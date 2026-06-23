import streamlit as st
import pandas as pd
import os
import json
import urllib.request
import urllib.parse
import math
import zipfile
import xml.etree.ElementTree as ET
from datetime import datetime
import folium
from folium.plugins import Draw, PolyLineTextPath
from folium.features import DivIcon
from streamlit_folium import st_folium
from branca.element import Element

# --- RASTERIO SAFELOAD ---
try:
    import rasterio
    from rasterio.warp import transform, transform_bounds
    RASTERIO_AVAILABLE = True
except ImportError:
    RASTERIO_AVAILABLE = False

# ==========================================
# CONSTANTS & SETUP
# ==========================================
FT_TO_M = 0.3048
M_TO_FT = 3.28084
MPH_TO_MS = 0.44704
MS_TO_MPH = 2.23694

MISSION_DIR = "missions"
SURFACES_DIR = "surfaces"
os.makedirs(MISSION_DIR, exist_ok=True)
os.makedirs(SURFACES_DIR, exist_ok=True)

if "locked_creator_center" not in st.session_state:
    st.session_state.locked_creator_center = [40.246860, -111.648667]
if "locked_editor_center" not in st.session_state:
    st.session_state.locked_editor_center = [40.246860, -111.648667]
if "locked_viewer_center" not in st.session_state:
    st.session_state.locked_viewer_center = [40.246860, -111.648667]

# Mavic 3 Multispectral Sensor Specs
SENSOR_W = 17.3  
SENSOR_H = 13.0
FOCAL_L = 12.3   
IMAGE_W = 5280   

CAM_DISPLAY_MAP = {
    "visible": "RGB Only",
    "narrow_band": "Multispectral Only",
    "visible,narrow_band": "RGB + Multispectral"
}
CAM_VAL_MAP = {v: k for k, v in CAM_DISPLAY_MAP.items()}

HARDWARE_MAP = {
    "Mavic 3 Multispectral (Drone: 0, Payload: 3)": {"drone_sub": "0", "payload_sub": "3"}
}

# ==========================================
# EXTERNAL DATA & ELEVATION HELPERS
# ==========================================
@st.cache_data(ttl=3600)
def fetch_uasfm_data(center_lat, center_lon, radius_deg=0.05):
    xmin = center_lon - radius_deg
    ymin = center_lat - radius_deg
    xmax = center_lon + radius_deg
    ymax = center_lat + radius_deg
    params = {
        "where": "1=1", "geometry": f"{xmin},{ymin},{xmax},{ymax}", "geometryType": "esriGeometryEnvelope",
        "inSR": "4326", "spatialRel": "esriSpatialRelIntersects", "outFields": "*", "outSR": "4326", "f": "geojson"
    }
    url = f"https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/FAA_UAS_FacilityMap_Data/FeatureServer/0/query?{urllib.parse.urlencode(params)}"
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        response = urllib.request.urlopen(req)
        data = json.loads(response.read().decode('utf-8'))
        if "features" in data and len(data["features"]) == 0:
            return {"type": "FeatureCollection", "features": []}
        return data if "error" not in data else None
    except Exception:
        return None

def get_haversine_dist(p1, p2):
    R = 6371000
    lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
    dlat, dlon = lat1 - lat2, lon1 - lon2
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def get_bearing(p1, p2):
    lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
    d_lon = lon2 - lon1
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

@st.cache_data(show_spinner=False, ttl=3600)
def get_elevations_open_elev(coords):
    url = "https://api.open-elevation.com/api/v1/lookup"
    locations = [{"latitude": lat, "longitude": lon} for lat, lon in coords]
    data = json.dumps({"locations": locations}).encode('utf-8')
    try:
        req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json', 'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req) as response:
            res_json = json.loads(response.read().decode('utf-8'))
            return [result['elevation'] for result in res_json['results']]
    except Exception as e:
        st.warning(f"Open-Elevation error: {e}")
        return [0] * len(coords)

@st.cache_data(show_spinner=False, ttl=3600)
def get_elevations_usgs(coords):
    elevations = []
    for lat, lon in coords:
        url = f"https://epqs.nationalmap.gov/v1/json?x={lon}&y={lat}&wkid=4326&units=Meters&includeDate=false"
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req) as response:
                res_json = json.loads(response.read().decode('utf-8'))
                val = res_json.get('value', 0)
                elevations.append(0 if str(val).lower() == 'null' or str(val).strip() == '' else float(val))
        except Exception as e:
            elevations.append(0)
    return elevations

@st.cache_data(show_spinner=False, ttl=3600)
def get_elevations_raster(coords, tif_path):
    if not RASTERIO_AVAILABLE:
        return [0] * len(coords)
    elevations = []
    try:
        with rasterio.open(tif_path) as src:
            nodata = src.nodata
            lons = [c[1] for c in coords]
            lats = [c[0] for c in coords]
            xs, ys = transform('EPSG:4326', src.crs, lons, lats)
            pts = list(zip(xs, ys))
            
            for val in src.sample(pts):
                v = float(val[0])
                if nodata is not None and math.isclose(v, nodata, rel_tol=1e-5):
                    elevations.append(0.0) # Fallback if point is outside the raster bounds
                else:
                    elevations.append(v)
    except Exception as e:
        st.warning(f"Error reading GeoTIFF: {e}")
        return [0] * len(coords)
    return elevations

def get_tif_bounds_wgs84(tif_path):
    if not RASTERIO_AVAILABLE:
        return None
    try:
        with rasterio.open(tif_path) as src:
            bounds = src.bounds
            wgs_bounds = transform_bounds(src.crs, 'EPSG:4326', *bounds)
            # Returns (min_lon, min_lat, max_lon, max_lat) -> we want [[min_lat, min_lon], [max_lat, max_lon]]
            return [[wgs_bounds[1], wgs_bounds[0]], [wgs_bounds[3], wgs_bounds[2]]]
    except Exception:
        return None

def get_elevations_batch(coords, source, tif_path=None):
    if not coords:
        return []
    if source == "USGS 3DEP (US High-Res)":
        return get_elevations_usgs(coords)
    elif source == "Local GeoTIFF" and tif_path and os.path.exists(tif_path):
        return get_elevations_raster(coords, tif_path)
    else:
        return get_elevations_open_elev(coords)

# ==========================================
# 3D ROTATION MATRIX FOOTPRINT CALCULATOR
# ==========================================
def get_photo_footprint(lat, lon, alt_ft, pitch, yaw):
    w, h, f = SENSOR_W, SENSOR_H, FOCAL_L
    corners = [(-w/2, h/2, -f), (w/2, h/2, -f), (w/2, -h/2, -f), (-w/2, -h/2, -f)]
    yaw_rad = math.radians(yaw)
    pitch_rad = math.radians(pitch + 90) 
    
    Rz = [[math.cos(yaw_rad), math.sin(yaw_rad), 0],
          [-math.sin(yaw_rad), math.cos(yaw_rad), 0], [0, 0, 1]]
    Rx = [[1, 0, 0],
          [0, math.cos(pitch_rad), -math.sin(pitch_rad)], [0, math.sin(pitch_rad), math.cos(pitch_rad)]]
    
    R = [[Rz[0][0]*Rx[0][0] + Rz[0][1]*Rx[1][0] + Rz[0][2]*Rx[2][0], 
          Rz[0][0]*Rx[0][1] + Rz[0][1]*Rx[1][1] + Rz[0][2]*Rx[2][1],
          Rz[0][0]*Rx[0][2] + Rz[0][1]*Rx[1][2] + Rz[0][2]*Rx[2][2]],
         [Rz[1][0]*Rx[0][0] + Rz[1][1]*Rx[1][0] + Rz[1][2]*Rx[2][0], 
          Rz[1][0]*Rx[0][1] + Rz[1][1]*Rx[1][1] + Rz[1][2]*Rx[2][1],
          Rz[1][0]*Rx[0][2] + Rz[1][1]*Rx[1][2] + Rz[1][2]*Rx[2][2]],
         [Rz[2][0]*Rx[0][0] + Rz[2][1]*Rx[1][0] + Rz[2][2]*Rx[2][0], 
          Rz[2][0]*Rx[0][1] + Rz[2][1]*Rx[1][1] + Rz[2][2]*Rx[2][1],
          Rz[2][0]*Rx[0][2] + Rz[2][1]*Rx[1][2] + Rz[2][2]*Rx[2][2]]]

    R_earth_ft = 20925646.3
    final_corners = []
    for corner in corners:
        ray = [R[0][0]*corner[0] + R[0][1]*corner[1] + R[0][2]*corner[2],
               R[1][0]*corner[0] + R[1][1]*corner[1] + R[1][2]*corner[2],
               R[2][0]*corner[0] + R[2][1]*corner[1] + R[2][2]*corner[2]]
        if ray[2] == 0: 
            continue
        t = -alt_ft / ray[2]
        dx_ft, dy_ft = ray[0] * t, ray[1] * t
        dlat = math.degrees(dy_ft / R_earth_ft)
        dlon = math.degrees(dx_ft / (R_earth_ft * math.cos(math.radians(lat))))
        final_corners.append([lat + dlat, lon + dlon])
    return final_corners

# ==========================================
# SESSION STATE INITIALIZATION & CALLBACKS
# ==========================================
if "alt_ft" not in st.session_state: st.session_state.alt_ft = 50.0
if "pitch" not in st.session_state: st.session_state.pitch = -60
if "trigger_type" not in st.session_state: st.session_state.trigger_type = "distance"
if "t_dist_val" not in st.session_state: st.session_state.t_dist_val = 9.0
if "target_gap_ft" not in st.session_state: st.session_state.target_gap_ft = 26.2
if "overlap_pct" not in st.session_state: st.session_state.overlap_pct = 70.0

if "e_alt_ft" not in st.session_state: st.session_state.e_alt_ft = 50.0
if "e_pitch" not in st.session_state: st.session_state.e_pitch = -60
if "e_trigger_type" not in st.session_state: st.session_state.e_trigger_type = "distance"
if "e_t_dist_val" not in st.session_state: st.session_state.e_t_dist_val = 9.0
if "e_target_gap_ft" not in st.session_state: st.session_state.e_target_gap_ft = 26.2
if "e_overlap_pct" not in st.session_state: st.session_state.e_overlap_pct = 70.0

def get_center_footprint(pitch, alt):
    if pitch == 0: return 999999.0  
    slant_dist = alt / math.sin(math.radians(abs(pitch)))
    return slant_dist * (SENSOR_W / FOCAL_L)

def sync_dist_to_overlap():
    fw = get_center_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    if fw > 0: st.session_state.overlap_pct = max(0.0, min(((fw - st.session_state.get('t_dist_val', 9.0)) / fw) * 100, 99.9))

def sync_overlap_to_dist():
    fw = get_center_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    st.session_state.t_dist_val = fw * (1 - (st.session_state.get('overlap_pct', 70.0) / 100))

def sync_gap_to_overlap():
    fw = get_center_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    if fw > 0: st.session_state.overlap_pct = max(0.0, min(((fw - st.session_state.get('target_gap_ft', 26.2)) / fw) * 100, 99.9))

def sync_overlap_to_gap():
    fw = get_center_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    st.session_state.target_gap_ft = fw * (1 - (st.session_state.get('overlap_pct', 70.0) / 100))

def sync_geometry():
    if st.session_state.get('trigger_type', 'distance') == 'distance': sync_dist_to_overlap()
    else: sync_gap_to_overlap()

def e_sync_dist_to_overlap():
    fw = get_center_footprint(st.session_state.get('e_pitch', -60.0), st.session_state.get('e_alt_ft', 50.0))
    if fw > 0: st.session_state.e_overlap_pct = max(0.0, min(((fw - st.session_state.get('e_t_dist_val', 9.0)) / fw) * 100, 99.9))

def e_sync_overlap_to_dist():
    fw = get_center_footprint(st.session_state.get('e_pitch', -60.0), st.session_state.get('e_alt_ft', 50.0))
    st.session_state.e_t_dist_val = fw * (1 - (st.session_state.get('e_overlap_pct', 70.0) / 100))

def e_sync_geometry():
    if st.session_state.get('e_trigger_type', 'distance') == 'distance': e_sync_dist_to_overlap()
    else: 
        fw = get_center_footprint(st.session_state.get('e_pitch', -60.0), st.session_state.get('e_alt_ft', 50.0))
        if fw > 0: st.session_state.e_overlap_pct = max(0.0, min(((fw - st.session_state.get('e_target_gap_ft', 26.2)) / fw) * 100, 99.9))

# ==========================================
# DATA EXTRACTION (FOR EDITOR)
# ==========================================
def parse_kmz_for_editing(full_path):
    ns = {'kml': 'http://www.opengis.net/kml/2.2', 'wpml': 'http://www.dji.com/wpmz/1.0.6'}
    meta = {
        "safe_takeoff_ft": 60.0, "trans_speed_mph": 22.0, "speed_m": 4.11, "speed_mph": 6.0,
        "alt_ft": 50.0, "pitch": -60.0, "trigger_type": "distance", 
        "t_val": 9.0, "photo_start_wp": 0, "coords": [], "camera_type": "visible",
        "drone_sub": "0", "payload_sub": "3"
    }
    with zipfile.ZipFile(full_path, 'r') as kmz:
        root_w = ET.fromstring(kmz.read('wpmz/waylines.wpml'))
        root_t = ET.fromstring(kmz.read('wpmz/template.kml'))
        
        safe_node = root_w.find('.//wpml:takeOffSecurityHeight', ns)
        if safe_node is not None: meta['safe_takeoff_ft'] = float(safe_node.text) * M_TO_FT
        trans_node = root_w.find('.//wpml:globalTransitionalSpeed', ns)
        if trans_node is not None: meta['trans_speed_mph'] = float(trans_node.text) * MS_TO_MPH
        speed_node = root_w.find('.//wpml:autoFlightSpeed', ns)
        if speed_node is not None: 
            meta['speed_m'] = float(speed_node.text)
            meta['speed_mph'] = float(speed_node.text) * MS_TO_MPH

        drone_info = root_t.find('.//wpml:droneInfo', ns)
        if drone_info is not None:
            d_sub = drone_info.find('.//wpml:droneSubEnumValue', ns)
            if d_sub is not None and d_sub.text: meta['drone_sub'] = d_sub.text
            
        payload_info = root_t.find('.//wpml:payloadInfo', ns)
        if payload_info is not None:
            p_sub = payload_info.find('.//wpml:payloadSubEnumValue', ns)
            if p_sub is not None and p_sub.text: meta['payload_sub'] = p_sub.text

        hw_key = "Mavic 3 Multispectral (Drone: 0, Payload: 3)"
        for k, v in HARDWARE_MAP.items():
            if v["drone_sub"] == meta['drone_sub'] and v["payload_sub"] == meta['payload_sub']:
                hw_key = k
        meta['hardware_key'] = hw_key

        pms = root_w.findall('.//kml:Placemark', ns)
        for i, pm in enumerate(pms):
            c_raw = pm.find('.//kml:coordinates', ns).text.strip().split(',')
            meta['coords'].append((float(c_raw[1]), float(c_raw[0]))) 
            
            if i == 0:
                alt_node = pm.find('.//wpml:executeHeight', ns)
                if alt_node is not None: meta['alt_ft'] = float(alt_node.text) * M_TO_FT
                pitch_node = pm.find('.//wpml:waypointGimbalHeadingParam/wpml:waypointGimbalPitchAngle', ns)
                if pitch_node is not None: meta['pitch'] = float(pitch_node.text)

            for ag in pm.findall('.//wpml:actionGroup', ns):
                t_type = ag.find('.//wpml:actionTriggerType', ns)
                if t_type is not None and 'multiple' in t_type.text:
                    meta['trigger_type'] = "distance" if "Distance" in t_type.text else "time"
                    t_param = ag.find('.//wpml:actionTriggerParam', ns)
                    if t_param is not None:
                        val = float(t_param.text)
                        meta['t_val'] = val * M_TO_FT if meta['trigger_type'] == 'distance' else val
                    start_idx = ag.find('.//wpml:actionGroupStartIndex', ns)
                    if start_idx is not None: meta['photo_start_wp'] = int(start_idx.text)
                
                for a in ag.findall('.//wpml:action', ns):
                    func = a.find('.//wpml:actionActuatorFunc', ns)
                    if func is not None and func.text == 'takePhoto':
                        params = a.find('.//wpml:actionActuatorFuncParam', ns)
                        if params is not None:
                            lens = params.find('.//wpml:payloadLensIndex', ns)
                            if lens is not None: meta['camera_type'] = lens.text
    return meta

# ==========================================
# CORE MISSION GENERATOR
# ==========================================
def generate_native_kmz_contents(coords, cfg, elev_source, tif_path):
    ms_ts = int(datetime.now().timestamp() * 1000)
    
    elevations = get_elevations_batch(coords, elev_source, tif_path)
    start_elev = elevations[0] if elevations else 0
    target_agl_m = cfg["alt_ft"] * FT_TO_M
    
    safe_m = cfg["safe_takeoff_ft"] * FT_TO_M
    trans_m = cfg["trans_speed_mph"] * MPH_TO_MS
    speed_m = cfg["speed_m"]
    lens_str = cfg.get("camera_type", "visible")
    drone_sub_enum = cfg.get("drone_sub", "0")
    payload_sub_enum = cfg.get("payload_sub", "3")
    
    total_dist_m = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1))
    total_duration = total_dist_m / speed_m if speed_m > 0 else 0
    pitch_val = cfg['pitch'] if 'pitch' in cfg else cfg.get('gimbal_pitch', -60)

    yaws = []
    for i in range(len(coords) - 1):
        ref_bearing = get_bearing(coords[i], coords[i+1])
        yaw = (ref_bearing + 90) % 360 if cfg['side'] == "right" else (ref_bearing - 90) % 360
        if yaw > 180: yaw -= 360
        yaws.append(int(yaw))

    template_placemarks = ""
    waylines_placemarks = ""
    g_id_template = 0  
    g_id_waylines = 0  

    for i, p in enumerate(coords):
        current_elev = elevations[i] if i < len(elevations) else start_elev
        terrain_diff = current_elev - start_elev
        alt_m = target_agl_m + terrain_diff
        
        current_yaw = yaws[0] if i == 0 else yaws[i-1]
        template_action_group = ""
        waylines_action_group = ""

        if i == 0:
            waylines_action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{g_id_waylines}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalHeadingYawBase>aircraft</wpml:gimbalHeadingYawBase>
              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
              <wpml:gimbalPitchRotateAngle>{pitch_val}</wpml:gimbalPitchRotateAngle>
              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>
              <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
              <wpml:gimbalRotateTime>10</wpml:gimbalRotateTime>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            g_id_waylines += 1

        if 0 < i < len(coords) - 1:
            next_yaw = yaws[i]
            diff = (next_yaw - current_yaw + 180) % 360 - 180
            path_mode = "clockwise" if diff >= 0 else "counterClockwise"
            
            yaw_action_block = f"""
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>rotateYaw</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:aircraftHeading>{next_yaw}</wpml:aircraftHeading>
              <wpml:aircraftPathMode>{path_mode}</wpml:aircraftPathMode>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            template_action_group += f"\n        <wpml:actionGroup>\n          <wpml:actionGroupId>{g_id_template}</wpml:actionGroupId>{yaw_action_block}"
            waylines_action_group += f"\n        <wpml:actionGroup>\n          <wpml:actionGroupId>{g_id_waylines}</wpml:actionGroupId>{yaw_action_block}"
            g_id_template += 1
            g_id_waylines += 1

        start_wp = cfg.get("photo_start_wp", 0)
        if i == start_wp:
            trigger_tag = "multipleDistance" if cfg["trigger_type"] == "distance" else "multipleTiming"
            interval_val = max(1.0, cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['interval_sec']
            
            photo_action_block = f"""
          <wpml:actionGroupStartIndex>{start_wp}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{len(coords)-1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>{trigger_tag}</wpml:actionTriggerType><wpml:actionTriggerParam>{interval_val:.2f}</wpml:actionTriggerParam></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
              <wpml:useGlobalPayloadLensIndex>0</wpml:useGlobalPayloadLensIndex>
              <wpml:payloadLensIndex>{lens_str}</wpml:payloadLensIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            template_action_group += f"\n        <wpml:actionGroup>\n          <wpml:actionGroupId>{g_id_template}</wpml:actionGroupId>{photo_action_block}"
            waylines_action_group += f"\n        <wpml:actionGroup>\n          <wpml:actionGroupId>{g_id_waylines}</wpml:actionGroupId>{photo_action_block}"
            g_id_template += 1
            g_id_waylines += 1

        if i < len(coords) - 1:
            waylines_action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{g_id_waylines}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i+1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>betweenAdjacentPoints</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalEvenlyRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalPitchRotateAngle>{pitch_val}</wpml:gimbalPitchRotateAngle>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            g_id_waylines += 1

        template_placemarks += f"""
      <Placemark>
        <Point><coordinates>{p[1]:.8f},{p[0]:.8f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:ellipsoidHeight>{alt_m:.1f}</wpml:ellipsoidHeight>
        <wpml:height>{alt_m:.1f}</wpml:height>
        <wpml:useGlobalHeight>0</wpml:useGlobalHeight>
        <wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>
        <wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>smoothTransition</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{current_yaw}</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
        </wpml:waypointHeadingParam>
        <wpml:gimbalPitchAngle>{pitch_val}</wpml:gimbalPitchAngle>
        <wpml:isRisky>0</wpml:isRisky>{template_action_group}
      </Placemark>"""

        waylines_placemarks += f"""
      <Placemark>
        <Point><coordinates>{p[1]:.8f},{p[0]:.8f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{alt_m:.1f}</wpml:executeHeight>
        <wpml:waypointSpeed>{speed_m:.2f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>smoothTransition</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{current_yaw}</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingAngleEnable>1</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>1</wpml:useStraightLine>
        <wpml:waypointGimbalHeadingParam>
          <wpml:waypointGimbalPitchAngle>{pitch_val}</wpml:waypointGimbalPitchAngle>
          <wpml:waypointGimbalYawAngle>0</wpml:waypointGimbalYawAngle>
        </wpml:waypointGimbalHeadingParam>
        <wpml:isRisky>0</wpml:isRisky>
        <wpml:waypointWorkType>0</wpml:waypointWorkType>{waylines_action_group}
      </Placemark>"""

    template_kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <wpml:createTime>{ms_ts}</wpml:createTime>
    <wpml:updateTime>{ms_ts}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>{safe_m:.1f}</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>{trans_m:.1f}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>77</wpml:droneEnumValue><wpml:droneSubEnumValue>{drone_sub_enum}</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:payloadInfo><wpml:payloadEnumValue>68</wpml:payloadEnumValue><wpml:payloadSubEnumValue>{payload_sub_enum}</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineCoordinateSysParam>
        <wpml:coordinateMode>WGS84</wpml:coordinateMode>
        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>
        <wpml:positioningType>GPS</wpml:positioningType>
      </wpml:waylineCoordinateSysParam>
      <wpml:autoFlightSpeed>{speed_m:.1f}</wpml:autoFlightSpeed>

      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>
      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>
      <wpml:payloadParam>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
        <wpml:imageFormat>{lens_str}</wpml:imageFormat>
      </wpml:payloadParam>
      <wpml:globalWaypointHeadingParam>
        <wpml:waypointHeadingMode>manually</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
        <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
        <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
      </wpml:globalWaypointHeadingParam>
      <wpml:globalWaypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:globalWaypointTurnMode>
      <wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>
      {template_placemarks}
    </Folder>
  </Document>
</kml>"""

    waylines_wpml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <wpml:createTime>{ms_ts}</wpml:createTime>
    <wpml:updateTime>{ms_ts}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>{safe_m:.1f}</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>{trans_m:.1f}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>77</wpml:droneEnumValue><wpml:droneSubEnumValue>{drone_sub_enum}</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:waylineAvoidLimitAreaMode>0</wpml:waylineAvoidLimitAreaMode>
      <wpml:payloadInfo><wpml:payloadEnumValue>68</wpml:payloadEnumValue><wpml:payloadSubEnumValue>{payload_sub_enum}</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>{total_dist_m:.2f}</wpml:distance>
      <wpml:duration>{total_duration:.2f}</wpml:duration>
      <wpml:autoFlightSpeed>{speed_m:.1f}</wpml:autoFlightSpeed>
      <wpml:payloadParam>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
        <wpml:imageFormat>{lens_str}</wpml:imageFormat>
      </wpml:payloadParam>
      {waylines_placemarks}
    </Folder>
  </Document>
</kml>"""

    return template_kml, waylines_wpml

# ==========================================
# UI NAVIGATION & LOGIC
# ==========================================
st.set_page_config(layout="wide", page_title="Flight Planner")
st.title("DJI Flight Planner")
page = st.radio("Navigation", ["Creator", "Editor", "Viewer"], horizontal=True, label_visibility="collapsed")

# --- CREATOR MODE ---
if page == 'Creator':
    with st.sidebar:
        st.header("1. Global Config")
        mission_name = st.text_input("Filename", "Mission_Flight")
        trans_speed_mph = st.number_input("Takeoff Speed (mph)", value=22.0)
        safe_takeoff_ft = st.number_input("Safe Takeoff Alt (ft)", value=60.0)
        
        st.header("2. Waypoint Settings")
        st.number_input("Relative Altitude (ft)", key="alt_ft", on_change=sync_geometry)
        st.info("❗Elevation is relative to the take off point, NOT the mission start point.")
        
        c_elev_source = st.selectbox("Elevation Source", ["Open-Elevation (Global)", "USGS 3DEP (US High-Res)", "Local GeoTIFF"], key="c_source")
        
        if c_elev_source == "Open-Elevation (Global)":
                st.warning("⚠️ Can be off by several dozen feet. Use with caution.")
        
        if c_elev_source == "USGS 3DEP (US High-Res)":
            st.warning("⏱️ USGS parses coordinates individually. Generating long missions may take a a copule minutes.")
            
        c_tif_path = None
        c_show_bounds = False
        if c_elev_source == "Local GeoTIFF":
            if not RASTERIO_AVAILABLE:
                st.error("Missing 'rasterio' library. Run `pip install rasterio pyproj` to use local GeoTIFFs.")
            else:
                tif_files = [f for f in os.listdir(SURFACES_DIR) if f.endswith((".tif", ".tiff"))]
                if tif_files:
                    selected_tif = st.selectbox("Select Surface File", tif_files, key="c_tif")
                    c_tif_path = os.path.join(SURFACES_DIR, selected_tif)
                    c_show_bounds = st.checkbox("Show GeoTIFF Boundaries on Map", value=True, key="c_bounds")
                else:
                    st.warning("No .tif files found in the 'surfaces' folder.")
        # -------------------------------

        st.slider("Gimbal Pitch (°)", -90, 0, key="pitch", on_change=sync_geometry)
        
        pitch_rad = math.radians(abs(st.session_state.pitch))
        D_ft_c = st.session_state.alt_ft / math.sin(pitch_rad) if pitch_rad > 0 else float('inf')
        gsd_cm = (D_ft_c * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_c != float('inf') else 0
        st.info(f"📐 Est. Ground GSD: {gsd_cm:.2f} cm/px")
        
        side = st.selectbox("Side of flight path", ["right", "left"])
        
        st.header("3. Hardware & Payload")
        hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()))
        drone_sub_enum = HARDWARE_MAP[hw_choice]["drone_sub"]
        payload_sub_enum = HARDWARE_MAP[hw_choice]["payload_sub"]
        
        cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"])
        camera_type = CAM_VAL_MAP[cam_choice]
        
        min_photo_interval_sec = 2.0 if "narrow_band" in camera_type else 0.7

        st.header("4. Trigger & Speed")
        photo_start_wp = st.number_input("Start Photos at Waypoint Index", min_value=0, value=0, step=1)
        st.radio("Type", ["distance", "time"], key="trigger_type", on_change=sync_geometry)
        
        if st.session_state.trigger_type == "distance":
            st.number_input("Interval (ft)", key="t_dist_val", min_value=1.0, on_change=sync_dist_to_overlap)
            st.number_input("Forward Overlap (%)", key="overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=sync_overlap_to_dist)
            manual_mph = st.number_input("Flight Speed (mph)", min_value=2.3, value=6.0)
            speed_m = manual_mph * MPH_TO_MS
            
            gap_m = max(1.0, st.session_state.t_dist_val * FT_TO_M)
            max_speed_m = gap_m / min_photo_interval_sec
            if speed_m > max_speed_m:
                st.error(f"⚠️ Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
        else:
            t_val_sec = st.number_input("Interval (sec)", min_value=min_photo_interval_sec, value=max(2.0, min_photo_interval_sec))
            auto_speed = st.checkbox("Auto-Calc Speed", True)
            if auto_speed:
                st.number_input("Target Gap (ft)", key="target_gap_ft", min_value=1.0, on_change=sync_gap_to_overlap)
                st.number_input("Forward Overlap (%)", key="overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=sync_overlap_to_gap)
                speed_m = min(max((st.session_state.target_gap_ft * FT_TO_M) / t_val_sec, 1.0), 10.0)
                st.info(f"🤖 Auto-Calculated Speed: {speed_m * MS_TO_MPH:.1f} mph")
            else:
                manual_mph = st.number_input("Manual Speed (mph)", min_value=2.3, value=6.0)
                speed_m = manual_mph * MPH_TO_MS
                current_gap = speed_m * M_TO_FT * t_val_sec
                fw = get_center_footprint(st.session_state.pitch, st.session_state.alt_ft)
                current_overlap = ((fw - current_gap) / fw) * 100 if fw > 0 else 0
                st.info(f"📊 Current Overlap: {max(0, min(current_overlap, 99.9)):.1f}%")

        st.header("5. Visuals")
        show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="creator_faa_toggle")
        if show_faa_airspace:
            st.write("#### Update restrictions of map center")
            if st.button("Update Map Center", key="btn_update_creator"):
                st.session_state.locked_creator_center = st.session_state.creator_center
                st.rerun()

    top_hud = st.container()

    m = folium.Map(location=st.session_state.locked_creator_center, zoom_start=17, tiles=None)
    folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m)
    
    
    if c_elev_source == "Local GeoTIFF" and c_tif_path and c_show_bounds:
        bounds = get_tif_bounds_wgs84(c_tif_path)
        if bounds:
            folium.Rectangle(
                bounds=bounds, color="#ff8800", weight=3, fill=True, fill_opacity=0.1,
                tooltip="Active GeoTIFF Boundary"
            ).add_to(m)
    

    if show_faa_airspace:
        uasfm_data = fetch_uasfm_data(st.session_state.locked_creator_center[0], st.session_state.locked_creator_center[1])
        if uasfm_data and uasfm_data.get("features"):
            folium.GeoJson(
                uasfm_data, name="FAA UASFM Grids",
                style_function=lambda x: {'fillColor': 'red' if x['properties'].get('CEILING', x['properties'].get('ceiling', -1)) == 0 else 'green', 'color': 'black', 'weight': 1, 'fillOpacity': 0.15},
                tooltip=folium.GeoJsonTooltip(fields=['CEILING'], aliases=['Max LAANC Altitude (ft):'])
            ).add_to(m)
        elif uasfm_data: 
            folium.Marker(st.session_state.locked_creator_center, icon=DivIcon(html='<div style="font-size: 12px; color: grey;">No FAA restrictions at this location</div>')).add_to(m)

    Draw(export=False, draw_options={'polyline':{'shapeOptions':{'color':'#00ffff','weight':5}}}).add_to(m)
    map_data = st_folium(m, width=1200, height=600, key=f"creator_map_{show_faa_airspace}_{c_elev_source}_{c_show_bounds}")

    if map_data and map_data.get("center"):
        st.session_state.creator_center = [map_data["center"]["lat"], map_data["center"]["lng"]]
        st.session_state.creator_zoom = map_data["zoom"]
        c_lat, c_lon = st.session_state.creator_center
        st.info(f"📍 Current Screen Center: **{c_lat:.6f}, {c_lon:.6f}** (Click 'Update' in sidebar to update restrictions in this area)")

    if map_data.get("all_drawings"):
        coords = [(c[1], c[0]) for c in map_data["all_drawings"][-1]['geometry']['coordinates']]
        total_dist_ft = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1)) * M_TO_FT
        
        gap_ft = max(1.0, st.session_state.t_dist_val * M_TO_FT) if st.session_state.trigger_type == "distance" else speed_m * t_val_sec * M_TO_FT
            
        if len(coords) > photo_start_wp:
            dist_to_start = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(photo_start_wp)) * M_TO_FT
            est_photos = int(max(0, total_dist_ft - dist_to_start) / gap_ft) + 1 if gap_ft > 0 else 0
        else:
            est_photos = 0

        with top_hud:
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}")
            c3.metric("Flight Speed", f"{speed_m * MS_TO_MPH:.1f} mph")
            st.write("---")
            
            existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d))]
            save_col1, save_col2, save_col3 = st.columns([2, 2, 2])
            with save_col1:
                save_option = st.selectbox("Save Destination", ["Root (missions/)", "Create New Folder...", "Custom Path..."] + existing_dirs)
            with save_col2:
                new_dir_name = st.text_input("New Folder Name", "New_Project") if save_option == "Create New Folder..." else ""
                custom_path_name = st.text_input("Absolute File Path", "/Users/connor/Desktop/") if save_option == "Custom Path..." else ""
            with save_col3:
                st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
                save_clicked = st.button("🚀 Save & Generate KMZ", use_container_width=True)

            if save_clicked:
                with st.spinner("Calculating terrain elevations and generating KMZ..."):
                    cfg = {
                        "safe_takeoff_ft": safe_takeoff_ft, "trans_speed_mph": trans_speed_mph, 
                        "alt_ft": st.session_state.alt_ft, "pitch": st.session_state.pitch, "side": side, 
                        "trigger_type": st.session_state.trigger_type, 
                        "interval_ft": st.session_state.t_dist_val if st.session_state.trigger_type == "distance" else 0, 
                        "interval_sec": t_val_sec if st.session_state.trigger_type == "time" else 0, 
                        "speed_m": speed_m, "photo_start_wp": int(photo_start_wp),
                        "camera_type": camera_type, "drone_sub": drone_sub_enum, "payload_sub": payload_sub_enum
                    }
                    
                    suffix = f"_H{int(st.session_state.alt_ft)}A{int(abs(st.session_state.pitch))}OL{int(st.session_state.overlap_pct)}"
                    final_filename = f"{mission_name}{suffix}"

                    if save_option == "Root (missions/)": final_dir = MISSION_DIR
                    elif save_option == "Create New Folder...": final_dir = os.path.join(MISSION_DIR, new_dir_name)
                    elif save_option == "Custom Path...": final_dir = custom_path_name
                    else: final_dir = os.path.join(MISSION_DIR, save_option)
                        
                    os.makedirs(final_dir, exist_ok=True)
                    final_filepath = os.path.join(final_dir, f"{final_filename}.kmz")

                    template_kml, waylines_wpml = generate_native_kmz_contents(coords, cfg, c_elev_source, c_tif_path)
                    with zipfile.ZipFile(final_filepath, 'w') as kmz:
                        kmz.writestr("wpmz/waylines.wpml", waylines_wpml)
                        kmz.writestr("wpmz/template.kml", template_kml)
                    
                st.success(f"Saved {final_filename}.kmz to {final_dir}/")
            st.divider()

# --- EDITOR MODE ---
elif page == 'Editor':
    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d))]
    
    col_dir, col_file, col_new = st.columns([2, 3, 1])
    with col_dir: selected_dir_name = st.selectbox("Select Folder", ["Root (missions/)"] + existing_dirs, key="edit_dir")
        
    active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
    kmz_files = [f for f in os.listdir(active_dir) if f.endswith(".kmz")]

    if not kmz_files:
        st.warning(f"No missions found in {selected_dir_name}.")
    else:
        with col_file: selected_kmz = st.selectbox("Select Mission to Edit", kmz_files)
        with col_new:
            st.markdown("<div style='margin-top: 32px;'></div>", unsafe_allow_html=True)
            make_new_file = st.checkbox("Make new file?", value=False)
            
        edit_name = f"{selected_kmz.replace('.kmz', '')}-edited" if make_new_file else selected_kmz.replace('.kmz', '')
        full_path = os.path.join(active_dir, selected_kmz)
        
        if 'editor_kmz' not in st.session_state or st.session_state.editor_kmz != full_path:
            st.session_state.editor_kmz = full_path
            meta = parse_kmz_for_editing(full_path)
            st.session_state.meta = meta
            st.session_state.editor_key = str(datetime.now().timestamp())
            
            st.session_state.e_alt_ft = meta['alt_ft']
            st.session_state.e_pitch = int(meta['pitch'])
            st.session_state.e_trigger_type = meta['trigger_type']
            if meta['trigger_type'] == 'distance':
                st.session_state.e_t_dist_val = meta['t_val']
                st.session_state.e_target_gap_ft = 26.2
            else:
                st.session_state.e_target_gap_ft = meta['speed_m'] * M_TO_FT * meta['t_val']
                st.session_state.e_t_dist_val = 9.0
            e_sync_geometry()
            
        meta = st.session_state.meta
        
        with st.sidebar:
            st.info(f"💾 Will save as: {edit_name}.kmz")
            st.header("Modify Parameters")
            e_safe = st.number_input("Safe Takeoff Alt (ft)", value=meta['safe_takeoff_ft'])
            e_trans = st.number_input("Takeoff Speed (mph)", value=meta['trans_speed_mph'])
            st.number_input("Relative Altitude (ft)", key="e_alt_ft", on_change=e_sync_geometry)
        
            st.info("❗Elevation is relative to the take off point, NOT the mission start point.")
            
            e_elev_source = st.selectbox("Elevation Source", ["Open-Elevation (Global)", "USGS 3DEP (US High-Res)", "Local GeoTIFF"], key="e_source")
            
            if e_elev_source == "Open-Elevation (Global)":
                st.warning("⚠️ Can be off by several dozen feet. Use with caution.")
                
            elif e_elev_source == "USGS 3DEP (US High-Res)":
                st.warning("⏱️ USGS parses coordinates individually. Generating long missions may take a couple of minutes.")
                
            e_tif_path = None
            e_show_bounds = False
            if e_elev_source == "Local GeoTIFF":
                if not RASTERIO_AVAILABLE:
                    st.error("Missing 'rasterio' library. Run `pip install rasterio pyproj` to use local GeoTIFFs.")
                else:
                    tif_files = [f for f in os.listdir(SURFACES_DIR) if f.endswith((".tif", ".tiff"))]
                    if tif_files:
                        selected_tif = st.selectbox("Select Surface File", tif_files, key="e_tif")
                        e_tif_path = os.path.join(SURFACES_DIR, selected_tif)
                        e_show_bounds = st.checkbox("Show GeoTIFF Boundaries on Map", value=True, key="e_bounds")
                    else:
                        st.warning("No .tif files found in the 'surfaces' folder.")
            # ----------------------------------------

            st.slider("Gimbal Pitch (°)", -90, 0, key="e_pitch", on_change=e_sync_geometry)

            pitch_rad_e = math.radians(abs(st.session_state.e_pitch))
            D_ft_e = st.session_state.e_alt_ft / math.sin(pitch_rad_e) if pitch_rad_e > 0 else float('inf')
            st.info(f"📐 Est. Ground GSD: {(D_ft_e * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_e != float('inf') else 0:.2f} cm/px")
            
            e_side = st.selectbox("Yaw Side", ["right", "left"])
            
            st.header("Hardware & Payload")
            e_hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()), index=list(HARDWARE_MAP.keys()).index(meta.get('hardware_key', "Mavic 3 Multispectral (Drone: 0, Payload: 3)")))
            e_drone_sub_enum = HARDWARE_MAP[e_hw_choice]["drone_sub"]
            e_payload_sub_enum = HARDWARE_MAP[e_hw_choice]["payload_sub"]
            
            current_cam_display = CAM_DISPLAY_MAP.get(meta.get('camera_type', 'visible'), "RGB Only")
            e_cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"], index=["RGB Only", "Multispectral Only", "RGB + Multispectral"].index(current_cam_display))
            e_camera_type = CAM_VAL_MAP[e_cam_choice]
            
            min_photo_interval_sec = 2.0 if "narrow_band" in e_camera_type else 0.7

            st.header("Trigger Settings")
            e_start_wp = st.number_input("Start Photos at WP", min_value=0, value=meta['photo_start_wp'], step=1)
            e_trigger = st.radio("Type", ["distance", "time"], key="e_trigger_type", on_change=e_sync_geometry)
            safe_e_speed = max(2.3, float(meta.get('speed_mph', 6.0)))
            
            if st.session_state.e_trigger_type == "distance":
                st.number_input("Interval (ft)", key="e_t_dist_val", min_value=1.0, on_change=e_sync_dist_to_overlap)
                st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=e_sync_overlap_to_dist)
                e_speed_m = st.number_input("Flight Speed (mph)", min_value=2.3, value=safe_e_speed) * MPH_TO_MS
                
                gap_m = max(1.0, st.session_state.e_t_dist_val * FT_TO_M)
                max_speed_m = gap_m / min_photo_interval_sec
                if e_speed_m > max_speed_m:
                    st.error(f"⚠️ Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
            else:
                if 'e_t_time_val' not in st.session_state: 
                    st.session_state.e_t_time_val = meta['t_val'] if meta['trigger_type'] == 'time' else max(2.0, min_photo_interval_sec)
                e_tval_sec = st.number_input("Interval (sec)", key="e_t_time_val", min_value=min_photo_interval_sec)
                e_auto_speed = st.checkbox("Auto-Calc Speed", True)
                if e_auto_speed:
                    st.number_input("Target Gap (ft)", key="e_target_gap_ft", min_value=1.0, on_change=e_sync_gap_to_overlap)
                    st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=e_sync_overlap_to_gap)
                    e_speed_m = min(max((st.session_state.e_target_gap_ft * FT_TO_M) / e_tval_sec, 1.0), 10.0)
                    st.info(f"🤖 Auto-Calculated Speed: {e_speed_m * MS_TO_MPH:.1f} mph")
                else:
                    e_speed_m = st.number_input("Manual Speed (mph)", min_value=2.3, value=safe_e_speed) * MPH_TO_MS
                    fw = get_center_footprint(st.session_state.e_pitch, st.session_state.e_alt_ft)
                    current_overlap = ((fw - (e_speed_m * M_TO_FT * e_tval_sec)) / fw) * 100 if fw > 0 else 0
                    st.info(f"📊 Current Overlap: {max(0, min(current_overlap, 99.9)):.1f}%")

            st.header("Visuals")
            show_footprints = st.checkbox("Show Image Footprints", value=True)
            show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="editor_faa_toggle")
            if show_faa_airspace:
                st.write("#### Update restrictions of map center")
                if st.button("Update Map Center", key="btn_update_editor"):
                    st.session_state.locked_editor_center = st.session_state.editor_center
                    st.rerun()

        top_hud = st.container()
        st.write("### 📍 Fine-Tune Flight Path Coordinates")
        
        df = pd.DataFrame(meta['coords'], columns=['Latitude', 'Longitude'])
        edited_df = st.data_editor(df, num_rows="dynamic", key=st.session_state.editor_key, use_container_width=True)
        current_coords = [(row['Latitude'], row['Longitude']) for _, row in edited_df.iterrows()]

        m_edit = folium.Map(location=st.session_state.locked_editor_center, zoom_start=18, tiles=None)
        folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_edit)
        
        # --- RASTERIO BOUNDS DRAWER (EDITOR) ---
        if e_elev_source == "Local GeoTIFF" and e_tif_path and e_show_bounds:
            bounds = get_tif_bounds_wgs84(e_tif_path)
            if bounds:
                folium.Rectangle(bounds=bounds, color="#ff8800", weight=3, fill=True, fill_opacity=0.1, tooltip="Active GeoTIFF Boundary").add_to(m_edit)
        # ---------------------------------------

        if show_faa_airspace:
            uasfm_data = fetch_uasfm_data(st.session_state.locked_editor_center[0], st.session_state.locked_editor_center[1])
            if uasfm_data and uasfm_data.get("features"):
                folium.GeoJson(
                    uasfm_data, name="FAA UASFM Grids",
                    style_function=lambda x: {'fillColor': 'red' if x['properties'].get('CEILING', x['properties'].get('ceiling', -1)) == 0 else 'green', 'color': 'black', 'weight': 1, 'fillOpacity': 0.15},
                    tooltip=folium.GeoJsonTooltip(fields=['CEILING'], aliases=['Max LAANC Altitude (ft):'])
                ).add_to(m_edit)
            elif uasfm_data:
                folium.Marker(st.session_state.locked_editor_center, icon=DivIcon(html='<div style="font-size: 10px; color: grey; width: 150px;">No FAA restrictions at this location</div>')).add_to(m_edit)

        Draw(export=False, draw_options={'polyline':{'shapeOptions':{'color':'#00ffff','weight':5}}}).add_to(m_edit)
        line = folium.PolyLine(current_coords, color="#00ffff", weight=5).add_to(m_edit)
        PolyLineTextPath(line, '  ►  ', repeat=True, offset=7, attributes={'fill': '#000000', 'font-weight': 'bold', 'font-size': '24', 'fill-opacity': '0.3'}).add_to(m_edit)
        
        gap_ft_preview = max(1.0, st.session_state.e_t_dist_val * M_TO_FT) if st.session_state.e_trigger_type == "distance" else e_speed_m * st.session_state.e_t_time_val * M_TO_FT

        yaws = []
        for i in range(len(current_coords) - 1):
            ref_bearing = get_bearing(current_coords[i], current_coords[i+1])
            yaws.append((ref_bearing + 90) % 360 if e_side == "right" else (ref_bearing - 90) % 360)

        cum_dist = [0.0]
        total_dist_ft = 0.0
        
        # --- BATCH ELEVATION UPDATED WITH SOURCE ---
        elevations = get_elevations_batch(current_coords, e_elev_source, e_tif_path)
        start_elev = elevations[0] if elevations else 0
        target_agl_ft = st.session_state.e_alt_ft
        
        for i in range(len(current_coords) - 1):
            dist = get_haversine_dist(current_coords[i], current_coords[i+1]) * M_TO_FT
            total_dist_ft += dist
            cum_dist.append(total_dist_ft)
            
            elev_diff_ft = (elevations[i+1] - elevations[i]) * M_TO_FT if elevations else 0.0
            mid_lat = (current_coords[i][0] + current_coords[i+1][0]) / 2
            mid_lon = (current_coords[i][1] + current_coords[i+1][1]) / 2
            
            folium.Marker(
                location=[mid_lat, mid_lon],
                icon=DivIcon(
                    icon_size=(120, 40), icon_anchor=(60, 20),
                    html=f'<div style="font-size: 12pt; color: #ffffff; text-shadow: 2px 2px 4px #000000, -1px -1px 0 #000, 1px -1px 0 #000, -1px 1px 0 #000, 1px 1px 0 #000; font-weight: bold; text-align: center; line-height: 1.2;">{dist:.1f} ft<br><span style="font-size: 10pt; color: #00ffff;">Elev Dif: {elev_diff_ft:+.1f} ft</span></div>'
                )
            ).add_to(m_edit)

        for i, c in enumerate(current_coords):
            pt_elev = elevations[i] if elevations else 0
            pt_alt_ft = target_agl_ft + ((pt_elev - start_elev) * M_TO_FT)
            folium.Marker(
                location=c,
                tooltip=f"<b>Waypoint {i}</b><br>Lat: {c[0]:.6f}<br>Lon: {c[1]:.6f}<br>Alt: {pt_alt_ft:.1f} ft",
                icon=DivIcon(icon_size=(24,24), icon_anchor=(12,12), html=f'<div style="font-size: 11pt; color: black; background: white; border-radius: 50%; text-align: center; border: 2px solid black; font-weight: bold; width: 24px; height: 24px; line-height: 20px;">{i}</div>')
            ).add_to(m_edit)
            
        if show_footprints and gap_ft_preview > 0 and e_start_wp < len(current_coords):
            current_dist = cum_dist[int(e_start_wp)]
            while current_dist <= total_dist_ft + 0.01:
                for i in range(len(cum_dist) - 1):
                    if cum_dist[i] <= current_dist <= cum_dist[i+1] + 0.001:
                        seg_len = cum_dist[i+1] - cum_dist[i]
                        if seg_len > 0:
                            frac = (current_dist - cum_dist[i]) / seg_len
                            lat = current_coords[i][0] + (current_coords[i+1][0] - current_coords[i][0]) * frac
                            lon = current_coords[i][1] + (current_coords[i+1][1] - current_coords[i][1]) * frac
                        else:
                            lat, lon = current_coords[i][0], current_coords[i][1]
                        
                        footprint = get_photo_footprint(lat, lon, st.session_state.e_alt_ft, st.session_state.e_pitch, yaws[i])
                        folium.Polygon(locations=footprint, color="darkorange", weight=1, fill=True, fill_opacity=0.15).add_to(m_edit)
                        folium.CircleMarker([lat, lon], radius=2.5, color="yellow", fill=True).add_to(m_edit)
                        break
                current_dist += gap_ft_preview

        map_data_edit = st_folium(m_edit, width=1200, height=600, key=f"editor_map_{show_faa_airspace}_{show_footprints}_{e_elev_source}_{e_show_bounds}")

        if map_data_edit and map_data_edit.get("center"):
            st.session_state.editor_center = [map_data_edit["center"]["lat"], map_data_edit["center"]["lng"]]
            st.session_state.editor_zoom = map_data_edit["zoom"]
            c_lat, c_lon = st.session_state.editor_center
            st.info(f"📍 Current Screen Center: **{c_lat:.6f}, {c_lon:.6f}**")

        final_coords = [(c[1], c[0]) for c in map_data_edit["all_drawings"][-1]['geometry']['coordinates']] if map_data_edit.get("all_drawings") and len(map_data_edit["all_drawings"]) > 0 else current_coords
        if map_data_edit.get("all_drawings") and len(map_data_edit["all_drawings"]) > 0: st.info("✏️ Using newly drawn line from the map.")

        with top_hud:
            total_dist_ft = sum(get_haversine_dist(final_coords[i], final_coords[i+1]) for i in range(len(final_coords)-1)) * M_TO_FT
            gap_ft = max(1.0, st.session_state.e_t_dist_val * M_TO_FT) if st.session_state.e_trigger_type == "distance" else e_speed_m * st.session_state.e_t_time_val * M_TO_FT
                
            if len(final_coords) > e_start_wp:
                dist_to_start = sum(get_haversine_dist(final_coords[i], final_coords[i+1]) for i in range(e_start_wp)) * M_TO_FT
                est_photos = int(max(0, total_dist_ft - dist_to_start) / gap_ft) + 1 if gap_ft > 0 else 0
            else:
                est_photos = 0
            
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}")
            c3.metric("Flight Speed", f"{e_speed_m * MS_TO_MPH:.1f} mph")

            if st.button("💾 Save & Update Mission"):
                with st.spinner("Calculating terrain elevations and generating KMZ..."):
                    new_cfg = {
                        "safe_takeoff_ft": e_safe, "trans_speed_mph": e_trans, "alt_ft": st.session_state.e_alt_ft,
                        "pitch": st.session_state.e_pitch, "side": e_side, "trigger_type": st.session_state.e_trigger_type,
                        "interval_ft": st.session_state.e_t_dist_val if st.session_state.e_trigger_type == "distance" else 0, 
                        "interval_sec": st.session_state.e_t_time_val if st.session_state.e_trigger_type == "time" else 0, 
                        "speed_m": e_speed_m, "photo_start_wp": int(e_start_wp), "camera_type": e_camera_type,
                        "drone_sub": e_drone_sub_enum, "payload_sub": e_payload_sub_enum
                    }
                    
                    suffix = f"_H{int(st.session_state.e_alt_ft)}A{int(abs(st.session_state.e_pitch))}OL{int(st.session_state.e_overlap_pct)}"
                    final_filename = f"{edit_name}{suffix}"

                    # --- UPDATED TO USE SELECTED SOURCE ---
                    template_kml, waylines_wpml = generate_native_kmz_contents(final_coords, new_cfg, e_elev_source, e_tif_path)
                    
                    final_filepath = os.path.join(active_dir, f"{final_filename}.kmz")
                    with zipfile.ZipFile(final_filepath, 'w') as kmz:
                        kmz.writestr("wpmz/waylines.wpml", waylines_wpml)
                        kmz.writestr("wpmz/template.kml", template_kml)
                        
                st.success(f"Successfully updated and saved as {final_filename}.kmz in {selected_dir_name}!")
            st.divider()

# ==========================================
# VIEWER MODE
# ==========================================
elif page == 'Viewer':
    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d))]
    
    col_dir, col_file, col_multi = st.columns([2, 3, 1])
    with col_dir: selected_dir_name = st.selectbox("Select Folder", ["Root (missions/)"] + existing_dirs, key="view_dir")
        
    active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
    kmz_files = [f for f in os.listdir(active_dir) if f.endswith(".kmz")]

    if not kmz_files:
        st.warning(f"No missions found in {selected_dir_name}.")
    else:
        with col_multi:
            st.markdown("<div style='margin-top: 32px;'></div>", unsafe_allow_html=True)
            view_multiple = st.checkbox("View multiple?", value=False)
            
        with col_file:
            if view_multiple: selected_kmzs = st.multiselect("Select Missions", kmz_files, default=[kmz_files[0]] if kmz_files else [])
            else:
                sel = st.selectbox("Select Mission", kmz_files)
                selected_kmzs = [sel] if sel else []
        
        with st.sidebar:
            show_footprints = st.checkbox("Show Image Footprints", value=True)
            show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="viewer_faa_toggle")
            if show_faa_airspace:
                st.write("#### Update restrictions of map center")
                if st.button("Update Map Center", key="btn_update_viewer"):
                    st.session_state.locked_viewer_center = st.session_state.viewer_center
                    st.rerun()

        if selected_kmzs:
            m_view = None
            grand_total_dist_ft = 0.0
            grand_total_photos = 0
            colors = ["#00ffff", "#ff00ff", "#00ff00", "#ffff00", "#ff8800"]
            
            for kmz_idx, current_kmz in enumerate(selected_kmzs):
                line_color = colors[kmz_idx % len(colors)]
                full_path = os.path.join(active_dir, current_kmz)
                ns = {'kml': 'http://www.opengis.net/kml/2.2', 'wpml': 'http://www.dji.com/wpmz/1.0.6'}
                
                with zipfile.ZipFile(full_path, 'r') as kmz:
                    root = ET.fromstring(kmz.read('wpmz/waylines.wpml'))
                    root_t = ET.fromstring(kmz.read('wpmz/template.kml'))
                
                meta = {"speed": 0, "pitch": -60, "mode": "None", "t_val": 0, "alt": 50.0, "safe_alt": 0, "start_idx": 0, "camera_type": "visible"}
                
                p_node = root.find('.//wpml:waypointGimbalHeadingParam/wpml:waypointGimbalPitchAngle', ns)
                if p_node is not None: meta['pitch'] = float(p_node.text)

                wp_data = []
                for pm in root.findall('.//kml:Placemark', ns):
                    idx = int(pm.find('.//wpml:index', ns).text)
                    c_raw = pm.find('.//kml:coordinates', ns).text.strip().split(',')
                    yaw = float(pm.find('.//wpml:waypointHeadingAngle', ns).text)
                    alt = float(pm.find('.//wpml:executeHeight', ns).text) * M_TO_FT
                    
                    target_yaw = yaw
                    for action_group in pm.findall('.//wpml:actionGroup', ns):
                        t_type = action_group.find('.//wpml:actionTriggerType', ns)
                        if t_type is not None and ('multiple' in t_type.text):
                            meta['mode'] = "Distance" if "Distance" in t_type.text else "Time"
                            t_param = action_group.find('.//wpml:actionTriggerParam', ns)
                            if t_param is not None: meta['t_val'] = float(t_param.text)
                            start_idx = action_group.find('.//wpml:actionGroupStartIndex', ns)
                            if start_idx is not None: meta['start_idx'] = int(start_idx.text)
                        
                        for a in action_group.findall('.//wpml:action', ns):
                            func = a.find('.//wpml:actionActuatorFunc', ns)
                            if func is not None:
                                if func.text == 'takePhoto':
                                    params = a.find('.//wpml:actionActuatorFuncParam', ns)
                                    if params is not None:
                                        lens = params.find('.//wpml:payloadLensIndex', ns)
                                        if lens is not None: meta['camera_type'] = lens.text
                                elif func.text == 'rotateYaw':
                                    params = a.find('.//wpml:actionActuatorFuncParam', ns)
                                    if params is not None:
                                        heading = params.find('.//wpml:aircraftHeading', ns)
                                        if heading is not None: target_yaw = float(heading.text)

                    wp_data.append({'lat': float(c_raw[1]), 'lon': float(c_raw[0]), 'yaw': yaw, 'target_yaw': target_yaw, 'alt': alt, 'index': idx})

                if wp_data:
                    if m_view is None:
                        m_view = folium.Map(location=st.session_state.locked_viewer_center, zoom_start=19, tiles=None)
                        folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_view)
                        
                        if show_faa_airspace:
                            uasfm_data = fetch_uasfm_data(st.session_state.locked_viewer_center[0], st.session_state.locked_viewer_center[1])
                            if uasfm_data and uasfm_data.get("features"):
                                    folium.GeoJson(
                                        uasfm_data, name="FAA UASFM Grids",
                                        style_function=lambda x: {'fillColor': 'red' if x['properties'].get('CEILING', x['properties'].get('ceiling', -1)) == 0 else 'green', 'color': 'black', 'weight': 1, 'fillOpacity': 0.15},
                                        tooltip=folium.GeoJsonTooltip(fields=['CEILING'], aliases=['Max LAANC Altitude (ft):'])
                                    ).add_to(m_view)
                            elif uasfm_data:
                                folium.Marker(st.session_state.locked_viewer_center, icon=DivIcon(html='<div style="font-size: 10px; color: grey; width: 150px;">No FAA restrictions at this location</div>')).add_to(m_view)
                    
                    line_coords = [[w['lat'], w['lon']] for w in wp_data]
                    line = folium.PolyLine(line_coords, color=line_color, weight=5).add_to(m_view)
                    PolyLineTextPath(line, '  ►  ', repeat=True, offset=7, attributes={'fill': '#000000', 'font-weight': 'bold', 'font-size': '24', 'fill-opacity': '0.3'}).add_to(m_view)
                    
                    cum_dist = [0.0]
                    total_dist_m = 0.0
                    for i in range(len(wp_data) - 1):
                        p1 = (wp_data[i]['lat'], wp_data[i]['lon'])
                        p2 = (wp_data[i+1]['lat'], wp_data[i+1]['lon'])
                        d = get_haversine_dist(p1, p2)
                        total_dist_m += d
                        cum_dist.append(total_dist_m)
                        
                        elev_diff_ft = wp_data[i+1]['alt'] - wp_data[i]['alt']
                        mid_lat = (p1[0] + p2[0]) / 2
                        mid_lon = (p1[1] + p2[1]) / 2
                        folium.Marker(
                            location=[mid_lat, mid_lon],
                            icon=DivIcon(icon_size=(120, 40), icon_anchor=(60, 20), html=f'<div style="font-size: 12pt; color: #ffffff; text-shadow: 2px 2px 4px #000000, -1px -1px 0 #000, 1px -1px 0 #000, -1px 1px 0 #000, 1px 1px 0 #000; font-weight: bold; text-align: center; line-height: 1.2;">{d * M_TO_FT:.1f} ft<br><span style="font-size: 10pt; color: #00ffff;">Elev Dif: {elev_diff_ft:+.1f} ft</span></div>')
                        ).add_to(m_view)
                    
                    grand_total_dist_ft += (total_dist_m * M_TO_FT)
                    gap = max(1.0, float(meta['t_val'])) if meta['mode'] == "Distance" else (meta['speed'] * meta['t_val'])
                    
                    for w in wp_data:
                        i = w['index']
                        length = 0.00012
                        end_lat = w['lat'] + length * math.cos(math.radians(w['target_yaw']))
                        end_lon = w['lon'] + length * math.sin(math.radians(w['target_yaw']))
                        folium.PolyLine([[w['lat'], w['lon']], [end_lat, end_lon]], color="#ff0000", weight=4).add_to(m_view)
                        
                        folium.Marker(
                            location=[w['lat'], w['lon']],
                            tooltip=f"<b>Waypoint {i}</b><br>Lat: {w['lat']:.6f}<br>Lon: {w['lon']:.6f}<br>Alt: {w['alt']:.1f} ft",
                            icon=DivIcon(icon_size=(24,24), icon_anchor=(12,12), html=f'<div style="font-size: 11pt; color: black; background: white; border-radius: 50%; text-align: center; border: 2px solid black; font-weight: bold; width: 24px; height: 24px; line-height: 20px;">{i}</div>')
                        ).add_to(m_view)
                    
                    photo_count = 0
                    if gap > 0 and meta['start_idx'] < len(wp_data):
                        current_dist = cum_dist[meta['start_idx']]
                        while current_dist <= total_dist_m + 0.01:
                            for i in range(len(cum_dist) - 1):
                                if cum_dist[i] <= current_dist <= cum_dist[i+1] + 0.001:
                                    seg_len = cum_dist[i+1] - cum_dist[i]
                                    if seg_len > 0:
                                        frac = (current_dist - cum_dist[i]) / seg_len
                                        lat = wp_data[i]['lat'] + (wp_data[i+1]['lat'] - wp_data[i]['lat']) * frac
                                        lon = wp_data[i]['lon'] + (wp_data[i+1]['lon'] - wp_data[i]['lon']) * frac
                                    else:
                                        lat, lon = wp_data[i]['lat'], wp_data[i]['lon']
                                    
                                    if show_footprints:
                                        yaw = wp_data[i]['target_yaw']
                                        footprint = get_photo_footprint(lat, lon, meta['alt']*M_TO_FT, meta['pitch'], yaw)
                                        folium.Polygon(locations=footprint, color="darkorange", weight=1, fill=True, fill_opacity=0.15).add_to(m_view)
                                        
                                    folium.CircleMarker([lat, lon], radius=2.5, color="yellow", fill=True).add_to(m_view)
                                    photo_count += 1
                                    break
                            current_dist += gap
                    
                    grand_total_photos += photo_count

            if m_view is not None:
                st.sidebar.header("Mission Metadata")
                if len(selected_kmzs) == 1:
                    cam_type = meta.get('camera_type', 'visible')
                    cam_display = CAM_DISPLAY_MAP.get(cam_type, "RGB Only")
                    hw_key = "Unknown Configuration"
                    for k, v in HARDWARE_MAP.items():
                        if v["drone_sub"] == meta.get('drone_sub', '') and v["payload_sub"] == meta.get('payload_sub', ''):
                            hw_key = k

                    st.sidebar.write(f"**Hardware Platform:** {hw_key}")
                    st.sidebar.write(f"**Camera Sensor:** {cam_display}")
                    st.sidebar.write(f"**Gimbal Pitch:** {meta['pitch']}°")
                    st.sidebar.write(f"**Safe Takeoff:** {meta['safe_alt']*M_TO_FT:.1f} ft")
                    st.sidebar.write(f"**Waypoint Alt:** {meta['alt']*M_TO_FT:.1f} ft")
                    st.sidebar.write(f"**Trigger:** {meta['mode']} ({meta['t_val']*M_TO_FT if meta['mode']=='Distance' else meta['t_val']:.1f})")
                    st.sidebar.write(f"**Calculated Photos:** {grand_total_photos}")
                else:
                    st.sidebar.success(f"Viewing {len(selected_kmzs)} combined missions.")
                    st.sidebar.write(f"**Total Aggregated Distance:** {grand_total_dist_ft:.1f} ft")
                    st.sidebar.write(f"**Total Aggregated Photos:** {grand_total_photos}")

                hud_html = f'''
                    <div style="position: fixed; bottom: 40px; left: 40px; width: 240px; background-color: rgba(255,255,255,0.9); border:2px solid #333; z-index:9999; padding: 15px; border-radius: 8px; font-family: sans-serif;">
                        <h4 style="margin:0 0 10px 0;">Mission Stats</h4>
                        <b>Total Distance:</b> {grand_total_dist_ft:.1f} ft<br>
                        <b>Total Photos:</b> {grand_total_photos}<br>
                        <p style="font-size: 11px; margin: 10px 0 0 0;">
                            <span style="color: #00ffff;">■</span> Path 
                            <span style="color: #ff0000;">■</span> Camera Yaw <br>
                            <span style="color: #ffff00;">●</span> Photo Spot
                            <span style="color: darkorange;">■</span> Photo Footprint
                        </p>
                    </div>
                '''
                m_view.get_root().html.add_child(Element(hud_html))
                map_data_view = st_folium(m_view, width=1200, height=600, key=f"viewer_map_{show_faa_airspace}_{show_footprints}")