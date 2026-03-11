import streamlit as st
import json
import os
import math
import zipfile
import xml.etree.ElementTree as ET
from datetime import datetime
import folium
from folium.plugins import Draw
from streamlit_folium import st_folium
from branca.element import Element

# ==========================================
# CONSTANTS & SETUP
# ==========================================
FT_TO_M = 0.3048
M_TO_FT = 3.28084
MISSION_DIR = "missions"
os.makedirs(MISSION_DIR, exist_ok=True)

# Mavic 3 Hasselblad Sensor Specs
SENSOR_W = 17.3  
FOCAL_L = 12.3   
IMAGE_W = 5280   

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

# ==========================================
# DATA EXTRACTION (FOR EDITOR)
# ==========================================
def parse_kmz_for_editing(full_path):
    ns = {'kml': 'http://www.opengis.net/kml/2.2', 'wpml': 'http://www.dji.com/wpmz/1.0.6'}
    meta = {
        "safe_takeoff_ft": 60.0, "trans_speed_fps": 32.0, "speed_m": 4.11,
        "alt_ft": 50.0, "pitch": -60.0, "trigger_type": "distance", 
        "t_val": 9.0, "photo_start_wp": 0, "coords": []
    }
    with zipfile.ZipFile(full_path, 'r') as kmz:
        root = ET.fromstring(kmz.read('wpmz/waylines.wpml'))
        
        safe_node = root.find('.//wpml:takeOffSecurityHeight', ns)
        if safe_node is not None: meta['safe_takeoff_ft'] = float(safe_node.text) * M_TO_FT
        
        trans_node = root.find('.//wpml:globalTransitionalSpeed', ns)
        if trans_node is not None: meta['trans_speed_fps'] = float(trans_node.text) * M_TO_FT

        speed_node = root.find('.//wpml:autoFlightSpeed', ns)
        if speed_node is not None: meta['speed_m'] = float(speed_node.text)

        pms = root.findall('.//kml:Placemark', ns)
        for i, pm in enumerate(pms):
            c_raw = pm.find('.//kml:coordinates', ns).text.strip().split(',')
            meta['coords'].append((float(c_raw[1]), float(c_raw[0]))) # Save lat, lon
            
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
                    if start_idx is not None:
                        meta['photo_start_wp'] = int(start_idx.text)

    return meta

# ==========================================
# CORE MISSION GENERATOR
# ==========================================
def generate_native_kmz_contents(coords, cfg):
    ms_ts = int(datetime.now().timestamp() * 1000)
    
    alt_m = cfg["alt_ft"] * FT_TO_M
    safe_m = cfg["safe_takeoff_ft"] * FT_TO_M
    trans_m = cfg["trans_speed_fps"] * FT_TO_M
    
    speed_m = cfg.get("speed_m")
    if speed_m is None:
        speed_m = min(max((cfg["target_gap_ft"] * FT_TO_M) / cfg["interval_sec"], 1.0), 10.0) if (cfg["auto_calc_speed"] and cfg["trigger_type"] == "time") else (cfg["manual_fps"] * FT_TO_M)

    total_dist_m = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1))
    total_duration = total_dist_m / speed_m if speed_m > 0 else 0
    pitch_val = cfg['pitch'] if 'pitch' in cfg else cfg.get('gimbal_pitch', -60)

    # 1. BUILD TEMPLATE.KML
    template_placemarks = ""
    for i, p in enumerate(coords):
        ref_bearing = get_bearing(coords[i], coords[i+1]) if i < len(coords)-1 else get_bearing(coords[i-1], coords[i])
        yaw = (ref_bearing + 90) % 360 if cfg['side'] == "right" else (ref_bearing - 90) % 360
        if yaw > 180: yaw -= 360

        template_photo_action = ""
        start_wp = cfg.get("photo_start_wp", 0)
        if i == start_wp:
            trigger_tag = "multipleDistance" if cfg["trigger_type"] == "distance" else "multipleTiming"
            interval_val = round(cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['interval_sec']
            template_photo_action = f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>0</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{start_wp}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{len(coords)-1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>{trigger_tag}</wpml:actionTriggerType><wpml:actionTriggerParam>{interval_val:.2f}</wpml:actionTriggerParam></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
              <wpml:useGlobalPayloadLensIndex>1</wpml:useGlobalPayloadLensIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""

        template_placemarks += f"""
      <Placemark>
        <Point><coordinates>{p[1]:.8f},{p[0]:.8f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:ellipsoidHeight>{alt_m:.1f}</wpml:ellipsoidHeight>
        <wpml:height>{alt_m:.1f}</wpml:height>
        <wpml:useGlobalHeight>1</wpml:useGlobalHeight>
        <wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>
        <wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>smoothTransition</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{int(yaw)}</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
        </wpml:waypointHeadingParam>
        <wpml:gimbalPitchAngle>{pitch_val}</wpml:gimbalPitchAngle>
        <wpml:isRisky>0</wpml:isRisky>
        {template_photo_action}
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
      <wpml:droneInfo><wpml:droneEnumValue>77</wpml:droneEnumValue><wpml:droneSubEnumValue>2</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:payloadInfo><wpml:payloadEnumValue>68</wpml:payloadEnumValue><wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
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
      <wpml:globalHeight>{alt_m:.1f}</wpml:globalHeight>
      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>
      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>
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

    # 2. BUILD WAYLINES.WPML
    waylines_placemarks = ""
    global_action_id = 0  

    for i, p in enumerate(coords):
        ref_bearing = get_bearing(coords[i], coords[i+1]) if i < len(coords)-1 else get_bearing(coords[i-1], coords[i])
        yaw = (ref_bearing + 90) % 360 if cfg['side'] == "right" else (ref_bearing - 90) % 360
        if yaw > 180: yaw -= 360

        action_group = ""
        
        if i == 0:
            action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{global_action_id}</wpml:actionGroupId>
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
            global_action_id += 1

        start_wp = cfg.get("photo_start_wp", 0)
        if i == start_wp:
            trigger_tag = "multipleDistance" if cfg["trigger_type"] == "distance" else "multipleTiming"
            interval_val = round(cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['interval_sec']
            action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{global_action_id}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{start_wp}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{len(coords)-1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>{trigger_tag}</wpml:actionTriggerType><wpml:actionTriggerParam>{interval_val:.2f}</wpml:actionTriggerParam></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
              <wpml:useGlobalPayloadLensIndex>1</wpml:useGlobalPayloadLensIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            global_action_id += 1

        if i < len(coords) - 1:
            action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{global_action_id}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i+1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>betweenAdjacentPoints</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalEvenlyRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalPitchRotateAngle>{pitch_val}</wpml:gimbalPitchAngle>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            global_action_id += 1

        waylines_placemarks += f"""
      <Placemark>
        <Point><coordinates>{p[1]:.8f},{p[0]:.8f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{alt_m:.1f}</wpml:executeHeight>
        <wpml:waypointSpeed>{speed_m:.2f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>smoothTransition</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{int(yaw)}</wpml:waypointHeadingAngle>
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
        {action_group}
        <wpml:waypointGimbalHeadingParam>
          <wpml:waypointGimbalPitchAngle>{pitch_val}</wpml:waypointGimbalPitchAngle>
          <wpml:waypointGimbalYawAngle>0</wpml:waypointGimbalYawAngle>
        </wpml:waypointGimbalHeadingParam>
        <wpml:isRisky>0</wpml:isRisky>
        <wpml:waypointWorkType>0</wpml:waypointWorkType>
      </Placemark>"""

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
      <wpml:droneInfo><wpml:droneEnumValue>77</wpml:droneEnumValue><wpml:droneSubEnumValue>2</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:waylineAvoidLimitAreaMode>0</wpml:waylineAvoidLimitAreaMode>
      <wpml:payloadInfo><wpml:payloadEnumValue>68</wpml:payloadEnumValue><wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>{total_dist_m:.2f}</wpml:distance>
      <wpml:duration>{total_duration:.2f}</wpml:duration>
      <wpml:autoFlightSpeed>{speed_m:.1f}</wpml:autoFlightSpeed>
      {waylines_placemarks}
    </Folder>
  </Document>
</kml>"""

    return template_kml, waylines_wpml

# ==========================================
# UI NAVIGATION & LOGIC
# ==========================================
st.set_page_config(layout="wide", page_title="ISLERS Control")

st.title("🛰️ ISLERS Flight Planner")
page = st.radio("Navigation", ["Creator", "Editor", "Viewer"], horizontal=True, label_visibility="collapsed")

# --- CREATOR MODE ---
if page == 'Creator':
    with st.sidebar:
        st.header("1. Global Config")
        mission_name = st.text_input("Filename", "New_flight")
        safe_takeoff_ft = st.number_input("Safe Takeoff Alt (ft)", value=60.0)
        trans_speed_fps = st.number_input("Takeoff Speed (fps)", value=32.0)
        
        st.header("2. Waypoint Settings")
        alt_ft = st.number_input("Relative Altitude (ft)", value=50.0)
        gsd_cm = (alt_ft * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W)
        st.info(f"📐 Est. GSD: {gsd_cm:.2f} cm/px")
        
        gimbal_pitch = st.slider("Gimbal Pitch (°)", -90, 0, -60)
        side = st.selectbox("Side", ["right", "left"])
        
        st.header("3. Trigger & Speed")
        photo_start_wp = st.number_input("Start Photos at Waypoint Index", min_value=0, value=0, step=1)
        trigger = st.radio("Type", ["distance", "time"])
        t_val = st.number_input("Interval (ft or sec)", 1.0)
        auto_speed = st.checkbox("Auto-Calc Speed", True)
        target_gap_ft = st.number_input("Target Gap (ft)", 5)
        manual_fps = st.number_input("Manual Speed (fps)", 5)

    speed_m = min(max((target_gap_ft * FT_TO_M) / t_val, 1.0), 10.0) if (auto_speed and trigger == "time") else (manual_fps * FT_TO_M)

    # 1. Create an empty container for the HUD to render ABOVE the map
    top_hud = st.container()

    # 2. Render Map
    m = folium.Map(location=[40.253, -111.640], zoom_start=17, tiles=None)
    folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m)
    Draw(export=False, draw_options={'polyline':{'shapeOptions':{'color':'#00ffff','weight':5}}}).add_to(m)
    map_data = st_folium(m, width=1200, height=600)

    # 3. Inject data and buttons back into the top container
    if map_data.get("all_drawings"):
        coords = [(c[1], c[0]) for c in map_data["all_drawings"][-1]['geometry']['coordinates']]
        total_dist_ft = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1)) * M_TO_FT
        
        gap_ft = round(t_val * FT_TO_M) * M_TO_FT if trigger == "distance" else (speed_m * t_val * M_TO_FT)
        if len(coords) > photo_start_wp:
            dist_to_start = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(photo_start_wp)) * M_TO_FT
            active_path_ft = max(0, total_dist_ft - dist_to_start)
            est_photos = int(active_path_ft / gap_ft) + 1 if gap_ft > 0 else 0
        else:
            est_photos = 0

        with top_hud:
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}")
            c3.metric("Flight Speed", f"{speed_m * M_TO_FT:.1f} fps")

            if st.button("🚀 Save & Generate KMZ"):
                cfg = {
                    "safe_takeoff_ft": safe_takeoff_ft, "trans_speed_fps": trans_speed_fps, 
                    "alt_ft": alt_ft, "pitch": gimbal_pitch, "side": side, 
                    "trigger_type": trigger, "interval_ft": t_val, "interval_sec": t_val, 
                    "speed_m": speed_m, "photo_start_wp": int(photo_start_wp)
                }
                template_kml, waylines_wpml = generate_native_kmz_contents(coords, cfg)
                with zipfile.ZipFile(f"missions/{mission_name}.kmz", 'w') as kmz:
                    kmz.writestr("wpmz/waylines.wpml", waylines_wpml)
                    kmz.writestr("wpmz/template.kml", template_kml)
                st.success(f"Saved {mission_name}.kmz to missions/")
            st.divider()

# --- EDITOR MODE ---
elif page == 'Editor':
    kmz_files = [f for f in os.listdir(MISSION_DIR) if f.endswith(".kmz")]
    if not kmz_files:
        st.warning("No missions found in missions/ directory.")
    else:
        st.info("💡 Editor modifies flight parameters for an existing route. To draw a new route, use Creator.")
        
        col1, col2 = st.columns([1, 2])
        with col1:
            selected_kmz = st.selectbox("Select Mission to Edit", kmz_files)
        
        full_path = os.path.join(MISSION_DIR, selected_kmz)
        meta = parse_kmz_for_editing(full_path)
        
        with st.sidebar:
            st.header("Modify Parameters")
            edit_name = st.text_input("Save As", value=selected_kmz.replace('.kmz', '_edited'))
            
            e_safe = st.number_input("Safe Takeoff Alt (ft)", value=meta['safe_takeoff_ft'])
            e_trans = st.number_input("Takeoff Speed (fps)", value=meta['trans_speed_fps'])
            e_alt = st.number_input("Relative Altitude (ft)", value=meta['alt_ft'])
            e_pitch = st.slider("Gimbal Pitch (°)", -90, 0, int(meta['pitch']))
            e_side = st.selectbox("Yaw Side", ["right", "left"])
            
            st.header("Trigger Settings")
            e_start_wp = st.number_input("Start Photos at WP", min_value=0, value=meta['photo_start_wp'], step=1)
            t_idx = 0 if meta['trigger_type'] == 'distance' else 1
            e_trigger = st.radio("Type", ["distance", "time"], index=t_idx)
            e_tval = st.number_input("Interval (ft or sec)", value=meta['t_val'])
            e_speed = st.number_input("Flight Speed (fps)", value=meta['speed_m'] * M_TO_FT)

        # Place Update Button ABOVE Map
        if st.button("💾 Save & Update Mission"):
            new_cfg = {
                "safe_takeoff_ft": e_safe, "trans_speed_fps": e_trans, "alt_ft": e_alt,
                "pitch": e_pitch, "side": e_side, "trigger_type": e_trigger,
                "interval_ft": e_tval, "interval_sec": e_tval, 
                "speed_m": e_speed * FT_TO_M, "photo_start_wp": int(e_start_wp)
            }
            template_kml, waylines_wpml = generate_native_kmz_contents(meta['coords'], new_cfg)
            with zipfile.ZipFile(f"missions/{edit_name}.kmz", 'w') as kmz:
                kmz.writestr("wpmz/waylines.wpml", waylines_wpml)
                kmz.writestr("wpmz/template.kml", template_kml)
            st.success(f"Successfully updated and saved as {edit_name}.kmz!")
        
        st.divider()

        if meta['coords']:
            m_edit = folium.Map(location=[meta['coords'][0][0], meta['coords'][0][1]], zoom_start=18, tiles=None)
            folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_edit)
            folium.PolyLine(meta['coords'], color="#00ffff", weight=5).add_to(m_edit)
            for i, c in enumerate(meta['coords']):
                folium.CircleMarker(c, radius=4, color="red", popup=f"WP {i}").add_to(m_edit)
            st_folium(m_edit, width=1200, height=600)

# --- VIEWER MODE ---
elif page == 'Viewer':
    kmz_files = [f for f in os.listdir(MISSION_DIR) if f.endswith(".kmz")]
    if not kmz_files:
        st.warning("No missions found in missions/ directory.")
    else:
        selected_kmz = st.selectbox("Select Mission to Inspect", kmz_files)
        full_path = os.path.join(MISSION_DIR, selected_kmz)
        ns = {'kml': 'http://www.opengis.net/kml/2.2', 'wpml': 'http://www.dji.com/wpmz/1.0.6'}
        
        with zipfile.ZipFile(full_path, 'r') as kmz:
            root = ET.fromstring(kmz.read('wpmz/waylines.wpml'))
        
        meta = {"speed": 0, "pitch": 0, "mode": "None", "t_val": 0, "alt": 0, "safe_alt": 0, "trans_speed": 0, "start_idx": 0}
        
        safe_node = root.find('.//wpml:takeOffSecurityHeight', ns)
        if safe_node is not None: meta['safe_alt'] = float(safe_node.text)
        
        speed_node = root.find('.//wpml:autoFlightSpeed', ns)
        if speed_node is not None: meta['speed'] = float(speed_node.text)

        wp0 = root.find('.//kml:Placemark', ns)
        if wp0 is not None:
            pitch_node = wp0.find('.//wpml:waypointGimbalHeadingParam/wpml:waypointGimbalPitchAngle', ns)
            if pitch_node is not None: meta['pitch'] = float(pitch_node.text)

        wp_data = []
        for pm in root.findall('.//kml:Placemark', ns):
            idx = int(pm.find('.//wpml:index', ns).text)
            c_raw = pm.find('.//kml:coordinates', ns).text.strip().split(',')
            yaw = float(pm.find('.//wpml:waypointHeadingAngle', ns).text)
            alt = float(pm.find('.//wpml:executeHeight', ns).text)
            wp_data.append({'lat': float(c_raw[1]), 'lon': float(c_raw[0]), 'yaw': yaw, 'alt': alt, 'index': idx})
            meta['alt'] = alt
            
            for action_group in pm.findall('.//wpml:actionGroup', ns):
                t_type = action_group.find('.//wpml:actionTriggerType', ns)
                if t_type is not None and ('multiple' in t_type.text):
                    meta['mode'] = "Distance" if "Distance" in t_type.text else "Time"
                    t_param = action_group.find('.//wpml:actionTriggerParam', ns)
                    if t_param is not None: meta['t_val'] = float(t_param.text)
                    start_idx = action_group.find('.//wpml:actionGroupStartIndex', ns)
                    if start_idx is not None: meta['start_idx'] = int(start_idx.text)

        if wp_data:
            m_view = folium.Map(location=[wp_data[0]['lat'], wp_data[0]['lon']], zoom_start=19, tiles=None)
            folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_view)
            folium.PolyLine([[w['lat'], w['lon']] for w in wp_data], color="#00ffff", weight=5).add_to(m_view)
            
            cum_dist = [0.0]
            total_dist_m = 0.0
            for i in range(len(wp_data) - 1):
                p1 = (wp_data[i]['lat'], wp_data[i]['lon'])
                p2 = (wp_data[i+1]['lat'], wp_data[i+1]['lon'])
                d = get_haversine_dist(p1, p2)
                total_dist_m += d
                cum_dist.append(total_dist_m)
            
            gap = int(meta['t_val']) if meta['mode'] == "Distance" else (meta['speed'] * meta['t_val'])
            
            for i in range(len(wp_data)):
                wp = wp_data[i]
                length = 0.00012
                end_lat = wp['lat'] + length * math.cos(math.radians(wp['yaw']))
                end_lon = wp['lon'] + length * math.sin(math.radians(wp['yaw']))
                folium.PolyLine([[wp['lat'], wp['lon']], [end_lat, end_lon]], color="#ff0000", weight=4).add_to(m_view)
                folium.CircleMarker([wp['lat'], wp['lon']], radius=3, color="white", fill=True).add_to(m_view)
            
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
                            folium.CircleMarker([lat, lon], radius=2.5, color="yellow", fill=True).add_to(m_view)
                            photo_count += 1
                            break
                    current_dist += gap

            st.sidebar.header("Mission Metadata")
            st.sidebar.write(f"**Gimbal Pitch:** {meta['pitch']}°")
            st.sidebar.write(f"**Safe Takeoff:** {meta['safe_alt']*M_TO_FT:.1f} ft")
            st.sidebar.write(f"**Waypoint Alt:** {meta['alt']*M_TO_FT:.1f} ft")
            st.sidebar.write(f"**Trigger:** {meta['mode']} ({meta['t_val']*M_TO_FT if meta['mode']=='Distance' else meta['t_val']:.1f})")
            st.sidebar.write(f"**Calculated Photos:** {photo_count}")

            hud_html = f'''
                <div style="position: fixed; bottom: 40px; left: 40px; width: 240px; background-color: rgba(255,255,255,0.9); 
                            border:2px solid #333; z-index:9999; padding: 15px; border-radius: 8px; font-family: sans-serif;">
                    <h4 style="margin:0 0 10px 0;">Mission Stats</h4>
                    <b>Distance:</b> {total_dist_m*M_TO_FT:.1f} ft<br>
                    <p style="font-size: 11px; margin: 10px 0 0 0;">
                        <span style="color: #00ffff;">■</span> Path 
                        <span style="color: #ff0000;">■</span> Camera Yaw 
                        <span style="color: #ffff00;">●</span> Photo Spot
                    </p>
                </div>
            '''
            m_view.get_root().html.add_child(Element(hud_html))
            st_folium(m_view, width=1200, height=600)