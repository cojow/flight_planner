import streamlit as st
import pandas as pd
import json
import os
import math
import zipfile
import xml.etree.ElementTree as ET
from datetime import datetime
import folium
from folium.plugins import Draw, PolyLineTextPath
from folium.features import DivIcon
from streamlit_folium import st_folium
from branca.element import Element

# ==========================================
# CONSTANTS & SETUP
# ==========================================
FT_TO_M = 0.3048
M_TO_FT = 3.28084
MPH_TO_MS = 0.44704
MS_TO_MPH = 2.23694

MISSION_DIR = "missions"
os.makedirs(MISSION_DIR, exist_ok=True)

# Mavic 3 Hasselblad Sensor Specs
SENSOR_W = 17.3  
FOCAL_L = 12.3   
IMAGE_W = 5280   

CAM_DISPLAY_MAP = {
    "visable": "RGB Only",
    "narrow_band": "Multispectral Only",
    "visable,narrow_band": "RGB + Multispectral"
}
CAM_VAL_MAP = {v: k for k, v in CAM_DISPLAY_MAP.items()}

# Scalable Hardware Dictionary
HARDWARE_MAP = {
    "Mavic 3 Multispectral (Drone: 0, Payload: 3)": {"drone_sub": "0", "payload_sub": "3"}
}

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

def get_footprint(pitch, alt):
    if pitch == 0: return 999999.0  
    slant_dist = alt / math.sin(math.radians(abs(pitch)))
    return slant_dist * (SENSOR_W / FOCAL_L)

def sync_dist_to_overlap():
    fw = get_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    if fw > 0: 
        val = st.session_state.get('t_dist_val', 9.0)
        st.session_state.overlap_pct = max(0.0, min(((fw - val) / fw) * 100, 99.9))

def sync_overlap_to_dist():
    fw = get_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    st.session_state.t_dist_val = fw * (1 - (st.session_state.get('overlap_pct', 70.0) / 100))

def sync_gap_to_overlap():
    fw = get_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    if fw > 0: 
        val = st.session_state.get('target_gap_ft', 26.2)
        st.session_state.overlap_pct = max(0.0, min(((fw - val) / fw) * 100, 99.9))

def sync_overlap_to_gap():
    fw = get_footprint(st.session_state.get('pitch', -60.0), st.session_state.get('alt_ft', 50.0))
    st.session_state.target_gap_ft = fw * (1 - (st.session_state.get('overlap_pct', 70.0) / 100))

def sync_geometry():
    if st.session_state.get('trigger_type', 'distance') == 'distance': sync_dist_to_overlap()
    else: sync_gap_to_overlap()

def e_sync_dist_to_overlap():
    fw = get_footprint(st.session_state.get('e_pitch', -60.0), st.session_state.get('e_alt_ft', 50.0))
    if fw > 0: 
        val = st.session_state.get('e_t_dist_val', 9.0)
        st.session_state.e_overlap_pct = max(0.0, min(((fw - val) / fw) * 100, 99.9))

def e_sync_overlap_to_dist():
    fw = get_footprint(st.session_state.get('e_pitch', -60.0), st.session_state.get('e_alt_ft', 50.0))
    st.session_state.e_t_dist_val = fw * (1 - (st.session_state.get('e_overlap_pct', 70.0) / 100))

def e_sync_gap_to_overlap():
    fw = get_footprint(st.session_state.get('e_pitch', -60.0), st.session_state.get('e_alt_ft', 50.0))
    if fw > 0: 
        val = st.session_state.get('e_target_gap_ft', 26.2)
        st.session_state.e_overlap_pct = max(0.0, min(((fw - val) / fw) * 100, 99.9))

def e_sync_overlap_to_gap():
    fw = get_footprint(st.session_state.get('e_pitch', -60.0), st.session_state.get('e_alt_ft', 50.0))
    st.session_state.e_target_gap_ft = fw * (1 - (st.session_state.get('e_overlap_pct', 70.0) / 100))

def e_sync_geometry():
    if st.session_state.get('e_trigger_type', 'distance') == 'distance': e_sync_dist_to_overlap()
    else: e_sync_gap_to_overlap()

# ==========================================
# DATA EXTRACTION (FOR EDITOR)
# ==========================================
def parse_kmz_for_editing(full_path):
    ns = {'kml': 'http://www.opengis.net/kml/2.2', 'wpml': 'http://www.dji.com/wpmz/1.0.6'}
    meta = {
        "safe_takeoff_ft": 60.0, "trans_speed_mph": 22.0, "speed_m": 4.11, "speed_mph": 1.0,
        "alt_ft": 50.0, "pitch": -60.0, "trigger_type": "distance", 
        "t_val": 9.0, "photo_start_wp": 0, "coords": [], "camera_type": "visable",
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
                    if start_idx is not None:
                        meta['photo_start_wp'] = int(start_idx.text)
                
                for a in ag.findall('.//wpml:action', ns):
                    func = a.find('.//wpml:actionActuatorFunc', ns)
                    if func is not None and func.text == 'takePhoto':
                        params = a.find('.//wpml:actionActuatorFuncParam', ns)
                        if params is not None:
                            lens = params.find('.//wpml:payloadLensIndex', ns)
                            if lens is not None:
                                meta['camera_type'] = lens.text
    return meta

# ==========================================
# CORE MISSION GENERATOR
# ==========================================
def generate_native_kmz_contents(coords, cfg):
    ms_ts = int(datetime.now().timestamp() * 1000)
    alt_m = cfg["alt_ft"] * FT_TO_M
    safe_m = cfg["safe_takeoff_ft"] * FT_TO_M
    trans_m = cfg["trans_speed_mph"] * MPH_TO_MS
    speed_m = cfg["speed_m"]
    lens_str = cfg.get("camera_type", "visable")
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
        <wpml:useGlobalHeight>1</wpml:useGlobalHeight>
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
      <wpml:globalHeight>{alt_m:.1f}</wpml:globalHeight>
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
st.set_page_config(layout="wide", page_title="ISLERS Control")

st.title("🛰️ Flight Planner")
page = st.radio("Navigation", ["Creator", "Editor", "Viewer"], horizontal=True, label_visibility="collapsed")

# --- CREATOR MODE ---
if page == 'Creator':
    with st.sidebar:
        st.header("1. Global Config")
        mission_name = st.text_input("Filename", "ISLERS_Flight")
        trans_speed_mph = st.number_input("Takeoff Speed (mph)", value=22.0)
        safe_takeoff_ft = st.number_input("Safe Takeoff Alt (ft)", value=60.0)
        
        st.header("2. Waypoint Settings")
        st.number_input("Relative Altitude (ft)", key="alt_ft", on_change=sync_geometry)
        st.slider("Gimbal Pitch (°)", -90, 0, key="pitch", on_change=sync_geometry)
        
        pitch_rad = math.radians(abs(st.session_state.pitch))
        D_ft_c = st.session_state.alt_ft / math.sin(pitch_rad) if pitch_rad > 0 else float('inf')
        gsd_cm = (D_ft_c * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_c != float('inf') else 0
        st.info(f"📐 Est. Ground GSD: {gsd_cm:.2f} cm/px")
        
        side = st.selectbox("Side", ["right", "left"])
        
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
                st.error(f"⚠️ Speed Too High! At {speed_m * MS_TO_MPH:.1f} mph, the camera must fire every {gap_m / speed_m:.2f}s. Your sensor requires {min_photo_interval_sec}s. The drone will drop photos. Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
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
                fw = get_footprint(st.session_state.pitch, st.session_state.alt_ft)
                current_overlap = ((fw - current_gap) / fw) * 100 if fw > 0 else 0
                st.info(f"📊 Current Overlap: {max(0, min(current_overlap, 99.9)):.1f}%")

    top_hud = st.container()

    m = folium.Map(location=[40.253, -111.640], zoom_start=17, tiles=None)
    folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m)
    Draw(export=False, draw_options={'polyline':{'shapeOptions':{'color':'#00ffff','weight':5}}}).add_to(m)
    map_data = st_folium(m, width=1200, height=600)

    if map_data.get("all_drawings"):
        coords = [(c[1], c[0]) for c in map_data["all_drawings"][-1]['geometry']['coordinates']]
        total_dist_ft = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1)) * M_TO_FT
        
        if st.session_state.trigger_type == "distance":
            gap_m = max(1.0, st.session_state.t_dist_val * FT_TO_M)
            gap_ft = gap_m * M_TO_FT
        else:
            gap_ft = speed_m * t_val_sec * M_TO_FT
            
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
            c3.metric("Flight Speed", f"{speed_m * MS_TO_MPH:.1f} mph")

            if st.button("🚀 Save & Generate KMZ"):
                cfg = {
                    "safe_takeoff_ft": safe_takeoff_ft, "trans_speed_mph": trans_speed_mph, 
                    "alt_ft": st.session_state.alt_ft, "pitch": st.session_state.pitch, "side": side, 
                    "trigger_type": st.session_state.trigger_type, 
                    "interval_ft": st.session_state.t_dist_val if st.session_state.trigger_type == "distance" else 0, 
                    "interval_sec": t_val_sec if st.session_state.trigger_type == "time" else 0, 
                    "speed_m": speed_m, "photo_start_wp": int(photo_start_wp),
                    "camera_type": camera_type,
                    "drone_sub": drone_sub_enum,
                    "payload_sub": payload_sub_enum
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
        col1, col2 = st.columns([3, 1])
        with col1:
            selected_kmz = st.selectbox("Select Mission to Edit", kmz_files)
        with col2:
            st.markdown("<div style='margin-top: 32px;'></div>", unsafe_allow_html=True)
            make_new_file = st.checkbox("Make new file?", value=False)
            
        base_name = selected_kmz.replace('.kmz', '')
        edit_name = f"{base_name}-edited" if make_new_file else base_name
        
        full_path = os.path.join(MISSION_DIR, selected_kmz)
        
        if 'editor_kmz' not in st.session_state or st.session_state.editor_kmz != selected_kmz:
            st.session_state.editor_kmz = selected_kmz
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
            st.slider("Gimbal Pitch (°)", -90, 0, key="e_pitch", on_change=e_sync_geometry)
            
            pitch_rad_e = math.radians(abs(st.session_state.e_pitch))
            D_ft_e = st.session_state.e_alt_ft / math.sin(pitch_rad_e) if pitch_rad_e > 0 else float('inf')
            e_gsd_cm = (D_ft_e * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_e != float('inf') else 0
            st.info(f"📐 Est. Ground GSD: {e_gsd_cm:.2f} cm/px")
            
            e_side = st.selectbox("Yaw Side", ["right", "left"])
            
            st.header("Hardware & Payload")
            e_hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()), index=list(HARDWARE_MAP.keys()).index(meta.get('hardware_key', "Mavic 3 Multispectral (Drone: 0, Payload: 3)")))
            e_drone_sub_enum = HARDWARE_MAP[e_hw_choice]["drone_sub"]
            e_payload_sub_enum = HARDWARE_MAP[e_hw_choice]["payload_sub"]
            
            current_cam_display = CAM_DISPLAY_MAP.get(meta.get('camera_type', 'visable'), "RGB Only")
            e_cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"], index=["RGB Only", "Multispectral Only", "RGB + Multispectral"].index(current_cam_display))
            e_camera_type = CAM_VAL_MAP[e_cam_choice]
            
            min_photo_interval_sec = 2.0 if "narrow_band" in e_camera_type else 0.7

            st.header("Trigger Settings")
            e_start_wp = st.number_input("Start Photos at WP", min_value=0, value=meta['photo_start_wp'], step=1)
            t_idx = 0 if meta['trigger_type'] == 'distance' else 1
            e_trigger = st.radio("Type", ["distance", "time"], key="e_trigger_type", on_change=e_sync_geometry)
            
            safe_e_speed = max(2.3, float(meta.get('speed_mph', 6.0)))
            
            if st.session_state.e_trigger_type == "distance":
                st.number_input("Interval (ft)", key="e_t_dist_val", min_value=1.0, on_change=e_sync_dist_to_overlap)
                st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=e_sync_overlap_to_dist)
                e_manual_mph = st.number_input("Flight Speed (mph)", min_value=2.3, value=safe_e_speed)
                e_speed_m = e_manual_mph * MPH_TO_MS
                
                gap_m = max(1.0, st.session_state.e_t_dist_val * FT_TO_M)
                max_speed_m = gap_m / min_photo_interval_sec
                if e_speed_m > max_speed_m:
                    st.error(f"⚠️ Speed Too High! At {e_speed_m * MS_TO_MPH:.1f} mph, the camera must fire every {gap_m / e_speed_m:.2f}s. Your sensor requires {min_photo_interval_sec}s. The drone will drop photos. Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
            else:
                if 'e_t_time_val' not in st.session_state: st.session_state.e_t_time_val = meta['t_val'] if meta['trigger_type'] == 'time' else max(2.0, min_photo_interval_sec)
                e_tval_sec = st.number_input("Interval (sec)", key="e_t_time_val", min_value=min_photo_interval_sec)
                e_auto_speed = st.checkbox("Auto-Calc Speed", True)
                if e_auto_speed:
                    st.number_input("Target Gap (ft)", key="e_target_gap_ft", min_value=1.0, on_change=e_sync_gap_to_overlap)
                    st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=e_sync_overlap_to_gap)
                    e_speed_m = min(max((st.session_state.e_target_gap_ft * FT_TO_M) / e_tval_sec, 1.0), 10.0)
                    st.info(f"🤖 Auto-Calculated Speed: {e_speed_m * MS_TO_MPH:.1f} mph")
                else:
                    e_manual_mph = st.number_input("Manual Speed (mph)", min_value=2.3, value=safe_e_speed)
                    e_speed_m = e_manual_mph * MPH_TO_MS
                    current_gap = e_speed_m * M_TO_FT * e_tval_sec
                    fw = get_footprint(st.session_state.e_pitch, st.session_state.e_alt_ft)
                    current_overlap = ((fw - current_gap) / fw) * 100 if fw > 0 else 0
                    st.info(f"📊 Current Overlap: {max(0, min(current_overlap, 99.9)):.1f}%")

        top_hud = st.container()
        st.write("### 📍 Fine-Tune Flight Path Coordinates")
        
        df = pd.DataFrame(meta['coords'], columns=['Latitude', 'Longitude'])
        edited_df = st.data_editor(df, num_rows="dynamic", key=st.session_state.editor_key, use_container_width=True)
        current_coords = [(row['Latitude'], row['Longitude']) for _, row in edited_df.iterrows()]

        m_edit = folium.Map(location=[current_coords[0][0], current_coords[0][1]], zoom_start=18, tiles=None)
        folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_edit)
        Draw(export=False, draw_options={'polyline':{'shapeOptions':{'color':'#00ffff','weight':5}}}).add_to(m_edit)
        
        line = folium.PolyLine(current_coords, color="#00ffff", weight=5).add_to(m_edit)
        PolyLineTextPath(line, '  ►  ', repeat=True, offset=7, attributes={'fill': '#000000', 'font-weight': 'bold', 'font-size': '24', 'fill-opacity': '0.3'}).add_to(m_edit)
        
        for i in range(len(current_coords) - 1):
            p1 = current_coords[i]
            p2 = current_coords[i+1]
            dist = get_haversine_dist(p1, p2) * M_TO_FT
            mid_lat = (p1[0] + p2[0]) / 2
            mid_lon = (p1[1] + p2[1]) / 2
            folium.Marker(
                location=[mid_lat, mid_lon],
                icon=DivIcon(
                    icon_size=(100, 20), icon_anchor=(50, 10),
                    html=f'<div style="font-size: 12pt; color: #ffffff; text-shadow: 2px 2px 4px #000000, -1px -1px 0 #000, 1px -1px 0 #000, -1px 1px 0 #000, 1px 1px 0 #000; font-weight: bold; text-align: center;">{dist:.1f} ft</div>'
                )
            ).add_to(m_edit)

        for i, c in enumerate(current_coords):
            folium.Marker(
                location=c,
                icon=DivIcon(
                    icon_size=(24,24), icon_anchor=(12,12),
                    html=f'<div style="font-size: 11pt; color: black; background: white; border-radius: 50%; text-align: center; border: 2px solid black; font-weight: bold; width: 24px; height: 24px; line-height: 20px;">{i}</div>'
                )
            ).add_to(m_edit)
            
        map_data_edit = st_folium(m_edit, width=1200, height=600)

        if map_data_edit.get("all_drawings") and len(map_data_edit["all_drawings"]) > 0:
            final_coords = [(c[1], c[0]) for c in map_data_edit["all_drawings"][-1]['geometry']['coordinates']]
            st.info("✏️ Using newly drawn line from the map.")
        else:
            final_coords = current_coords

        with top_hud:
            total_dist_ft = sum(get_haversine_dist(final_coords[i], final_coords[i+1]) for i in range(len(final_coords)-1)) * M_TO_FT
            
            if st.session_state.e_trigger_type == "distance":
                gap_m = max(1.0, st.session_state.e_t_dist_val * FT_TO_M)
                gap_ft = gap_m * M_TO_FT
            else:
                gap_ft = e_speed_m * st.session_state.e_t_time_val * M_TO_FT
                
            if len(final_coords) > e_start_wp:
                dist_to_start = sum(get_haversine_dist(final_coords[i], final_coords[i+1]) for i in range(e_start_wp)) * M_TO_FT
                active_path_ft = max(0, total_dist_ft - dist_to_start)
                est_photos = int(active_path_ft / gap_ft) + 1 if gap_ft > 0 else 0
            else:
                est_photos = 0
            
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}")
            c3.metric("Flight Speed", f"{e_speed_m * MS_TO_MPH:.1f} mph")

            if st.button("💾 Save & Update Mission"):
                new_cfg = {
                    "safe_takeoff_ft": e_safe, "trans_speed_mph": e_trans, "alt_ft": st.session_state.e_alt_ft,
                    "pitch": st.session_state.e_pitch, "side": e_side, "trigger_type": st.session_state.e_trigger_type,
                    "interval_ft": st.session_state.e_t_dist_val if st.session_state.e_trigger_type == "distance" else 0, 
                    "interval_sec": st.session_state.e_t_time_val if st.session_state.e_trigger_type == "time" else 0, 
                    "speed_m": e_speed_m, "photo_start_wp": int(e_start_wp),
                    "camera_type": e_camera_type,
                    "drone_sub": e_drone_sub_enum,
                    "payload_sub": e_payload_sub_enum
                }
                template_kml, waylines_wpml = generate_native_kmz_contents(final_coords, new_cfg)
                with zipfile.ZipFile(f"missions/{edit_name}.kmz", 'w') as kmz:
                    kmz.writestr("wpmz/waylines.wpml", waylines_wpml)
                    kmz.writestr("wpmz/template.kml", template_kml)
                st.success(f"Successfully updated and saved as {edit_name}.kmz!")
            st.divider()

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
            root_t = ET.fromstring(kmz.read('wpmz/template.kml'))
        
        meta = {"speed": 0, "pitch": 0, "mode": "None", "t_val": 0, "alt": 0, "safe_alt": 0, "trans_speed": 0, "start_idx": 0, "camera_type": "visable"}
        
        safe_node = root.find('.//wpml:takeOffSecurityHeight', ns)
        if safe_node is not None: meta['safe_alt'] = float(safe_node.text)
        
        speed_node = root.find('.//wpml:autoFlightSpeed', ns)
        if speed_node is not None: meta['speed'] = float(speed_node.text)
        
        # Read from template to ensure accuracy
        drone_info = root_t.find('.//wpml:droneInfo', ns)
        if drone_info is not None:
            d_sub = drone_info.find('.//wpml:droneSubEnumValue', ns)
            if d_sub is not None: meta['drone_sub'] = d_sub.text
            
        payload_info = root_t.find('.//wpml:payloadInfo', ns)
        if payload_info is not None:
            p_sub = payload_info.find('.//wpml:payloadSubEnumValue', ns)
            if p_sub is not None: meta['payload_sub'] = p_sub.text

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
                
                for a in action_group.findall('.//wpml:action', ns):
                    func = a.find('.//wpml:actionActuatorFunc', ns)
                    if func is not None and func.text == 'takePhoto':
                        params = a.find('.//wpml:actionActuatorFuncParam', ns)
                        if params is not None:
                            lens = params.find('.//wpml:payloadLensIndex', ns)
                            if lens is not None:
                                meta['camera_type'] = lens.text

        if wp_data:
            m_view = folium.Map(location=[wp_data[0]['lat'], wp_data[0]['lon']], zoom_start=19, tiles=None)
            folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_view)
            
            line_coords = [[w['lat'], w['lon']] for w in wp_data]
            
            line = folium.PolyLine(line_coords, color="#00ffff", weight=5).add_to(m_view)
            PolyLineTextPath(line, '  ►  ', repeat=True, offset=7, attributes={'fill': '#000000', 'font-weight': 'bold', 'font-size': '24', 'fill-opacity': '0.3'}).add_to(m_view)
            
            cum_dist = [0.0]
            total_dist_m = 0.0
            
            for i in range(len(wp_data) - 1):
                p1 = (wp_data[i]['lat'], wp_data[i]['lon'])
                p2 = (wp_data[i+1]['lat'], wp_data[i+1]['lon'])
                d = get_haversine_dist(p1, p2)
                total_dist_m += d
                cum_dist.append(total_dist_m)
                
                mid_lat = (p1[0] + p2[0]) / 2
                mid_lon = (p1[1] + p2[1]) / 2
                folium.Marker(
                    location=[mid_lat, mid_lon],
                    icon=DivIcon(
                        icon_size=(100, 20), icon_anchor=(50, 10),
                        html=f'<div style="font-size: 12pt; color: #ffffff; text-shadow: 2px 2px 4px #000000, -1px -1px 0 #000, 1px -1px 0 #000, -1px 1px 0 #000, 1px 1px 0 #000; font-weight: bold; text-align: center;">{d * M_TO_FT:.1f} ft</div>'
                    )
                ).add_to(m_view)
            
            gap = max(1.0, float(meta['t_val'])) if meta['mode'] == "Distance" else (meta['speed'] * meta['t_val'])
            
            for w in wp_data:
                i = w['index']
                length = 0.00012
                end_lat = w['lat'] + length * math.cos(math.radians(w['yaw']))
                end_lon = w['lon'] + length * math.sin(math.radians(w['yaw']))
                folium.PolyLine([[w['lat'], w['lon']], [end_lat, end_lon]], color="#ff0000", weight=4).add_to(m_view)
                
                folium.Marker(
                    location=[w['lat'], w['lon']],
                    icon=DivIcon(
                        icon_size=(24,24), icon_anchor=(12,12),
                        html=f'<div style="font-size: 11pt; color: black; background: white; border-radius: 50%; text-align: center; border: 2px solid black; font-weight: bold; width: 24px; height: 24px; line-height: 20px;">{i}</div>'
                    )
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
                            folium.CircleMarker([lat, lon], radius=2.5, color="yellow", fill=True).add_to(m_view)
                            photo_count += 1
                            break
                    current_dist += gap

            cam_type = meta.get('camera_type', 'visable')
            cam_display = CAM_DISPLAY_MAP.get(cam_type, "RGB Only")
            hw_key = "Unknown Configuration"
            for k, v in HARDWARE_MAP.items():
                if v["drone_sub"] == meta.get('drone_sub', '') and v["payload_sub"] == meta.get('payload_sub', ''):
                    hw_key = k

            st.sidebar.header("Mission Metadata")
            st.sidebar.write(f"**Hardware Platform:** {hw_key}")
            st.sidebar.write(f"**Camera Sensor:** {cam_display}")
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