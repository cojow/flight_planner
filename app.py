import streamlit as st
import json
import os
import math
import zipfile
import io
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
SENSOR_W = 17.3  # mm
FOCAL_L = 12.3   # mm
IMAGE_W = 5280   # px

if 'page' not in st.session_state:
    st.session_state.page = 'planner'

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
# PART 1: NATIVE KMZ GENERATOR
# ==========================================
def generate_native_wpml(coords, cfg):
    ms_ts = int(datetime.now().timestamp() * 1000)
    alt_m = cfg["alt_ft"] * FT_TO_M
    speed_m = cfg["speed_m"]
    
    total_dist_m = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1))
    total_duration = total_dist_m / speed_m

    waypoints_xml = ""
    for i, p in enumerate(coords):
        ref_bearing = get_bearing(coords[i], coords[i+1]) if i < len(coords)-1 else get_bearing(coords[i-1], coords[i])
        yaw = (ref_bearing + 90) % 360 if cfg['side'] == "right" else (ref_bearing - 90) % 360

        action_group = ""
        if i == 0:
            trigger_type = "multipleDistance" if cfg["trigger_type"] == "distance" else "multipleTiming"
            interval_val = (cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['interval_sec']
            action_group = f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>0</wpml:actionGroupId><wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{len(coords)-1}</wpml:actionGroupEndIndex><wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>{trigger_type}</wpml:actionTriggerType>
            <wpml:actionTriggerParam>{interval_val:.3f}</wpml:actionTriggerParam></wpml:actionTrigger>
          <wpml:action><wpml:actionId>0</wpml:actionId><wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam></wpml:action>
        </wpml:actionGroup>"""

        waypoints_xml += f"""
      <Placemark>
        <Point><coordinates>{p[1]},{p[0]},{alt_m:.3f}</coordinates></Point>
        <wpml:index>{i}</wpml:index><wpml:executeHeight>{alt_m:.3f}</wpml:executeHeight>
        <wpml:waypointSpeed>{speed_m:.2f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam><wpml:waypointHeadingMode>fixed</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{yaw:.1f}</wpml:waypointHeadingAngle><wpml:waypointHeadingAngleEnable>1</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode></wpml:waypointHeadingParam>
        <wpml:waypointTurnParam><wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist></wpml:waypointTurnParam>
        <wpml:useStraightLine>1</wpml:useStraightLine><wpml:isRisky>0</wpml:isRisky><wpml:waypointWorkType>0</wpml:waypointWorkType>
        {action_group}
      </Placemark>"""

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document><wpml:createTime>{ms_ts}</wpml:createTime><wpml:updateTime>{ms_ts}</wpml:updateTime>
    <wpml:missionConfig><wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode><wpml:finishAction>goHome</wpml:finishAction>
      <wpml:droneInfo><wpml:droneEnumValue>77</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:payloadInfo><wpml:payloadEnumValue>68</wpml:payloadEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder><wpml:templateType>waypoint</wpml:templateType><wpml:templateId>0</wpml:templateId><wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>{total_dist_m:.2f}</wpml:distance><wpml:duration>{total_duration:.2f}</wpml:duration>
      <wpml:autoFlightSpeed>{speed_m:.2f}</wpml:autoFlightSpeed><wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:startActionGroup>
        <wpml:action><wpml:actionId>0</wpml:actionId><wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam><wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
            <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable><wpml:gimbalPitchRotateAngle>{cfg['pitch']}</wpml:gimbalPitchRotateAngle>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam></wpml:action>
        <wpml:action><wpml:actionId>1</wpml:actionId><wpml:actionActuatorFunc>setFocusType</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam><wpml:cameraFocusType>manual</wpml:cameraFocusType><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam></wpml:action>
        <wpml:action><wpml:actionId>2</wpml:actionId><wpml:actionActuatorFunc>focus</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam><wpml:isInfiniteFocus>1</wpml:isInfiniteFocus><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam></wpml:action>
      </wpml:startActionGroup>
      {waypoints_xml}</Folder></Document></kml>"""

# ==========================================
# MAIN UI
# ==========================================
st.set_page_config(layout="wide", page_title="ISLERS Control")

col_t, col_n = st.columns([4, 1])
col_t.title("🛰️ ISLERS Mission Control")
if col_n.button("🔄 Switch to " + ("Viewer" if st.session_state.page == 'planner' else "Planner")):
    st.session_state.page = 'viewer' if st.session_state.page == 'planner' else 'planner'
    st.rerun()

# --- PLANNER MODE ---
if st.session_state.page == 'planner':
    with st.sidebar:
        st.header("Flight Settings")
        mission_name = st.text_input("Mission Name", "Provo_Mission_1")
        alt_ft = st.number_input("Altitude (ft)", 10, 400, 30)
        
        # GSD CALCULATOR
        gsd_cm = (alt_ft * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W)
        st.info(f"📐 Est. GSD: {gsd_cm:.2f} cm/px ({gsd_cm/2.54:.2f} in/px)")
        
        gimbal_pitch = st.slider("Gimbal Pitch", -90, 0, -60)
        side = st.selectbox("Side", ["right", "left"])
        
        st.header("Trigger")
        trigger = st.radio("Type", ["distance", "time"])
        t_val = st.number_input("Interval (ft or sec)", 2)
        
        st.header("Speed")
        auto_speed = st.checkbox("Auto-Calc Speed", True)
        target_gap_ft = st.number_input("Target Gap (ft)", 26.2)
        manual_fps = st.number_input("Manual Speed (fps)", 13.5)

    speed_m = min(max((target_gap_ft * FT_TO_M) / t_val, 1.0), 10.0) if (auto_speed and trigger == "time") else (manual_fps * FT_TO_M)

    m = folium.Map(location=[40.24, -111.65], zoom_start=16, tiles=None)
    folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m)
    Draw(export=False, draw_options={'polyline':{'shapeOptions':{'color':'#00ffff','weight':5}}}).add_to(m)
    map_data = st_folium(m, width=1200, height=600)

    if map_data.get("all_drawings"):
        coords = [(c[1], c[0]) for c in map_data["all_drawings"][-1]['geometry']['coordinates']]
        total_dist_ft = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1)) * M_TO_FT
        photo_gap_ft = t_val if trigger == "distance" else (speed_m * t_val * M_TO_FT)
        est_photos = int(total_dist_ft / photo_gap_ft) + 1
        
        st.divider()
        c1, c2, c3 = st.columns(3)
        c1.metric("Distance", f"{total_dist_ft:.1f} ft")
        c2.metric("Est. Photos", f"{est_photos}")
        c3.metric("Speed", f"{speed_m * M_TO_FT:.1f} fps")

        if st.button("🚀 Save & Generate KMZ"):
            cfg = {"alt_ft": alt_ft, "pitch": gimbal_pitch, "side": side, "trigger_type": trigger, "interval_ft": t_val, "interval_sec": t_val, "speed_m": speed_m}
            wpml = generate_native_wpml(coords, cfg)
            with zipfile.ZipFile(f"missions/{mission_name}.kmz", 'w') as kmz:
                kmz.writestr("wpmz/waylines.wpml", wpml)
                kmz.writestr("wpmz/template.kml", wpml)
            st.success(f"Saved {mission_name}.kmz to missions/")

# --- VIEWER MODE ---
else:
    st.subheader("🔍 Mission Inspector")
    kmz_files = [f for f in os.listdir(MISSION_DIR) if f.endswith(".kmz")]
    if not kmz_files:
        st.warning("No missions found.")
    else:
        selected_kmz = st.selectbox("Select KMZ", kmz_files)
        full_path = os.path.join(MISSION_DIR, selected_kmz)
        ns = {'kml': 'http://www.opengis.net/kml/2.2', 'wpml': 'http://www.dji.com/wpmz/1.0.6'}
        
        with zipfile.ZipFile(full_path, 'r') as kmz:
            root = ET.fromstring(kmz.read('wpmz/waylines.wpml'))
        
        meta = {"speed": 0, "pitch": 0, "mode": "None", "t_val": 0, "alt": 0}
        s_node = root.find('.//wpml:autoFlightSpeed', ns)
        if s_node is not None: meta['speed'] = float(s_node.text)

        wp_data = []
        for pm in root.findall('.//kml:Placemark', ns):
            c_raw = pm.find('.//kml:coordinates', ns).text.strip().split(',')
            yaw = float(pm.find('.//wpml:waypointHeadingAngle', ns).text)
            alt = float(pm.find('.//wpml:executeHeight', ns).text)
            wp_data.append({'lat': float(c_raw[1]), 'lon': float(c_raw[0]), 'yaw': yaw})
            meta['alt'] = alt
            
            t_p = pm.find('.//wpml:actionTriggerParam', ns)
            t_t = pm.find('.//wpml:actionTriggerType', ns)
            if t_p is not None:
                meta['t_val'] = float(t_p.text)
                meta['mode'] = "Distance" if "Distance" in t_t.text else "Time"
            
            p_node = pm.find('.//wpml:gimbalPitchRotateAngle', ns)
            if p_node is not None: meta['pitch'] = float(p_node.text)

        if wp_data:
            m_view = folium.Map(location=[wp_data[0]['lat'], wp_data[0]['lon']], zoom_start=19, tiles=None)
            folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_view)
            
            folium.PolyLine([[w['lat'], w['lon']] for w in wp_data], color="#00ffff", weight=5).add_to(m_view)
            gap = meta['t_val'] if meta['mode'] == "Distance" else (meta['speed'] * meta['t_val'])
            total_dist_m = 0
            
            for i in range(len(wp_data)):
                wp = wp_data[i]
                length = 0.00015
                end_lat = wp['lat'] + length * math.cos(math.radians(wp['yaw']))
                end_lon = wp['lon'] + length * math.sin(math.radians(wp['yaw']))
                folium.PolyLine([[wp['lat'], wp['lon']], [end_lat, end_lon]], color="#ff0000", weight=4).add_to(m_view)
                folium.CircleMarker([wp['lat'], wp['lon']], radius=3, color="white", fill=True).add_to(m_view)
                
                if i < len(wp_data) - 1:
                    p1, p2 = (wp_data[i]['lat'], wp_data[i]['lon']), (wp_data[i+1]['lat'], wp_data[i+1]['lon'])
                    seg_dist = get_haversine_dist(p1, p2)
                    total_dist_m += seg_dist
                    if gap > 0:
                        for j in range(1, int(seg_dist / gap) + 1):
                            frac = (j * gap) / seg_dist
                            folium.CircleMarker([p1[0] + (p2[0]-p1[0])*frac, p1[1] + (p2[1]-p1[1])*frac], radius=2.5, color="yellow", fill=True).add_to(m_view)

            hud_html = f'''
                <div style="position: fixed; bottom: 40px; left: 40px; width: 240px; background-color: rgba(255,255,255,0.9); 
                            border:2px solid #333; z-index:9999; padding: 15px; border-radius: 8px; font-family: sans-serif;">
                    <h4 style="margin:0 0 10px 0;">Mission HUD</h4>
                    <b>Altitude:</b> {meta['alt']*M_TO_FT:.0f} ft<br>
                    <b>Speed:</b> {meta['speed']*M_TO_FT:.1f} fps<br>
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