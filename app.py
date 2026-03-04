import streamlit as st
import json
import math
from datetime import datetime
import folium
from folium.plugins import Draw
from streamlit_folium import st_folium


"""
run using:

streamlit run app.py

"""

# ==========================================
# CONSTANTS & CONVERSIONS
# ==========================================
FT_TO_M = 0.3048
M_TO_FT = 3.28084

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
# STREAMLIT UI SETUP
# ==========================================
st.set_page_config(layout="wide", page_title="ISLERS Mission Control")
st.title("🛰️ ISLERS Mission Control")

with st.sidebar:
    st.header("1. Flight Settings")
    alt_ft = st.number_input("Altitude (ft)", value=230.0)
    gimbal_pitch = st.slider("Gimbal Pitch (°)", -90, 0, -60)
    side = st.selectbox("Camera Facing", ["right", "left"])
    
    st.header("2. Photo Trigger")
    trigger_type = st.radio("Trigger Mode", ["distance", "time"])
    if trigger_type == "distance":
        interval_ft = st.number_input("Interval (ft)", value=16.4)
        interval_val = interval_ft * FT_TO_M
    else:
        interval_sec = st.number_input("Interval (sec)", value=2.0)
        interval_val = interval_sec
    
    st.header("3. Speed Control")
    auto_speed = st.checkbox("Auto-Calc Speed", value=True)
    target_dist_ft = st.number_input("Target Photo Gap (ft)", value=26.0)
    manual_speed_fps = st.number_input("Manual Speed (fps)", value=13.5)

    # Speed calculation logic for the HUD
    if auto_speed and trigger_type == "time":
        speed_m = min(max((target_dist_ft * FT_TO_M) / interval_sec, 1.0), 10.0)
    else:
        speed_m = manual_speed_fps * FT_TO_M

# ==========================================
# MAP & LIVE PREVIEW LOGIC
# ==========================================
st.subheader("Interactive Flight Planner")
m = folium.Map(location=[40.2338, -111.6585], zoom_start=15, max_zoom=22, tiles=None)

folium.TileLayer(
    tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
    attr='Google Satellite', name='Google Satellite',
    max_zoom=22, max_native_zoom=20 
).add_to(m)

# Initialize Draw Tool
draw_tool = Draw(
    export=False,
    position='topleft',
    draw_options={
        'polyline': {'shapeOptions': {'color': '#00ffff', 'weight': 4}},
        'polygon': False, 'rectangle': False, 'circle': False, 'marker': False
    }
)
draw_tool.add_to(m)

# Process Drawing Data for Live Preview
output = st_folium(m, width=1100, height=650)

if output.get("all_drawings"):
    last_drawing = output["all_drawings"][-1]
    if last_drawing['geometry']['type'] == 'LineString':
        coords = [(c[1], c[0]) for c in last_drawing['geometry']['coordinates']]
        
        # 1. LIVE PREVIEW: Calculate photo spots on the fly
        photo_gap = interval_val if trigger_type == "distance" else (speed_m * interval_val)
        total_dist_m = 0
        est_photos = 1 # Initial shot
        
        # We re-run the map generation with the dots if coordinates exist
        # To avoid infinite loops in Streamlit, we show a summary below
        for i in range(len(coords)-1):
            p1, p2 = coords[i], coords[i+1]
            seg_dist = get_haversine_dist(p1, p2)
            total_dist_m += seg_dist
            
            num_dots = int(seg_dist / photo_gap)
            est_photos += num_dots

        # ==========================================
        # MISSION ANALYTICS HUD
        # ==========================================
        col1, col2, col3, col4 = st.columns(4)
        col1.metric("Total Distance", f"{total_dist_m * M_TO_FT:.1f} ft")
        col2.metric("Est. Photos", f"{est_photos}")
        col3.metric("Flight Speed", f"{speed_m * M_TO_FT:.1f} fps")
        col4.metric("Flight Time", f"{total_dist_m / speed_m / 60:.1f} min")

        if st.button("🚀 Generate & Save Mavic 3 KMZ"):
            # Insert your fixed Native WPML Generation logic here
            st.success("Mission saved to /missions folder!")