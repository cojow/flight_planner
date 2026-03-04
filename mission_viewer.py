import os
import zipfile
import xml.etree.ElementTree as ET
import math
import webbrowser
import folium
from branca.element import Element

# ==========================================
# CONFIGURATION
# ==========================================
KMZ_TO_VIEW = "missions/Mavic3_Feet_Native_20260303_210230.kmz" 
USE_FEET = True  # Set to True to see HUD results in Feet/FPS

# Conversion Constant
M_TO_FT = 3.28084

def get_haversine_dist(p1, p2):
    R = 6371000 
    lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
    dlat, dlon = lat1 - lat2, lon1 - lon2
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def parse_dji_kmz(kmz_path):
    waypoints = []
    meta = {"speed": 0, "pitch": 0, "mode": "None", "trigger_val": 0, "altitude": 0}
    
    if not os.path.exists(kmz_path):
        return None, None, f"Error: {kmz_path} not found."

    with zipfile.ZipFile(kmz_path, 'r') as kmz:
        wpml_data = kmz.read('wpmz/waylines.wpml')
        root = ET.fromstring(wpml_data)
    
    ns = {'kml': 'http://www.opengis.net/kml/2.2', 'wpml': 'http://www.dji.com/wpmz/1.0.6'}

    # 1. Extract Global Speed & Stats
    speed_node = root.find('.//wpml:autoFlightSpeed', ns)
    if speed_node is not None: meta['speed'] = float(speed_node.text)

    # 2. Extract Waypoints & Heading
    for pm in root.findall('.//kml:Placemark', ns):
        coords_node = pm.find('.//kml:coordinates', ns)
        yaw_node = pm.find('.//wpml:waypointHeadingAngle', ns)
        alt_node = pm.find('.//wpml:executeHeight', ns)
        
        if coords_node is not None:
            # Clean up whitespace/line breaks from coordinate string
            coords = coords_node.text.strip().replace('\n', '').replace(' ', '').split(',')
            if len(coords) >= 2:
                waypoints.append({
                    'lat': float(coords[1]), 
                    'lon': float(coords[0]), 
                    'yaw': float(yaw_node.text) if yaw_node is not None else 0.0
                })
                if alt_node is not None: meta['altitude'] = float(alt_node.text)

        # 3. Enhanced Action Detection (Trigger Params)
        # Look for the new 1.0.6 triggerParam tag
        trigger_param = pm.find('.//wpml:actionTriggerParam', ns)
        trigger_type = pm.find('.//wpml:actionTriggerType', ns)
        
        if trigger_param is not None and trigger_type is not None:
            meta['trigger_val'] = float(trigger_param.text)
            meta['mode'] = "Distance" if "Distance" in trigger_type.text else "Time"

        # Check for Gimbal Pitch inside action groups
        pitch_node = pm.find('.//wpml:gimbalPitchRotateAngle', ns)
        if pitch_node is not None: meta['pitch'] = float(pitch_node.text)

    return waypoints, meta, None

def run_inspector(kmz_path):
    waypoints, meta, error = parse_dji_kmz(kmz_path)
    if not waypoints:
        print(f"Error: Could not find valid waypoints in {kmz_path}")
        return

    # Create Map with Ultra-Zoom
    m = folium.Map(location=[waypoints[0]['lat'], waypoints[0]['lon']], 
                   zoom_start=18, max_zoom=22, tiles=None)

    folium.TileLayer(
        tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
        attr='Google Satellite', name='Google Satellite',
        max_zoom=22, max_native_zoom=20 
    ).add_to(m)

    # 1. Draw Path (Cyan)
    path_coords = [[w['lat'], w['lon']] for w in waypoints]
    folium.PolyLine(path_coords, color="#00ffff", weight=5, opacity=0.8).add_to(m)

    # 2. Draw Camera Vectors (Red)
    for wp in waypoints:
        length = 0.00015 
        end_lat = wp['lat'] + length * math.cos(math.radians(wp['yaw']))
        end_lon = wp['lon'] + length * math.sin(math.radians(wp['yaw']))
        folium.PolyLine([[wp['lat'], wp['lon']], [end_lat, end_lon]], color="#ff0000", weight=4).add_to(m)
        folium.CircleMarker([wp['lat'], wp['lon']], radius=4, color="white", fill=True, fill_color="black").add_to(m)

    # 3. Calculate Intervals & Yellow Dots
    photo_gap = meta['trigger_val'] if meta['mode'] == "Distance" else (meta['speed'] * meta['trigger_val'])
    total_dist, est_photos = 0, 0
    
    if photo_gap > 0:
        for i in range(len(waypoints) - 1):
            p1, p2 = (waypoints[i]['lat'], waypoints[i]['lon']), (waypoints[i+1]['lat'], waypoints[i+1]['lon'])
            seg_dist = get_haversine_dist(p1, p2)
            total_dist += seg_dist
            num_dots = int(seg_dist / photo_gap)
            for j in range(1, num_dots + 1):
                frac = (j * photo_gap) / seg_dist
                lat = p1[0] + (p2[0] - p1[0]) * frac
                lon = p1[1] + (p2[1] - p1[1]) * frac
                folium.CircleMarker([lat, lon], radius=2.5, color="#ffd600", fill=True).add_to(m)
                est_photos += 1
        est_photos += 1 # First shot

    # 4. HUD Logic (Unit Conversion)
    if USE_FEET:
        display_alt = f"{meta['altitude'] * M_TO_FT:.0f} ft"
        display_speed = f"{meta['speed'] * M_TO_FT:.1f} fps"
        display_dist = f"{total_dist * M_TO_FT:.0f} ft"
        t_val = meta['trigger_val'] * M_TO_FT if meta['mode'] == "Distance" else meta['trigger_val']
        t_unit = "ft" if meta['mode'] == "Distance" else "s"
    else:
        display_alt = f"{meta['altitude']:.1f} m"
        display_speed = f"{meta['speed']:.1f} m/s"
        display_dist = f"{total_dist:.0f} m"
        t_val = meta['trigger_val']
        t_unit = "m" if meta['mode'] == "Distance" else "s"

    hud_html = f'''
        <div style="position: fixed; bottom: 40px; left: 40px; width: 260px; background-color: rgba(255,255,255,0.95); 
                    border:2px solid #333; z-index:9999; padding: 15px; border-radius: 8px; font-family: sans-serif; box-shadow: 3px 3px 10px rgba(0,0,0,0.2);">
            <h3 style="margin:0 0 10px 0; color: #1a73e8;">Mission Inspector</h3>
            <b>Altitude:</b> {display_alt}<br>
            <b>Trigger:</b> {meta['mode']} ({t_val:.1f}{t_unit})<br>
            <b>Speed:</b> {display_speed}<br>
            <b>Gimbal Pitch:</b> {meta['pitch']}°<br>
            <hr style="margin: 10px 0;">
            <b>Total Distance:</b> {display_dist}<br>
            <b style="font-size: 1.1em; color: #d32f2f;">Total Photos: {est_photos}</b><br>
            <p style="font-size: 11px; margin: 10px 0 0 0;">
                <span style="color: #00ffff;">■</span> Path 
                <span style="color: #ff0000;">■</span> Camera Yaw 
                <span style="color: #ffd600;">●</span> Photo Spot
            </p>
        </div>
    '''
    m.get_root().html.add_child(Element(hud_html))

    out_file = "mission_inspector.html"
    m.save(out_file)
    webbrowser.open("file://" + os.path.realpath(out_file))

if __name__ == "__main__":
    target = KMZ_TO_VIEW
    if not os.path.exists(target):
        if os.path.exists("missions"):
            files = [f for f in os.listdir("missions") if f.endswith(".kmz")]
            if files: target = os.path.join("missions", sorted(files)[-1])
    
    run_inspector(target)