"""
Flight Log Tracker - a lightweight, read-only companion to the main Flight
Planner app. Points at a shared folder (e.g. a Box Drive-synced folder) full
of .kmz mission files and plots every flight found there on one map, so a
team can each upload the flights they've run and everyone sees the combined
history. See FLIGHT_TRACKER_README.txt for setup/usage instructions.
"""
import streamlit as st
import pandas as pd
import os
import json
import re
import math
import zipfile
import platform
import subprocess
import urllib.request
import urllib.parse
import xml.etree.ElementTree as ET
from datetime import datetime

import folium
from streamlit_folium import st_folium

st.set_page_config(page_title="Flight Log Tracker", layout="wide", page_icon="🛰️")

FT_TO_M = 0.3048
M_TO_FT = 1 / FT_TO_M

# Matches the naming convention the main app saves missions with, e.g.
# "Mission_Fly_H100A90OL75" or "Mission_Fly_H100A90OL75SO65" for mapping
# missions - lets height/pitch/overlap be read straight from the filename
# instead of having to re-derive them from raw waypoint geometry.
SUFFIX_RE = re.compile(r'_(Fly|Pilot)_H(\d+)A(\d+)OL(\d+)(?:SO(\d+))?$')

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".tracker_config.json")

PALETTE = ["#00e5ff", "#ff4dd2", "#ffe135", "#4dff4d", "#ff9500", "#ff4d4d", "#a680ff", "#66ffc2"]


def load_config():
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH) as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


def save_config(cfg):
    try:
        with open(CONFIG_PATH, 'w') as f:
            json.dump(cfg, f)
    except Exception:
        pass


def update_config(**kwargs):
    """Merges into the existing saved config rather than overwriting it, so
    setting one preference (e.g. the folder path) doesn't wipe out others
    (e.g. the include-subdirectories toggle)."""
    cfg = load_config()
    cfg.update(kwargs)
    save_config(cfg)


def pick_folder_dialog(prompt_title):
    """Opens the OS-native folder picker (AppleScript on macOS, tkinter on Windows)."""
    current_os = platform.system()
    folder_path = None

    if current_os == "Darwin":
        script = f'''
        tell application "System Events"
            activate
            set f to choose folder with prompt "{prompt_title}"
            return POSIX path of f
        end tell
        '''
        try:
            res = subprocess.run(['osascript', '-e', script], capture_output=True, text=True)
            if res.returncode == 0 and res.stdout.strip():
                folder_path = res.stdout.strip()
        except Exception as e:
            st.error(f"Failed to open Mac folder picker: {e}")
    elif current_os == "Windows":
        import tkinter as tk
        from tkinter import filedialog
        try:
            root = tk.Tk()
            root.withdraw()
            root.attributes('-topmost', True)
            folder = filedialog.askdirectory(master=root, title=prompt_title)
            root.destroy()
            if folder:
                folder_path = folder
        except Exception as e:
            st.error(f"Failed to open Windows folder picker: {e}")

    return folder_path


def get_haversine_dist(p1, p2):
    R = 6371000
    lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
    dlat, dlon = lat1 - lat2, lon1 - lon2
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def get_bearing(p1, p2):
    lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
    d_lon = lon2 - lon1
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


@st.cache_data(show_spinner=False, ttl=3600)
def fetch_uasfm_data(center_lat, center_lon, radius_deg=0.05):
    """FAA UAS Facility Map (LAANC ceiling) grid data around a point."""
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


def parse_flight_kmz(path):
    """
    Pulls just what the tracker needs to show for one flight: its path,
    a rough facing direction for the view cone, and the height/pitch/
    overlap/photo-count summary. Height/pitch/overlap are read from the
    _Fly_HxxAxxOLxx filename suffix when present (exact, and avoids
    re-deriving them from full footprint geometry); anything missing falls
    back to reading the KMZ directly.
    """
    fname = os.path.basename(path)
    base = fname[:-4] if fname.lower().endswith('.kmz') else fname
    result = {
        'file': fname, 'name': base, 'platform': 'Unknown',
        'alt_ft': None, 'pitch': None, 'overlap_pct': None, 'side_overlap_pct': None,
        'photo_count': 0, 'coords': [], 'heading0': 0.0, 'created': None,
        'mtime': os.path.getmtime(path), 'error': None,
    }

    m = SUFFIX_RE.search(base)
    if m:
        result['platform'] = m.group(1)
        result['alt_ft'] = float(m.group(2))
        result['pitch'] = float(m.group(3))
        result['overlap_pct'] = float(m.group(4))
        if m.group(5):
            result['side_overlap_pct'] = float(m.group(5))
        result['name'] = base[:m.start()] or base

    try:
        with zipfile.ZipFile(path, 'r') as kmz:
            names = kmz.namelist()
            is_fly = any(n.startswith('wpmz/') for n in names)
            if result['platform'] == 'Unknown':
                result['platform'] = 'Fly' if is_fly else 'Pilot'
            waylines_name = next((n for n in names if n.endswith('waylines.wpml')), None)
            template_name = next((n for n in names if n.endswith('template.kml')), None)
            if not waylines_name:
                result['error'] = 'No waylines.wpml found inside the .kmz'
                return result
            root_w = ET.fromstring(kmz.read(waylines_name))
            root_t = ET.fromstring(kmz.read(template_name)) if template_name else None
    except Exception as e:
        result['error'] = str(e)
        return result

    if root_t is not None:
        ct = root_t.find('.//{*}createTime')
        if ct is not None and ct.text:
            try:
                result['created'] = datetime.fromtimestamp(int(ct.text) / 1000)
            except Exception:
                pass

    coords = []
    photo_action_count = 0
    has_multi_trigger = False
    gap_ft = None
    start_idx = 0
    speed_m = None
    heading0 = None

    speed_node = root_w.find('.//{*}autoFlightSpeed')
    if speed_node is not None and speed_node.text:
        speed_m = float(speed_node.text)

    for i, pm in enumerate(root_w.findall('.//{*}Placemark')):
        c_node = pm.find('.//{*}coordinates')
        if c_node is not None:
            c_raw = c_node.text.strip().split(',')
            coords.append((float(c_raw[1]), float(c_raw[0])))

        if i == 0:
            if result['alt_ft'] is None:
                alt_node = pm.find('.//{*}executeHeight')
                if alt_node is not None and alt_node.text:
                    result['alt_ft'] = float(alt_node.text) * M_TO_FT
            if result['pitch'] is None:
                pitch_node = pm.find('.//{*}waypointGimbalHeadingParam/{*}waypointGimbalPitchAngle')
                if pitch_node is not None and pitch_node.text:
                    result['pitch'] = abs(float(pitch_node.text))
                else:
                    # DJI Fly doesn't carry that tag - pitch is set once via
                    # the first waypoint's gimbalRotate action instead.
                    for a in pm.findall('.//{*}action'):
                        func = a.find('.//{*}actionActuatorFunc')
                        if func is not None and func.text == 'gimbalRotate':
                            p_angle = a.find('.//{*}actionActuatorFuncParam/{*}gimbalPitchRotateAngle')
                            if p_angle is not None and p_angle.text:
                                result['pitch'] = abs(float(p_angle.text))
                            break
            heading_node = pm.find('.//{*}waypointHeadingAngle')
            if heading_node is not None and heading_node.text:
                heading0 = float(heading_node.text)

        for ag in pm.findall('.//{*}actionGroup'):
            t_type = ag.find('.//{*}actionTriggerType')
            if t_type is not None and t_type.text and 'multiple' in t_type.text:
                has_multi_trigger = True
                trigger_type = 'distance' if 'Distance' in t_type.text else 'time'
                t_param = ag.find('.//{*}actionTriggerParam')
                if t_param is not None and t_param.text:
                    val = float(t_param.text)
                    gap_ft = val * M_TO_FT if trigger_type == 'distance' else (speed_m or 0) * val * M_TO_FT
                s_idx = ag.find('.//{*}actionGroupStartIndex')
                if s_idx is not None and s_idx.text:
                    start_idx = int(s_idx.text)
            for a in ag.findall('.//{*}action'):
                func = a.find('.//{*}actionActuatorFunc')
                if func is not None and func.text == 'takePhoto':
                    photo_action_count += 1

    result['coords'] = coords
    if heading0 is not None:
        result['heading0'] = heading0
    elif len(coords) >= 2:
        result['heading0'] = get_bearing(coords[0], coords[1])

    if has_multi_trigger and gap_ft and len(coords) >= 2 and start_idx < len(coords):
        total_dist_ft = sum(get_haversine_dist(coords[i], coords[i + 1]) for i in range(len(coords) - 1)) * M_TO_FT
        dist_to_start = sum(get_haversine_dist(coords[i], coords[i + 1]) for i in range(start_idx)) * M_TO_FT
        result['photo_count'] = int(max(0, total_dist_ft - dist_to_start) / gap_ft) + 1
    else:
        result['photo_count'] = photo_action_count

    return result


def periodic_cone_points(coords, target_count=8, min_spacing_m=10):
    """
    Samples (lat, lon, heading) at roughly evenly-spaced intervals along a
    flight path, so view cones can be drawn periodically along the line
    rather than just once at the start. Spacing is derived from the path's
    total length so short and long flights both get a reasonable number of
    cones instead of a fixed absolute spacing.
    """
    if len(coords) < 2:
        return [(coords[0][0], coords[0][1], 0.0)] if coords else []

    cum = [0.0]
    total = 0.0
    for i in range(len(coords) - 1):
        total += get_haversine_dist(coords[i], coords[i + 1])
        cum.append(total)
    if total <= 0:
        return [(coords[0][0], coords[0][1], get_bearing(coords[0], coords[1]))]

    spacing = max(min_spacing_m, total / target_count)
    points = []
    target = 0.0
    seg = 0
    while target <= total + 1e-6:
        while seg < len(cum) - 2 and cum[seg + 1] < target:
            seg += 1
        seg_len = cum[seg + 1] - cum[seg]
        frac = (target - cum[seg]) / seg_len if seg_len > 0 else 0.0
        lat = coords[seg][0] + (coords[seg + 1][0] - coords[seg][0]) * frac
        lon = coords[seg][1] + (coords[seg + 1][1] - coords[seg][1]) * frac
        points.append((lat, lon, get_bearing(coords[seg], coords[seg + 1])))
        target += spacing
    return points


def add_view_cone(m, origin, heading_deg, color, length_m=12, half_fov_deg=20):
    """A small fixed-size FOV wedge - a visual indicator of facing
    direction, not a to-scale image footprint."""
    lat0, lon0 = origin

    def offset(bearing_deg, dist_m):
        dlat = (dist_m * math.cos(math.radians(bearing_deg))) / 111320.0
        dlon = (dist_m * math.sin(math.radians(bearing_deg))) / (111320.0 * math.cos(math.radians(lat0)) or 1e-9)
        return (lat0 + dlat, lon0 + dlon)

    left = offset(heading_deg - half_fov_deg, length_m)
    right = offset(heading_deg + half_fov_deg, length_m)
    folium.Polygon(
        locations=[origin, left, right],
        color=color, weight=1, fill=True, fill_opacity=0.4, fill_color=color,
    ).add_to(m)


def build_tooltip_html(f):
    date_str = (f['created'] or datetime.fromtimestamp(f['mtime'])).strftime('%b %d, %Y')
    alt_str = f"{f['alt_ft']:.0f} ft" if f['alt_ft'] is not None else "n/a"
    pitch_str = f"{f['pitch']:.0f}°" if f['pitch'] is not None else "n/a"
    ol_str = f"{f['overlap_pct']:.0f}%" if f['overlap_pct'] is not None else "n/a"
    if f.get('side_overlap_pct') is not None:
        ol_str += f" (side {f['side_overlap_pct']:.0f}%)"
    return f"""
    <div style="font-family: sans-serif; font-size: 12px; line-height: 1.5;">
      <b>{f['name']}</b> &middot; {f['platform']}<br>
      Date: {date_str}<br>
      Photos: {f['photo_count']}<br>
      Height: {alt_str}<br>
      Pitch: {pitch_str}<br>
      Overlap: {ol_str}
    </div>
    """


def build_map(flights, show_faa=False, cone_count=8):
    all_pts = []
    for f in flights:
        all_pts.extend(f['coords'])
    if not all_pts:
        return None

    center = all_pts[0]
    m = folium.Map(location=center, zoom_start=17, tiles=None)
    folium.TileLayer(
        tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
        attr='Google', max_zoom=22, max_native_zoom=20,
    ).add_to(m)

    if show_faa:
        uasfm_data = fetch_uasfm_data(center[0], center[1])
        if uasfm_data and uasfm_data.get("features"):
            folium.GeoJson(
                uasfm_data, name="FAA UASFM Grids",
                style_function=lambda x: {
                    'fillColor': 'red' if x['properties'].get('CEILING', x['properties'].get('ceiling', -1)) == 0 else 'green',
                    'color': 'black', 'weight': 1, 'fillOpacity': 0.15,
                },
                tooltip=folium.GeoJsonTooltip(fields=['CEILING'], aliases=['Max LAANC Altitude (ft):']),
            ).add_to(m)

    for idx, f in enumerate(flights):
        color = PALETTE[idx % len(PALETTE)]
        folium.PolyLine(
            f['coords'], color=color, weight=4, opacity=0.85,
            tooltip=folium.Tooltip(build_tooltip_html(f), sticky=True),
        ).add_to(m)
        for lat, lon, heading in periodic_cone_points(f['coords'], target_count=cone_count):
            add_view_cone(m, (lat, lon), heading, color)

    lats = [p[0] for p in all_pts]
    lons = [p[1] for p in all_pts]
    m.fit_bounds([[min(lats), min(lons)], [max(lats), max(lons)]])
    return m


# ==========================================
# UI
# ==========================================
st.markdown("""
<style>
html, body { overflow: hidden !important; }
[data-testid="stAppViewContainer"] { overflow: hidden !important; }
[data-testid="stHeader"] { background: transparent !important; height: 2.2rem !important; }
[data-testid="stMainBlockContainer"] { padding: 0 !important; margin: 0 !important; max-width: 100% !important; }
[data-testid="stMain"] { overflow: hidden !important; height: 100vh !important; }
[data-testid="stMainBlockContainer"] > div:first-child { gap: 0 !important; }
footer { display: none !important; }

/* Base layer: the map fills the whole viewport */
.st-key-map_layer { position: fixed !important; inset: 0 !important; z-index: 0 !important; }
.st-key-map_layer iframe { height: 100vh !important; width: 100vw !important; display: block !important; }

/* Floating panels, on top of the map - both anchored to the right edge */
.st-key-settings_panel {
    position: fixed !important; top: 16px !important; right: 16px !important; z-index: 9999 !important;
    background: rgba(255,255,255,0.95) !important; backdrop-filter: blur(6px);
    border-radius: 12px !important; padding: 10px 14px !important; width: 300px !important;
    box-shadow: 0 4px 18px rgba(0,0,0,0.22) !important;
}
.st-key-list_trigger_panel {
    position: fixed !important; bottom: 16px !important; right: 16px !important; z-index: 9999 !important;
    background: rgba(255,255,255,0.95) !important; backdrop-filter: blur(6px);
    border-radius: 12px !important; padding: 10px 14px !important; width: 220px !important;
    box-shadow: 0 4px 18px rgba(0,0,0,0.22) !important; text-align: right !important;
}

/* Flight list slide-up, covering the bottom of the map */
.st-key-flight_panel {
    position: fixed !important; bottom: 0 !important; left: 0 !important; right: 0 !important;
    width: 100vw !important; height: 50vh !important; z-index: 10000 !important;
    background: white !important; overflow-y: auto !important; padding: 24px !important;
    box-shadow: 0 -8px 24px rgba(0,0,0,0.32) !important;
}

/* Compact widgets inside the floating panels */
.st-key-settings_panel [data-testid="stTextInput"] input, .st-key-settings_panel button { font-size: 0.85rem !important; }
.st-key-settings_panel h1, .st-key-list_trigger_panel h1 { font-size: 1.1rem !important; margin: 0 0 4px 0 !important; }
</style>
""", unsafe_allow_html=True)

if 'flights_dir' not in st.session_state:
    st.session_state.flights_dir = load_config().get('flights_dir', '')
if 'show_flight_list' not in st.session_state:
    st.session_state.show_flight_list = False
if 'show_faa' not in st.session_state:
    st.session_state.show_faa = False
if 'include_subdirs' not in st.session_state:
    st.session_state.include_subdirs = load_config().get('include_subdirs', False)

flights_dir = st.session_state.flights_dir
flights, ok_flights, bad_flights = [], [], []
if flights_dir and os.path.isdir(flights_dir):
    if st.session_state.include_subdirs:
        kmz_paths = sorted(
            os.path.join(root, fname)
            for root, _dirs, files in os.walk(flights_dir)
            for fname in files if fname.lower().endswith('.kmz')
        )
    else:
        kmz_paths = sorted(
            os.path.join(flights_dir, f) for f in os.listdir(flights_dir) if f.lower().endswith('.kmz')
        )
    flights = [parse_flight_kmz(p) for p in kmz_paths]
    ok_flights = [f for f in flights if f['coords'] and not f['error']]
    bad_flights = [f for f in flights if f['error'] or not f['coords']]

# --- Base layer: full-page map ---
with st.container(key="map_layer"):
    flight_map = build_map(ok_flights, show_faa=st.session_state.show_faa) if ok_flights else None
    if flight_map:
        st_folium(flight_map, use_container_width=True, height=1000, returned_objects=[])
    else:
        st.info(
            "No flights to show yet. Set your shared folder and upload a .kmz to get started."
            if flights_dir else
            "Set your shared flight folder in the panel on the right to get started."
        )

# --- Top-right floating panel: identity, folder settings, upload, FAA toggle ---
with st.container(key="settings_panel"):
    st.markdown("# 🛰️ Flight Log Tracker")
    with st.popover("⚙️ Settings", use_container_width=True):
        st.caption("Point this at your shared Box-synced flight-log folder.")
        typed_dir = st.text_input(
            "Shared flight folder", value=st.session_state.flights_dir,
            placeholder="e.g. /Users/you/Library/CloudStorage/Box-Box/Flight Logs",
        )
        if st.button("📂 Browse"):
            picked = pick_folder_dialog("Select the shared flight-log folder")
            if picked:
                typed_dir = picked
        if typed_dir != st.session_state.flights_dir:
            st.session_state.flights_dir = typed_dir
            update_config(flights_dir=typed_dir)
            if typed_dir and not os.path.isdir(typed_dir):
                try:
                    os.makedirs(typed_dir, exist_ok=True)
                except Exception as e:
                    st.error(f"Can't access or create that folder: {e}")
            st.rerun()

        st.checkbox(
            "Include subdirectories", key="include_subdirs",
            help="Also scan folders inside the shared flight folder for .kmz files.",
            on_change=lambda: update_config(include_subdirs=st.session_state.include_subdirs),
        )

        st.divider()
        uploaded = st.file_uploader("Upload completed flight(s) (.kmz)", type=["kmz"], accept_multiple_files=True)
        if uploaded and flights_dir:
            saved = []
            for uf in uploaded:
                dest = os.path.join(flights_dir, uf.name)
                if os.path.exists(dest):
                    base, ext = os.path.splitext(uf.name)
                    dest = os.path.join(flights_dir, f"{base}_{datetime.now().strftime('%Y%m%d%H%M%S')}{ext}")
                with open(dest, 'wb') as out:
                    out.write(uf.getbuffer())
                saved.append(os.path.basename(dest))
            st.success(f"Uploaded: {', '.join(saved)}")
            st.rerun()

        if bad_flights:
            st.divider()
            st.caption(f"⚠️ {len(bad_flights)} file(s) couldn't be read:")
            for f in bad_flights:
                st.caption(f"- {f['file']}: {f['error'] or 'no coordinates found'}")

    st.checkbox("Show FAA restrictions", key="show_faa")

# --- Bottom-right floating panel: flight count + list toggle ---
with st.container(key="list_trigger_panel"):
    st.markdown(f"**{len(ok_flights)} flight{'s' if len(ok_flights) != 1 else ''} logged**")
    if st.button("📋 Flight List", use_container_width=True):
        st.session_state.show_flight_list = not st.session_state.show_flight_list
        st.rerun()

# --- Slide-over flight list, covering half the map ---
if st.session_state.show_flight_list:
    with st.container(key="flight_panel"):
        c1, c2 = st.columns([5, 1])
        with c1:
            st.markdown("## Flight list")
        with c2:
            if st.button("✕", key="close_flight_list"):
                st.session_state.show_flight_list = False
                st.rerun()

        query = st.text_input("Search by name or date", placeholder="e.g. Testy or 2026-07")
        rows_source = ok_flights
        if query:
            q = query.strip().lower()
            rows_source = [
                f for f in ok_flights
                if q in f['name'].lower()
                or q in (f['created'] or datetime.fromtimestamp(f['mtime'])).strftime('%Y-%m-%d').lower()
            ]

        rows = []
        for f in sorted(rows_source, key=lambda x: x['created'] or datetime.fromtimestamp(x['mtime']), reverse=True):
            rows.append({
                'Date': (f['created'] or datetime.fromtimestamp(f['mtime'])).strftime('%Y-%m-%d'),
                'Name': f['name'],
                'Platform': f['platform'],
                'Height (ft)': f['alt_ft'],
                'Pitch (°)': f['pitch'],
                'Overlap (%)': f['overlap_pct'],
                'Photos': f['photo_count'],
            })
        st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)
