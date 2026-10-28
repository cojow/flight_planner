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
import platform
import subprocess
import urllib.request
import urllib.parse
from datetime import datetime

import folium
from streamlit_folium import st_folium

# Reading and drawing flights is shared with the Flight Planner's
# "Show Logged Flights" layer (app.py), so both show flights the same way.
from logged_flights import add_flights_layer, flight_date, load_config, load_flights, update_config

st.set_page_config(page_title="Flight Log Tracker", layout="wide", page_icon="🛰️")

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

    add_flights_layer(m, flights, cone_count=cone_count, details="hover")

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
ok_flights, bad_flights = load_flights(flights_dir, st.session_state.include_subdirs)

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
        # The uploader gets a new key after each save, which empties it; with a
        # fixed key it still held the files after st.rerun(), so every rerun
        # saved them again under a new timestamped name.
        uploaded = st.file_uploader("Upload completed flight(s) (.kmz)", type=["kmz"], accept_multiple_files=True,
                                    key=f"upload_{st.session_state.get('upload_count', 0)}")
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
            st.session_state.upload_count = st.session_state.get('upload_count', 0) + 1
            st.toast(f"Uploaded: {', '.join(saved)}")
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
                or q in flight_date(f).strftime('%Y-%m-%d').lower()
            ]

        rows = []
        for f in sorted(rows_source, key=flight_date, reverse=True):
            rows.append({
                'Date': flight_date(f).strftime('%Y-%m-%d'),
                'Name': f['name'],
                'Platform': f['platform'],
                'Height (ft)': f['alt_ft'],
                'Pitch (°)': f['pitch'],
                'Overlap (%)': f['overlap_pct'],
                'Photos': f['photo_count'],
            })
        st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)
