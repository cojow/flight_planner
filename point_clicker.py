

'''
Run using the following in the terminal:

python point_clicker.py

'''
import os
import webbrowser
import json
import threading
import time
from datetime import datetime
from flask import Flask, request, jsonify
from flask_cors import CORS
import folium
from folium.plugins import Draw
from branca.element import Element

app = Flask(__name__)
CORS(app)

# --- MISSION DIRECTORY ---
MISSION_FOLDER = os.path.join(os.getcwd(), "missions")
os.makedirs(MISSION_FOLDER, exist_ok=True)

@app.route('/save', methods=['POST'])
def save_geojson():
    try:
        data = request.json
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"path_{ts}.geojson"
        filepath = os.path.join(MISSION_FOLDER, filename)
        
        with open(filepath, "w") as f:
            json.dump(data, f)
        
        latest_path = os.path.join(MISSION_FOLDER, "latest_path.geojson")
        with open(latest_path, "w") as f:
            json.dump(data, f)
            
        print(f"--- SUCCESS: Saved {filename} ---")
        return jsonify({"status": "success", "file": filename})
    except Exception as e:
        print(f"Server Error: {e}")
        return jsonify({"status": "error"}), 500

def create_map():
    # --- ULTRA ZOOM CONFIGURATION ---
    # max_zoom: How far the UI lets you scroll in
    # max_native_zoom: The highest resolution the tiles actually provide (usually 19 or 20)
    m = folium.Map(
        location=[40.2338, -111.6585], 
        zoom_start=18,
        max_zoom=22, 
        tiles=None # We define tiles below to apply zoom limits
    )

    # Adding Google Satellite with high-zoom support
    folium.TileLayer(
        tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
        attr='Google Satellite',
        name='Google Satellite',
        max_zoom=22,
        max_native_zoom=20 
    ).add_to(m)

    draw = Draw(
        export=False, 
        position='topleft',
        draw_options={
            'polyline': {'shapeOptions': {'color': '#00ffff', 'weight': 5}},
            'polygon': False, 'rectangle': False, 'circle': False, 'marker': False
        }
    )
    draw.add_to(m)

    # Bulletproof Javascript Map Finder
    script_content = """
    <script>
    function waitForMap() {
        var map_el = document.querySelector('.folium-map');
        var map_id = map_el.id;
        var map_obj = window[map_id];

        if (typeof map_obj !== 'undefined' && map_obj !== null) {
            console.log("Map ready for high-precision clicking.");
            map_obj.on('draw:created', function (e) {
                var geojson = e.layer.toGeoJSON();
                fetch('http://127.0.0.1:5000/save', {
                    method: 'POST',
                    mode: 'cors',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(geojson)
                })
                .then(res => res.json())
                .then(data => alert("MISSION SAVED: " + data.file))
                .catch(err => alert("SAVE ERROR: " + err));
            });
        } else {
            setTimeout(waitForMap, 100);
        }
    }
    document.addEventListener('DOMContentLoaded', waitForMap);
    </script>
    """
    m.get_root().html.add_child(Element(script_content))
    
    map_path = os.path.join(os.getcwd(), "point_picker.html")
    m.save(map_path)
    return map_path

if __name__ == "__main__":
    threading.Thread(target=lambda: app.run(port=5000, debug=False, use_reloader=False), daemon=True).start()
    time.sleep(1)
    path = create_map()
    webbrowser.open("file://" + path)
    print("Ultra-Zoom Map Active. Zoom in to level 22 for precision.")
    while True: time.sleep(1)