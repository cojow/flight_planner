"""
Reading and drawing logged flights: the .kmz mission files in a shared
flight-log folder (e.g. a Box Drive-synced "Flight Logs" folder).

Used by both the standalone Flight Log Tracker (py/flight_tracker.py) and the
Flight Planner's "Show Logged Flights" layer (app.py), so the two always read
and draw flights the same way. Nothing here depends on Streamlit.
"""
import functools
import json
import math
import os
import re
import xml.etree.ElementTree as ET
import zipfile
from datetime import datetime
from html import escape

import folium

FT_TO_M = 0.3048
M_TO_FT = 1 / FT_TO_M

# Matches the naming convention the main app saves missions with, e.g.
# "Mission_Fly_H100A90OL75" or "Mission_Fly_H100A90OL75SO65" for mapping
# missions - lets height/pitch/overlap be read straight from the filename
# instead of having to re-derive them from raw waypoint geometry.
SUFFIX_RE = re.compile(r'_(Fly|Pilot)_H(\d+)A(\d+)OL(\d+)(?:SO(\d+))?$')

# At the repository root, not beside this file: the tracker moved into py/,
# and the saved folder preference stayed where it was (and gitignored).
CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".tracker_config.json")

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


def kmz_paths(folder, include_subdirs=False):
    """Every .kmz in a folder (and, if asked, the folders inside it), sorted."""
    if not folder or not os.path.isdir(folder):
        return []
    if include_subdirs:
        return sorted(os.path.join(root, f) for root, _dirs, files in os.walk(folder)
                      for f in files if f.lower().endswith('.kmz'))
    return sorted(os.path.join(folder, f) for f in os.listdir(folder) if f.lower().endswith('.kmz'))


@functools.lru_cache(maxsize=4096)
def _parse_cached(path, mtime):
    # mtime is only part of the cache key: a file is read again only after it changes.
    return parse_flight_kmz(path)


def load_flights(folder, include_subdirs=False):
    """
    (flights that can be drawn, files that couldn't be read). Each file is
    parsed once and re-read only when it changes, so a folder of many flights
    stays quick to show on every rerun.
    """
    flights = []
    for path in kmz_paths(folder, include_subdirs):
        try:
            flights.append(_parse_cached(path, os.path.getmtime(path)))
        except OSError as e:          # removed or unreadable since it was listed
            flights.append({'file': os.path.basename(path), 'name': os.path.basename(path),
                            'coords': [], 'error': str(e)})
    ok = [f for f in flights if f['coords'] and not f.get('error')]
    bad = [f for f in flights if f.get('error') or not f['coords']]
    return ok, bad


def flight_date(f):
    return f['created'] or datetime.fromtimestamp(f['mtime'])


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
        color=color, weight=1, fill=True, fill_opacity=0.4, fill_color=color, interactive=False,
    ).add_to(m)


def build_tooltip_html(f):
    date_str = flight_date(f).strftime('%b %d, %Y')
    alt_str = f"{f['alt_ft']:.0f} ft" if f['alt_ft'] is not None else "n/a"
    pitch_str = f"{f['pitch']:.0f}°" if f['pitch'] is not None else "n/a"
    ol_str = f"{f['overlap_pct']:.0f}%" if f['overlap_pct'] is not None else "n/a"
    if f.get('side_overlap_pct') is not None:
        ol_str += f" (side {f['side_overlap_pct']:.0f}%)"
    return f"""
    <div style="font-family: sans-serif; font-size: 12px; line-height: 1.5;">
      <b>{escape(f['name'])}</b> &middot; {f['platform']}<br>
      Date: {date_str}<br>
      Photos: {f['photo_count']}<br>
      Height: {alt_str}<br>
      Pitch: {pitch_str}<br>
      Overlap: {ol_str}
    </div>
    """


def add_flights_layer(m, flights, cone_count=8, details="hover"):
    """
    Draw flights on a folium map: one coloured line per flight, with view
    cones along it (cone_count of them; 0 for none). details="hover" shows each
    flight's summary when the mouse is over its line (the standalone tracker);
    details="click" shows it only when the line is clicked (the planner, where
    hover pop-ups get in the way).
    """
    for idx, f in enumerate(flights):
        color = PALETTE[idx % len(PALETTE)]
        html = build_tooltip_html(f)
        popup_or_tooltip = ({'tooltip': folium.Tooltip(html, sticky=True)} if details == "hover"
                            else {'popup': folium.Popup(html, max_width=280)})
        folium.PolyLine(f['coords'], color=color, weight=4, opacity=0.85, **popup_or_tooltip).add_to(m)
        if cone_count:
            for lat, lon, heading in periodic_cone_points(f['coords'], target_count=cone_count):
                add_view_cone(m, (lat, lon), heading, color)
