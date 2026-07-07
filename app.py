import streamlit as st
import pandas as pd
import os
import json
import urllib.request
import urllib.parse
import math
import zipfile
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta
import re
from geopy.geocoders import Nominatim
import folium
from folium.plugins import Draw, PolyLineTextPath
from folium.features import DivIcon
from streamlit_folium import st_folium
from branca.element import Element
import shutil
from PIL import Image
import subprocess
import shlex
import time
import matplotlib.pyplot as plt
import textwrap
import ctypes
import ctypes.util
import platform


# --- RASTERIO SAFELOAD ---
try:
    import rasterio
    from rasterio.warp import transform, transform_bounds
    RASTERIO_AVAILABLE = True
except ImportError:
    RASTERIO_AVAILABLE = False

# ==========================================
# MTP BRIDGE (direct libmtp bindings for DJI Fly controller transfers)
# ==========================================
# Inlined directly into app.py (rather than kept as a separate module) so the
# app has no import-path dependency on where a second file happens to live -
# a plain `import mtp_bridge` only works when that file sits next to app.py,
# and moving it elsewhere (e.g. into an archive/diagnostics folder) silently
# breaks the DJI Fly Transfer tab with no obvious error.
#
# Two things this replaces the stock `mtp-*` CLI tools for:
#
# - `mtp-sendfile` derives both the on-device filename and the PTP object-
#   format code from the LOCAL file's name/extension, ignoring the desired
#   remote name entirely. Some controllers' MTP responders also reject the
#   generic/unknown format code that .kmz/.zip/.txt files fall into, while
#   accepting recognized media types. libmtp's API lets us set the
#   destination filename and the object-format code independently, so a
#   file can be labeled as an accepted type (e.g. JPEG) while still landing
#   under the real name we want.
#
# - `mtp-folders` and `mtp-files` always enumerate the ENTIRE device object
#   store (every photo, video, thumbnail, cache and log file - tens of
#   thousands of objects on a well-used controller), even though we only
#   ever care about one small, known subtree
#   (Android/data/dji.go.v5/files/waypoint). libmtp's
#   `LIBMTP_Get_Files_And_Folders` lists only the direct children of one
#   folder at a time, so walking down to that subtree touches a few dozen
#   objects instead of the whole device - the difference between minutes
#   and well under a second.
#
# Requires libmtp's shared library to be installed (the same dependency the
# `mtp-*` CLI tools already require).
try:
    LIBMTP_FILETYPE_FOLDER = 0
    LIBMTP_FILETYPE_JPEG = 14
    LIBMTP_FILES_AND_FOLDERS_ROOT = 0xffffffff

    class _RawDeviceEntry(ctypes.Structure):
        _fields_ = [
            ("vendor", ctypes.c_char_p),
            ("vendor_id", ctypes.c_uint16),
            ("product", ctypes.c_char_p),
            ("product_id", ctypes.c_uint16),
            ("device_flags", ctypes.c_uint32),
        ]

    class _RawDevice(ctypes.Structure):
        _fields_ = [
            ("device_entry", _RawDeviceEntry),
            ("bus_location", ctypes.c_uint32),
            ("devnum", ctypes.c_uint8),
        ]

    class _MTPFile(ctypes.Structure):
        pass

    _MTPFile._fields_ = [
        ("item_id", ctypes.c_uint32),
        ("parent_id", ctypes.c_uint32),
        ("storage_id", ctypes.c_uint32),
        ("filename", ctypes.c_char_p),
        ("filesize", ctypes.c_uint64),
        ("modificationdate", ctypes.c_long),
        ("filetype", ctypes.c_int),
        ("next", ctypes.POINTER(_MTPFile)),
    ]

    class _MTPError(ctypes.Structure):
        pass

    _MTPError._fields_ = [
        ("errornumber", ctypes.c_int),
        ("error_text", ctypes.c_char_p),
        ("next", ctypes.POINTER(_MTPError)),
    ]

    class MTPBridgeError(Exception):
        pass

    def _mtp_load_library(candidates, friendly_name):
        last_err = None
        for cand in candidates:
            if not cand:
                continue
            try:
                return ctypes.CDLL(cand)
            except OSError as e:
                last_err = e
        raise MTPBridgeError(f"Could not load {friendly_name}. Tried: {candidates}. Last error: {last_err}")

    def _mtp_find_libmtp():
        found = ctypes.util.find_library("mtp")
        candidates = [
            found,
            "libmtp.dylib",
            "libmtp.9.dylib",
            "/opt/homebrew/lib/libmtp.dylib",
            "/usr/local/lib/libmtp.dylib",
            "libmtp.so.9",
            "libmtp.so",
            "libmtp-9.dll",
            "libmtp.dll",
        ]
        return _mtp_load_library(candidates, "libmtp")

    def _mtp_find_libc():
        found = ctypes.util.find_library("c")
        candidates = [found, "libc.dylib", "libc.so.6", "msvcrt.dll"]
        return _mtp_load_library(candidates, "the C runtime library")

    _mtp = _mtp_find_libmtp()
    _libc = _mtp_find_libc()

    _mtp.LIBMTP_Init.restype = None

    _mtp.LIBMTP_Detect_Raw_Devices.argtypes = [
        ctypes.POINTER(ctypes.POINTER(_RawDevice)),
        ctypes.POINTER(ctypes.c_int),
    ]
    _mtp.LIBMTP_Detect_Raw_Devices.restype = ctypes.c_int

    _mtp.LIBMTP_Open_Raw_Device_Uncached.argtypes = [ctypes.POINTER(_RawDevice)]
    _mtp.LIBMTP_Open_Raw_Device_Uncached.restype = ctypes.c_void_p

    _mtp.LIBMTP_Release_Device.argtypes = [ctypes.c_void_p]
    _mtp.LIBMTP_Release_Device.restype = None

    _mtp.LIBMTP_Get_Files_And_Folders.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
    _mtp.LIBMTP_Get_Files_And_Folders.restype = ctypes.POINTER(_MTPFile)

    _mtp.LIBMTP_new_file_t.restype = ctypes.POINTER(_MTPFile)

    _mtp.LIBMTP_destroy_file_t.argtypes = [ctypes.POINTER(_MTPFile)]
    _mtp.LIBMTP_destroy_file_t.restype = None

    _mtp.LIBMTP_Send_File_From_File.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.POINTER(_MTPFile),
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    _mtp.LIBMTP_Send_File_From_File.restype = ctypes.c_int

    _mtp.LIBMTP_Get_File_To_File.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint32,
        ctypes.c_char_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    _mtp.LIBMTP_Get_File_To_File.restype = ctypes.c_int

    _mtp.LIBMTP_Delete_Object.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    _mtp.LIBMTP_Delete_Object.restype = ctypes.c_int

    _mtp.LIBMTP_Get_Errorstack.argtypes = [ctypes.c_void_p]
    _mtp.LIBMTP_Get_Errorstack.restype = ctypes.POINTER(_MTPError)

    _mtp.LIBMTP_Clear_Errorstack.argtypes = [ctypes.c_void_p]
    _mtp.LIBMTP_Clear_Errorstack.restype = None

    _libc.strdup.argtypes = [ctypes.c_char_p]
    _libc.strdup.restype = ctypes.c_void_p

    _libc.free.argtypes = [ctypes.c_void_p]
    _libc.free.restype = None

    _mtp.LIBMTP_Init()

    def _mtp_collect_errorstack(device_ptr):
        messages = []
        err_ptr = _mtp.LIBMTP_Get_Errorstack(device_ptr)
        while err_ptr:
            err = err_ptr.contents
            if err.error_text:
                messages.append(err.error_text.decode("utf-8", "replace"))
            err_ptr = err.next
        _mtp.LIBMTP_Clear_Errorstack(device_ptr)
        return messages

    class MTPSession:
        """
        Wraps a single libmtp device connection for the duration of a whole
        operation (one scan, one transfer), instead of spawning a fresh
        `mtp-*` process - and therefore a fresh device session - for every
        single step. This avoids two problems observed with the CLI-based
        approach: the slow full-device enumeration `mtp-folders`/`mtp-files`
        always perform, and the device renumbering object IDs between
        separate process connections.

        Use as a context manager:
            with MTPSession() as mtp:
                waypoint_id = mtp.resolve_path(["Android", "data", "dji.go.v5", "files", "waypoint"])
                children = mtp.list_children(waypoint_id)
        """

        def __init__(self):
            self.device = None

        def __enter__(self):
            raw_device_list = ctypes.POINTER(_RawDevice)()
            num_devices = ctypes.c_int(0)
            ret = _mtp.LIBMTP_Detect_Raw_Devices(ctypes.byref(raw_device_list), ctypes.byref(num_devices))
            if ret != 0 or num_devices.value == 0:
                raise MTPBridgeError("No MTP device detected.")
            try:
                self.device = _mtp.LIBMTP_Open_Raw_Device_Uncached(ctypes.byref(raw_device_list[0]))
            finally:
                _libc.free(ctypes.cast(raw_device_list, ctypes.c_void_p))
            if not self.device:
                raise MTPBridgeError("Failed to open MTP device session.")
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            if self.device:
                _mtp.LIBMTP_Release_Device(self.device)
                self.device = None
            return False

        def list_children(self, parent_id, storage_id=0):
            """
            Returns the DIRECT children of `parent_id` as a list of dicts
            with keys id/name/is_folder/size. Does not recurse and does not
            touch anything outside this one folder - the key difference
            from `mtp-files`/`mtp-folders`, which always walk the entire
            device.
            """
            head = _mtp.LIBMTP_Get_Files_And_Folders(self.device, storage_id, parent_id)
            items = []
            p = head
            while p:
                f = p.contents
                items.append({
                    "id": f.item_id,
                    "name": f.filename.decode("utf-8", "replace") if f.filename else "",
                    "is_folder": f.filetype == LIBMTP_FILETYPE_FOLDER,
                    "size": f.filesize,
                })
                p = f.next
            if head:
                _mtp.LIBMTP_destroy_file_t(head)
            return items

        def find_child(self, parent_id, name):
            """Returns the id of the direct child of `parent_id` named `name`, or None."""
            for item in self.list_children(parent_id):
                if item["name"] == name:
                    return item["id"]
            return None

        def resolve_path(self, names, start_id=LIBMTP_FILES_AND_FOLDERS_ROOT):
            """Walks a chain of folder names, returning the final folder's id, or None if any segment is missing."""
            current = start_id
            for name in names:
                current = self.find_child(current, name)
                if current is None:
                    return None
            return current

        def pull_file(self, file_id, local_path):
            ret = _mtp.LIBMTP_Get_File_To_File(self.device, file_id, local_path.encode("utf-8"), None, None)
            return ret == 0

        def delete_object(self, object_id):
            ret = _mtp.LIBMTP_Delete_Object(self.device, object_id)
            return ret == 0

        def send_disguised_file(self, local_path, remote_filename, parent_folder_id,
                                 disguise_filetype=LIBMTP_FILETYPE_JPEG, storage_id=0):
            """
            Pushes the bytes of `local_path`, landing inside
            `parent_folder_id` and named exactly `remote_filename`. The PTP
            object-format code is set to `disguise_filetype` (default:
            JPEG) rather than being derived from `remote_filename`'s
            extension - this is what lets non-media files (like .kmz)
            through devices that reject the generic/unknown format code.
            Returns (success: bool, message: str).
            """
            if not os.path.exists(local_path):
                return False, f"Local file not found: {local_path}"

            file_struct = _mtp.LIBMTP_new_file_t()
            try:
                file_struct.contents.parent_id = int(parent_folder_id)
                file_struct.contents.storage_id = int(storage_id)
                file_struct.contents.filesize = os.path.getsize(local_path)
                file_struct.contents.filetype = int(disguise_filetype)

                name_ptr = _libc.strdup(remote_filename.encode("utf-8"))
                file_struct.contents.filename = ctypes.cast(name_ptr, ctypes.c_char_p)

                send_ret = _mtp.LIBMTP_Send_File_From_File(
                    self.device, local_path.encode("utf-8"), file_struct, None, None
                )

                if send_ret != 0:
                    errors = _mtp_collect_errorstack(self.device)
                    detail = "; ".join(errors) if errors else "unknown error"
                    return False, f"Send failed: {detail}"

                return True, f"Sent as '{remote_filename}' (item ID {file_struct.contents.item_id})"
            finally:
                _mtp.LIBMTP_destroy_file_t(file_struct)

    MTP_BRIDGE_AVAILABLE = True
except Exception:
    MTP_BRIDGE_AVAILABLE = False

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

# Updated Hardware Map with DJI Fly
HARDWARE_MAP = {
    "DJI Fly (RC2 / Mini / Air Series)": {
        "drone_enum": "67", "drone_sub": "0", 
        "payload_enum": "43", "payload_sub": "0", 
        "is_dji_fly": True
        },

    "DJI Pilot 2 (Mavic 3M) (Drone: 0, Payload: 3)": {
        "drone_enum": "77", "drone_sub": "0", 
        "payload_enum": "68", "payload_sub": "3", 
        "is_dji_fly": False
        }
    }

# ==========================================
# EXTERNAL DATA & ELEVATION HELPERS
# ==========================================

def kill_macos_hijackers():
    """Kills macOS background apps that lock the MTP port."""
    subprocess.run("killall -9 PTPCamera", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    subprocess.run("killall -9 'Image Capture Extension'", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(1)

# The path, as a chain of folder names, from the device root down to where
# DJI Fly keeps its dummy mission slots. Walking down this one specific
# chain with targeted per-folder queries touches a few dozen objects total;
# `mtp-folders`/`mtp-files` touch the device's entire object store (tens of
# thousands of objects on a well-used controller) to find the same thing.
WAYPOINT_PATH = ["Android", "data", "dji.go.v5", "files", "waypoint"]
UUID_RE = re.compile(r'^[A-F0-9]{8}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{12}$', re.IGNORECASE)

def fetch_controller_nests_and_previews(cache_dir="missions/.cache", pull_thumbnails=True):
    """
    Scans the RC 2 for dummy missions and caches our custom hijacked
    thumbnails, using targeted per-folder libmtp queries instead of a
    full-device file enumeration.
    """
    kill_macos_hijackers()
    os.makedirs(cache_dir, exist_ok=True)

    if not MTP_BRIDGE_AVAILABLE:
        return {}, None

    try:
        with MTPSession() as session:
            waypoint_id = session.resolve_path(WAYPOINT_PATH)
            if waypoint_id is None:
                return {}, None

            waypoint_children = session.list_children(waypoint_id)
            nests = {}
            preview_id = None
            for item in waypoint_children:
                if not item["is_folder"]:
                    continue
                if item["name"] == "map_preview":
                    preview_id = item["id"]
                elif UUID_RE.match(item["name"]):
                    nests[item["name"].upper()] = item["id"]

            if not pull_thumbnails or not preview_id:
                return nests, preview_id

            preview_children = session.list_children(preview_id)

            for uuid, folder_id in nests.items():
                cache_path = os.path.join(cache_dir, f"{uuid}.jpg")
                file_id_to_pull = None

                # Check A: the subfolder under map_preview named after this mission's
                # UUID - this is the location DJI Fly's native UI actually reads
                # mission preview thumbnails from.
                preview_subfolder = next((it for it in preview_children if it["is_folder"] and it["name"] == uuid), None)
                if preview_subfolder:
                    sub_match = next((it for it in session.list_children(preview_subfolder["id"]) if it["name"] == f"{uuid}.jpg"), None)
                    if sub_match:
                        file_id_to_pull = sub_match["id"]

                # Check B: did we inject it directly into the UUID's own mission folder?
                if file_id_to_pull is None:
                    own_match = next((it for it in session.list_children(folder_id) if it["name"] == f"{uuid}.jpg"), None)
                    if own_match:
                        file_id_to_pull = own_match["id"]

                # Check C: did we inject it flat into the map_preview folder?
                if file_id_to_pull is None:
                    flat_match = next((it for it in preview_children if it["name"] == f"{uuid}.jpg"), None)
                    if flat_match:
                        file_id_to_pull = flat_match["id"]

                # If we found our custom hijacked image, pull it to the Mac
                if file_id_to_pull is not None:
                    session.pull_file(file_id_to_pull, cache_path)

            return nests, preview_id
    except MTPBridgeError:
        return {}, None
    except Exception:
        return {}, None

def push_mission_to_nest(local_kmz_path, target_uuid):
    """
    Pushes a local KMZ (and its paired JPG thumbnail, if present) to an
    existing dummy mission slot on the RC 2, replacing whatever's there.
    Runs the whole operation - locating folders, purging old files, pushing
    new ones - over a single continuous device connection rather than many
    separate `mtp-*` process invocations, which avoids both the slow
    full-device enumeration those tools do and the device's tendency to
    renumber object IDs between separate connections.
    """
    kill_macos_hijackers()

    if not MTP_BRIDGE_AVAILABLE:
        return False, "MTP bridge unavailable (is libmtp installed?)."

    try:
        with MTPSession() as session:
            waypoint_id = session.resolve_path(WAYPOINT_PATH)
            if waypoint_id is None:
                return False, "Could not locate the 'waypoint' folder on the controller."

            waypoint_children = session.list_children(waypoint_id)
            target_folder_id = next((it["id"] for it in waypoint_children if it["is_folder"] and it["name"] == target_uuid), None)
            map_preview_id = next((it["id"] for it in waypoint_children if it["is_folder"] and it["name"] == "map_preview"), None)

            if not target_folder_id:
                return False, "Target folder missing from controller. Please scan again."

            # The subfolder under map_preview named after this mission's UUID - this
            # is the location DJI Fly's native UI actually reads preview thumbnails from.
            preview_subfolder_id = session.find_child(map_preview_id, target_uuid) if map_preview_id else None

            # 1. PURGE OLD FILES
            deleted_something = False
            for item in session.list_children(target_folder_id):
                if session.delete_object(item["id"]):
                    deleted_something = True

            if map_preview_id:
                for item in session.list_children(map_preview_id):
                    if not item["is_folder"] and target_uuid in item["name"]:
                        if session.delete_object(item["id"]):
                            deleted_something = True

            if preview_subfolder_id:
                for item in session.list_children(preview_subfolder_id):
                    if session.delete_object(item["id"]):
                        deleted_something = True

            # 2. THE FUSE COOLDOWN (CRITICAL FIX)
            # If we deleted files, Android needs time to update its SQLite MediaStore DB.
            # If we push instantly, Android drops the incoming file into the void.
            if deleted_something:
                time.sleep(3.5)

            # 3. PUSH NEW FILES
            # NOTE: the `mtp-sendfile` CLI is not used here. It derives both the
            # on-device filename AND the PTP object-format code from the LOCAL
            # file's name/extension, ignoring the desired remote name entirely,
            # and this controller's MTP responder rejects the generic/unknown
            # format code that .kmz files fall into. MTPSession talks to
            # libmtp directly so the destination filename and format code
            # can be set independently (the KMZ is sent disguised as a
            # JPEG to get past the rejection; the JPG thumbnail is already
            # a real JPEG so no disguise is needed).
            subprocess.run(f'xattr -c {shlex.quote(local_kmz_path)}', shell=True)
            remote_kmz = f"{target_uuid}.kmz"
            ok_kmz, msg_kmz = session.send_disguised_file(local_kmz_path, remote_kmz, target_folder_id)
            if not ok_kmz:
                return False, f"KMZ transfer failed: {msg_kmz}"

            local_jpg_path = local_kmz_path.replace('.kmz', '.jpg')
            if os.path.exists(local_jpg_path):
                subprocess.run(f'xattr -c {shlex.quote(local_jpg_path)}', shell=True)
                remote_jpg = f"{target_uuid}.jpg"

                session.send_disguised_file(local_jpg_path, remote_jpg, target_folder_id)
                if map_preview_id:
                    session.send_disguised_file(local_jpg_path, remote_jpg, map_preview_id)
                if preview_subfolder_id:
                    session.send_disguised_file(local_jpg_path, remote_jpg, preview_subfolder_id)

            return True, "Success. (Reminder: Ensure DJI Fly is closed on the RC 2 before opening!)"
    except MTPBridgeError as e:
        return False, str(e)

def export_mission_kmz_from_strings(template_kml_str, waylines_wpml_str, output_kmz_path, is_dji_fly):
    """
    Exports the KMZ from memory strings differently depending on the hardware map.
    - is_dji_fly = True: Enforces the strict 'wpmz/' parent directory architecture.
    - is_dji_fly = False: Uses standard root-level zipping for DJI Pilot.
    """

    with zipfile.ZipFile(output_kmz_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        if is_dji_fly:
            # Forces the files inside a 'wpmz/' folder inside the zip
            kmz.writestr('wpmz/template.kml', template_kml_str)
            if waylines_wpml_str:
                kmz.writestr('wpmz/waylines.wpml', waylines_wpml_str)
                
        else:
            # Zips the files directly into the root of the archive (Pilot behavior)
            kmz.writestr('template.kml', template_kml_str)
            if waylines_wpml_str:
                kmz.writestr('waylines.wpml', waylines_wpml_str)

def is_dji_fly_kmz(kmz_path):
    """
    Determines whether a .kmz was built for DJI Fly (files nested under a
    wpmz/ folder) vs DJI Pilot (files at the zip root) - mirrors exactly how
    export_mission_kmz_from_strings lays out each platform's package, so it
    works regardless of filename (including files predating the Pilot/Fly
    filename prefix).
    """
    try:
        with zipfile.ZipFile(kmz_path, 'r') as kmz:
            return any(name.startswith('wpmz/') for name in kmz.namelist())
    except Exception:
        return False

@st.cache_data(show_spinner=False, ttl=3600)
def get_coords_from_search(query):
    """Parses a lat,lon string or geocodes an address to return [lat, lon]."""
    if not query:
        return None
    match = re.match(r"^\s*(-?\d+(\.\d+)?)\s*,\s*(-?\d+(\.\d+)?)\s*$", query)
    if match:
        return [float(match.group(1)), float(match.group(3))]
    try:
        geolocator = Nominatim(user_agent="dji_flight_planner_app")
        location = geolocator.geocode(query)
        if location:
            return [location.latitude, location.longitude]
    except Exception:
        pass
    return None

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
                    elevations.append(0.0)
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
            return [[wgs_bounds[1], wgs_bounds[0]], [wgs_bounds[3], wgs_bounds[2]]]
    except Exception:
        return None

# Name-text color bands by relative altitude, in 40 ft increments (0-39,
# 40-79, ... 360-399), so missions sharing the same flight path but flown at
# different heights/angles are visually distinguishable at a glance. Anything
# 400 ft and up falls through to ALTITUDE_COLOR_400_PLUS.
ALTITUDE_COLOR_BANDS = [
    (40, '#00FFFF'),   # 0-39 ft: cyan
    (80, '#00FF9C'),   # 40-79 ft: spring green
    (120, '#7FFF00'),  # 80-119 ft: chartreuse
    (160, '#FFFF00'),  # 120-159 ft: yellow
    (200, '#FFC400'),  # 160-199 ft: amber
    (240, '#FF9100'),  # 200-239 ft: orange
    (280, '#FF3D00'),  # 240-279 ft: red-orange
    (320, '#FF1493'),  # 280-319 ft: deep pink
    (360, '#FF00FF'),  # 320-359 ft: magenta
    (400, '#BF00FF'),  # 360-399 ft: purple
]
ALTITUDE_COLOR_400_PLUS = '#7B2FFF'  # 400+ ft: violet

def get_altitude_color(alt_ft):
    for threshold, color in ALTITUDE_COLOR_BANDS:
        if alt_ft < threshold:
            return color
    return ALTITUDE_COLOR_400_PLUS

def generate_name_thumbnail(mission_name, alt_ft, pitch, overlap_pct, output_filepath, coords=None):
    """
    Generates a 16:9 dark-mode thumbnail: mission name and key flight
    parameters (height, angle, overlap) on the left, so the plan is
    identifiable at a glance without needing the encoded filename suffix,
    and a small true-shape line drawing of the flight path in a thin
    bordered box on the right, captioned "flight path". The name's color
    is banded by altitude (see ALTITUDE_COLOR_BANDS) so missions sharing a
    path but flown at different heights are easy to tell apart.
    """
    fig_w, fig_h = 8, 4.5
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), facecolor='#121212')
    ax.set_facecolor('#121212')

    # --- LEFT: name + flight parameters ---
    # Wrap the name so it doesn't run off the edges. Underscores are treated
    # as spaces here so textwrap can break at word boundaries instead of
    # hard-splitting mid-word (e.g. "Fly_Mission_Flight").
    display_name = mission_name.replace('_', ' ')
    wrapped_name = "\n".join(textwrap.wrap(display_name, width=12)) or display_name

    ax.text(0.27, 0.68, wrapped_name,
            color=get_altitude_color(alt_ft),
            fontsize=42,
            ha='center',
            va='center',
            weight='bold',
            transform=ax.transAxes)

    info_text = f"H: {alt_ft:.0f} ft\nA: {abs(pitch):.0f}°\nOL: {overlap_pct:.0f}%"
    ax.text(0.27, 0.24, info_text,
            color='#FFFFFF',
            fontsize=26,
            ha='center',
            va='center',
            linespacing=1.6,
            transform=ax.transAxes)

    # --- RIGHT: flight path drawing in a thin bordered box ---
    if coords and len(coords) >= 2:
        box_size_in = 2.1
        box_cx_in, box_cy_in = 6.3, 2.35
        box_x0_frac = (box_cx_in - box_size_in / 2) / fig_w
        box_y0_frac = (box_cy_in - box_size_in / 2) / fig_h
        box_w_frac = box_size_in / fig_w
        box_h_frac = box_size_in / fig_h

        ax.add_patch(plt.Rectangle(
            (box_x0_frac, box_y0_frac), box_w_frac, box_h_frac,
            transform=ax.transAxes, facecolor='none', edgecolor='#FFFFFF', linewidth=1.2
        ))

        # Project lat/lon to local planar inches (equirectangular approx, fine
        # for small local flight areas) so the drawn shape isn't distorted,
        # then scale/center it to fit inside the box while preserving aspect.
        lats = [c[0] for c in coords]
        lons = [c[1] for c in coords]
        lat0 = sum(lats) / len(lats)
        xs_m = [(lon - lons[0]) * math.cos(math.radians(lat0)) for lon in lons]
        ys_m = [(lat - lats[0]) for lat in lats]
        width_m = max(xs_m) - min(xs_m) or 1e-9
        height_m = max(ys_m) - min(ys_m) or 1e-9
        mid_x = (max(xs_m) + min(xs_m)) / 2
        mid_y = (max(ys_m) + min(ys_m)) / 2
        scale = 0.72 * box_size_in / max(width_m, height_m)

        xs_frac = [(box_cx_in + (x - mid_x) * scale) / fig_w for x in xs_m]
        ys_frac = [(box_cy_in + (y - mid_y) * scale) / fig_h for y in ys_m]

        ax.plot(xs_frac, ys_frac, color='#FFFFFF', linewidth=1.6, transform=ax.transAxes)

        ax.text(box_x0_frac + box_w_frac / 2, box_y0_frac - 0.07, "flight path",
                color='#AAAAAA',
                fontsize=13,
                ha='center',
                va='center',
                transform=ax.transAxes)

    # Strip away all axes, borders, and margins
    ax.axis('off')
    plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
    plt.margins(0, 0)

    # Save as a standard low-footprint JPG
    plt.savefig(output_filepath, format='jpg', dpi=120, bbox_inches='tight', pad_inches=0)
    plt.close()

def get_elevations_batch(coords, source, tif_path=None):
    if not coords:
        return []
    if source == "USGS 3DEP (US High-Res)":
        return get_elevations_usgs(coords)
    elif source == "Local GeoTIFF" and tif_path and os.path.exists(tif_path):
        return get_elevations_raster(coords, tif_path)
    else:
        return get_elevations_open_elev(coords)

def pick_folder_dialog(prompt_title):
    """
    Opens the OS-native folder picker (AppleScript on macOS, tkinter on
    Windows) and returns the chosen absolute path, or None if cancelled,
    unsupported, or the picker failed.
    """
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
            root.attributes('-topmost', True)  # Forces window to the front on Windows
            folder = filedialog.askdirectory(master=root, title=prompt_title)
            root.destroy()
            if folder:
                folder_path = folder
        except Exception as e:
            st.error(f"Failed to open Windows folder picker: {e}")

    return folder_path

def get_exif_datetime(filepath):
    """Extracts the exact time the photo was taken from EXIF data."""
    try:
        with Image.open(filepath) as img:
            exif = img._getexif()
            if not exif:
                return None
            for tag, value in exif.items():
                if tag == 36867: 
                    return datetime.strptime(value, "%Y:%m:%d %H:%M:%S")
    except Exception:
        pass
    return None

def st_group_images_by_time(source_folder, output_folder, target_date, gap_minutes=5):
    """Streamlit-adapted function to filter and group images by time gaps."""
    valid_extensions = {'.jpg', '.jpeg', '.png', '.tif', '.tiff'}
    os.makedirs(output_folder, exist_ok=True)
    
    image_data = []
    try:
        files = os.listdir(source_folder)
    except Exception as e:
        st.error(f"Error accessing source directory: {e}")
        return

    progress_bar = st.progress(0, text="Scanning files for EXIF data...")
    
    for i, filename in enumerate(files):
        ext = os.path.splitext(filename)[1].lower()
        if ext in valid_extensions:
            filepath = os.path.join(source_folder, filename)
            taken_time = get_exif_datetime(filepath)
            
            if taken_time and taken_time.date() == target_date:
                image_data.append({'path': filepath, 'name': filename, 'time': taken_time})
        
        progress_bar.progress((i + 1) / len(files), text=f"Scanning files... ({i+1}/{len(files)})")
                
    if not image_data:
        progress_bar.empty()
        st.warning(f"No images found for {target_date.strftime('%Y-%m-%d')} in the source folder.")
        return

    image_data.sort(key=lambda x: x['time'])

    groups = []
    current_group = [image_data[0]]
    gap_threshold = timedelta(minutes=gap_minutes)

    for i in range(1, len(image_data)):
        time_diff = image_data[i]['time'] - image_data[i-1]['time']
        if time_diff <= gap_threshold:
            current_group.append(image_data[i])
        else:
            groups.append(current_group)
            current_group = [image_data[i]]
            
    if current_group:
        groups.append(current_group)

    progress_bar.empty()
    st.info(f"Found {len(groups)} distinct flight groups.")
    
    copy_progress = st.progress(0, text="Copying images to group folders...")
    total_images = sum(len(g) for g in groups)
    copied = 0
    
    for i, group in enumerate(groups):
        group_start_datetime = group[0]['time'].strftime("%Y-%m-%d_%H-%M-%S")
        folder_name = f"Group_{i+1}_{group_start_datetime}"
        folder_path = os.path.join(output_folder, folder_name)
        
        os.makedirs(folder_path, exist_ok=True)
        
        for img in group:
            target_path = os.path.join(folder_path, img['name'])
            shutil.copy2(img['path'], target_path) 
            copied += 1
            copy_progress.progress(copied / total_images, text=f"Copying images... ({copied}/{total_images})")
            
    copy_progress.empty()
    st.success(f"Successfully sorted {total_images} images into {len(groups)} folders at '{output_folder}'.")

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

def interpolate_path(coords, gap_m, return_frac=False):
    """
    Breaks down a corner-based path into physical waypoints for DJI Fly
    compatibility. When return_frac=True, also returns a parallel list of
    (segment_index, frac) tuples describing each output point's position
    along the original path - used to interpolate other per-waypoint values
    (like elevation) consistently with the same points, without having to
    re-query them for every densified waypoint.
    """
    if gap_m <= 0 or len(coords) < 2:
        if return_frac:
            return coords, [(i, 0.0) for i in range(len(coords))]
        return coords

    dense_coords = [coords[0]]
    fracs = [(0, 0.0)]
    cum_dist = [0.0]
    total_dist_m = 0.0

    for i in range(len(coords)-1):
        d = get_haversine_dist(coords[i], coords[i+1])
        total_dist_m += d
        cum_dist.append(total_dist_m)

    target_dist = gap_m
    while target_dist <= total_dist_m + 0.001:
        for i in range(len(cum_dist) - 1):
            if cum_dist[i] <= target_dist <= cum_dist[i+1] + 0.001:
                seg_len = cum_dist[i+1] - cum_dist[i]
                if seg_len > 0:
                    frac = (target_dist - cum_dist[i]) / seg_len
                    lat = coords[i][0] + (coords[i+1][0] - coords[i][0]) * frac
                    lon = coords[i][1] + (coords[i+1][1] - coords[i][1]) * frac
                    dense_coords.append((lat, lon))
                    fracs.append((i, frac))
                break
        target_dist += gap_m

    if get_haversine_dist(dense_coords[-1], coords[-1]) > gap_m * 0.1:
        dense_coords.append(coords[-1])
        fracs.append((len(coords) - 2, 1.0))

    if return_frac:
        return dense_coords, fracs
    return dense_coords

def interpolate_elevations(elevations, fracs):
    """
    Linearly interpolates elevation values at each (segment_index, frac)
    position produced by interpolate_path(return_frac=True), so densified
    waypoints get a sensible elevation without re-querying the elevation
    source for every one of them.
    """
    return [
        elevations[i] + (elevations[min(i + 1, len(elevations) - 1)] - elevations[i]) * frac
        for i, frac in fracs
    ]

# ==========================================
# SESSION STATE INITIALIZATION & SAFE CALLBACKS
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

# Bulletproof getter intercepting any NoneType errors caused by Streamlit clearing fields
def safe_get_float(key, default_val):
    if key not in st.session_state:
        return default_val
    val = st.session_state[key]
    if val is None:
        return default_val
    try:
        return float(val)
    except (TypeError, ValueError):
        return default_val

def get_center_footprint(pitch, alt):
    if pitch == 0: return 999999.0  
    slant_dist = alt / math.sin(math.radians(abs(pitch)))
    return slant_dist * (SENSOR_W / FOCAL_L)

def sync_dist_to_overlap():
    fw = get_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    if fw > 0: 
        st.session_state.overlap_pct = max(0.0, min(((fw - safe_get_float('t_dist_val', 9.0)) / fw) * 100, 99.9))

def sync_overlap_to_dist():
    fw = get_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    st.session_state.t_dist_val = fw * (1 - (safe_get_float('overlap_pct', 70.0) / 100))

def sync_gap_to_overlap():
    fw = get_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    if fw > 0: 
        st.session_state.overlap_pct = max(0.0, min(((fw - safe_get_float('target_gap_ft', 26.2)) / fw) * 100, 99.9))

def sync_overlap_to_gap():
    fw = get_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    st.session_state.target_gap_ft = fw * (1 - (safe_get_float('overlap_pct', 70.0) / 100))

def sync_geometry():
    if st.session_state.get('trigger_type', 'distance') == 'distance': 
        sync_dist_to_overlap()
    else: 
        sync_gap_to_overlap()

def e_sync_dist_to_overlap():
    fw = get_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    if fw > 0: 
        st.session_state.e_overlap_pct = max(0.0, min(((fw - safe_get_float('e_t_dist_val', 9.0)) / fw) * 100, 99.9))

def e_sync_overlap_to_dist():
    fw = get_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    st.session_state.e_t_dist_val = fw * (1 - (safe_get_float('e_overlap_pct', 70.0) / 100))

def e_sync_gap_to_overlap():
    fw = get_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    if fw > 0: 
        st.session_state.e_overlap_pct = max(0.0, min(((fw - safe_get_float('e_target_gap_ft', 26.2)) / fw) * 100, 99.9))

def e_sync_overlap_to_gap():
    fw = get_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    st.session_state.e_target_gap_ft = fw * (1 - (safe_get_float('e_overlap_pct', 70.0) / 100))

def e_sync_geometry():
    if st.session_state.get('e_trigger_type', 'distance') == 'distance': 
        e_sync_dist_to_overlap()
    else: 
        e_sync_gap_to_overlap()

# ==========================================
# DATA EXTRACTION (FOR EDITOR)
# ==========================================
def parse_kmz_for_editing(full_path):
    meta = {
        "safe_takeoff_ft": 60.0, "trans_speed_mph": 22.0, "speed_m": 4.11, "speed_mph": 6.0,
        "alt_ft": 50.0, "pitch": -60.0, "trigger_type": "distance", 
        "t_val": 9.0, "photo_start_wp": 0, "coords": [], "camera_type": "visible",
        "drone_sub": "0", "payload_sub": "3"
    }
    with zipfile.ZipFile(full_path, 'r') as kmz:
        waylines_file = [name for name in kmz.namelist() if name.endswith('waylines.wpml')][0]
        template_file = [name for name in kmz.namelist() if name.endswith('template.kml')][0]
        root_w = ET.fromstring(kmz.read(waylines_file))
        root_t = ET.fromstring(kmz.read(template_file))
        
        # Check namespaces generically in case of DJI Fly (1.0.2)
        safe_node = root_w.find('.//{*}takeOffSecurityHeight')
        if safe_node is not None: meta['safe_takeoff_ft'] = float(safe_node.text) * M_TO_FT
        trans_node = root_w.find('.//{*}globalTransitionalSpeed')
        if trans_node is not None: meta['trans_speed_mph'] = float(trans_node.text) * MS_TO_MPH
        speed_node = root_w.find('.//{*}autoFlightSpeed')
        if speed_node is not None: 
            meta['speed_m'] = float(speed_node.text)
            meta['speed_mph'] = float(speed_node.text) * MS_TO_MPH

        drone_info = root_t.find('.//{*}droneInfo')
        if drone_info is not None:
            d_sub = drone_info.find('.//{*}droneSubEnumValue')
            if d_sub is not None and d_sub.text: meta['drone_sub'] = d_sub.text
            
        payload_info = root_t.find('.//{*}payloadInfo')
        if payload_info is not None:
            p_sub = payload_info.find('.//{*}payloadSubEnumValue')
            if p_sub is not None and p_sub.text: meta['payload_sub'] = p_sub.text

        hw_key = "Mavic 3 Multispectral (Drone: 0, Payload: 3)"
        for k, v in HARDWARE_MAP.items():
            if v["drone_sub"] == meta['drone_sub'] and v["payload_sub"] == meta['payload_sub']:
                hw_key = k
        meta['hardware_key'] = hw_key

        pms = root_w.findall('.//{*}Placemark')
        for i, pm in enumerate(pms):
            c_node = pm.find('.//{*}coordinates')
            if c_node is not None:
                c_raw = c_node.text.strip().split(',')
                meta['coords'].append((float(c_raw[1]), float(c_raw[0]))) 
            
            if i == 0:
                alt_node = pm.find('.//{*}executeHeight')
                if alt_node is not None: meta['alt_ft'] = float(alt_node.text) * M_TO_FT
                pitch_node = pm.find('.//{*}waypointGimbalHeadingParam/{*}waypointGimbalPitchAngle')
                if pitch_node is not None: meta['pitch'] = float(pitch_node.text)

            for ag in pm.findall('.//{*}actionGroup'):
                t_type = ag.find('.//{*}actionTriggerType')
                if t_type is not None and 'multiple' in t_type.text:
                    meta['trigger_type'] = "distance" if "Distance" in t_type.text else "time"
                    t_param = ag.find('.//{*}actionTriggerParam')
                    if t_param is not None:
                        val = float(t_param.text)
                        meta['t_val'] = val * M_TO_FT if meta['trigger_type'] == 'distance' else val
                    start_idx = ag.find('.//{*}actionGroupStartIndex')
                    if start_idx is not None: meta['photo_start_wp'] = int(start_idx.text)
                
                for a in ag.findall('.//{*}action'):
                    func = a.find('.//{*}actionActuatorFunc')
                    if func is not None and func.text == 'takePhoto':
                        params = a.find('.//{*}actionActuatorFuncParam')
                        if params is not None:
                            lens = params.find('.//{*}payloadLensIndex')
                            if lens is not None: meta['camera_type'] = lens.text
    return meta

# ==========================================
# CORE MISSION GENERATOR
# ==========================================
def generate_native_kmz_contents(coords, cfg, elev_source, tif_path):
    is_dji_fly = cfg.get("is_dji_fly", False)
    wpml_ns = "1.0.2" if is_dji_fly else "1.0.6"
    
    if is_dji_fly:
        gap_m = max(1.0, cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['speed_m'] * cfg['interval_sec']
        # Query elevations only for the original corner waypoints (same as
        # DJI Pilot), then interpolate for the densified in-between points.
        # Re-querying every dense waypoint's elevation was the choke point
        # here, especially with USGS 3DEP, which looks up one coordinate
        # at a time.
        corner_elevations = get_elevations_batch(coords, elev_source, tif_path)
        coords, fracs = interpolate_path(coords, gap_m, return_frac=True)
        elevations = interpolate_elevations(corner_elevations, fracs)
    else:
        elevations = get_elevations_batch(coords, elev_source, tif_path)

    ms_ts = int(datetime.now().timestamp() * 1000)

    start_elev = elevations[0] if elevations else 0
    target_agl_m = cfg["alt_ft"] * FT_TO_M
    
    safe_m = cfg["safe_takeoff_ft"] * FT_TO_M
    trans_m = cfg["trans_speed_mph"] * MPH_TO_MS
    speed_m = cfg["speed_m"]
    lens_str = cfg.get("camera_type", "visible")
    drone_enum = cfg.get("drone_enum", "77")
    drone_sub_enum = cfg.get("drone_sub", "0")
    payload_enum = cfg.get("payload_enum", "68")
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
    
    template_extra = ""
    waylines_extra = ""
    if not is_dji_fly:
        template_extra = f"""
        <wpml:gimbalPitchAngle>{pitch_val}</wpml:gimbalPitchAngle>
        <wpml:isRisky>0</wpml:isRisky>"""
        waylines_extra = f"""
        <wpml:waypointGimbalHeadingParam>
          <wpml:waypointGimbalPitchAngle>{pitch_val}</wpml:waypointGimbalPitchAngle>
          <wpml:waypointGimbalYawAngle>0</wpml:waypointGimbalYawAngle>
        </wpml:waypointGimbalHeadingParam>
        <wpml:isRisky>0</wpml:isRisky>
        <wpml:waypointWorkType>0</wpml:waypointWorkType>"""

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

        if 0 < i < len(coords) - 1 and not is_dji_fly:
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

        start_wp = 0 if is_dji_fly else cfg.get("photo_start_wp", 0)
        
        if is_dji_fly:
            if i >= start_wp:
                photo_action_block = f"""
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
                template_action_group += f"\n        <wpml:actionGroup>\n          <wpml:actionGroupId>{g_id_template}</wpml:actionGroupId>{photo_action_block}"
                waylines_action_group += f"\n        <wpml:actionGroup>\n          <wpml:actionGroupId>{g_id_waylines}</wpml:actionGroupId>{photo_action_block}"
                g_id_template += 1
                g_id_waylines += 1
        else:
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

        if i < len(coords) - 1 and not is_dji_fly:
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
        </wpml:waypointHeadingParam>{template_extra}{template_action_group}
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
        <wpml:useStraightLine>1</wpml:useStraightLine>{waylines_extra}{waylines_action_group}
      </Placemark>"""

    template_kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/{wpml_ns}">
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
      <wpml:droneInfo><wpml:droneEnumValue>{drone_enum}</wpml:droneEnumValue><wpml:droneSubEnumValue>{drone_sub_enum}</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:payloadInfo><wpml:payloadEnumValue>{payload_enum}</wpml:payloadEnumValue><wpml:payloadSubEnumValue>{payload_sub_enum}</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
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
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/{wpml_ns}">
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
      <wpml:droneInfo><wpml:droneEnumValue>{drone_enum}</wpml:droneEnumValue><wpml:droneSubEnumValue>{drone_sub_enum}</wpml:droneSubEnumValue></wpml:droneInfo>
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
page = st.radio("Navigation", ["Creator", "Editor", "Viewer  |", "Photo Sorter", "DJI Fly Transfer"], horizontal=True, label_visibility="collapsed")

# --- CREATOR MODE ---
if page == 'Creator':
    with st.sidebar:
        st.header("1. Hardware & Payload")
        hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()))
        drone_enum = HARDWARE_MAP[hw_choice]["drone_enum"]
        drone_sub_enum = HARDWARE_MAP[hw_choice]["drone_sub"]
        payload_enum = HARDWARE_MAP[hw_choice]["payload_enum"]
        payload_sub_enum = HARDWARE_MAP[hw_choice]["payload_sub"]
        is_dji_fly = HARDWARE_MAP[hw_choice].get("is_dji_fly", False)
        
        if is_dji_fly:
            st.warning("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash saving will be disabled if you exceed this.")
        
        cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"])
        camera_type = CAM_VAL_MAP[cam_choice]
        min_photo_interval_sec = 2.0 if "narrow_band" in camera_type else 0.7

        st.header("2. Global Config")
        mission_name = st.text_input("Filename", "Mission_Flight")
        trans_speed_mph = st.number_input("Takeoff Speed (mph)", value=22.0, step=1.0)
        safe_takeoff_ft = st.number_input("Safe Takeoff Alt (ft)", value=60.0, step=1.0)
        
        st.header("3. Waypoint Settings")
        st.number_input("Relative Altitude (ft)", value=60.0, key="alt_ft", step=1.0, on_change=sync_geometry)
        st.info("❗Elevation is relative to the take off point, NOT the mission start point.")

        c_elev_source = st.selectbox("Elevation Source", ["USGS 3DEP (US High-Res)", "Open-Elevation (Global)", "Local GeoTIFF"], key="c_source")
        if c_elev_source == "Open-Elevation (Global)":
            st.warning("Can be off by several dozen feet. Use with caution.")
        elif c_elev_source == "USGS 3DEP (US High-Res)":
            st.warning("USGS parses coordinates individually. Generating long missions may take a few seconds.")
            
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

        st.slider("Gimbal Pitch (°)", -90, 0, value= -60, key="pitch", on_change=sync_geometry)
        
        current_pitch = safe_get_float('pitch', -60.0)
        pitch_rad = math.radians(abs(current_pitch))
        current_alt = safe_get_float('alt_ft', 50.0)
        D_ft_c = current_alt / math.sin(pitch_rad) if pitch_rad > 0 else float('inf')
        gsd_cm = (D_ft_c * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_c != float('inf') else 0
        st.info(f"Est. Ground GSD: {gsd_cm:.2f} cm/px")
        
        side = st.selectbox("Side of flight path", ["right", "left"])
        
        st.header("4. Trigger & Speed")
        photo_start_wp = st.number_input("Start Photos at Waypoint Index", min_value=0, value=0, step=1)
        st.radio("Type", ["distance", "time"], key="trigger_type", on_change=sync_geometry)
        
        if st.session_state.get('trigger_type', 'distance') == "distance":
            st.number_input("Interval (ft)", key="t_dist_val", min_value=1.0, step=1.0, on_change=sync_dist_to_overlap)
            st.number_input("Forward Overlap (%)", key="overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=sync_overlap_to_dist)
            manual_mph = st.number_input("Flight Speed (mph)", min_value=2.3, step=1.0, value=4.0)
            speed_m = manual_mph * MPH_TO_MS
            
            gap_m = max(1.0, safe_get_float('t_dist_val', 9.0) * FT_TO_M)
            max_speed_m = gap_m / min_photo_interval_sec
            if speed_m > max_speed_m:
                st.error(f"Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
        else:
            t_val_sec = st.number_input("Interval (sec)", min_value=min_photo_interval_sec, value=max(2.0, min_photo_interval_sec))
            auto_speed = st.checkbox("Auto-Calc Speed", True)
            if auto_speed:
                st.number_input("Target Gap (ft)", key="target_gap_ft", min_value=1.0, on_change=sync_gap_to_overlap)
                st.number_input("Forward Overlap (%)", key="overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=sync_overlap_to_gap)
                speed_m = min(max((safe_get_float('target_gap_ft', 26.2) * FT_TO_M) / t_val_sec, 1.0), 10.0)
                st.info(f"Auto-Calculated Speed: {speed_m * MS_TO_MPH:.1f} mph")
            else:
                manual_mph = st.number_input("Manual Speed (mph)", min_value=2.3, value=6.0, step=1.0)
                speed_m = manual_mph * MPH_TO_MS
                current_gap = speed_m * M_TO_FT * t_val_sec
                fw = get_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
                current_overlap = ((fw - current_gap) / fw) * 100 if fw > 0 else 0
                st.info(f"Current Overlap: {max(0, min(current_overlap, 99.9)):.1f}%")

        st.header("5. Visuals")
        show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="creator_faa_toggle")
        if show_faa_airspace:
            st.write("#### Update restrictions of map center")
            if st.button("Update Map Center", key="btn_update_creator"):
                st.session_state.locked_creator_center = st.session_state.creator_center
                st.rerun()

    # --- SAVE DESTINATION (always visible; does not reset when the drawn line changes) ---
    if "c_browsed_dir" not in st.session_state:
        st.session_state.c_browsed_dir = None

    def _clear_c_browsed_dir():
        st.session_state.c_browsed_dir = None

    @st.dialog("Create New Folder")
    def _c_new_folder_dialog():
        folder_name_input = st.text_input("Folder Name", key="c_popup_new_folder_name")
        if st.button("Create", key="c_popup_create_folder_btn"):
            if folder_name_input.strip():
                os.makedirs(os.path.join(MISSION_DIR, folder_name_input.strip()), exist_ok=True)
                st.success(f"Created '{folder_name_input.strip()}'")
                st.rerun()
            else:
                st.warning("Enter a folder name.")

    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
    save_col1, save_col2, save_col3, save_col4 = st.columns([3, 3, 0.6, 0.6])
    with save_col1:
        save_option = st.selectbox(
            "Save Destination", ["Root (missions/)"] + existing_dirs,
            key="c_save_option", on_change=_clear_c_browsed_dir
        )
    with save_col2:
        new_dir_name = st.text_input("New Folder Name", "New_Project", key="c_new_dir_name") if save_option == "Create New Folder..." else ""
    with save_col3:
        st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
        if st.button("📂", key="c_btn_browse_dir", help="Browse for a save directory", use_container_width=True):
            picked = pick_folder_dialog("Select Save Directory")
            if picked:
                st.session_state.c_browsed_dir = picked
                st.rerun()
    with save_col4:
        st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
        if st.button("＋", key="c_btn_new_folder_popup", help="Create a new empty folder", use_container_width=True):
            _c_new_folder_dialog()

    if st.session_state.c_browsed_dir:
        st.caption(f"📁 Saving to custom path: {st.session_state.c_browsed_dir}")
    st.write("---")

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
    map_data = st_folium(m, width=1200, height=600, key="creator_map")

    if map_data and map_data.get("center"):
        st.session_state.creator_center = [map_data["center"]["lat"], map_data["center"]["lng"]]
        st.session_state.creator_zoom = map_data["zoom"]
        c_lat, c_lon = st.session_state.creator_center
        st.info(f"Current Screen Center: {c_lat:.6f}, {c_lon:.6f} (Click 'Update' in sidebar to update restrictions in this area)")

    search_col1, search_col2 = st.columns([3, 1])
    with search_col1:
        c_search_query = st.text_input("Jump to Address or Lat/Lon", key="c_search_input", placeholder="e.g. 1600 Pennsylvania Ave or 40.25, -111.64")
    with search_col2:
        st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
        if st.button("Search Location", key="c_btn_search", use_container_width=True):
            with st.spinner("Searching..."):
                new_coords = get_coords_from_search(c_search_query)
                if new_coords:
                    st.session_state.locked_creator_center = new_coords
                    st.session_state.creator_center = new_coords
                    st.rerun()
                else:
                    st.error("Location not found. Try a different query.")
    st.write("---")

    if map_data.get("all_drawings"):
        coords = [(c[1], c[0]) for c in map_data["all_drawings"][-1]['geometry']['coordinates']]
        total_dist_ft = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1)) * M_TO_FT
        
        gap_ft = max(1.0, safe_get_float('t_dist_val', 9.0) ) if st.session_state.get('trigger_type', 'distance') == "distance" else speed_m * t_val_sec #* M_TO_FT
            
        if len(coords) > photo_start_wp:
            dist_to_start = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(photo_start_wp)) * M_TO_FT
            est_photos = int(max(0, total_dist_ft - dist_to_start) / gap_ft) + 1 if gap_ft > 0 else 0
        else:
            est_photos = 0

        save_disabled = False
        if is_dji_fly and est_photos > 99:
            st.error("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash please reduce your distance or increase the interval.")
            save_disabled = True

        with top_hud:
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}" + (" / 99" if is_dji_fly else ""))
            with c3:
                st.markdown("<div style='margin-top: 10px;'></div>", unsafe_allow_html=True)
                save_clicked = st.button("Save & Generate KMZ", use_container_width=True, disabled=save_disabled)
            st.write("---")

            if save_clicked:
                with st.spinner("Calculating terrain elevations and generating KMZ..."):
                    cfg = {
                        "safe_takeoff_ft": safe_takeoff_ft, "trans_speed_mph": trans_speed_mph,
                        "alt_ft": safe_get_float('alt_ft', 50.0), "pitch": safe_get_float('pitch', -60.0), "side": side,
                        "trigger_type": st.session_state.get('trigger_type', 'distance'),
                        "interval_ft": safe_get_float('t_dist_val', 9.0) if st.session_state.get('trigger_type', 'distance') == "distance" else 0.0,
                        "interval_sec": t_val_sec if st.session_state.get('trigger_type', 'distance') == "time" else 0.0,
                        "speed_m": speed_m, "photo_start_wp": int(photo_start_wp),
                        "camera_type": camera_type, "drone_sub": drone_sub_enum, "payload_sub": payload_sub_enum,
                        "is_dji_fly": is_dji_fly
                    }

                    platform_prefix = "Fly" if is_dji_fly else "Pilot"
                    prefixed_name = f"{mission_name}_{platform_prefix}"
                    suffix = f"_H{int(safe_get_float('alt_ft', 50.0))}A{int(abs(safe_get_float('pitch', -60.0)))}OL{int(safe_get_float('overlap_pct', 70.0))}"
                    final_filename = f"{prefixed_name}{suffix}"

                    if st.session_state.c_browsed_dir:
                        final_dir = st.session_state.c_browsed_dir
                    elif save_option == "Root (missions/)": final_dir = MISSION_DIR
                    elif save_option == "Create New Folder...": final_dir = os.path.join(MISSION_DIR, new_dir_name)
                    else: final_dir = os.path.join(MISSION_DIR, save_option)

                    os.makedirs(final_dir, exist_ok=True)
                    final_filepath = os.path.join(final_dir, f"{final_filename}.kmz")

                    template_kml, waylines_wpml = generate_native_kmz_contents(coords, cfg, c_elev_source, c_tif_path)
                    export_mission_kmz_from_strings(
                        template_kml_str=template_kml,
                        waylines_wpml_str=waylines_wpml,
                        output_kmz_path=final_filepath,
                        is_dji_fly=is_dji_fly
                    )
                    thumbnail_path = final_filepath.replace('.kmz', '.jpg')
                    generate_name_thumbnail(
                        prefixed_name, safe_get_float('alt_ft', 50.0), safe_get_float('pitch', -60.0),
                        safe_get_float('overlap_pct', 70.0), thumbnail_path, coords=coords
                    )

                st.success(f"Saved {final_filename}.kmz to {final_dir}/")
            st.divider()

# --- EDITOR MODE ---
elif page == 'Editor':
    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
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

            if meta['coords']:
                st.session_state.locked_editor_center = list(meta['coords'][0])
                st.session_state.editor_center = list(meta['coords'][0])
            
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
            st.header("1. Hardware & Payload")
            e_hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()), index=list(HARDWARE_MAP.keys()).index(meta.get('hardware_key', "DJI Fly (RC2 / Mini / Air Series)")))
            e_drone_enum = HARDWARE_MAP[e_hw_choice]["drone_enum"]
            e_drone_sub_enum = HARDWARE_MAP[e_hw_choice]["drone_sub"]
            e_payload_enum = HARDWARE_MAP[e_hw_choice]["payload_enum"]
            e_payload_sub_enum = HARDWARE_MAP[e_hw_choice]["payload_sub"]
            e_is_dji_fly = HARDWARE_MAP[e_hw_choice].get("is_dji_fly", False)
            
            if e_is_dji_fly:
                st.warning("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash saving will be disabled if you exceed this.")
            
            current_cam_display = CAM_DISPLAY_MAP.get(meta.get('camera_type', 'visible'), "RGB Only")
            e_cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"], index=["RGB Only", "Multispectral Only", "RGB + Multispectral"].index(current_cam_display))
            e_camera_type = CAM_VAL_MAP[e_cam_choice]
            min_photo_interval_sec = 2.0 if "narrow_band" in e_camera_type else 0.7
            
            st.info(f"Will save as: {edit_name}.kmz")
            st.header("2. Modify Parameters")
            e_safe = st.number_input("Safe Takeoff Alt (ft)", value=meta['safe_takeoff_ft'])
            e_trans = st.number_input("Takeoff Speed (mph)", value=meta['trans_speed_mph'])
            st.number_input("Relative Altitude (ft)", value=60.0, key="e_alt_ft", step=1.0, on_change=e_sync_geometry)
            st.info("❗Elevation is relative to the take off point, NOT the mission start point.")

            e_elev_source = st.selectbox("Elevation Source", ["Open-Elevation (Global)", "USGS 3DEP (US High-Res)", "Local GeoTIFF"], key="e_source")
            if e_elev_source == "Open-Elevation (Global)":
                st.warning("Can be off by several dozen feet. Use with caution.")
            elif e_elev_source == "USGS 3DEP (US High-Res)":
                st.warning("USGS parses coordinates individually. Generating long missions may take a few seconds.")
                
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

            st.slider("Gimbal Pitch (°)", -90, 0, value= -60, key="e_pitch", on_change=e_sync_geometry)

            current_e_pitch = safe_get_float('e_pitch', -60.0)
            pitch_rad_e = math.radians(abs(current_e_pitch))
            current_e_alt = safe_get_float('e_alt_ft', 50.0)
            D_ft_e = current_e_alt / math.sin(pitch_rad_e) if pitch_rad_e > 0 else float('inf')
            st.info(f"Est. Ground GSD: {(D_ft_e * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_e != float('inf') else 0:.2f} cm/px")
            
            e_side = st.selectbox("Yaw Side", ["right", "left"])

            st.header("3. Trigger Settings")
            e_start_wp = st.number_input("Start Photos at WP", min_value=0, value=meta['photo_start_wp'], step=1)
            e_trigger = st.radio("Type", ["distance", "time"], key="e_trigger_type", on_change=e_sync_geometry)
            safe_e_speed = max(2.3, float(meta.get('speed_mph', 6.0)))
            
            if st.session_state.get('e_trigger_type', 'distance') == "distance":
                st.number_input("Interval (ft)", key="e_t_dist_val", min_value=1.0, step=1.0, on_change=e_sync_dist_to_overlap)
                st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=e_sync_overlap_to_dist)
                e_speed_m = st.number_input("Flight Speed (mph)", min_value=2.3, step=1.0, value=safe_e_speed) * MPH_TO_MS
                
                gap_m = max(1.0, safe_get_float('e_t_dist_val', 9.0) * FT_TO_M)
                max_speed_m = gap_m / min_photo_interval_sec
                if e_speed_m > max_speed_m:
                    st.error(f"Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
            else:
                if 'e_t_time_val' not in st.session_state: 
                    st.session_state.e_t_time_val = meta['t_val'] if meta['trigger_type'] == 'time' else max(2.0, min_photo_interval_sec)
                e_tval_sec = st.number_input("Interval (sec)", key="e_t_time_val", min_value=min_photo_interval_sec)
                e_auto_speed = st.checkbox("Auto-Calc Speed", True)
                if e_auto_speed:
                    st.number_input("Target Gap (ft)", key="e_target_gap_ft", min_value=1.0, on_change=e_sync_gap_to_overlap)
                    st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0, on_change=e_sync_overlap_to_gap)
                    e_speed_m = min(max((safe_get_float('e_target_gap_ft', 26.2) * FT_TO_M) / e_tval_sec, 1.0), 10.0)
                    st.info(f"Auto-Calculated Speed: {e_speed_m * MS_TO_MPH:.1f} mph")
                else:
                    e_speed_m = st.number_input("Manual Speed (mph)", min_value=2.3, value=safe_e_speed, step=1.0) * MPH_TO_MS
                    fw = get_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
                    current_overlap = ((fw - (e_speed_m * M_TO_FT * e_tval_sec)) / fw) * 100 if fw > 0 else 0
                    st.info(f"Current Overlap: {max(0, min(current_overlap, 99.9)):.1f}%")

            st.header("4. Visuals")
            show_footprints = st.checkbox("Show Image Footprints", value=True)
            show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="editor_faa_toggle")
            if show_faa_airspace:
                st.write("#### Update restrictions of map center")
                if st.button("Update Map Center", key="btn_update_editor"):
                    st.session_state.locked_editor_center = st.session_state.editor_center
                    st.rerun()

        top_hud = st.container()
        st.write("### Fine-Tune Flight Path Coordinates")
        
        df = pd.DataFrame(meta['coords'], columns=['Latitude', 'Longitude'])
        edited_df = st.data_editor(df, num_rows="dynamic", key=st.session_state.editor_key, use_container_width=True)
        current_coords = [(row['Latitude'], row['Longitude']) for _, row in edited_df.iterrows()]

        m_edit = folium.Map(location=st.session_state.locked_editor_center, zoom_start=18, tiles=None)
        folium.TileLayer(tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', attr='Google', max_zoom=22, max_native_zoom=20).add_to(m_edit)
        
        if e_elev_source == "Local GeoTIFF" and e_tif_path and e_show_bounds:
            bounds = get_tif_bounds_wgs84(e_tif_path)
            if bounds:
                folium.Rectangle(bounds=bounds, color="#ff8800", weight=3, fill=True, fill_opacity=0.1, tooltip="Active GeoTIFF Boundary").add_to(m_edit)

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
        
        gap_ft_preview = max(1.0, safe_get_float('e_t_dist_val', 9.0) ) if st.session_state.get('e_trigger_type', 'distance') == "distance" else e_speed_m * safe_get_float('e_t_time_val', 2.0) * M_TO_FT#* M_TO_FT

        yaws = []
        for i in range(len(current_coords) - 1):
            ref_bearing = get_bearing(current_coords[i], current_coords[i+1])
            yaws.append((ref_bearing + 90) % 360 if e_side == "right" else (ref_bearing - 90) % 360)

        cum_dist = [0.0]
        total_dist_ft = 0.0
        
        elevations = get_elevations_batch(current_coords, e_elev_source, e_tif_path)
        start_elev = elevations[0] if elevations else 0
        target_agl_ft = safe_get_float('e_alt_ft', 50.0)
        
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
                        
                        footprint = get_photo_footprint(lat, lon, safe_get_float('e_alt_ft', 50.0), safe_get_float('e_pitch', -60.0), yaws[i])
                        folium.Polygon(locations=footprint, color="darkorange", weight=1, fill=True, fill_opacity=0.15).add_to(m_edit)
                        folium.CircleMarker([lat, lon], radius=2.5, color="yellow", fill=True).add_to(m_edit)
                        break
                current_dist += gap_ft_preview

        map_data_edit = st_folium(m_edit, width=1200, height=600, key="editor_map")

        if map_data_edit and map_data_edit.get("center"):
            st.session_state.editor_center = [map_data_edit["center"]["lat"], map_data_edit["center"]["lng"]]
            st.session_state.editor_zoom = map_data_edit["zoom"]
            c_lat, c_lon = st.session_state.editor_center
            st.info(f"Current Screen Center: {c_lat:.6f}, {c_lon:.6f}")

        e_search_col1, e_search_col2 = st.columns([3, 1])
        with e_search_col1:
            e_search_query = st.text_input("Jump to Address or Lat/Lon", key="e_search_input", placeholder="e.g. 1600 Pennsylvania Ave or 40.25, -111.64")
        with e_search_col2:
            st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
            if st.button("Search Location", key="e_btn_search", use_container_width=True):
                with st.spinner("Searching..."):
                    new_coords = get_coords_from_search(e_search_query)
                    if new_coords:
                        st.session_state.locked_editor_center = new_coords
                        st.session_state.editor_center = new_coords
                        st.rerun()
                    else:
                        st.error("Location not found. Try a different query.")
        st.write("---")

        final_coords = [(c[1], c[0]) for c in map_data_edit["all_drawings"][-1]['geometry']['coordinates']] if map_data_edit.get("all_drawings") and len(map_data_edit["all_drawings"]) > 0 else current_coords
        if map_data_edit.get("all_drawings") and len(map_data_edit["all_drawings"]) > 0: st.info("Using newly drawn line from the map.")

        with top_hud:
            total_dist_ft = sum(get_haversine_dist(final_coords[i], final_coords[i+1]) for i in range(len(final_coords)-1)) * M_TO_FT
            gap_ft = max(1.0, safe_get_float('e_t_dist_val', 9.0) ) if st.session_state.get('e_trigger_type', 'distance') == "distance" else e_speed_m * safe_get_float('e_t_time_val', 2.0) * M_TO_FT #* M_TO_FT
                
            if len(final_coords) > e_start_wp:
                dist_to_start = sum(get_haversine_dist(final_coords[i], final_coords[i+1]) for i in range(e_start_wp)) * M_TO_FT
                est_photos = int(max(0, total_dist_ft - dist_to_start) / gap_ft) + 1 if gap_ft > 0 else 0
            else:
                est_photos = 0
            
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}" + (" / 99" if e_is_dji_fly else ""))
            c3.metric("Flight Speed", f"{e_speed_m * MS_TO_MPH:.1f} mph")

            save_disabled = False
            if e_is_dji_fly and est_photos > 99:
                st.error("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash please reduce your distance or increase the interval.")
                save_disabled = True

            if st.button("Save & Update Mission", disabled=save_disabled):
                with st.spinner("Calculating terrain elevations and generating KMZ..."):
                    new_cfg = {
                        "safe_takeoff_ft": e_safe, "trans_speed_mph": e_trans, "alt_ft": safe_get_float('e_alt_ft', 50.0),
                        "pitch": safe_get_float('e_pitch', -60.0), "side": e_side, "trigger_type": st.session_state.get('e_trigger_type', 'distance'),
                        "interval_ft": safe_get_float('e_t_dist_val', 9.0) if st.session_state.get('e_trigger_type', 'distance') == "distance" else 0.0, 
                        "interval_sec": safe_get_float('e_t_time_val', 2.0) if st.session_state.get('e_trigger_type', 'distance') == "time" else 0.0, 
                        "speed_m": e_speed_m, "photo_start_wp": int(e_start_wp), "camera_type": e_camera_type,
                        "drone_sub": e_drone_sub_enum, "payload_sub": e_payload_sub_enum,
                        "is_dji_fly": e_is_dji_fly
                    }
                    
                    e_platform_prefix = "Fly" if e_is_dji_fly else "Pilot"
                    e_prefixed_name = f"{edit_name}_{e_platform_prefix}"
                    suffix = f"_H{int(safe_get_float('e_alt_ft', 50.0))}A{int(abs(safe_get_float('e_pitch', -60.0)))}OL{int(safe_get_float('e_overlap_pct', 70.0))}"
                    final_filename = f"{e_prefixed_name}{suffix}"

                    template_kml, waylines_wpml = generate_native_kmz_contents(final_coords, new_cfg, e_elev_source, e_tif_path)

                    final_filepath = os.path.join(active_dir, f"{final_filename}.kmz")
                    export_mission_kmz_from_strings(
                        template_kml_str=template_kml,
                        waylines_wpml_str=waylines_wpml,
                        output_kmz_path=final_filepath,
                        is_dji_fly=e_is_dji_fly
                    )

                    thumbnail_path = final_filepath.replace('.kmz', '.jpg')
                    generate_name_thumbnail(
                        e_prefixed_name, safe_get_float('e_alt_ft', 50.0), safe_get_float('e_pitch', -60.0),
                        safe_get_float('e_overlap_pct', 70.0), thumbnail_path, coords=final_coords
                    )

                st.success(f"Successfully updated and saved as {final_filename}.kmz in {selected_dir_name}!")
            st.divider()

# ==========================================
# VIEWER MODE
# ==========================================
elif page == 'Viewer  |':
    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
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

                try:
                    with zipfile.ZipFile(full_path, 'r') as kmz:
                        waylines_file = [name for name in kmz.namelist() if name.endswith('waylines.wpml')][0]
                        template_file = [name for name in kmz.namelist() if name.endswith('template.kml')][0]
                        root = ET.fromstring(kmz.read(waylines_file))
                        root_t = ET.fromstring(kmz.read(template_file))
                except Exception as e:
                    st.error(f"Could not read KMZ file: {e}")
                    continue
                
                meta = {"speed": 0, "pitch": -60, "mode": "None", "t_val": 0, "alt": 50.0, "safe_alt": 0, "start_idx": 0, "camera_type": "visible"}

                p_node = root.find('.//{*}waypointGimbalHeadingParam/{*}waypointGimbalPitchAngle')
                if p_node is not None: meta['pitch'] = float(p_node.text)
                speed_node = root.find('.//{*}autoFlightSpeed')
                if speed_node is not None: meta['speed'] = float(speed_node.text)

                wp_data = []
                for pm in root.findall('.//{*}Placemark'):
                    idx_node = pm.find('.//{*}index')
                    idx = int(idx_node.text) if idx_node is not None else len(wp_data)
                    c_node = pm.find('.//{*}coordinates')
                    if c_node is None: continue
                    c_raw = c_node.text.strip().split(',')
                    yaw_node = pm.find('.//{*}waypointHeadingAngle')
                    yaw = float(yaw_node.text) if yaw_node is not None else 0.0
                    alt_node = pm.find('.//{*}executeHeight')
                    alt = float(alt_node.text) * M_TO_FT if alt_node is not None else 0.0
                    
                    target_yaw = yaw
                    for action_group in pm.findall('.//{*}actionGroup'):
                        t_type = action_group.find('.//{*}actionTriggerType')
                        if t_type is not None:
                            if 'multiple' in t_type.text:
                                meta['mode'] = "Distance" if "Distance" in t_type.text else "Time"
                                t_param = action_group.find('.//{*}actionTriggerParam')
                                if t_param is not None: meta['t_val'] = float(t_param.text)
                                start_idx = action_group.find('.//{*}actionGroupStartIndex')
                                if start_idx is not None: meta['start_idx'] = int(start_idx.text)
                            elif 'reachPoint' in t_type.text:
                                pass
                        
                        for a in action_group.findall('.//{*}action'):
                            func = a.find('.//{*}actionActuatorFunc')
                            if func is not None:
                                if func.text == 'takePhoto':
                                    params = a.find('.//{*}actionActuatorFuncParam')
                                    if params is not None:
                                        lens = params.find('.//{*}payloadLensIndex')
                                        if lens is not None: meta['camera_type'] = lens.text
                                elif func.text == 'rotateYaw':
                                    params = a.find('.//{*}actionActuatorFuncParam')
                                    if params is not None:
                                        heading = params.find('.//{*}aircraftHeading')
                                        if heading is not None: target_yaw = float(heading.text)

                    wp_data.append({'lat': float(c_raw[1]), 'lon': float(c_raw[0]), 'yaw': yaw, 'target_yaw': target_yaw, 'alt': alt, 'index': idx})

                if wp_data:
                    if m_view is None:
                        if 'current_viewer_file' not in st.session_state or st.session_state.current_viewer_file != selected_kmzs[0]:
                            st.session_state.current_viewer_file = selected_kmzs[0]
                            st.session_state.locked_viewer_center = [wp_data[0]['lat'], wp_data[0]['lon']]
                        
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
                    is_dense_mission = (len(wp_data) > 4 and meta['mode'] == "None")
                    
                    if is_dense_mission:
                        for w in wp_data:
                            if show_footprints:
                                yaw = w['target_yaw']
                                footprint = get_photo_footprint(w['lat'], w['lon'], w['alt'], meta['pitch'], yaw)
                                folium.Polygon(locations=footprint, color="darkorange", weight=1, fill=True, fill_opacity=0.15).add_to(m_view)
                            folium.CircleMarker([w['lat'], w['lon']], radius=2.5, color="yellow", fill=True).add_to(m_view)
                            photo_count += 1
                    elif gap > 0 and meta['start_idx'] < len(wp_data):
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

                    st.sidebar.write(f"Hardware Platform: {hw_key}")
                    st.sidebar.write(f"Camera Sensor: {cam_display}")
                    st.sidebar.write(f"Gimbal Pitch: {meta['pitch']}°")
                    st.sidebar.write(f"Safe Takeoff: {meta['safe_alt']*M_TO_FT:.1f} ft")
                    st.sidebar.write(f"Waypoint Alt: {meta['alt']*M_TO_FT:.1f} ft")
                    st.sidebar.write(f"Trigger: {'Dense Waypoints (DJI Fly)' if meta['mode'] == 'None' else meta['mode']} ({meta['t_val']*M_TO_FT if meta['mode']=='Distance' else meta['t_val']:.1f})")
                    st.sidebar.write(f"Calculated Photos: {grand_total_photos}")
                else:
                    st.sidebar.success(f"Viewing {len(selected_kmzs)} combined missions.")
                    st.sidebar.write(f"Total Aggregated Distance: {grand_total_dist_ft:.1f} ft")
                    st.sidebar.write(f"Total Aggregated Photos: {grand_total_photos}")

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
                map_data_view = st_folium(m_view, width=1200, height=600, key="viewer_map")
            
                if map_data_view and map_data_view.get("center"):
                    st.session_state.viewer_center = [map_data_view["center"]["lat"], map_data_view["center"]["lng"]]
                    st.session_state.viewer_zoom = map_data_view["zoom"]
                    c_lat, c_lon = st.session_state.viewer_center
                    st.sidebar.info(f"Current Screen Center: {c_lat:.6f}, {c_lon:.6f} (Click 'Update' in sidebar to update restrictions in this area)")

# ==========================================
# PHOTO SORTER MODE
# ==========================================
elif page == 'Photo Sorter':
    st.header("Photo Sorter")
    st.write("Automatically group drone photos into separate folders based on the time they were taken.")
    st.write("This is for specifically for drones the use DJI Fly.")

    # Initialize default paths in session state so they don't reset
    if "sorter_source" not in st.session_state:
        st.session_state.sorter_source = os.path.expanduser("~")
    if "sorter_output" not in st.session_state:
        st.session_state.sorter_output = os.path.join(os.path.expanduser("~"), "Output")

    def pick_source_folder():
        folder_path = pick_folder_dialog("Select Source Directory")
        if folder_path:
            st.session_state.sorter_source = folder_path

    def pick_output_folder():
        folder_path = pick_folder_dialog("Select Output Directory")
        if folder_path:
            st.session_state.sorter_output = folder_path

    col1, col2 = st.columns(2)
    
    with col1:
        c1_text, c1_btn = st.columns([5, 1])
        with c1_text:
            st.text_input("Source Directory (Where the photos are currently)", key="sorter_source")
        with c1_btn:
            st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
            st.button("📂", key="btn_pick_source", on_click=pick_source_folder, help="Browse for Source Directory")
            
        target_date = st.date_input("Target Date")

    with col2:
        c2_text, c2_btn = st.columns([5, 1])
        with c2_text:
            st.text_input("Output Directory (Where to create the group folders)", key="sorter_output")
        with c2_btn:
            st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
            st.button("📂", key="btn_pick_output", on_click=pick_output_folder, help="Browse for Output Directory")
            
        gap_minutes = st.number_input("Time Gap (minutes)", min_value=1, value=5, step=1, help="If the time between two sequential photos exceeds this gap, a new folder is created.")
        
    st.write("---")
    submit_btn = st.button("🚀 Sort Photos", use_container_width=True)
    
    if submit_btn:
        source_dir = st.session_state.sorter_source
        output_dir = st.session_state.sorter_output
        
        if not source_dir or not os.path.exists(source_dir):
            st.error("The source directory does not exist or is invalid.")
        elif not output_dir:
            st.error("Please provide an output directory.")
        else:
            with st.spinner("Processing images..."):
                st_group_images_by_time(source_dir, output_dir, target_date, gap_minutes)

# ==========================================
# BATCH TRANSFER MODE
# ==========================================
elif page == 'DJI Fly Transfer':
    st.header("DJI Fly Batch Mission Transfer")
    st.write("Assign local flight plans (left) to overwrite existing missions on the RC 2 (right).")
    
    if 'rc_nests' not in st.session_state:
        st.session_state.rc_nests = {}
        st.session_state.preview_id = None

    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("1. Source Missions")
        existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
        selected_dir_name = st.selectbox("Select Local Folder", ["Root (missions/)"] + existing_dirs, key="batch_dir")
        
        active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
        # This page is DJI Fly-only (MTP transfer to the RC 2's dummy mission
        # slots doesn't apply to DJI Pilot missions), so Pilot-format .kmz
        # files are filtered out rather than just listed alongside Fly ones.
        kmz_files = [
            f for f in os.listdir(active_dir)
            if f.endswith(".kmz") and is_dji_fly_kmz(os.path.join(active_dir, f))
        ]

        if not kmz_files:
            st.warning(f"No DJI Fly missions found in {selected_dir_name}.")
        else:
            st.info(f"Found {len(kmz_files)} missions ready for transfer.")
            
    with col2:
        st.subheader("2. Controller Nests")
        st.write("Connect the RC 2 via USB, power on, and close Android File Transfer.")
        if st.button("🔄 Scan RC 2 & Pull Previews", use_container_width=True):
            with st.spinner("Scanning MTP and downloading thumbnails... (This takes a few seconds)"):
                st.session_state.rc_nests, st.session_state.preview_id = fetch_controller_nests_and_previews()
                
        if st.session_state.rc_nests:
            st.success(f"Found {len(st.session_state.rc_nests)} authorized mission slots.")
        else:
            st.warning("No controller connected, or no dummy missions found.")

    st.write("---")
    
    # --- The Visual Mapping UI ---
    if kmz_files and st.session_state.rc_nests:
        st.subheader("3. Assign & Transfer")
        
        max_rows = max(len(kmz_files), len(st.session_state.rc_nests))
        num_rows = st.number_input("Number of missions to assign", min_value=1, max_value=max_rows, value=min(3, max_rows))
        
        transfer_map = {}

        # Header formatting
        h1, h2, h3 = st.columns([4, 1, 4])
        h1.markdown("**Local Mission** (What to push)")
        h3.markdown("**Controller Target** (What will be overwritten)")

        # Dynamic Visual Rows
        # Once a local mission or nest is picked in one row, it's excluded from
        # the other rows' dropdowns - keeps the lists shrinking as you assign,
        # instead of showing already-used options, so it's easier to work
        # through what's left.
        #
        # This is done in two passes: first resolve what each row's value
        # will be (keeping any value the user already explicitly set, and
        # picking sensible non-overlapping defaults for the rest against a
        # shrinking pool), then render each row's dropdown with exclusions
        # computed from that fully-resolved picture. Reading st.session_state
        # naively mid-loop instead would mix fresh values (for rows already
        # rendered this run) with stale, previous-run values (for rows not
        # yet reached) - and on first load, before any row has a value at
        # all, would produce no exclusions whatsoever.
        def _resolve_row_values(all_options, session_key_prefix, placeholder):
            remaining = list(all_options)
            resolved = {}
            for j in range(int(num_rows)):
                state_key = f"{session_key_prefix}{j}"
                if state_key in st.session_state:
                    # This row has already been rendered before, even if the
                    # user has since cleared it back to the placeholder -
                    # respect that as an intentional "nothing assigned here"
                    # rather than auto-filling it from the remaining pool,
                    # which would keep it (and whatever got auto-filled)
                    # wrongly excluded from every other row's options.
                    existing = st.session_state[state_key]
                    resolved[j] = existing if (existing == placeholder or existing in remaining) else placeholder
                elif remaining:
                    resolved[j] = remaining[0]
                else:
                    resolved[j] = placeholder
                if resolved[j] in remaining:
                    remaining.remove(resolved[j])
            return resolved

        resolved_locs = _resolve_row_values(kmz_files, "loc_", "--- Select Local Mission ---")
        resolved_nests = _resolve_row_values(list(st.session_state.rc_nests.keys()), "nest_", "--- Select Target Nest ---")

        for i in range(int(num_rows)):
            st.markdown(f"**Assignment {i+1}**")
            row_c1, row_c2, row_c3 = st.columns([4, 1, 4])

            chosen_locs_elsewhere = {v for j, v in resolved_locs.items() if j != i} - {"--- Select Local Mission ---"}
            chosen_nests_elsewhere = {v for j, v in resolved_nests.items() if j != i} - {"--- Select Target Nest ---"}

            row_local_options = ["--- Select Local Mission ---"] + [
                k for k in kmz_files if k not in chosen_locs_elsewhere
            ]
            row_nest_options = ["--- Select Target Nest ---"] + [
                n for n in st.session_state.rc_nests.keys() if n not in chosen_nests_elsewhere
            ]

            with row_c1:
                default_loc = row_local_options.index(resolved_locs[i]) if resolved_locs[i] in row_local_options else 0
                loc_choice = st.selectbox(f"Local {i}", row_local_options, index=default_loc, key=f"loc_{i}", label_visibility="collapsed")

                # Show Local Preview Image
                if loc_choice != "--- Select Local Mission ---":
                    local_jpg = os.path.join(active_dir, loc_choice.replace('.kmz', '.jpg'))
                    if os.path.exists(local_jpg):
                        st.image(local_jpg, use_container_width=True)
                    else:
                        st.info("No custom title card generated.")

            with row_c2:
                # Add a visual arrow pointing from local to remote
                st.markdown("<h1 style='text-align: center; color: gray; margin-top: 40px;'>➔</h1>", unsafe_allow_html=True)

            with row_c3:
                default_nest = row_nest_options.index(resolved_nests[i]) if resolved_nests[i] in row_nest_options else 0
                nest_choice = st.selectbox(f"Nest {i}", row_nest_options, index=default_nest, key=f"nest_{i}", label_visibility="collapsed")
                
                # Show Cached Controller Preview Image
                if nest_choice != "--- Select Target Nest ---":
                    cached_jpg = os.path.join("missions/.cache", f"{nest_choice}.jpg")
                    if os.path.exists(cached_jpg):
                        st.image(cached_jpg, use_container_width=True, caption=f"Current: {nest_choice[-8:]}")
                    else:
                        st.info("Native Dummy Mission\n\n*(Preview unreadable over USB until overridden)*")
                
            if loc_choice != "--- Select Local Mission ---" and nest_choice != "--- Select Target Nest ---":
                transfer_map[loc_choice] = nest_choice
            
            st.write("---")
                    
        if st.button("🚀 Execute Visual Transfer", use_container_width=True):
            if not transfer_map:
                st.warning("No valid pairs assigned! Select a local mission and a target nest.")
            else:
                st.session_state.last_transfer_checklist = []  # reset for this batch

                progress_bar = st.progress(0, text="Initializing transfer...")
                total_tasks = len(transfer_map)
                completed = 0

                for kmz_name, target_uuid in transfer_map.items():
                    local_path = os.path.join(active_dir, kmz_name)
                    target_folder_id = st.session_state.rc_nests[target_uuid]

                    progress_bar.progress(completed / total_tasks, text=f"Transferring {kmz_name}...")

                    # Call the MTP helper we updated earlier
                    success, error_msg = push_mission_to_nest(local_path, target_uuid)

                    if success:
                        st.success(f"✅ Transferred **{kmz_name}** into slot `{target_uuid}`")
                        st.session_state.last_transfer_checklist.append((kmz_name, target_uuid))
                    else:
                        st.error(f"❌ Failed to transfer **{kmz_name}**: {error_msg}")

                    completed += 1
                    time.sleep(1.5) # Let the Android File System breathe

                progress_bar.progress(1.0, text="Batch transfer complete!")

        # DJI Fly caches each mission's thumbnail privately and won't pick up a
        # newly-pushed one on its own - not even after a full power cycle (see
        # session notes). Opening a mission in DJI Fly and saving it once is
        # the only thing that's been found to force a refresh, so surface a
        # checklist of exactly what was just transferred rather than leaving
        # the user to remember on their own.
        if st.session_state.get("last_transfer_checklist"):
            st.write("---")
            st.subheader("📋 Manual Thumbnail Refresh Checklist")
            st.info(
                "DJI Fly caches each mission's thumbnail privately and won't pick up "
                "the new one automatically - not even after a full power cycle. Open "
                "each mission below in DJI Fly and save it once to force its "
                "thumbnail to refresh on the controller. The picture shown is what that "
                "mission currently still looks like on the controller's screen (the UUID "
                "itself isn't visible there), so you can spot the right one in DJI Fly's list."
            )
            for kmz_name, target_uuid in st.session_state.last_transfer_checklist:
                check_col1, check_col2 = st.columns([1, 5])
                with check_col1:
                    cached_jpg = os.path.join("missions/.cache", f"{target_uuid}.jpg")
                    if os.path.exists(cached_jpg):
                        st.image(cached_jpg, width=1600)
                    else:
                        st.caption("(no preview cached - scan the RC 2 to fetch one)")
                with check_col2:
                    st.checkbox(f"{kmz_name} → `{target_uuid}`", key=f"refresh_check_{target_uuid}")