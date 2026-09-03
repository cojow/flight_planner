import streamlit as st
import streamlit.components.v1 as components
import pandas as pd
import os
import json
import urllib.request
import urllib.parse
import math
import zipfile
import xml.etree.ElementTree as ET
import base64
import mimetypes
from datetime import datetime, timedelta
import re
import uuid
from geopy.geocoders import Nominatim
import folium
from folium.plugins import Draw, PolyLineTextPath

# Optional: only the experimental decomposed coverage strategy needs shapely.
# Imported defensively so a missing install degrades to "that option is
# unavailable" rather than taking the whole app down on startup.
try:
    from shapely.geometry import Polygon as ShapelyPolygon, box as shapely_box
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
from folium.features import DivIcon
from streamlit_folium import st_folium
from branca.element import Element
import shutil
from PIL import Image
import subprocess
import shlex
import time
import matplotlib
# Streamlit sets MPLBACKEND=Agg on import, which only takes effect if
# matplotlib hasn't already picked a backend - as long as `import streamlit`
# (line 1) runs before matplotlib.pyplot's first import, this is currently
# redundant. But generate_name_thumbnail() renders on Streamlit's own
# per-rerun worker thread, and any interactive backend (e.g. this box's
# default, TkAgg) crashes the whole process when driven off the main thread
# - reproduced live as a hard segfault (Tcl_AsyncDelete: async handler
# deleted by the wrong thread). Setting it explicitly here means that stays
# true even if these imports are ever reordered (an autoformatter's import
# sort would do it) or the app is ever run standalone without Streamlit.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import textwrap
import ctypes
import ctypes.util
import platform
import io
import logging

# DJI Fly Transfer diagnostics: fetch_controller_nests_and_previews() and
# push_mission_to_nest() used to swallow every exception and fall back to a
# generic "no controller found" message, which made a real bug (wrong
# device, a broken MTP/WPD call, etc.) indistinguishable from the user
# simply not having the controller plugged in. This logger prints the real
# exception - with traceback - to the terminal streamlit was launched from,
# so a genuine failure is diagnosable instead of silently disabled.
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("dji_fly_transfer")


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
                # The raw-device enumeration above succeeded (the OS sees the
                # controller on the bus), but libmtp couldn't actually claim
                # it - almost always because another app already has it
                # open. On Mac, Image Capture/Preview/Photos auto-launch and
                # grab any newly connected camera/MTP device before you get
                # a chance to; on Linux, gvfs/gphoto2 auto-mounting the
                # device does the same thing. This is the single most common
                # failure users hit, so name it here rather than in the
                # generic "device not detected" message above.
                raise MTPBridgeError(
                    "Detected the controller, but couldn't open a session with it - it's likely "
                    "already claimed by another app. On Mac, check for Preview, Photos, or Image "
                    "Capture (they auto-launch when a camera/MTP device connects) and quit "
                    "whichever opened, then Scan again without unplugging the controller."
                )
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
    # DEBUG, not a warning: this bridge is expected to fail to load on
    # Windows (no libmtp there) and on any Mac/Linux box without libmtp
    # installed - by design, WPDSession covers Windows instead. Only worth
    # surfacing loudly if it turns out to be the platform's only option;
    # get_mtp_session_class() does that check and logs accordingly.
    logger.debug("libmtp MTP bridge unavailable", exc_info=True)
    MTP_BRIDGE_AVAILABLE = False

# ==========================================
# WPD BRIDGE (Windows Portable Devices - native Windows MTP access)
# ==========================================
# libmtp needs raw USB access to the controller, which on Windows means
# replacing its driver with WinUSB via Zadig - and that breaks normal
# Explorer/MTP access to the controller until the driver is swapped back.
# Windows Portable Devices (WPD) is Windows' own driver stack for MTP
# devices - the same one Explorer and Android File Transfer already use -
# so talking to the controller through it needs no driver replacement at
# all. This mirrors MTPSession's small interface (list_children/
# find_child/resolve_path/pull_file/delete_object/send_disguised_file) so
# fetch_controller_nests_and_previews/push_mission_to_nest don't need to
# know which backend is active.
#
# The WPD PROPERTYKEY/GUID constants below (WPD_OBJECT_NAME, WPD_OBJECT_
# FORMAT, WPD_CONTENT_TYPE_FOLDER, etc.) come from the Windows SDK's
# PortableDevice.h. They're C-header DEFINE_PROPERTYKEY/DEFINE_GUID
# constants, not COM interface members, so comtypes can't generate them
# the way it generates the interfaces themselves - they have to be
# transcribed by hand.
#
# A handful of WPD interface methods are marked plain 'in' in the shipped
# typelib where the real C++ headers declare '[out]'/'[in,out]'
# (IPortableDeviceKeyCollection.GetCount/GetAt among them) - a known quirk
# of WPD's typelib metadata - so those calls below pass an explicit ctypes
# byref() output slot instead of relying on comtypes' usual automatic
# in/out marshalling.
try:
    import comtypes
    import comtypes.client as _wpd_cc
    from ctypes import byref, c_ulong, c_ubyte, c_wchar_p

    _wpd_cc.GetModule("PortableDeviceApi.dll")
    _wpd_cc.GetModule("PortableDeviceTypes.dll")
    import comtypes.gen.PortableDeviceApiLib as _WpdApi
    import comtypes.gen.PortableDeviceTypesLib as _WpdTypes
    from comtypes.gen._1F001332_1A57_4934_BE31_AFFC99F4EE0A_0_1_0 import (
        _tagpropertykey as _WPD_PROPERTYKEY,
        tag_inner_PROPVARIANT as _WPD_PROPVARIANT,
    )

    def _wpd_key(fmtid, pid):
        return _WPD_PROPERTYKEY(comtypes.GUID(fmtid), pid)

    _FMTID_WPD_OBJECT = "{EF6B490D-5CD8-437A-AFFC-DA8B60EE4A3C}"
    _FMTID_WPD_CLIENT = "{204D9F0C-2292-4080-9F42-40664E70F859}"
    _FMTID_WPD_RESOURCE = "{E81E79BE-34F0-41BF-B53F-F1A06AE87842}"

    WPD_OBJECT_PARENT_ID = _wpd_key(_FMTID_WPD_OBJECT, 3)
    WPD_OBJECT_NAME = _wpd_key(_FMTID_WPD_OBJECT, 4)
    WPD_OBJECT_FORMAT = _wpd_key(_FMTID_WPD_OBJECT, 6)
    WPD_OBJECT_CONTENT_TYPE = _wpd_key(_FMTID_WPD_OBJECT, 7)
    WPD_OBJECT_SIZE = _wpd_key(_FMTID_WPD_OBJECT, 11)
    WPD_OBJECT_ORIGINAL_FILE_NAME = _wpd_key(_FMTID_WPD_OBJECT, 12)
    WPD_CLIENT_NAME_KEY = _wpd_key(_FMTID_WPD_CLIENT, 2)
    WPD_CLIENT_MAJOR_VERSION = _wpd_key(_FMTID_WPD_CLIENT, 3)
    WPD_CLIENT_MINOR_VERSION = _wpd_key(_FMTID_WPD_CLIENT, 4)
    WPD_CLIENT_REVISION = _wpd_key(_FMTID_WPD_CLIENT, 5)
    WPD_CLIENT_SECURITY_QUALITY_OF_SERVICE = _wpd_key(_FMTID_WPD_CLIENT, 8)
    WPD_RESOURCE_DEFAULT = _wpd_key(_FMTID_WPD_RESOURCE, 0)

    WPD_CONTENT_TYPE_FOLDER = comtypes.GUID("{27E2E392-A111-48E0-AB0C-E17705A05F85}")
    # PTP/MTP object-format code 0x3801 ("EXIF/JPEG") - the same format
    # LIBMTP_FILETYPE_JPEG (used above for the macOS/Linux path) maps to.
    # Used to disguise the .kmz as a photo so the controller's own MTP
    # responder accepts it instead of rejecting an unrecognized format.
    WPD_OBJECT_FORMAT_EXIF = comtypes.GUID("{38010000-AE6C-4804-98BA-C57B46965FE7}")
    WPD_DEVICE_OBJECT_ID = "DEVICE"
    _VT_LPWSTR = 31
    _STGM_READ = 0x00000000

    def _wpd_make_key_collection(keys):
        coll = _wpd_cc.CreateObject(_WpdTypes.PortableDeviceKeyCollection, interface=_WpdApi.IPortableDeviceKeyCollection)
        for k in keys:
            coll.Add(byref(k))
        return coll

    def _wpd_make_values():
        return _wpd_cc.CreateObject(_WpdTypes.PortableDeviceValues, interface=_WpdApi.IPortableDeviceValues)

    def _wpd_propvariant_str(value):
        """Builds a VT_LPWSTR PROPVARIANT wrapping `value`. Returns (propvariant,
        buffer) - the caller must keep `buffer` alive for as long as the
        propvariant is in use, since ctypes does not otherwise keep the
        underlying string alive on its own."""
        buf = c_wchar_p(value)
        pv = _WPD_PROPVARIANT()
        pv.vt = _VT_LPWSTR
        pv.__MIDL____MIDL_itf_PortableDeviceApi_0001_00000001.pwszVal = buf
        return pv, buf

    def _wpd_get_string(values, key, default=""):
        try:
            return values.GetStringValue(byref(key)) or default
        except Exception:
            return default

    def _wpd_get_guid_str(values, key):
        try:
            return str(values.GetGuidValue(byref(key)))
        except Exception:
            return None

    def _wpd_get_u64(values, key, default=0):
        try:
            return values.GetUnsignedLargeIntegerValue(byref(key))
        except Exception:
            return default

    # IPortableDeviceManager::GetDevices() - the "normal" way to list WPD
    # devices - has been observed to report zero devices for controllers
    # that Explorer can browse into just fine over the same WPD stack (seen
    # live against an RC 2: Explorer opens and lists its folders normally
    # while GetDevices() returns an empty list moments later, even after
    # RefreshDeviceList() and a re-plug). Enumerating the WPD device
    # interface directly via SetupAPI - the same low-level mechanism the
    # shell itself relies on to notice a portable device's arrival - finds
    # the device reliably where GetDevices() doesn't, so that's used here
    # instead. GetDevices() is kept as a fallback in case some other
    # machine/device combination hits the reverse case.
    _GUID_DEVINTERFACE_WPD = "{6AC27878-A6FA-4155-BA85-F98F491D4F33}"
    _DIGCF_PRESENT = 0x2
    _DIGCF_DEVICEINTERFACE = 0x10
    from ctypes import wintypes as _wintypes

    class _WPD_GUID(ctypes.Structure):
        _fields_ = [("Data1", _wintypes.DWORD), ("Data2", _wintypes.WORD), ("Data3", _wintypes.WORD),
                    ("Data4", _wintypes.BYTE * 8)]

    class _SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
        _fields_ = [("cbSize", _wintypes.DWORD), ("InterfaceClassGuid", _WPD_GUID),
                    ("Flags", _wintypes.DWORD), ("Reserved", ctypes.POINTER(_wintypes.ULONG))]

    def _wpd_load_setupapi():
        """
        Loads setupapi.dll as its own independent handle rather than via the
        process-wide, name-cached `ctypes.windll.setupapi` proxy. Streamlit
        re-execs this whole module on every rerun (every widget interaction,
        not just app startup), which redefines the ctypes Structure types
        below fresh each time and re-points argtypes/restype at them - fine
        on its own, but `ctypes.windll.setupapi` is a single object shared
        process-wide, so a rerun's thread reassigning its argtypes while a
        still-finishing previous rerun's thread is mid-call on the same
        function is a genuine data race (observed as a hard segfault, not a
        Python exception, when driven through the live Streamlit app - a
        standalone single-threaded reproduction of the same calls didn't
        crash, which pointed at cross-rerun/thread shared state rather than
        the calls themselves). A fresh WinDLL instance per call keeps each
        rerun's argtypes/restype configuration local to that call.
        """
        dll = ctypes.WinDLL("setupapi.dll")
        dll.SetupDiGetClassDevsW.restype = _wintypes.HANDLE
        dll.SetupDiGetClassDevsW.argtypes = [ctypes.POINTER(_WPD_GUID), _wintypes.LPCWSTR, _wintypes.HANDLE, _wintypes.DWORD]
        dll.SetupDiEnumDeviceInterfaces.restype = _wintypes.BOOL
        dll.SetupDiEnumDeviceInterfaces.argtypes = [_wintypes.HANDLE, ctypes.c_void_p, ctypes.POINTER(_WPD_GUID), _wintypes.DWORD, ctypes.POINTER(_SP_DEVICE_INTERFACE_DATA)]
        dll.SetupDiGetDeviceInterfaceDetailW.restype = _wintypes.BOOL
        dll.SetupDiGetDeviceInterfaceDetailW.argtypes = [_wintypes.HANDLE, ctypes.POINTER(_SP_DEVICE_INTERFACE_DATA), ctypes.c_void_p, _wintypes.DWORD, ctypes.POINTER(_wintypes.DWORD), ctypes.c_void_p]
        dll.SetupDiDestroyDeviceInfoList.restype = _wintypes.BOOL
        dll.SetupDiDestroyDeviceInfoList.argtypes = [_wintypes.HANDLE]
        return dll

    def _wpd_enumerate_device_paths():
        """Returns the device paths of all present WPD-class device interfaces,
        found via SetupAPI directly rather than IPortableDeviceManager."""
        _setupapi = _wpd_load_setupapi()
        guid = _WPD_GUID()
        ctypes.memmove(byref(guid), byref(comtypes.GUID(_GUID_DEVINTERFACE_WPD)), ctypes.sizeof(_WPD_GUID))
        h_dev_info = _setupapi.SetupDiGetClassDevsW(byref(guid), None, None, _DIGCF_PRESENT | _DIGCF_DEVICEINTERFACE)
        if not h_dev_info or h_dev_info == _wintypes.HANDLE(-1).value:
            return []
        paths = []
        try:
            index = 0
            while True:
                if_data = _SP_DEVICE_INTERFACE_DATA()
                if_data.cbSize = ctypes.sizeof(_SP_DEVICE_INTERFACE_DATA)
                if not _setupapi.SetupDiEnumDeviceInterfaces(h_dev_info, None, byref(guid), index, byref(if_data)):
                    break
                required = _wintypes.DWORD(0)
                _setupapi.SetupDiGetDeviceInterfaceDetailW(h_dev_info, byref(if_data), None, 0, byref(required), None)
                buf = ctypes.create_string_buffer(required.value)
                ctypes.cast(buf, ctypes.POINTER(_wintypes.DWORD))[0] = 8  # cbSize of the detail struct header
                if _setupapi.SetupDiGetDeviceInterfaceDetailW(h_dev_info, byref(if_data), buf, required, byref(required), None):
                    paths.append(ctypes.wstring_at(ctypes.addressof(buf) + 4))
                index += 1
        finally:
            _setupapi.SetupDiDestroyDeviceInfoList(h_dev_info)
        return paths

    class WPDSession:
        """
        Windows-native equivalent of MTPSession, built on the Windows
        Portable Devices COM API instead of libmtp. See the WPD bridge
        comment above for why this exists as a separate backend.

        Use as a context manager, exactly like MTPSession:
            with WPDSession() as session:
                waypoint_id = session.resolve_path(WAYPOINT_PATH)
                children = session.list_children(waypoint_id)
        """

        def __init__(self):
            self.device = None
            self.content = None
            self._props = None
            self._resources = None
            self._com_initialized = False

        def __enter__(self):
            try:
                comtypes.CoInitialize()
                self._com_initialized = True
            except OSError:
                pass  # already initialized on this thread

            device_paths = _wpd_enumerate_device_paths()
            if not device_paths:
                # Fall back to IPortableDeviceManager in case some other
                # machine/device combination needs it instead (see the
                # comment above _wpd_enumerate_device_paths).
                mgr = _wpd_cc.CreateObject(_WpdApi.PortableDeviceManager, interface=_WpdApi.IPortableDeviceManager)
                _, count = mgr.GetDevices(None, 0)
                if count:
                    arr = (c_wchar_p * count)()
                    arr, count = mgr.GetDevices(arr, count)
                    device_paths = [arr[0]]
            if not device_paths:
                raise MTPBridgeError("No MTP device detected.")
            device_id = device_paths[0]

            client_info = _wpd_make_values()
            client_info.SetStringValue(byref(WPD_CLIENT_NAME_KEY), "DJI Flight Planner")
            client_info.SetUnsignedIntegerValue(byref(WPD_CLIENT_MAJOR_VERSION), 1)
            client_info.SetUnsignedIntegerValue(byref(WPD_CLIENT_MINOR_VERSION), 0)
            client_info.SetUnsignedIntegerValue(byref(WPD_CLIENT_REVISION), 0)
            # SECURITY_IMPERSONATION - the value Microsoft's own WPD samples use.
            client_info.SetUnsignedIntegerValue(byref(WPD_CLIENT_SECURITY_QUALITY_OF_SERVICE), 2)

            try:
                self.device = _wpd_cc.CreateObject(_WpdApi.PortableDevice, interface=_WpdApi.IPortableDevice)
                self.device.Open(device_id, client_info)
            except Exception as e:
                raise MTPBridgeError(f"Failed to open WPD device session: {e}")
            self.content = self.device.Content()
            self._props = self.content.Properties()
            self._resources = self.content.Transfer()
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            if self.device:
                try:
                    self.device.Close()
                except Exception:
                    pass
                self.device = None
            # self.content/_props/_resources hold comtypes COM interface
            # pointers whose Release() calls fire from Python's refcounting
            # GC whenever these attributes are dropped - which, left to
            # attribute lookup + normal `with` block teardown, happens AFTER
            # this method returns and the WPDSession instance itself goes
            # out of scope. That's after CoUninitialize() below has already
            # torn down this thread's COM apartment, and releasing a COM
            # pointer post-uninitialize is undefined behavior - reproduced
            # live as a hard segfault when driven through Streamlit (whose
            # script-runner thread model made the GC timing land squarely in
            # that window; a plain single-threaded reproduction didn't hit
            # it). Clearing them here forces those Release() calls while the
            # apartment is still valid, before CoUninitialize() runs.
            self._resources = None
            self._props = None
            self.content = None
            if self._com_initialized:
                comtypes.CoUninitialize()
            return False

        def list_children(self, parent_id):
            """Returns the DIRECT children of `parent_id` as a list of dicts
            with keys id/name/is_folder/size - same shape as MTPSession's."""
            keys = _wpd_make_key_collection([
                WPD_OBJECT_NAME, WPD_OBJECT_ORIGINAL_FILE_NAME,
                WPD_OBJECT_CONTENT_TYPE, WPD_OBJECT_SIZE,
            ])
            items = []
            for object_id in self.content.EnumObjects(0, parent_id, None):
                values = self._props.GetValues(object_id, keys)
                name = _wpd_get_string(values, WPD_OBJECT_ORIGINAL_FILE_NAME) or _wpd_get_string(values, WPD_OBJECT_NAME)
                is_folder = _wpd_get_guid_str(values, WPD_OBJECT_CONTENT_TYPE) == str(WPD_CONTENT_TYPE_FOLDER)
                items.append({
                    "id": object_id, "name": name, "is_folder": is_folder,
                    "size": _wpd_get_u64(values, WPD_OBJECT_SIZE),
                })
            return items

        def find_child(self, parent_id, name):
            """Returns the id of the direct child of `parent_id` named `name`, or None."""
            for item in self.list_children(parent_id):
                if item["name"] == name:
                    return item["id"]
            return None

        def resolve_path(self, names, start_id=WPD_DEVICE_OBJECT_ID):
            """
            Walks a chain of folder names, returning the final folder's id,
            or None if any segment is missing.

            WPD inserts an extra storage node (e.g. "Internal shared
            storage") between the device root and its actual filesystem
            root that MTP/libmtp doesn't surface as a real folder - the
            same WAYPOINT_PATH that resolves directly against MTPSession's
            root would 404 on its very first segment ("Android") here.
            This storage node's WPD_OBJECT_CONTENT_TYPE is
            WPD_CONTENT_TYPE_STORAGE_CONTAINER, not WPD_CONTENT_TYPE_FOLDER,
            so it comes back from list_children() with is_folder=False even
            though it enumerates children exactly like a folder does - so
            this can't filter on is_folder to find it. If a segment isn't
            found as a direct child but the current level holds exactly one
            object overall, that's this storage node - transparently
            descend into it and retry the same segment once.
            """
            current = start_id
            for name in names:
                found = self.find_child(current, name)
                if found is None:
                    siblings = self.list_children(current)
                    if len(siblings) == 1:
                        found = self.find_child(siblings[0]["id"], name)
                if found is None:
                    return None
                current = found
            return current

        def pull_file(self, file_id, local_path):
            try:
                buf_size, stream = self._resources.GetStream(file_id, byref(WPD_RESOURCE_DEFAULT), _STGM_READ, 0)
                chunk = max(buf_size, 65536)
                with open(local_path, "wb") as out:
                    while True:
                        data, read = stream.RemoteRead(chunk)
                        if not read:
                            break
                        out.write(bytes(data)[:read])
                        if read < chunk:
                            break
                return True
            except Exception:
                return False

        def delete_object(self, object_id):
            try:
                pv, _buf = _wpd_propvariant_str(object_id)
                coll = _wpd_cc.CreateObject(
                    _WpdTypes.PortableDevicePropVariantCollection,
                    interface=_WpdApi.IPortableDevicePropVariantCollection,
                )
                coll.Add(byref(pv))
                self.content.Delete(0, coll, None)
                return True
            except Exception:
                return False

        def send_disguised_file(self, local_path, remote_filename, parent_folder_id,
                                 disguise_filetype=None, storage_id=0):
            """
            Pushes the bytes of `local_path`, landing inside
            `parent_folder_id` and named exactly `remote_filename`, with
            WPD_OBJECT_FORMAT set to `disguise_filetype` (default: the
            EXIF/JPEG format code) rather than left to whatever WPD would
            infer from the extension - matches MTPSession's disguise
            behavior for non-media files like .kmz. `storage_id` is
            accepted for call-signature compatibility with MTPSession but
            unused here (the destination is fully implied by
            `parent_folder_id`). Returns (success: bool, message: str).
            """
            if not os.path.exists(local_path):
                return False, f"Local file not found: {local_path}"
            disguise_format = disguise_filetype if disguise_filetype is not None else WPD_OBJECT_FORMAT_EXIF

            values = _wpd_make_values()
            values.SetStringValue(byref(WPD_OBJECT_PARENT_ID), parent_folder_id)
            values.SetStringValue(byref(WPD_OBJECT_NAME), remote_filename)
            values.SetStringValue(byref(WPD_OBJECT_ORIGINAL_FILE_NAME), remote_filename)
            values.SetUnsignedLargeIntegerValue(byref(WPD_OBJECT_SIZE), os.path.getsize(local_path))
            values.SetGuidValue(byref(WPD_OBJECT_FORMAT), disguise_format)

            try:
                stream, buf_size, _cookie = self.content.CreateObjectWithPropertiesAndData(values, 0, None)
            except Exception as e:
                return False, f"Send failed: {e}"

            chunk = max(buf_size, 65536)
            try:
                with open(local_path, "rb") as f:
                    while True:
                        data = f.read(chunk)
                        if not data:
                            break
                        written = 0
                        while written < len(data):
                            buf = (c_ubyte * (len(data) - written)).from_buffer_copy(data[written:])
                            n = stream.RemoteWrite(buf, len(buf))
                            if not n:
                                raise MTPBridgeError("Write stalled (0 bytes written)")
                            written += n
                stream.Commit(0)
                return True, f"Sent as '{remote_filename}'"
            except Exception as e:
                return False, f"Send failed: {e}"

    WPD_BRIDGE_AVAILABLE = True
except Exception:
    # DEBUG, not a warning: expected to fail on Mac/Linux (no comtypes
    # there) - by design, MTPSession covers those instead. See the same
    # note on the MTP bridge's except block above.
    logger.debug("WPD bridge unavailable", exc_info=True)
    WPD_BRIDGE_AVAILABLE = False


def get_mtp_session_class():
    """
    Picks the MTP backend for the current OS: WPDSession (native Windows
    Portable Devices - no driver replacement needed) on Windows, MTPSession
    (libmtp) elsewhere. Returns None if the platform's backend isn't
    available, in which case the DJI Fly Transfer tab's MTP features are
    disabled but the rest of the app is unaffected.
    """
    if platform.system() == "Windows":
        if not WPD_BRIDGE_AVAILABLE:
            logger.warning("WPD bridge unavailable on Windows - DJI Fly Transfer's MTP features are disabled. Run with logging.DEBUG (or see the WPD bridge's except block) for the underlying import/setup error.")
        return WPDSession if WPD_BRIDGE_AVAILABLE else None
    if not MTP_BRIDGE_AVAILABLE:
        logger.warning("libmtp MTP bridge unavailable on %s - DJI Fly Transfer's MTP features are disabled. Is libmtp installed?", platform.system())
    return MTPSession if MTP_BRIDGE_AVAILABLE else None


# ==========================================
# CONSTANTS & SETUP
# ==========================================
FT_TO_M = 0.3048
M_TO_FT = 3.28084
MPH_TO_MS = 0.44704
MS_TO_MPH = 2.23694

# ---------------------------------------------------------------------------
# Basemap tiles
# ---------------------------------------------------------------------------
# Esri's ArcGIS Online basemaps, which are free to use with attribution. These
# replaced mt1.google.com, which is Google's undocumented INTERNAL tile
# endpoint - it works, but using it outside the Maps Platform API is against
# their terms, which is a problem the moment this app is shared or published.
#
# Google's "lyrs=y" was a single combined satellite+labels hybrid layer. Esri
# splits those into two services, so the labels go on as a second transparent
# overlay to keep the same look. Esri also orders its path {z}/{y}/{x}, not
# the {z}/{x}/{y} most providers use.
#
# max_native_zoom is 19, NOT 20. Past 19 Esri serves an identical 2521-byte
# "no data" placeholder across most of the world - verified byte-for-byte at
# Provo and rural Nevada; only dense urban areas (e.g. Manhattan) carry real
# z20. Capping the native zoom at 19 makes Leaflet upscale those tiles for
# deeper zooms, which looks soft but is never blank.
ESRI_TILE_BASE = "https://server.arcgisonline.com/ArcGIS/rest/services"
ESRI_IMAGERY_URL = f"{ESRI_TILE_BASE}/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}"
ESRI_LABELS_URL = f"{ESRI_TILE_BASE}/Reference/World_Boundaries_and_Places/MapServer/tile/{{z}}/{{y}}/{{x}}"
ESRI_STREET_TILE_URL = ESRI_TILE_BASE + "/World_Street_Map/MapServer/tile/{z}/{y}/{x}"
ESRI_ATTR = ("Tiles &copy; Esri &mdash; Source: Esri, Maxar, Earthstar Geographics, "
             "and the GIS User Community")
BASEMAP_MAX_ZOOM = 22
BASEMAP_MAX_NATIVE_ZOOM = 19


def add_basemap(fmap):
    """
    Put the shared satellite basemap (imagery + place labels) on a folium map.

    Every map in the app goes through here so the tile source, attribution and
    zoom caps can't drift apart between the Creator, Editor and Viewer - they
    were three separately maintained copies of the same TileLayer line before.
    """
    folium.TileLayer(
        tiles=ESRI_IMAGERY_URL, attr=ESRI_ATTR, name="Satellite",
        max_zoom=BASEMAP_MAX_ZOOM, max_native_zoom=BASEMAP_MAX_NATIVE_ZOOM,
    ).add_to(fmap)
    # Labels ride on top of the imagery but still in Leaflet's tile pane, so
    # they stay underneath the flight path and waypoint markers.
    folium.TileLayer(
        tiles=ESRI_LABELS_URL, attr=ESRI_ATTR, name="Place labels",
        overlay=True, control=False,
        max_zoom=BASEMAP_MAX_ZOOM, max_native_zoom=BASEMAP_MAX_NATIVE_ZOOM,
    ).add_to(fmap)
    return fmap

MISSION_DIR = "missions"
SURFACES_DIR = "surfaces"
os.makedirs(MISSION_DIR, exist_ok=True)
os.makedirs(SURFACES_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Multi-user mode: per-session mission isolation
# ---------------------------------------------------------------------------
# A local `streamlit run app.py` is one pilot, one machine, one missions/
# folder they can also browse in Finder/Explorer - that has to keep working
# exactly as before. A shared deployment (e.g. a class on Streamlit Community
# Cloud) is a single running process serving every visitor at once, with no
# per-user filesystem at all: everyone was pointed at that same missions/
# folder, so any student could see, overwrite, or delete another student's
# saved missions, and the same for the shared .creator_presets.json.
#
# Off by default, so nothing changes for a local install. An instructor
# deploying this turns it on with one setting - either an environment
# variable (FLIGHT_PLANNER_MULTI_USER=1, works on any host) or the same key
# in .streamlit/secrets.toml (Streamlit Community Cloud's own "Secrets"
# panel, no shell access needed). SURFACES_DIR is deliberately NOT scoped
# this way - it holds GeoTIFFs the instructor pre-loads for everyone to read,
# never written to from inside the app, so there is nothing to isolate.
def _multi_user_mode():
    val = os.environ.get("FLIGHT_PLANNER_MULTI_USER")
    if val is None:
        try:
            val = st.secrets.get("FLIGHT_PLANNER_MULTI_USER")
        except Exception:
            val = None
    return str(val).strip().lower() in ("1", "true", "yes", "on")


MULTI_USER_MODE = _multi_user_mode()

if MULTI_USER_MODE:
    # session_state is what's genuinely per-browser-session in Streamlit (the
    # script itself re-runs top to bottom for every user), so the isolation
    # anchors on a random id stored there rather than anything filesystem- or
    # request-derived. Generated once and kept for the life of the tab.
    if "session_id" not in st.session_state:
        st.session_state.session_id = uuid.uuid4().hex[:10]
    MISSION_DIR = os.path.join(MISSION_DIR, "_sessions", st.session_state.session_id)
    os.makedirs(MISSION_DIR, exist_ok=True)

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

# Half of the sensor's vertical field of view (~27.9 deg here). The gimbal
# tilt is applied about the sensor's height axis, so at any pitch shallower
# than this the top edge of the frame points at or above the horizon and the
# footprint has no finite ground projection - its far edge runs off to
# infinity. Derived from the optics rather than hard-coded so it stays
# correct if the sensor constants above are ever changed for another camera.
VERT_HALF_FOV_DEG = math.degrees(math.atan((SENSOR_H / 2.0) / FOCAL_L))

# Shallowest gimbal tilt an area-mapping mission will accept. The bare
# half-FOV is an asymptote - the footprint diverges as it is approached, so
# overlap/spacing derived from it become meaningless long before it - so keep
# a margin that still leaves a finite, usable footprint.
MIN_MAPPING_PITCH_DEG = VERT_HALF_FOV_DEG + 5.0

# How the camera may be aimed on an area-mapping mission, relative to the
# direction of travel.
#   "parallel" - camera looks along the flight line. The tilt then falls in
#     the along-track plane, so the imaged strip stays centred on the drone
#     and every pass runs straight down its strip: the computed path keeps
#     the shape of the drawn area instead of being pushed sideways. It also
#     puts the sensor's long axis across-track, widening the swath by
#     SENSOR_W/SENSOR_H (~33% here) for proportionally fewer flight lines.
#   "right"/"left" - camera looks across the flight line to that side, the
#     corridor-mission aim. The tilt now falls across-track, so the imaged
#     strip sits off to one side and the flight lines have to be offset to
#     compensate - which, because the offset flips with each direction
#     change, visibly distorts the path away from the drawn shape.
MAPPING_CAMERA_SIDES = ["parallel", "right", "left"]

# Smallest drawn area worth sweeping, in square feet. Anything at or below this
# is a degenerate shape rather than a small one - duplicate vertices, or points
# that all fall on a line - and has no interior to cover. Such a shape used to
# yield a two-waypoint "mission" that looked valid enough to save but had no
# flyable path in it. A genuinely small plot is still allowed: a 3 ft square is
# well clear of this and simply comes out as one short pass.
MIN_MAPPABLE_AREA_FT2 = 1.0


def mapping_yaw_mode(side):
    """Which footprint geometry a mapping camera side implies."""
    return "forward" if side == "parallel" else "perpendicular"

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
    """Kills macOS background apps that lock the MTP port. No-op on other platforms."""
    if platform.system() != "Darwin":
        return
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

    Returns (nests, preview_id, error). error is None for the everyday
    "nothing to report" cases (no bridge on this platform's install, no
    device plugged in, DJI Fly's folder layout not found) - those aren't
    bugs and the caller's existing "no controller connected" messaging
    already covers them. It's a message string only when something
    unexpected happened, so the caller can show specifically what broke
    instead of the same generic message for both cases; either way, the
    real exception (if any) is always logged with a traceback so it's
    visible in the terminal streamlit was launched from.
    """
    kill_macos_hijackers()
    os.makedirs(cache_dir, exist_ok=True)

    SessionClass = get_mtp_session_class()
    if SessionClass is None:
        return {}, None, "MTP bridge unavailable on this platform (Windows: is comtypes installed? Other platforms: is libmtp installed? - see the terminal log for the underlying error)."

    try:
        with SessionClass() as session:
            waypoint_id = session.resolve_path(WAYPOINT_PATH)
            if waypoint_id is None:
                return {}, None, None

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
                return nests, preview_id, None

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

            return nests, preview_id, None
    except MTPBridgeError as e:
        # "No MTP device detected" is the everyday case (nothing plugged
        # in, or not yet recognized) - not worth alarming the user with.
        # Anything else from this exception type (e.g. a device WAS found
        # but opening a session with it failed) is unexpected.
        message = str(e)
        if "No MTP device detected" in message:
            return {}, None, None
        logger.warning("MTP/WPD bridge error while scanning controller: %s", message)
        return {}, None, message
    except Exception as e:
        logger.exception("Unexpected error while scanning controller for DJI Fly missions")
        return {}, None, f"{type(e).__name__}: {e}"

def push_mission_to_nest(local_kmz_path, target_uuid):
    """
    Pushes a local KMZ (and its paired JPG thumbnail, if present) to an
    existing dummy mission slot on the RC 2, replacing whatever's there.
    Runs the whole operation - locating folders, purging old files, pushing
    new ones - over a single continuous device connection rather than many
    separate `mtp-*` process invocations, which avoids both the slow
    full-device enumeration those tools do and the device's tendency to
    renumber object IDs between separate connections.

    Also garbage-collects the shared map_preview folder of stale thumbnails
    (this mission's old one, plus any orphaned from since-removed dummy
    slots) each time it's called, since that shared pool is otherwise never
    revisited and DJI Fly's own thumbnail cache appears to occasionally
    mismatch a mission to a neighboring file once it's grown large.
    """
    kill_macos_hijackers()

    SessionClass = get_mtp_session_class()
    if SessionClass is None:
        return False, "MTP bridge unavailable (Windows: is comtypes installed? Other platforms: is libmtp installed?)."

    try:
        with SessionClass() as session:
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
                # The shared map_preview folder holds one flat thumbnail per
                # mission ever pushed, but nothing else in the app ever
                # revisits it, so files belonging to OTHER dummy mission
                # slots - including slots that don't even exist as a
                # waypoint folder anymore - pile up here indefinitely. DJI
                # Fly's own on-controller thumbnail cache reads from this
                # same shared folder, and unlike our own exact-uuid lookups
                # (fetch_controller_nests_and_previews), whatever resolution
                # logic it uses internally is opaque and appears to
                # sometimes grab a neighboring file instead of the intended
                # one - most likely right after a bulk transfer touches many
                # of these in quick succession. We can't fix DJI Fly's own
                # logic, so instead shrink the pool of stale candidates it
                # has to pick from: delete the current mission's old
                # thumbnail (about to be replaced) plus any orphaned one
                # left over from a since-removed dummy slot.
                # Compared case-insensitively since UUID_RE (and the device
                # itself) don't guarantee consistent casing between a
                # waypoint folder's own name and the filenames we pushed
                # against it.
                valid_uuids = {it["name"].upper() for it in waypoint_children if it["is_folder"] and it["name"] != "map_preview"}
                for item in session.list_children(map_preview_id):
                    if item["is_folder"]:
                        continue
                    name, ext = os.path.splitext(item["name"])
                    if ext.lower() != ".jpg" or not UUID_RE.match(name):
                        continue
                    if name.upper() == target_uuid.upper() or name.upper() not in valid_uuids:
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
            # format code that .kmz files fall into. Both session backends
            # (MTPSession/libmtp on macOS/Linux, WPDSession on Windows) talk
            # to the device directly so the destination filename and format
            # code can be set independently (the KMZ is sent disguised as a
            # JPEG to get past the rejection; the JPG thumbnail is already
            # a real JPEG so no disguise is needed).
            if platform.system() == "Darwin":
                subprocess.run(f'xattr -c {shlex.quote(local_kmz_path)}', shell=True)
            remote_kmz = f"{target_uuid}.kmz"
            ok_kmz, msg_kmz = session.send_disguised_file(local_kmz_path, remote_kmz, target_folder_id)
            if not ok_kmz:
                return False, f"KMZ transfer failed: {msg_kmz}"

            local_jpg_path = kmz_companion_path(local_kmz_path)
            if os.path.exists(local_jpg_path):
                if platform.system() == "Darwin":
                    subprocess.run(f'xattr -c {shlex.quote(local_jpg_path)}', shell=True)
                remote_jpg = f"{target_uuid}.jpg"

                session.send_disguised_file(local_jpg_path, remote_jpg, target_folder_id)
                if map_preview_id:
                    session.send_disguised_file(local_jpg_path, remote_jpg, map_preview_id)
                if preview_subfolder_id:
                    session.send_disguised_file(local_jpg_path, remote_jpg, preview_subfolder_id)

            return True, "Success. (Reminder: Ensure DJI Fly is closed on the RC 2 before opening!)"
    except MTPBridgeError as e:
        logger.warning("MTP/WPD bridge error while pushing %s to nest %s: %s", local_kmz_path, target_uuid, e)
        return False, str(e)
    except Exception as e:
        # Previously uncaught here - any exception besides MTPBridgeError
        # (a COM error mid-transfer, a bad file path, etc.) would propagate
        # out of this function and crash the whole Streamlit script run,
        # aborting a batch transfer with no clean error for the remaining
        # missions in the batch. Logged with a traceback and reported back
        # as a normal failure instead.
        logger.exception("Unexpected error while pushing %s to nest %s", local_kmz_path, target_uuid)
        return False, f"Unexpected error: {type(e).__name__}: {e}"

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

def offer_kmz_download(container, scope, filepath=None, filename=None):
    """
    "Download KMZ" button for the most recently saved mission in one Creator/
    Editor flow, so the file actually leaves the server rather than only
    existing in a folder no one but the app itself can reach.

    Only renders in MULTI_USER_MODE. Locally, the save folder IS the user's
    own missions/ directory - they're already looking straight at the file in
    Finder/Explorer, so a second copy offered through the browser's download
    flow is pure redundancy. In multi-user mode it's the only way a saved
    mission gets onto a student's own computer at all - there's no
    filesystem for them to browse to, and once their tab closes the
    session's mission folder is gone for good (see MULTI_USER_MODE above
    generate_mapping_flight_path's session_id block). A no-op call is cheap
    enough that every save flow can call this unconditionally rather than
    each needing its own MULTI_USER_MODE check.

    Pass filepath/filename right after a successful save to remember it;
    call with just (container, scope) on every other render to redraw the
    same button. That split is needed because st.button/this whole save
    branch only evaluates True on the exact rerun the click happened on - a
    download button placed only inside that branch would vanish again the
    instant the user touched anything else, which defeats the purpose for a
    student who saves a mission and then, say, nudges the altitude field
    before remembering to grab the file.
    """
    if not MULTI_USER_MODE:
        return
    key = f"_last_saved_kmz_{scope}"
    if filepath is not None:
        st.session_state[key] = {"path": filepath, "name": filename}
    saved = st.session_state.get(key)
    if saved and os.path.exists(saved["path"]):
        with open(saved["path"], "rb") as f:
            container.download_button(
                "⬇️ Download KMZ", f.read(), file_name=saved["name"],
                mime="application/vnd.google-earth.kmz", key=f"dl_{scope}",
                width='stretch',
            )


def is_kmz_file(filename):
    """
    Whether `filename` is a .kmz, regardless of how the extension is cased.

    Windows filesystems are case-insensitive, so a mission that has been round
    -tripped through an SD card, a controller, or an email can come back named
    ".KMZ" and is the same file as far as the OS is concerned. A case-sensitive
    endswith(".kmz") skipped those silently: the file sat in the folder and
    opened fine in Explorer, but never appeared in any mission list and gave no
    error saying why. Files this app writes are always lowercase, so this only
    widens what gets picked up - it never changes what gets saved.
    """
    return filename.lower().endswith(".kmz")


def kmz_companion_path(kmz_path, new_ext=".jpg"):
    """
    Path of the file paired with a mission - its thumbnail unless told
    otherwise - derived from the mission's own path.

    Uses splitext rather than replace('.kmz', ...) so it strips a ".KMZ" just
    as happily as a ".kmz", and so it can only ever rewrite the extension:
    a plain replace also rewrites any earlier ".kmz" occurrence in the path,
    which a parent folder named e.g. "old.kmz backups" would trigger.
    """
    return os.path.splitext(kmz_path)[0] + new_ext


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

class ElevationLookupError(Exception):
    """Raised when NONE of a batch's points could get a real elevation
    reading from any source - see get_elevations_batch."""
    pass

# All three fetchers below use None (never 0) to mark a failed/missing
# reading. 0 is a legitimate elevation (sea level, and a fair amount of
# low-lying US terrain), so using it as a failure sentinel too - as this
# code used to - makes a failed lookup indistinguishable from "this point
# really is at sea level." That matters a lot here: elevation differences
# between waypoints get baked directly into each waypoint's commanded
# altitude (see generate_native_kmz_contents), so one silently-wrong 0
# among otherwise-correct hilly-terrain readings shows up as a real,
# uncommanded altitude cliff in the flight path at that exact waypoint.
# get_elevations_batch is where every caller actually gets its elevations
# from - it fills any None here with a nearby real reading before
# returning, so downstream code never has to check for None itself.

@st.cache_data(show_spinner=False, ttl=3600)
def get_elevations_open_elev(coords):
    url = "https://api.open-elevation.com/api/v1/lookup"
    locations = [{"latitude": lat, "longitude": lon} for lat, lon in coords]
    data = json.dumps({"locations": locations}).encode('utf-8')
    try:
        req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json', 'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req) as response:
            res_json = json.loads(response.read().decode('utf-8'))
            results = res_json['results']
            return [r.get('elevation') for r in results]
    except Exception as e:
        st.warning(f"Open-Elevation error: {e}")
        return [None] * len(coords)

@st.cache_data(show_spinner=False, ttl=3600)
def get_elevations_usgs(coords):
    elevations = []
    for lat, lon in coords:
        url = f"https://epqs.nationalmap.gov/v1/json?x={lon}&y={lat}&wkid=4326&units=Meters&includeDate=false"
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req) as response:
                res_json = json.loads(response.read().decode('utf-8'))
                val = res_json.get('value')
                elevations.append(None if val is None or str(val).lower() == 'null' or str(val).strip() == '' else float(val))
        except Exception as e:
            elevations.append(None)
    return elevations

@st.cache_data(show_spinner=False, ttl=3600)
def get_elevations_raster(coords, tif_path):
    if not RASTERIO_AVAILABLE:
        return [None] * len(coords)
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
                    elevations.append(None)
                else:
                    elevations.append(v)
    except Exception as e:
        st.warning(f"Error reading GeoTIFF: {e}")
        return [None] * len(coords)
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

def latlon_to_global_px(lat, lon, zoom, tile_size=256):
    """Web Mercator lat/lon -> pixel coordinates on the full world map at a given zoom."""
    lat_rad = math.radians(max(min(lat, 85.05112878), -85.05112878))
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n * tile_size
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n * tile_size
    return x, y

@st.cache_data(show_spinner=False, ttl=3600)
def fetch_basemap_image(min_lat, min_lon, max_lat, max_lon, target_px=420):
    """
    Fetches a Google roadmap tile mosaic cropped exactly to the given
    lat/lon bounding box, for use as street context behind a thumbnail's
    flight-path drawing. Returns None (caller falls back to a plain
    background) if the network is unavailable or the fetch fails.
    """
    tile_size = 256
    lat0 = (min_lat + max_lat) / 2
    lon0 = (min_lon + max_lon) / 2
    width_m = get_haversine_dist((lat0, min_lon), (lat0, max_lon))
    height_m = get_haversine_dist((min_lat, lon0), (max_lat, lon0))
    span_m = max(width_m, height_m, 1e-6)

    meters_per_px = 156543.03392 * math.cos(math.radians(lat0))
    zoom = int(round(math.log2(meters_per_px * target_px / span_m)))
    # Clamped to the basemap's real coverage: past z19 Esri returns a "no
    # data" placeholder, which would paste a grey grid behind the thumbnail.
    zoom = max(3, min(BASEMAP_MAX_NATIVE_ZOOM, zoom))

    x0, y0 = latlon_to_global_px(max_lat, min_lon, zoom, tile_size)
    x1, y1 = latlon_to_global_px(min_lat, max_lon, zoom, tile_size)
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))

    tile_x0, tile_x1 = int(x0 // tile_size), int(x1 // tile_size)
    tile_y0, tile_y1 = int(y0 // tile_size), int(y1 // tile_size)

    mosaic = Image.new('RGB', ((tile_x1 - tile_x0 + 1) * tile_size, (tile_y1 - tile_y0 + 1) * tile_size), '#e8e8e0')

    try:
        for tx in range(tile_x0, tile_x1 + 1):
            for ty in range(tile_y0, tile_y1 + 1):
                url = ESRI_STREET_TILE_URL.format(z=zoom, x=tx, y=ty)
                req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
                with urllib.request.urlopen(req, timeout=5) as response:
                    tile_img = Image.open(io.BytesIO(response.read())).convert('RGB')
                mosaic.paste(tile_img, ((tx - tile_x0) * tile_size, (ty - tile_y0) * tile_size))
    except Exception:
        return None

    crop_box = (
        int(x0 - tile_x0 * tile_size), int(y0 - tile_y0 * tile_size),
        int(x1 - tile_x0 * tile_size), int(y1 - tile_y0 * tile_size),
    )
    return mosaic.crop(crop_box)

def generate_name_thumbnail(mission_name, alt_ft, pitch, overlap_pct, output_filepath, coords=None, photo_count=None):
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

    # The controller's own preview display crops roughly the outer 7% off
    # every edge of these thumbnails, so all content is kept within a safe
    # margin well inside that crop zone rather than using the full canvas.
    MARGIN = 0.10

    # --- LEFT: name + flight parameters ---
    # Wrap the name so it doesn't run off the edges. Underscores are treated
    # as spaces here so textwrap can break at word boundaries instead of
    # hard-splitting mid-word (e.g. "Fly_Mission_Flight").
    display_name = mission_name.replace('_', ' ')
    wrapped_name = "\n".join(textwrap.wrap(display_name, width=11)) or display_name

    left_x = MARGIN + (1 - 2 * MARGIN) * 0.28

    ax.text(left_x, 1 - MARGIN - (1 - 2 * MARGIN) * 0.20, wrapped_name,
            color=get_altitude_color(alt_ft),
            fontsize=38,
            ha='center',
            va='center',
            weight='bold',
            transform=ax.transAxes)

    photo_line = f"Photos: {photo_count:.0f}\n" if photo_count is not None else ""
    info_text = f"{photo_line}H: {alt_ft:.0f} ft\nA: {abs(pitch):.0f}°\nOL: {overlap_pct:.0f}%"
    ax.text(left_x, MARGIN + (1 - 2 * MARGIN) * 0.20, info_text,
            color='#FFFFFF',
            fontsize=24,
            ha='center',
            va='center',
            linespacing=1.6,
            transform=ax.transAxes)

    # --- RIGHT: flight path drawing in a thin bordered box ---
    if coords and len(coords) >= 2:
        box_size_in = 1.75
        box_cx_in = fig_w * (MARGIN + (1 - 2 * MARGIN) * 0.74)
        box_cy_in = fig_h * 0.52
        box_x0_frac = (box_cx_in - box_size_in / 2) / fig_w
        box_y0_frac = (box_cy_in - box_size_in / 2) / fig_h
        box_w_frac = box_size_in / fig_w
        box_h_frac = box_size_in / fig_h

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

        # The box shows this exact same square region of ground the path was
        # just scaled into, converted back to lat/lon, so a fetched basemap
        # tile lines up pixel-for-pixel with the drawn path on top of it.
        box_extent_m = box_size_in / scale
        box_min_lon = lons[0] + (mid_x - box_extent_m / 2) / math.cos(math.radians(lat0))
        box_max_lon = lons[0] + (mid_x + box_extent_m / 2) / math.cos(math.radians(lat0))
        box_min_lat = lats[0] + (mid_y - box_extent_m / 2)
        box_max_lat = lats[0] + (mid_y + box_extent_m / 2)

        basemap_img = fetch_basemap_image(box_min_lat, box_min_lon, box_max_lat, box_max_lon)
        if basemap_img is not None:
            ax.imshow(
                basemap_img, extent=[box_x0_frac, box_x0_frac + box_w_frac, box_y0_frac, box_y0_frac + box_h_frac],
                transform=ax.transAxes, aspect='auto', zorder=1,
            )

        ax.add_patch(plt.Rectangle(
            (box_x0_frac, box_y0_frac), box_w_frac, box_h_frac,
            transform=ax.transAxes, facecolor='none', edgecolor='#FFFFFF', linewidth=1.2, zorder=2,
        ))

        path_color = get_altitude_color(alt_ft)
        ax.plot(
            xs_frac, ys_frac, color=path_color, linewidth=1.8, transform=ax.transAxes, zorder=3,
            path_effects=[pe.Stroke(linewidth=3.4, foreground='#000000', alpha=0.55), pe.Normal()],
        )

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

def _fill_missing_elevations(coords, elevations):
    """
    Replaces each failed (None) elevation with the reading from the
    nearest OTHER point in this same batch that did succeed - "nearby" in
    the literal geographic sense, via haversine distance - rather than
    guessing. Returns (filled, num_missing); num_missing == len(coords)
    means not a single point in the batch got a real reading, so there was
    nothing to fall back on at all.
    """
    valid_idxs = [i for i, e in enumerate(elevations) if e is not None]
    if not valid_idxs:
        return elevations, len(elevations)

    filled = list(elevations)
    num_missing = 0
    for i, e in enumerate(elevations):
        if e is not None:
            continue
        num_missing += 1
        nearest_idx = min(valid_idxs, key=lambda j: get_haversine_dist(coords[i], coords[j]))
        filled[i] = elevations[nearest_idx]
    return filled, num_missing

def get_elevations_batch(coords, source, tif_path=None):
    if not coords:
        return []
    if source == "USGS 3DEP (US High-Res)":
        raw = get_elevations_usgs(coords)
    elif source == "Local GeoTIFF" and tif_path and os.path.exists(tif_path):
        raw = get_elevations_raster(coords, tif_path)
    else:
        raw = get_elevations_open_elev(coords)

    filled, num_missing = _fill_missing_elevations(coords, raw)
    if num_missing == len(coords):
        raise ElevationLookupError(
            f"Could not get elevation data from {source} for any point in this mission. "
            "Check your network connection (or the GeoTIFF path/coverage) and try again."
        )
    if num_missing:
        st.warning(f"{num_missing} waypoint(s) had no {source} elevation reading - used the nearest successful nearby reading instead.")
    return filled

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

def default_group_folder_name(index, group):
    """The auto-generated folder name for one group - shared by the fully
    automatic sort and as the pre-filled default when naming groups by hand,
    so a name left untouched in the review UI produces the identical folder
    name the automatic path would have used."""
    group_start_datetime = group[0]['time'].strftime("%Y-%m-%d_%H-%M-%S")
    return f"Group_{index + 1}_{group_start_datetime}"


def find_photo_groups(source_folder, target_date, gap_minutes=5):
    """
    Scans source_folder for images taken on target_date and splits them into
    groups wherever the gap between two sequential photos exceeds
    gap_minutes. Read-only - nothing is copied or created on disk here, so
    this can run on its own as a preview step before the user decides how
    (or whether) to name each group.
    """
    valid_extensions = {'.jpg', '.jpeg', '.png', '.tif', '.tiff'}

    image_data = []
    try:
        files = os.listdir(source_folder)
    except Exception as e:
        st.error(f"Error accessing source directory: {e}")
        return []

    progress_bar = st.progress(0, text="Scanning files for EXIF data...")

    for i, filename in enumerate(files):
        ext = os.path.splitext(filename)[1].lower()
        if ext in valid_extensions:
            filepath = os.path.join(source_folder, filename)
            taken_time = get_exif_datetime(filepath)

            if taken_time and taken_time.date() == target_date:
                image_data.append({'path': filepath, 'name': filename, 'time': taken_time})

        progress_bar.progress((i + 1) / len(files), text=f"Scanning files... ({i+1}/{len(files)})")

    progress_bar.empty()

    if not image_data:
        st.warning(f"No images found for {target_date.strftime('%Y-%m-%d')} in the source folder.")
        return []

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

    st.info(f"Found {len(groups)} distinct flight groups.")
    return groups


def copy_photo_groups(groups, output_folder, group_names=None):
    """
    Copies each group's photos into its own folder under output_folder.
    group_names, if given, supplies one folder name per group (already
    sanitized/deduped by the caller) - any entry that's falsy falls back to
    that group's default auto-generated name, same as the fully automatic
    path uses for all of them.
    """
    os.makedirs(output_folder, exist_ok=True)

    copy_progress = st.progress(0, text="Copying images to group folders...")
    total_images = sum(len(g) for g in groups)
    copied = 0

    for i, group in enumerate(groups):
        folder_name = (group_names[i] if group_names else None) or default_group_folder_name(i, group)
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
    """
    Ground footprint of one photo as a list of [lat, lon] corners, or None
    if the camera is too shallow for all four corner rays to reach the
    ground (see VERT_HALF_FOV_DEG). Callers must skip drawing on None -
    there is no meaningful polygon to show at that tilt.
    """
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
        # ray[2] >= 0 means this corner looks at or above the horizon and
        # never meets the ground. Projecting it anyway gives a negative t -
        # a point BEHIND the camera - which folds the quad into a bowtie
        # whose drawn area *shrinks* as the camera flattens, when physically
        # it should grow without bound. Bail out so callers skip drawing
        # rather than render a silently wrong footprint. (project_footprint_ft
        # already guards this the same way; this function was missed.)
        if ray[2] >= 0:
            return None
        t = -alt_ft / ray[2]
        dx_ft, dy_ft = ray[0] * t, ray[1] * t
        dlat = math.degrees(dy_ft / R_earth_ft)
        dlon = math.degrees(dx_ft / (R_earth_ft * math.cos(math.radians(lat))))
        final_corners.append([lat + dlat, lon + dlon])
    return final_corners

def interpolate_path(coords, gap_m, return_frac=False):
    """
    Breaks down a corner-based path into physical waypoints for DJI Fly
    compatibility, spaced at gap_m along the path's total cumulative distance
    so photo cadence is set by the whole route, not reset at each bend. Every
    original vertex - not just the first and last - is also always kept as an
    exact point, snapping out any interval-spaced point that landed within
    gap_m/10 of one.

    That corner precision matters beyond just fidelity to the drawn path: on
    a mapping mission, a vertex is exactly where one pass ends and the turn
    to the next begins. The old distance-only walk had no reason to land a
    point there, so the photo nearest a strip's edge often fell some
    distance short of it, leaving a sliver of the area right at the boundary
    with less coverage than the rest of the strip. Anchoring a point to every
    corner fixes that regardless of how the interval happens to phase
    against the path's bends.

    When return_frac=True, also returns a parallel list of (segment_index,
    frac) tuples describing each output point's position along the original
    path - used to interpolate other per-waypoint values (like elevation)
    consistently with the same points, without having to re-query them for
    every densified waypoint. A point sitting exactly on vertex i (other than
    the last) is recorded as (i, 0.0) - the start of the segment leaving that
    vertex - except the final vertex, which has no outgoing segment and is
    recorded as (len(coords)-2, 1.0) instead.
    """
    if gap_m <= 0 or len(coords) < 2:
        if return_frac:
            return coords, [(i, 0.0) for i in range(len(coords))]
        return coords

    cum_dist = [0.0]
    total_dist_m = 0.0
    for i in range(len(coords) - 1):
        total_dist_m += get_haversine_dist(coords[i], coords[i+1])
        cum_dist.append(total_dist_m)

    # Natural interval points, exactly as before.
    interval_points = []  # (distance, seg_idx, frac, lat, lon)
    target_dist = gap_m
    while target_dist <= total_dist_m + 0.001:
        for i in range(len(cum_dist) - 1):
            if cum_dist[i] <= target_dist <= cum_dist[i+1] + 0.001:
                seg_len = cum_dist[i+1] - cum_dist[i]
                if seg_len > 0:
                    frac = (target_dist - cum_dist[i]) / seg_len
                    lat = coords[i][0] + (coords[i+1][0] - coords[i][0]) * frac
                    lon = coords[i][1] + (coords[i+1][1] - coords[i][1]) * frac
                    interval_points.append((target_dist, i, frac, lat, lon))
                break
        target_dist += gap_m

    # Every original vertex, exactly.
    corner_points = []  # same shape as interval_points
    for i, (lat, lon) in enumerate(coords):
        if i < len(coords) - 1:
            corner_points.append((cum_dist[i], i, 0.0, lat, lon))
        else:
            corner_points.append((cum_dist[i], len(coords) - 2, 1.0, lat, lon))

    snap_tol = gap_m * 0.1
    merged = corner_points + [
        p for p in interval_points
        if not any(abs(p[0] - c[0]) <= snap_tol for c in corner_points)
    ]
    merged.sort(key=lambda p: p[0])

    dense_coords = [(p[3], p[4]) for p in merged]
    fracs = [(p[1], p[2]) for p in merged]

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
# MAPPING MISSION (AREA COVERAGE) PATH GENERATOR
# ==========================================
FT_PER_DEG_LAT = 111320 * M_TO_FT  # ~365,223 ft per degree of latitude

def project_footprint_ft(alt_ft, pitch, yaw_deg):
    """
    Projects the four camera-sensor corners onto flat ground and returns
    them as (east_ft, north_ft) offsets from the nadir point. This is the
    same pinhole/rotation math as get_photo_footprint (the footprints drawn
    in the Viewer), just in a local planar frame instead of lat/lon, so any
    overlap derived from it matches what the app actually images. Returns
    None if the camera is too shallow for the rays to reach the ground.
    """
    w, h, f = SENSOR_W, SENSOR_H, FOCAL_L
    corners = [(-w/2, h/2, -f), (w/2, h/2, -f), (w/2, -h/2, -f), (-w/2, -h/2, -f)]
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch + 90)

    Rz = [[math.cos(yaw_rad), math.sin(yaw_rad), 0],
          [-math.sin(yaw_rad), math.cos(yaw_rad), 0], [0, 0, 1]]
    Rx = [[1, 0, 0],
          [0, math.cos(pitch_rad), -math.sin(pitch_rad)], [0, math.sin(pitch_rad), math.cos(pitch_rad)]]
    R = [[sum(Rz[r][k] * Rx[k][col] for k in range(3)) for col in range(3)] for r in range(3)]

    pts = []
    for corner in corners:
        ray = [sum(R[r][k] * corner[k] for k in range(3)) for r in range(3)]
        if ray[2] >= 0:
            return None  # ray points at or above the horizon - never hits ground
        t = -alt_ft / ray[2]
        pts.append((ray[0] * t, ray[1] * t))
    return pts

def _min_section_width_ft(pts, samples=33):
    """
    Narrowest second-axis (y) width of a footprint polygon, measured across its
    first-axis (x) span. Callers hand the points in already so that x is the
    axis the gimbal tilt runs along; pass them transposed to measure the other
    way round.

    The footprint is a clean rectangle only at nadir; any gimbal tilt turns it
    into a trapezoid that tapers toward the near edge. Spacing flight lines by
    the WIDEST part of that trapezoid leaves unimaged wedges between adjacent
    strips, so line spacing has to come from the narrowest part instead - in
    effect the width of the largest rectangle that fits inside the footprint.
    Returns the same value as the bounding box at nadir, and progressively
    tighter spacing as the camera tilts.

    Both ends of the span are probed as well as the interior samples: a
    trapezoid always reaches its narrowest at the near parallel edge, so
    sampling only at sample-cell centres (as this did) never actually visited
    the minimum and over-reported the usable width by up to ~5% at steep tilt,
    which quietly ate into the requested side overlap. The end probes sit a
    hair inside the span rather than exactly on it: the projection leaves the
    two corners of an edge differing by ~1e-14 ft, enough that a probe placed
    exactly at the extreme falls outside one of the two adjoining edges and
    finds a single crossing instead of a pair.
    """
    xs = [p[0] for p in pts]
    lo, hi = min(xs), max(xs)
    if hi - lo < 1e-9:
        return 0.0
    n = len(pts)
    inset = (hi - lo) * 1e-6
    probes = [lo + inset, hi - inset] + [lo + (hi - lo) * (k + 0.5) / samples for k in range(samples)]
    narrowest = float('inf')
    for x in probes:
        crossings = []
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            if abs(x2 - x1) < 1e-12:
                continue
            if (x1 - x) * (x2 - x) <= 0:
                t = (x - x1) / (x2 - x1)
                crossings.append(y1 + t * (y2 - y1))
        if len(crossings) >= 2:
            narrowest = min(narrowest, max(crossings) - min(crossings))
    return 0.0 if narrowest == float('inf') else narrowest


def footprint_extents_ft(alt_ft, pitch, side="right", yaw_mode="perpendicular"):
    """
    True ground-footprint dimensions (feet) for the mission's camera
    orientation, derived by projecting the sensor corners:
    - along_ft: extent parallel to the flight line (drives frontal overlap)
    - cross_ft: extent perpendicular to it (drives side overlap / swath)
    - offset_ft: how far the footprint's cross-track center sits from the
      point directly below the drone (0 at nadir; grows with tilt)
    - along_offset_ft: the same thing along-track, signed in the direction of
      travel (0 at nadir; positive means the camera images ground ahead of
      the drone rather than beneath it)
    Computed with the flight line running east so along = |x|, cross = |y|.

    yaw_mode picks how the camera is aimed relative to the direction of
    travel, and must match what generate_native_kmz_contents actually writes:
    - "forward": camera aimed along the direction of travel ("parallel" in the
      UI). Puts the sensor's LONG axis across-track - a 33% wider swath for
      the same altitude, so proportionally fewer flight lines and photos. It
      also keeps any tilt in the along-track plane, so the cross-track offset
      stays 0 and the drone flies straight down each strip's centerline.
    - "perpendicular": camera yawed 90 deg off the flight line, tilting to
      the chosen side. The corridor/line-mission aim, where the point is to
      image something beside the flight path.

    Tilt stretches the footprint into a trapezoid ALONG THE AXIS IT TILTS
    ABOUT, so which axis is which swaps between the two modes, and the usable
    rectangle has to be measured accordingly: it spans the full extent along
    the tilt axis, and is as wide as the narrowest section across it.
    Measuring both from the x axis regardless (as this did) was right only for
    "forward". In "perpendicular" it read the trapezoid's pinched corners as
    the swath - near zero, collapsing line spacing to the 1 ft floor - and
    took the widest (far) edge as the along-track extent instead of the usable
    narrow one, so realized frontal overlap fell well short of the request and
    went negative, i.e. left gaps, past roughly -45 deg.
    """
    if yaw_mode == "forward":
        yaw = 90.0  # flight line runs east, so "forward" is east
    else:
        yaw = 180.0 if side == "right" else 0.0
    pts = project_footprint_ft(alt_ft, pitch, yaw)
    if pts is None:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if yaw_mode == "forward":
        # Tilt runs along-track: parallel edges lie across-track.
        along_ft = max(xs) - min(xs)
        cross_ft = _min_section_width_ft(pts)
    else:
        # Tilt runs across-track: the trapezoid is the same shape turned 90
        # deg, so the two axes trade roles.
        cross_ft = max(ys) - min(ys)
        along_ft = _min_section_width_ft([(y, x) for x, y in pts])
    offset_ft = abs((max(ys) + min(ys)) / 2.0)
    along_offset_ft = (max(xs) + min(xs)) / 2.0
    return along_ft, cross_ft, offset_ft, along_offset_ft

def mapping_camera_geometry(alt_ft, pitch, frontal_overlap_pct, side_overlap_pct, side="parallel"):
    """
    Camera coverage geometry for area-mapping missions, all in feet. Overlap
    is based on the TRUE projected ground footprint (footprint_extents_ft),
    not a center-slant approximation, so the requested frontal/side overlap
    is the overlap the images actually get at any gimbal angle:
    - interval_ft: photo spacing along each pass (from frontal overlap)
    - spacing_ft: distance between adjacent passes (from side overlap)
    - offset_ft: horizontal distance from the drone to the center of what
      the camera actually images - the flight lines are shifted by this so
      the *imaged* strips, not the drone itself, line up over the area.
      Always 0 for the "parallel" camera side, since aiming the camera along
      the direction of travel keeps any tilt in the along-track plane; that
      is what lets the computed path keep the drawn area's shape.
    - along_offset_ft: the along-track counterpart, non-zero only for
      "parallel" with tilt (the camera then images ground ahead of the drone)
    Pitch shallower than MIN_MAPPING_PITCH_DEG is clamped - the ground
    footprint stretches toward infinity as the camera approaches the
    horizon, so overlap and spacing derived from it stop meaning anything.
    """
    yaw_mode = mapping_yaw_mode(side)
    clamped_pitch = -min(90.0, max(MIN_MAPPING_PITCH_DEG, abs(pitch)))
    extents = footprint_extents_ft(alt_ft, clamped_pitch, side, yaw_mode)
    if extents is None:
        # Defensive only: MIN_MAPPING_PITCH_DEG is derived to sit above the
        # horizon threshold, so the clamp above should always project. Fall
        # back to nadir, which is valid at every altitude, rather than to
        # another shallow angle - the previous fallback here retried at a
        # pitch that was itself below the threshold, so it returned None
        # again and the unpack below raised TypeError.
        extents = footprint_extents_ft(alt_ft, -90.0, side, yaw_mode)
    footprint_w_ft, footprint_h_ft, offset_ft, along_offset_ft = extents
    return {
        "footprint_w_ft": footprint_w_ft,
        "footprint_h_ft": footprint_h_ft,
        "offset_ft": offset_ft,
        "along_offset_ft": along_offset_ft,
        "yaw_mode": yaw_mode,
        "interval_ft": max(1.0, footprint_w_ft * (1.0 - frontal_overlap_pct / 100.0)),
        "spacing_ft": max(1.0, footprint_h_ft * (1.0 - side_overlap_pct / 100.0)),
    }

def extract_polygon_from_map_data(map_data):
    """
    Pulls the most recently drawn polygon (or rectangle - leaflet exports
    those as polygons too) from an st_folium result as a list of (lat, lon)
    vertices, or None if nothing polygonal has been drawn.
    """
    if not map_data or not map_data.get("all_drawings"):
        return None
    for drawing in reversed(map_data["all_drawings"]):
        geom = drawing.get("geometry", {})
        if geom.get("type") == "Polygon" and geom.get("coordinates"):
            ring = geom["coordinates"][0]
            coords = [(c[1], c[0]) for c in ring]
            if len(coords) > 1 and coords[0] == coords[-1]:
                coords = coords[:-1]
            if len(coords) >= 3:
                return coords
    return None

def _strip_spans(poly, lo, hi, min_span_ft=1.0):
    """
    The disjoint x-intervals the area actually occupies inside the horizontal
    band [lo, hi], smallest x first.

    This one measurement is the whole difference between the two coverage
    strategies. The classic sweep takes only the min and max x across a band,
    so on any shape with a notch - a U, an L, a star - every pass runs straight
    across the gap and photographs ground that was never drawn. Splitting the
    band into the spans that are genuinely inside the polygon lets each strip
    fly only the parts that need covering.
    """
    minx, _, maxx, _ = poly.bounds
    inter = poly.intersection(shapely_box(minx - 1.0, lo, maxx + 1.0, hi))
    if inter.is_empty:
        return []
    spans = []
    for g in getattr(inter, "geoms", [inter]):
        if g.is_empty or getattr(g, "area", 0.0) <= 0.0:
            continue
        gx0, _, gx1, _ = g.bounds
        if gx1 - gx0 >= min_span_ft:
            spans.append((gx0, gx1))
    spans.sort()
    return spans


def _decomposed_passes(rot_pts, centers, ymin, ymax, half_strip, runout):
    """
    Build the pass list by boustrophedon cell decomposition instead of one
    full-width pass per strip. Returns (path_rot, num_cells).

    Strips are cut into their real spans, then spans are grouped into cells:
    walking upward, a span that cleanly continues exactly one span from the
    strip below stays in that cell; anything else - a span splitting in two, two
    merging into one, or a span appearing from nothing - closes the cells
    involved and opens fresh ones. Those split/merge events are precisely the
    critical points of a boustrophedon decomposition. Each cell is then flown
    serpentine, and the cells are visited nearest-end-first.
    """
    poly = ShapelyPolygon(rot_pts)
    if not poly.is_valid:
        # Hand-drawn outlines can self-touch; buffer(0) heals the ring without
        # moving it, and may hand back a MultiPolygon, which the rest handles.
        poly = poly.buffer(0)
    if poly.is_empty:
        return [], 0

    strips = []
    for k, c in enumerate(centers):
        lo = ymin if k == 0 else c - half_strip
        hi = ymax if k == len(centers) - 1 else c + half_strip
        strips.append(_strip_spans(poly, lo, hi))

    cells, open_cells, prev_spans = [], {}, []
    for k, spans in enumerate(strips):
        fwd = {i: [] for i in range(len(prev_spans))}
        back = {j: [] for j in range(len(spans))}
        for j, (a0, a1) in enumerate(spans):
            for i, (b0, b1) in enumerate(prev_spans):
                if min(a1, b1) - max(a0, b0) > 0:      # the spans overlap in x
                    fwd[i].append(j)
                    back[j].append(i)
        new_open = {}
        for j, (x0, x1) in enumerate(spans):
            src = back[j]
            if len(src) == 1 and len(fwd[src[0]]) == 1 and src[0] in open_cells:
                ci = open_cells[src[0]]                # clean 1:1 continuation
            else:
                cells.append([])                       # split, merge or new
                ci = len(cells) - 1
            cells[ci].append((k, x0, x1))
            new_open[j] = ci
        open_cells, prev_spans = new_open, spans

    # Serpentine inside each cell, alternating direction strip to strip.
    cell_runs = []
    for cell in cells:
        run = []
        for idx, (k, x0, x1) in enumerate(cell):
            y = centers[k]
            lo, hi = x0 - runout, x1 + runout
            run.append(((lo, y), (hi, y)) if idx % 2 == 0 else ((hi, y), (lo, y)))
        if run:
            cell_runs.append(run)

    # Visit cells nearest-first, entering each from whichever end is closer.
    ordered, remaining, cur = [], list(range(len(cell_runs))), None
    while remaining:
        if cur is None:
            pick = remaining[0]
        else:
            pick = min(remaining, key=lambda i: min(
                math.hypot(cell_runs[i][0][0][0] - cur[0], cell_runs[i][0][0][1] - cur[1]),
                math.hypot(cell_runs[i][-1][1][0] - cur[0], cell_runs[i][-1][1][1] - cur[1])))
        remaining.remove(pick)
        run = cell_runs[pick]
        if cur is not None:
            d_start = math.hypot(run[0][0][0] - cur[0], run[0][0][1] - cur[1])
            d_end = math.hypot(run[-1][1][0] - cur[0], run[-1][1][1] - cur[1])
            if d_end < d_start:
                run = [(b, a) for (a, b) in reversed(run)]
        ordered.extend(run)
        cur = ordered[-1][1]

    path_rot = []
    for a, b in ordered:
        path_rot += [a, b]
    return path_rot, len(cell_runs)


def generate_mapping_flight_path(boundary_coords, alt_ft, pitch, frontal_overlap_pct, side_overlap_pct,
                                 side="parallel", runout_intervals=1.0, line_bearing_deg=None,
                                 decompose=False):
    """
    Builds a serpentine (lawnmower) flight path whose camera footprints
    fully cover the drawn boundary polygon, honoring altitude, gimbal
    pitch, and frontal/side overlap. The drone path itself runs a little
    outside the boundary: each pass spans the area's width across the strip
    it covers, plus runout_intervals photo intervals at either end so the
    boundary itself is not the last frame (0 still covers the area, since
    the footprint reaches half its own length past the final photo).

    line_bearing_deg fixes the compass bearing the passes run along; None
    picks the bearing that needs the fewest passes. With the "parallel" camera side the imaged
    strip stays centred on the drone, so each pass runs straight down its
    strip and the path keeps the drawn area's shape; with "right"/"left" and
    an oblique gimbal the flight lines are offset sideways so the *imaged*
    strips (not the drone) land on the target area, and because the camera
    stays on the drone's chosen side that offset flips with each direction
    change - which is what pushes the path out of the drawn shape.

    Returns (path_coords, info) where path_coords is the corner-waypoint
    serpentine as (lat, lon) tuples, or (None, None) for a degenerate
    boundary.
    """
    if not boundary_coords or len(boundary_coords) < 3:
        return None, None

    geom = mapping_camera_geometry(alt_ft, pitch, frontal_overlap_pct, side_overlap_pct, side)
    fh = geom["footprint_h_ft"]
    offset, spacing = geom["offset_ft"], geom["spacing_ft"]

    # Project to a local planar frame in feet (equirectangular approximation,
    # fine at flight-area scale).
    lat0 = sum(c[0] for c in boundary_coords) / len(boundary_coords)
    lon0 = sum(c[1] for c in boundary_coords) / len(boundary_coords)
    cos_lat = math.cos(math.radians(lat0))
    pts = [((c[1] - lon0) * cos_lat * FT_PER_DEG_LAT, (c[0] - lat0) * FT_PER_DEG_LAT) for c in boundary_coords]

    # Reject a shape with no interior before trying to sweep it. The shoelace
    # area is 0 for duplicated vertices and for any set of collinear points,
    # however far apart they are, so this catches both.
    area_ft2 = abs(sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
                       for i in range(len(pts)))) / 2.0
    if area_ft2 < MIN_MAPPABLE_AREA_FT2:
        return None, None

    # Sweep parallel to whichever boundary edge leaves the SMALLEST extent
    # perpendicular to the sweep. That perpendicular extent is exactly what
    # the pass count is derived from below, so minimizing it minimizes passes
    # (and therefore turns and flight time). For a convex polygon the optimal
    # sweep direction is always parallel to one of its edges, so testing the
    # edges is sufficient; for concave ones it remains a good heuristic.
    # Sweeping along the LONGEST edge - the previous rule - usually picks the
    # same direction but is not equivalent, and could cost an extra pass.
    # A fixed bearing overrides the search. Bearings are compass degrees
    # (clockwise from north) while this frame measures counter-clockwise from
    # east, hence the 90 - b; passes are bidirectional, so only 0..180 is
    # distinct and anything outside wraps into it.
    n = len(pts)
    if line_bearing_deg is not None:
        sweep_ang = math.radians(90.0 - (float(line_bearing_deg) % 180.0))
    else:
        sweep_ang, best_width = 0.0, float('inf')
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            if math.hypot(x2 - x1, y2 - y1) < 1e-9:
                continue  # duplicate vertex - no direction to take from it
            ang = math.atan2(y2 - y1, x2 - x1)
            ca_t, sa_t = math.cos(-ang), math.sin(-ang)
            ys_t = [x * sa_t + y * ca_t for x, y in pts]
            width = max(ys_t) - min(ys_t)
            if width < best_width - 1e-9:
                best_width, sweep_ang = width, ang
    ca, sa = math.cos(-sweep_ang), math.sin(-sweep_ang)
    rot = [(x * ca - y * sa, x * sa + y * ca) for x, y in pts]

    ys = [pt[1] for pt in rot]
    xs_all = [pt[0] for pt in rot]
    ymin, ymax = min(ys), max(ys)
    height = ymax - ymin

    # Imaged-strip centers: evenly respaced so the first/last strips exactly
    # reach the boundary's extents (actual side overlap comes out >= requested).
    if fh >= height:
        centers = [(ymin + ymax) / 2.0]
    else:
        n_passes = math.ceil((height - fh) / spacing) + 1
        step = (height - fh) / (n_passes - 1)
        centers = [ymin + fh / 2.0 + k * step for k in range(n_passes)]

    def band_x_range(lo, hi):
        """x-extent of the boundary within the horizontal band [lo, hi]."""
        xs = [x for x, y in rot if lo <= y <= hi]
        for i in range(n):
            x1, y1 = rot[i]
            x2, y2 = rot[(i + 1) % n]
            for y_line in (lo, (lo + hi) / 2.0, hi):
                if (y1 - y_line) * (y2 - y_line) < 0:
                    t = (y_line - y1) / (y2 - y1)
                    xs.append(x1 + t * (x2 - x1))
        if not xs:
            xs = xs_all
        return min(xs), max(xs)

    side_sign = 1.0 if side == "right" else -1.0
    runout = geom["interval_ft"] * max(0.0, float(runout_intervals))
    # A pass only has to span the strip it is RESPONSIBLE for - the ground
    # between it and its neighbours - not the full width of its own footprint.
    # The footprint does reach fh/2 to either side, but ground that far off is
    # imaged directly by a nearer pass, which is already sized for it. Taking
    # the band at the full fh (as this did) stretched every pass to the widest
    # part of a band fh tall; at high side overlap fh is several times the line
    # spacing, so on any shape whose width varies every pass came out nearly as
    # long as the widest part of the whole area, regardless of how narrow the
    # area was at that pass. The outermost passes still open out to the block
    # edge, since there is no neighbour beyond them.
    half_strip = (centers[1] - centers[0]) / 2.0 if len(centers) > 1 else 0.0

    num_cells = 1
    if decompose and SHAPELY_AVAILABLE:
        # Experimental strategy: cut each strip into the spans that are really
        # inside the area rather than one full-width pass. Everything above -
        # sweep bearing, strip centres, run-out - is shared with the classic
        # path, so the two can be compared like for like.
        path_rot, num_cells = _decomposed_passes(rot, centers, ymin, ymax, half_strip, runout)
        if not path_rot:
            decompose = False       # nothing usable came back; fall through

    if not (decompose and SHAPELY_AVAILABLE):
        path_rot = []
        for k, c in enumerate(centers):
            band_lo = ymin if k == 0 else c - half_strip
            band_hi = ymax if k == len(centers) - 1 else c + half_strip
            x_lo, x_hi = band_x_range(band_lo, band_hi)
            # Run-out past each edge, so the boundary is not the very last
            # frame. The footprint is centred on the drone, so a photo taken AT
            # the boundary already images fw/2 beyond it and zero run-out still
            # covers the area - the old fixed fw/2 extension here pushed the
            # drone that far out again, laying a full extra footprint of photos
            # outside the area at both ends of every pass without covering any
            # more of it.
            x_lo -= runout
            x_hi += runout
            eastbound = (k % 2 == 0)
            # Camera looks to the drone's chosen side of travel; place the
            # drone on the opposite side of the strip so the footprint lands
            # on it.
            y_drone = c + (side_sign * offset if eastbound else -side_sign * offset)
            if eastbound:
                path_rot += [(x_lo, y_drone), (x_hi, y_drone)]
            else:
                path_rot += [(x_hi, y_drone), (x_lo, y_drone)]

    # Rotate back and unproject.
    ca2, sa2 = math.cos(sweep_ang), math.sin(sweep_ang)
    path_coords = []
    for x, y in path_rot:
        gx = x * ca2 - y * sa2
        gy = x * sa2 + y * ca2
        path_coords.append((lat0 + gy / FT_PER_DEG_LAT, lon0 + gx / (FT_PER_DEG_LAT * cos_lat)))

    info = dict(geom)
    # Decomposition can emit more than one pass per strip, so the pass count is
    # taken from the path itself rather than the strip count.
    info["num_passes"] = len(path_rot) // 2
    info["num_cells"] = num_cells
    info["decomposed"] = bool(decompose and SHAPELY_AVAILABLE)
    # Compass bearing the passes actually run along, so the UI can report what
    # the automatic search picked (and seed the manual control from it).
    info["line_bearing_deg"] = (90.0 - math.degrees(sweep_ang)) % 180.0
    info["runout_ft"] = runout
    # Each pass contributes exactly two waypoints, so path segment j runs
    # along a pass when j is even and is a turn/connector between passes when
    # j is odd. The camera is yawed off the direction of travel, so on a
    # connector it points along the strips instead of across them - those
    # photos image the wrong thing and, on DJI Fly, burn the 99-photo budget.
    # Hand the connector indices to the generator so it can skip them.
    info["connector_segments"] = [j for j in range(max(0, len(path_coords) - 1)) if j % 2 == 1]
    return path_coords, info


def photo_index_ranges(n_coords, connector_segments):
    """
    Waypoint index ranges that should be photographed, as the maximal runs of
    consecutive non-connector segments. DJI Pilot needs these because it fires
    from an interval trigger over a waypoint range rather than per-waypoint,
    so skipping the turns means emitting one trigger per pass.
    """
    skip = set(connector_segments or ())
    ranges, start = [], None
    for j in range(max(0, n_coords - 1)):
        if j in skip:
            if start is not None:
                ranges.append((start, j))
                start = None
        elif start is None:
            start = j
    if start is not None:
        ranges.append((start, n_coords - 1))
    return ranges


def dense_photo_segments(fracs, connector_segments):
    """
    For each densified mapping waypoint (as interpolate_path's (segment_index,
    frac) pairs), the original path segment whose photo-taking status and
    heading it should use.

    Normally that is just the point's own (outgoing) segment. But
    interpolate_path now always places a point exactly on every original
    vertex, and a vertex directly between a pass and a connector is
    simultaneously the END of one pass and the START of whatever comes next.
    If that next thing is a connector, attributing the point to it would
    silently drop a photo - or, if takes_photo overrode that, aim it along
    the connector instead of across the strip - exactly at a strip's edge,
    which is precisely where mapping coverage most depends on a photo
    landing. So an exact vertex (frac == 0.0) prefers whichever adjacent
    segment is a photo-taking pass, when the two disagree; interior points
    (0 < frac < 1) only ever belong to the one segment they're on regardless.
    """
    skip = set(connector_segments or ())
    out = []
    for seg, frac in fracs:
        if frac == 0.0 and seg in skip and seg > 0 and (seg - 1) not in skip:
            out.append(seg - 1)
        else:
            out.append(seg)
    return out


def photo_gap_m(trigger_type, interval_ft, interval_sec, speed_m):
    """
    Metres between photos, derived exactly the way generate_native_kmz_contents
    derives it - a distance trigger is the interval itself, a time trigger is
    speed x time. Kept in metres throughout: the Creator used to divide a
    distance in FEET by a time-trigger gap in METRES, inflating the estimate by
    ~3.3x on time-triggered missions.
    """
    if trigger_type == "distance":
        return max(1.0, interval_ft * FT_TO_M)
    return max(1.0, speed_m * interval_sec)


def count_corridor_photos(coords, gap_m, is_dji_fly, photo_start_wp=0):
    """
    Photos a corridor (drawn-line) mission will actually take.

    DJI Fly is exact rather than estimated: the generator densifies the line and
    puts one photo on every resulting waypoint, and interpolate_path also pins a
    waypoint to each drawn corner - so distance/interval undercounts by roughly
    one photo per corner, and by more as the interval grows. It also ignores
    photo_start_wp (the generator forces 0 for DJI Fly), so this ignores it too.

    DJI Pilot fires from an interval trigger running on the aircraft, so
    distance over interval - counted from the start waypoint - is right there.
    """
    if not coords or len(coords) < 2 or gap_m <= 0:
        return 0
    if is_dji_fly:
        return len(interpolate_path(coords, gap_m))
    start = max(0, min(int(photo_start_wp), len(coords) - 1))
    dist_m = sum(get_haversine_dist(coords[i], coords[i + 1])
                 for i in range(start, len(coords) - 1))
    return int(dist_m / gap_m) + 1


def major_waypoint_indices(points, turn_deg=20.0, max_run=12):
    """
    Waypoint indices worth annotating on a densified (DJI Fly) mission.

    DJI Fly stores one waypoint per photo, so a route drawn from a handful of
    clicks comes back as 90+ points, and a label on every leg is unreadable.
    The points that still carry meaning are the ones the route actually turns
    at - the corners originally clicked on a corridor mission, and the ends of
    each pass on a mapping mission - so those are kept, plus a periodic
    fill-in so a long straight run is not left completely unlabelled.
    """
    n = len(points)
    if n < 3:
        return list(range(n))
    keep = {0, n - 1}
    for i in range(1, n - 1):
        turn = abs((get_bearing(points[i], points[i + 1])
                    - get_bearing(points[i - 1], points[i]) + 180) % 360 - 180)
        if turn > turn_deg:
            keep.add(i)
    ordered = sorted(keep)
    filled = []
    for a, b in zip(ordered, ordered[1:]):
        filled.append(a)
        span = b - a
        if span > max_run:
            steps = math.ceil(span / max_run)
            for k in range(1, steps):
                filled.append(a + round(k * span / steps))
    filled.append(ordered[-1])
    return sorted(set(filled))


def count_mapping_photos(path_coords, interval_ft, connector_segments, is_dji_fly=True):
    """
    Number of photos a mapping path will actually take, so the Creator's
    estimate can't drift from what gets written into the KMZ. DJI Fly is exact
    - it mirrors the generator's own densification. DJI Pilot is a close
    estimate, since its interval trigger runs on the aircraft.
    """
    if not path_coords or len(path_coords) < 2:
        return 0
    if is_dji_fly:
        gap_m = max(1.0, interval_ft * FT_TO_M)
        _dense, fracs = interpolate_path(path_coords, gap_m, return_frac=True)
        skip = set(connector_segments or ())
        return sum(1 for seg in dense_photo_segments(fracs, connector_segments) if seg not in skip)

    total = 0
    for r_start, r_end in photo_index_ranges(len(path_coords), connector_segments):
        seg_ft = sum(
            get_haversine_dist(path_coords[j], path_coords[j + 1]) for j in range(r_start, r_end)
        ) * M_TO_FT
        total += int(seg_ft / interval_ft) + 1 if interval_ft > 0 else 0
    return total

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

CREATOR_PRESETS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".creator_presets.json")

# Every Creator sidebar parameter a preset can capture/restore - deliberately
# excludes the Filename field, per the user's request. Includes mapping_mode
# itself, so loading a preset also switches between mapping/line mode.
CREATOR_PRESET_KEYS = [
    "mapping_mode", "hw_choice", "cam_choice",
    "trans_speed_mph", "safe_takeoff_ft",
    "alt_ft", "map_alt_ft",
    "c_source", "c_tif", "c_bounds",
    "pitch", "map_pitch", "side", "map_side",
    "photo_start_wp", "trigger_type",
    "t_dist_val", "overlap_pct", "manual_mph_dist",
    "t_val_sec", "auto_speed", "target_gap_ft", "manual_mph_time",
    "map_front_ol", "map_side_ol", "map_speed_mph",
    "map_fix_bearing", "map_bearing", "map_runout",
]


def load_creator_presets():
    if not os.path.exists(CREATOR_PRESETS_FILE):
        return {}
    try:
        with open(CREATOR_PRESETS_FILE, encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception as e:
        # Flag it rather than just presenting "no presets". An unreadable file
        # still holds the user's presets, and the next save would overwrite it
        # with a fresh one-entry dict, destroying whatever was recoverable.
        st.session_state["_c_preset_load_error"] = f"{type(e).__name__}: {e}"
        return {}


def save_creator_presets(presets):
    """
    Write the presets file. Returns None on success or the error text on
    failure - swallowing it silently meant a failed write (read-only checkout,
    no permission, disk full) still reported "Saved preset", and the preset was
    simply gone the next time the app started.
    """
    try:
        with open(CREATOR_PRESETS_FILE, 'w', encoding="utf-8") as f:
            json.dump(presets, f, indent=2)
        return None
    except Exception as e:
        return f"{type(e).__name__}: {e}"


README_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "README.md")
# Matches README bullets of the form "* **Widget Label**: explanation text",
# capturing everything up to the next such bullet (or a blank line/EOF).
README_BULLET_RE = re.compile(r'^\* \*\*(.+?)\*\*:[ \t]*(.+?)(?=\n\*\s*\*\*|\n\n|\Z)', re.DOTALL | re.MULTILINE)


def load_param_help():
    # Re-parsed on every call rather than cached, so editing README.md and
    # reloading the app picks up the change immediately - the whole point of
    # sourcing tooltips from the README instead of duplicating them here.
    help_map = {}
    try:
        with open(README_PATH, encoding="utf-8") as f:
            text = f.read()
    except Exception:
        return help_map
    for m in README_BULLET_RE.finditer(text):
        label = m.group(1).strip()
        help_map[label] = ' '.join(m.group(2).split())
    return help_map


PARAM_HELP = load_param_help()


def param_help(label):
    return PARAM_HELP.get(label)


def finite_center_footprint(pitch, alt):
    """
    Along-track footprint in feet, or None when this tilt has no bounded ground
    footprint at all.

    Along-track ground footprint (feet), used to convert between photo interval
    and forward overlap. Derived from the true projected footprint, so overlap
    matches the images at any gimbal angle.

    The frame reaches VERT_HALF_FOV_DEG above the lens axis, so once the tilt is
    shallower than that the top of every image is aimed at or above the horizon
    and the footprint runs to infinity - hence None rather than a number. This
    replaced a version that answered 999999 in that case (a sentinel meant only
    for the pitch==0 divide-by-zero guard); callers took it at face value, which
    turned a request for 70% overlap into a ~300,000 ft photo interval written
    straight into the mission, and made three separate read-outs report 99.9%
    overlap for a footprint that had no far edge.
    """
    if pitch == 0:
        return None
    extents = footprint_extents_ft(alt, pitch)
    return extents[0] if extents else None

def format_overlap(fw_ft, gap_ft):
    """
    Forward overlap for a photo spacing, as text ready to display.

    Returns a plain statement instead of a number in the two cases where a
    percentage would be a lie:
    - fw_ft is None: the gimbal is shallower than the sensor's vertical
      half-FOV, so the footprint has no far edge and overlap is undefined.
      These read-outs used to divide by the 999999 sentinel and cheerfully
      report 99.9%.
    - the spacing exceeds the footprint: overlap is genuinely negative, which
      means consecutive photos do not touch. Clamping that to 0% (as the
      Viewer did) hides an actual gap in coverage.
    """
    if not fw_ft or fw_ft <= 0:
        return "n/a at this gimbal pitch"
    if not gap_ft or gap_ft <= 0:
        return "n/a"
    pct = (1 - gap_ft / fw_ft) * 100
    if pct < 0:
        return f"none - {gap_ft - fw_ft:.0f} ft gap between consecutive photos"
    return f"{min(pct, 99.9):.1f}%"


def sync_dist_to_overlap():
    # No finite footprint -> overlap is undefined; leave the field alone rather
    # than writing a meaningless 99.9%.
    fw = finite_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    if fw:
        st.session_state.overlap_pct = max(0.0, min(((fw - safe_get_float('t_dist_val', 9.0)) / fw) * 100, 99.9))

def sync_overlap_to_dist():
    fw = finite_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    if fw:
        st.session_state.t_dist_val = fw * (1 - (safe_get_float('overlap_pct', 70.0) / 100))

def sync_gap_to_overlap():
    fw = finite_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    if fw:
        st.session_state.overlap_pct = max(0.0, min(((fw - safe_get_float('target_gap_ft', 26.2)) / fw) * 100, 99.9))

def sync_overlap_to_gap():
    fw = finite_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
    if fw:
        st.session_state.target_gap_ft = fw * (1 - (safe_get_float('overlap_pct', 70.0) / 100))

def sync_geometry():
    if st.session_state.get('trigger_type', 'distance') == 'distance': 
        sync_dist_to_overlap()
    else: 
        sync_gap_to_overlap()

def e_sync_dist_to_overlap():
    # As on the Creator side: skip entirely when there is no finite footprint.
    fw = finite_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    if fw:
        st.session_state.e_overlap_pct = max(0.0, min(((fw - safe_get_float('e_t_dist_val', 9.0)) / fw) * 100, 99.9))

def e_sync_overlap_to_dist():
    fw = finite_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    if fw:
        st.session_state.e_t_dist_val = fw * (1 - (safe_get_float('e_overlap_pct', 70.0) / 100))

def e_sync_gap_to_overlap():
    fw = finite_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    if fw:
        st.session_state.e_overlap_pct = max(0.0, min(((fw - safe_get_float('e_target_gap_ft', 26.2)) / fw) * 100, 99.9))

def e_sync_overlap_to_gap():
    fw = finite_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
    if fw:
        st.session_state.e_target_gap_ft = fw * (1 - (safe_get_float('e_overlap_pct', 70.0) / 100))

def e_sync_geometry():
    if st.session_state.get('e_trigger_type', 'distance') == 'distance': 
        e_sync_dist_to_overlap()
    else: 
        e_sync_gap_to_overlap()

# ==========================================
# DATA EXTRACTION (FOR EDITOR)
# ==========================================
def strip_flight_suffix(name):
    """
    Strips a previously-applied _Fly/_Pilot platform marker and
    _HxxAxxOLxx(SOxx) parameter suffix from a mission base name, so it can
    be cleanly regenerated from the current (possibly edited) altitude/
    pitch/overlap/platform values on save instead of stacking a second
    suffix on top of the old one.
    """
    return re.sub(r'(?:_(?:Fly|Pilot))?_H\d+A\d+OL\d+(?:SO\d+)?$', '', name)

# Characters that are legal in a macOS filename (this app's original
# platform - only '/' is illegal there) but raise a raw, unhandled OSError
# on Windows: '<>:"/\|?*' and control characters. Windows also forbids a
# trailing dot or space, which a plain character strip won't catch.
_WINDOWS_ILLEGAL_FILENAME_CHARS_RE = re.compile(r'[<>:"/\\|?*\x00-\x1f]')

def sanitize_filename_component(name):
    """
    Cleans a user-typed mission name so it's safe to build a filename from
    on any OS. Applied once, right where the name comes off its text_input
    widget, so every downstream use (filename, thumbnail label, success
    message) stays consistent with what actually got saved.
    """
    cleaned = _WINDOWS_ILLEGAL_FILENAME_CHARS_RE.sub('', name).strip().rstrip('.')
    return cleaned or "Mission"

def line_intersection_local(p_a, bearing_a, p_b, bearing_b):
    """
    Intersects two infinite lines - each given as a point plus a compass
    bearing - using a local-planar approximation valid for small areas.

    The true crossing point races toward infinity as the two lines approach
    parallel, so ordinary coordinate rounding noise gets amplified there -
    but "how close to parallel" alone isn't a reliable proxy for whether
    that's actually happened, because it ignores scale. A mapping serpentine
    can produce a connector whose bearing is only a few degrees shy of
    exactly reversing the pass it leaves - the true corner there is a
    perfectly well-defined, stable point (see recover_corners), just an
    acute one - while a genuine "there and back" survey leg is a real
    reversal that races off just as an angle test would predict.

    So rather than pre-judging from the angle, the intersection is always
    computed, then trusted only if it lands within a generous multiple of
    how far apart p_a and p_b themselves are - a direct measurement of
    whether the math actually blew up, not a guess based on angle alone.
    Genuine blow-ups land literally millions of times farther out than that
    (verified: a synthetic reversal put the raw result ~2.7 million times
    the anchor separation away), so this cleanly separates the two cases
    with a lot of headroom either side.
    """
    lat0, lon0 = p_a
    lon_scale = 111320.0 * math.cos(math.radians(lat0)) or 1e-9

    def to_local(lat, lon):
        return ((lon - lon0) * lon_scale, (lat - lat0) * 110540.0)

    def from_local(x, y):
        return (lat0 + y / 110540.0, lon0 + x / lon_scale)

    a1 = to_local(*p_a)
    d1 = (math.sin(math.radians(bearing_a)), math.cos(math.radians(bearing_a)))
    a2 = to_local(*p_b)
    d2 = (math.sin(math.radians(bearing_b)), math.cos(math.radians(bearing_b)))

    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-9:
        return None  # exactly parallel - no intersection exists at all

    t = ((a2[0] - a1[0]) * d2[1] - (a2[1] - a1[1]) * d2[0]) / denom
    ix, iy = a1[0] + t * d1[0], a1[1] + t * d1[1]

    baseline = math.hypot(a2[0] - a1[0], a2[1] - a1[1])
    if baseline > 1e-9:
        dist_from_a1 = math.hypot(ix - a1[0], iy - a1[1])
        dist_from_a2 = math.hypot(ix - a2[0], iy - a2[1])
        if min(dist_from_a1, dist_from_a2) > 50.0 * baseline:
            return None

    return from_local(ix, iy)

def recover_corners(coords, cluster_start, cluster_end):
    """
    Recovers the true original waypoint(s) hidden inside one bend cluster
    of a DJI Fly-densified path, as a list of 1 or 2 points to splice in
    where the cluster was - far more precise than approximating with
    whichever nearby dense sample happens to be closest (samples land at
    fixed cumulative-distance steps from the path's start, not reset at
    each corner, so the nearest one can be off by up to half a photo
    interval). This matters because a mission opened for editing and
    re-saved with only a parameter changed (e.g. altitude) re-derives its
    dense waypoints from whatever gets recovered here.

    Most turns - including shallow ones, and regardless of how many dense
    samples happen to land in the transition zone - are a single true
    vertex, so the direct intersection of the incoming and outgoing legs
    is tried first and used whenever numerically stable (see
    line_intersection_local).

    That direct intersection is only unstable for a near U-turn, where
    the two legs nearly reverse each other - which is also exactly the
    case where a "there and back" survey leg genuinely does turn via two
    close but distinct waypoints (the true end of the outbound run and
    true start of the inbound run), not one sharp vertex. When the
    cluster spans 2+ flagged points, it captures the short connecting
    chord between those two points, with its own clean bearing distinct
    from both legs; recovering each true corner as the (stable)
    intersection of that chord with its neighboring leg handles this case
    both accurately and precisely.

    Returns None if there isn't a clean straight run of at least 2 points
    on both sides (a corner too close to the path's very start/end).
    """
    if cluster_start - 2 < 0 or cluster_end + 2 >= len(coords):
        return None

    leg_in_bearing = get_bearing(coords[cluster_start - 2], coords[cluster_start - 1])
    leg_out_bearing = get_bearing(coords[cluster_end + 1], coords[cluster_end + 2])

    corner = line_intersection_local(coords[cluster_start - 1], leg_in_bearing, coords[cluster_end + 1], leg_out_bearing)
    if corner is not None:
        return [corner]

    if cluster_end > cluster_start:
        chord_bearing = get_bearing(coords[cluster_start], coords[cluster_end])
        corner_a = line_intersection_local(coords[cluster_start - 1], leg_in_bearing, coords[cluster_start], chord_bearing)
        corner_b = line_intersection_local(coords[cluster_end], chord_bearing, coords[cluster_end + 1], leg_out_bearing)
        if corner_a is not None and corner_b is not None:
            return [corner_a, corner_b]

    lat0, lon0 = coords[cluster_start - 1]
    lat1, lon1 = coords[cluster_end + 1]
    return [((lat0 + lat1) / 2, (lon0 + lon1) / 2)]

def dedensify_coords(coords, angle_tol_deg=1.0):
    """
    Collapses a DJI Fly-densified polyline (extra waypoints inserted at even
    spacing along each straight segment, for photo triggering) back down to
    just the original corner vertices, by keeping only points where the
    flight bearing actually changes. Densified points along a straight run
    share an identical bearing (pure lat/lon interpolation), so any real
    turn - even a shallow one - stands out well above float noise.

    A real corner rarely lands exactly on a dense sample (samples are placed
    at fixed cumulative-distance steps from the start, not reset at each
    corner), so the single "cut" segment that crosses it shows a bearing
    change at both its ends - two adjacent flagged points for one corner.
    Adjacent flags are merged into one cluster, and each cluster's true
    waypoint(s) are recovered via recover_corners.
    """
    if len(coords) <= 2:
        return list(coords)
    bend_idxs = []
    for i in range(1, len(coords) - 1):
        b_in = get_bearing(coords[i - 1], coords[i])
        b_out = get_bearing(coords[i], coords[i + 1])
        diff = abs((b_in - b_out + 180) % 360 - 180)
        if diff > angle_tol_deg:
            bend_idxs.append(i)

    clusters = []
    cluster = []
    for idx in bend_idxs:
        if cluster and idx != cluster[-1] + 1:
            clusters.append(cluster)
            cluster = []
        cluster.append(idx)
    if cluster:
        clusters.append(cluster)

    corners = [coords[0]]
    for cluster in clusters:
        recovered = recover_corners(coords, cluster[0], cluster[-1])
        corners.extend(recovered if recovered is not None else [coords[cluster[len(cluster) // 2]]])
    corners.append(coords[-1])
    return corners

def parse_kmz_for_editing(full_path):
    meta = {
        "safe_takeoff_ft": 60.0, "trans_speed_mph": 22.0, "speed_m": 4.11, "speed_mph": 6.0,
        "alt_ft": 50.0, "pitch": -60.0, "trigger_type": "distance", 
        "t_val": 9.0, "photo_start_wp": 0, "coords": [], "camera_type": "visible",
        "drone_sub": "0", "payload_sub": "3"
    }
    with zipfile.ZipFile(full_path, 'r') as kmz:
        is_fly = any(name.startswith('wpmz/') for name in kmz.namelist())
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

        raw_coords = []
        pms = root_w.findall('.//{*}Placemark')
        for i, pm in enumerate(pms):
            c_node = pm.find('.//{*}coordinates')
            if c_node is not None:
                c_raw = c_node.text.strip().split(',')
                raw_coords.append((float(c_raw[1]), float(c_raw[0])))

            if i == 0:
                alt_node = pm.find('.//{*}executeHeight')
                if alt_node is not None: meta['alt_ft'] = float(alt_node.text) * M_TO_FT
                pitch_node = pm.find('.//{*}waypointGimbalHeadingParam/{*}waypointGimbalPitchAngle')
                if pitch_node is not None:
                    meta['pitch'] = float(pitch_node.text)
                else:
                    # DJI Fly waypoints don't carry this tag at all - pitch is
                    # instead set once via the first waypoint's gimbalRotate
                    # action (see the is_dji_fly branch in
                    # generate_native_kmz_contents).
                    for a in pm.findall('.//{*}action'):
                        func = a.find('.//{*}actionActuatorFunc')
                        if func is not None and func.text == 'gimbalRotate':
                            p_angle = a.find('.//{*}actionActuatorFuncParam/{*}gimbalPitchRotateAngle')
                            if p_angle is not None and p_angle.text:
                                meta['pitch'] = float(p_angle.text)
                            break

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

        if is_fly and len(raw_coords) >= 2:
            # DJI Fly missions store every densified photo waypoint, not the
            # original drawn corners, and use a per-waypoint reachPoint
            # trigger rather than Pilot's single multipleDistance/
            # multipleTiming action, so trigger_type/t_val/photo_start_wp
            # never match above. Recover the corners by keeping only points
            # where the flight direction actually changes (dense points
            # along one straight segment share an identical bearing), and
            # recover the photo spacing from the distance between the first
            # two dense points. Without this, re-saving the mission would
            # re-densify the already-dense coordinate list on top of itself.
            meta['trigger_type'] = "distance"
            meta['t_val'] = get_haversine_dist(raw_coords[0], raw_coords[1]) * M_TO_FT
            meta['photo_start_wp'] = 0
            meta['coords'] = dedensify_coords(raw_coords)
        else:
            meta['coords'] = raw_coords
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
        corner_coords = coords  # pre-densification, for per-segment headings
        coords, fracs = interpolate_path(coords, gap_m, return_frac=True)
        elevations = interpolate_elevations(corner_elevations, fracs)
        # Mapping missions mark their turn/connector segments so no photo is
        # taken while crossing between passes (see generate_mapping_flight_path).
        # dense_photo_segments resolves each densified waypoint to the ORIGINAL
        # segment whose photo-taking status (and, below, heading) it should use
        # - ordinarily its own outgoing segment, except at a vertex shared
        # between a pass and a connector, which it attributes to the pass.
        no_photo_segments = cfg.get("no_photo_segments")
        photo_segments = dense_photo_segments(fracs, no_photo_segments)
        skip_segments = set(no_photo_segments or ())
        takes_photo = [seg not in skip_segments for seg in photo_segments]
    else:
        elevations = get_elevations_batch(coords, elev_source, tif_path)
        takes_photo = [True] * len(coords)
        corner_coords, fracs = coords, None  # DJI Pilot flies the corners as-is

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

    # Must stay in step with footprint_extents_ft's yaw_mode - the overlap and
    # line spacing a mapping mission is built from assume this exact aim.
    yaw_mode = cfg.get("camera_yaw_mode", "perpendicular")

    def aim(bearing):
        if yaw_mode == "forward":
            yaw = bearing % 360
        else:
            yaw = (bearing + 90) % 360 if cfg['side'] == "right" else (bearing - 90) % 360
        return int(yaw - 360 if yaw > 180 else yaw)

    yaws = [aim(get_bearing(coords[i], coords[i+1])) for i in range(len(coords) - 1)]

    # Each waypoint's heading normally comes from the leg it just flew, which
    # for an interior densified point is exactly right - interpolate_path now
    # keeps every point within the ONE original segment it was placed on, so
    # consecutive dense points never straddle a bend. The one case that still
    # needs help is a vertex shared between a pass and a connector: it takes
    # its photo-or-not status from photo_segments (the pass, not whichever of
    # the two the vertex happened to be recorded against), and its heading has
    # to agree - otherwise a photo at the edge of a strip could still end up
    # aimed along the connector instead of across the strip.
    # Gated on the mapping mission's connector list rather than on the yaw mode:
    # a pass/connector vertex needs its heading pinned to the pass under either
    # camera aim, and only a mapping mission has connectors to begin with.
    wp_headings = None
    if is_dji_fly and cfg.get("no_photo_segments") and len(corner_coords) >= 2:
        seg_aim = [aim(get_bearing(corner_coords[j], corner_coords[j+1]))
                   for j in range(len(corner_coords) - 1)]
        wp_headings = [seg_aim[min(seg, len(seg_aim) - 1)] for seg in photo_segments]

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
        
        if wp_headings is not None and i < len(wp_headings):
            current_yaw = wp_headings[i]
        else:
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
            if i >= start_wp and takes_photo[i]:
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

                # DJI Pilot fires from an interval trigger spanning a waypoint
                # range, rather than per-waypoint like DJI Fly, so connector
                # legs are skipped by emitting one trigger per pass instead of
                # a single one covering the whole route. Missions that don't
                # ask for that (every corridor/line mission) still get exactly
                # one group spanning everything, as before.
                photo_ranges = cfg.get("photo_index_ranges") or [(start_wp, len(coords) - 1)]
                for r_start, r_end in photo_ranges:
                    if r_end <= r_start:
                        continue  # a single point can't host an interval trigger
                    photo_action_block = f"""
          <wpml:actionGroupStartIndex>{r_start}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{r_end}</wpml:actionGroupEndIndex>
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


# Streamlit's default expanded sidebar is ~300px; used to align the map area
# to start right where it ends. Locked below (not user-resizable) so this
# stays accurate.
SIDEBAR_W = "300px"
# Rendered height of the merged title+tabs header bar below - kept as a
# constant (rather than measured) so the main content's padding-top and the
# map area's fixed top offset always agree with each other.
HEADER_H = "72px"

st.markdown(f"""
<style>
html, body {{ overflow: hidden !important; }}
[data-testid="stAppViewContainer"] {{ overflow: hidden !important; }}
[data-testid="stHeader"] {{ background: transparent !important; height: 2.2rem !important; }}
[data-testid="stMainBlockContainer"] {{ padding: 0 !important; padding-top: {HEADER_H} !important; margin: 0 !important; max-width: 100% !important; }}
[data-testid="stMain"] {{ overflow-y: auto !important; height: 100vh !important; }}
[data-testid="stMainBlockContainer"] > div:first-child {{ gap: 0 !important; }}
footer {{ display: none !important; }}

/* Lock the sidebar to a fixed width - no user resize handle. */
[data-testid="stSidebar"] {{ width: {SIDEBAR_W} !important; min-width: {SIDEBAR_W} !important; max-width: {SIDEBAR_W} !important; }}
[data-testid="stSidebarResizeHandle"] {{ display: none !important; pointer-events: none !important; }}
[data-testid="stSidebar"] div[style*="cursor: col-resize"] {{ display: none !important; pointer-events: none !important; }}

/* Single unified header bar: title + tabs together, spanning the full
   width (over both the sidebar and the content area) so there's no seam. */
.st-key-app_header {{
    position: fixed !important; top: 0 !important; left: 0 !important; right: 0 !important;
    z-index: 1000010 !important; display: flex !important; align-items: center !important;
    gap: 16px !important; padding: 10px 16px !important;
    background: rgba(255,255,255,0.96) !important; backdrop-filter: blur(6px);
    box-shadow: 0 2px 10px rgba(0,0,0,0.18) !important;
}}
.st-key-app_header [data-testid="stHorizontalBlock"] {{ width: 100% !important; align-items: center !important; }}
.st-key-app_header h1 {{ font-size: 1.6rem !important; margin: 0 !important; white-space: nowrap !important; }}

/* Map area: the region below the header and right of the sidebar. Its top
   bar (folder/save selectors + flight summary) and the map itself are flex
   children so the map always exactly fills whatever space the top bar
   doesn't use - bounded by the sidebar (left), header (top), and the
   screen edges (right/bottom). */
.st-key-map_area {{
    position: fixed !important; top: {HEADER_H} !important; left: {SIDEBAR_W} !important; right: 0 !important; bottom: 0 !important;
    width: auto !important; height: auto !important;
    display: flex !important; flex-direction: column !important; overflow: hidden !important;
}}
/* Streamlit inserts an extra stLayoutWrapper div between map_area and each
   keyed container below it - that wrapper, not top_bar/map_layer directly,
   is the actual flex item, so the grow/shrink rules must target it via
   :has() rather than the .st-key-* class itself. */
.st-key-map_area > div:has(> .st-key-top_bar) {{ flex: 0 0 auto !important; width: 100% !important; }}
.st-key-map_area > div:has(> .st-key-map_layer) {{ flex: 1 1 auto !important; min-height: 0 !important; height: auto !important; width: 100% !important; overflow: hidden !important; }}

.st-key-map_area .st-key-top_bar {{
    position: static !important; width: 100% !important;
    display: flex !important; align-items: flex-start !important;
    gap: 24px !important; flex-wrap: wrap !important; padding: 12px 16px !important;
    background: rgba(255,255,255,0.95) !important; backdrop-filter: blur(6px);
    box-shadow: 0 2px 10px rgba(0,0,0,0.18) !important;
    z-index: 1 !important;
}}
.st-key-map_area .st-key-top_bar [data-testid="stHorizontalBlock"] {{ flex: 1 1 auto !important; }}

/* Cascade a definite height down through every wrapper div between
   map_layer and the actual map iframe, so `height: 100%` resolves at each
   level instead of the iframe falling back to its 150px UA default. */
.st-key-map_area .st-key-map_layer {{ position: relative !important; height: 100% !important; width: 100% !important; display: flex !important; flex-direction: column !important; }}
.st-key-map_area .st-key-map_layer [data-testid="stElementContainer"]:has(iframe) {{ flex: 1 1 auto !important; min-height: 0 !important; height: auto !important; }}
.st-key-map_area .st-key-map_layer [data-testid="stElementContainer"]:has(iframe) > div {{ height: 100% !important; }}
.st-key-map_area .st-key-map_layer iframe {{ width: 100% !important; height: 100% !important; display: block !important; }}

/* Notices (warnings/info/captions/status text) float over the top of the
   map, translucent, and never affect the map's size - taken out of the flex
   flow and anchored to map_layer's own top edge, which the flex layout
   above already positions correctly below top_bar regardless of its
   height. */
.st-key-map_area .st-key-map_layer > div:has(> .st-key-notices) {{
    position: absolute !important; top: 0 !important; left: 0 !important; right: 0 !important;
    height: auto !important; width: auto !important; z-index: 5 !important; pointer-events: none !important;
}}
.st-key-notices {{
    background: rgba(255,255,255,0.82) !important; backdrop-filter: blur(3px);
    padding: 4px 16px !important; width: 100% !important; box-sizing: border-box !important;
}}
.st-key-notices > div {{ width: 100% !important; }}
.st-key-notices [data-testid="stElementContainer"] {{ margin-bottom: 2px !important; width: 100% !important; }}
.st-key-notices [data-testid="stMarkdownContainer"] {{ width: 100% !important; }}
.st-key-notices .stAlert {{ padding: 4px 12px !important; width: 100% !important; box-sizing: border-box !important; }}
/* The 99-photo override tick box sits in this banner and should read as a
   small aside under the error, not as another headline control. */
.st-key-notices .stCheckbox {{ pointer-events: auto !important; padding: 0 12px !important; }}
.st-key-notices .stCheckbox p {{ font-size: 0.78rem !important; }}

/* Small floating badge, bottom-right of the screen, for the map's current
   center coordinates - out of the way of both the search toggle (bottom
   left) and the notices banner (top of map). */
.st-key-screen_center {{
    position: fixed !important; bottom: 16px !important; right: 16px !important; z-index: 1000006 !important;
    width: fit-content !important; max-width: calc(100vw - {SIDEBAR_W} - 32px) !important;
    background: rgba(255,255,255,0.82) !important; backdrop-filter: blur(3px);
    padding: 4px 12px !important; border-radius: 8px !important; box-shadow: 0 2px 8px rgba(0,0,0,0.15) !important;
}}
.st-key-screen_center [data-testid="stMarkdownContainer"] div {{ font-size: 0.8rem !important; color: #31333F !important; white-space: nowrap !important; }}

/* Floating side panel for secondary content (e.g. the Editor's coordinate
   table) - right-aligned so it never collides with the sidebar. */
.st-key-side_panel {{
    position: fixed !important; top: 245px !important; right: 16px !important; z-index: 1000004 !important;
    width: 400px !important; max-height: calc(100vh - 265px) !important; overflow-y: auto !important;
    background: rgba(255,255,255,0.95) !important; backdrop-filter: blur(6px);
    border-radius: 12px !important; padding: 14px 16px !important;
    box-shadow: 0 4px 18px rgba(0,0,0,0.22) !important;
}}

/* Small "jump to address" toggle button, bottom-left of the map, next to
   the sidebar. */
.st-key-search_toggle {{
    position: fixed !important; bottom: 16px !important; left: calc({SIDEBAR_W} + 16px) !important;
    z-index: 1000006 !important;
}}

/* The address-search bar itself, opened by that button - spans the bottom
   of the screen (over the map, not the sidebar). */
.st-key-search_bar {{
    position: fixed !important; bottom: 0 !important; left: {SIDEBAR_W} !important; right: 0 !important;
    width: auto !important;
    z-index: 1000007 !important; padding: 14px 16px !important;
    background: rgba(255,255,255,0.96) !important; backdrop-filter: blur(6px);
    box-shadow: 0 -2px 10px rgba(0,0,0,0.18) !important;
}}

/* Photo Sorter / DJI Fly Transfer have no map, so they scroll normally, but
   without any of the above panels' compact sizing their headings read as
   noticeably larger than the rest of the app - this brings them in line. */
.st-key-page_body {{ padding: 16px 24px !important; }}
.st-key-page_body h1 {{ font-size: 1.5rem !important; }}
.st-key-page_body h2, .st-key-page_body h3 {{ font-size: 1.15rem !important; }}

/* The script-only embed just below the stylesheet runs the search bar's
   outside-click listener and renders nothing. components.html(height=0)
   already collapses it, but the container still leaves a hairline margin/
   padding gap in some browsers, so this belt-and-suspenders rule zeroes
   that out too. Collapsed via height/overflow rather than display:none so
   the iframe is still rendered and its script is guaranteed to execute. */
.st-key-search_autoclose_script {{
    height: 0 !important; min-height: 0 !important; overflow: hidden !important;
    margin: 0 !important; padding: 0 !important;
}}
</style>
""", unsafe_allow_html=True)

# Auto-close the address search bar on an outside click. st.markdown() can't
# run <script> tags (React strips them from dangerouslySetInnerHTML), so this
# goes through an iframe instead, reaching back into the main page via
# window.parent.document - a same-origin, well-established pattern for this
# exact limitation. Guarded so the (page-persistent) listener is only ever
# bound once, no matter how many times Streamlit reruns the script.
#
# components.html (not st.html/st.markdown) is what this needs: given an HTML
# string it embeds it as-is in an iframe that permits JavaScript and
# same-origin access to the app, whereas st.html sanitises the markup through
# DOMPurify and would strip the script outright. The markup here is a fixed
# literal, never user input, so that untrusted-content caveat doesn't apply.
# (st.iframe loads a URL, not an HTML string, and isn't the right tool here -
# and the Streamlit version this app targets doesn't even have it, which is
# exactly the bug that put st.iframe here in the first place: it was reverted
# to this from a copy that predated the components.html fix - see git history
# before reapplying that "fix" a third time.)
#
# This embed is script-only and renders nothing, so it must take up no space:
# height=0 collapses it, backed up by the CSS rule on
# .st-key-search_autoclose_script above for the rare browser that still
# leaves a hairline gap.
with st.container(key="search_autoclose_script"):
    components.html("""
<script>
(function() {
    const doc = window.parent.document;
    const win = window.parent;
    if (doc.__searchOutsideClickBound) return;
    doc.__searchOutsideClickBound = true;

    function closeIfOpen() {
        const bar = doc.querySelector('.st-key-search_bar');
        if (!bar) return;
        const toggle = doc.querySelector('.st-key-search_toggle');
        const btn = toggle ? toggle.querySelector('button') : null;
        if (btn) btn.click();
    }

    doc.addEventListener('click', function(e) {
        const bar = doc.querySelector('.st-key-search_bar');
        if (!bar) return;
        const toggle = doc.querySelector('.st-key-search_toggle');
        if (bar.contains(e.target) || (toggle && toggle.contains(e.target))) return;
        closeIfOpen();
    }, true);

    // A click on the map lands inside its iframe, which never bubbles a
    // click event out to this parent document - the only signal the parent
    // gets is that focus moved into an <iframe>, surfaced as a window blur.
    win.addEventListener('blur', function() {
        setTimeout(function() {
            if (doc.activeElement && doc.activeElement.tagName === 'IFRAME') {
                closeIfOpen();
            }
        }, 0);
    });
})();
</script>
""", height=0)

HEADING_RE = re.compile(r'^(#{1,6})[ \t]+(.+?)[ \t]*$', re.MULTILINE)


def _github_slug(heading_text):
    # Approximates GitHub's own heading-anchor slugs, since the README's
    # #anchor links are written to match those (and so work unmodified when
    # read on GitHub) - strip markdown emphasis, lowercase, drop anything
    # that isn't a word char/space/hyphen, then collapse whitespace to "-".
    text = re.sub(r'[*_`]', '', heading_text).strip().lower()
    text = re.sub(r'[^\w\s-]', '', text)
    return re.sub(r'\s+', '-', text)


MD_IMAGE_RE = re.compile(r'!\[([^\]]*)\]\(([^)\s]+)\)')
HTML_IMG_SRC_RE = re.compile(r'(<img\b[^>]*\bsrc\s*=\s*")([^"]+)(")', re.IGNORECASE)


def _inline_local_images(text, base_dir):
    """
    GitHub serves README.md's images by fetching the real file at its
    relative path, so `BYU_Specific_information/images/foo.png` just works
    there. Streamlit's dev server has no route for that path though - it
    falls back to serving the app's own index.html for anything unmatched
    (with a 200, not a 404), so the browser tries to decode that HTML as an
    image and shows a broken icon instead. Swap local image paths for base64
    data URIs here so the in-app README dialog is self-contained, while
    leaving the README file itself untouched for GitHub to render normally.
    """
    def _data_uri(path):
        if path.startswith(("http://", "https://", "data:")):
            return None
        full_path = os.path.join(base_dir, path)
        if not os.path.isfile(full_path):
            return None
        mime_type = mimetypes.guess_type(full_path)[0] or "application/octet-stream"
        with open(full_path, "rb") as f:
            encoded = base64.b64encode(f.read()).decode("ascii")
        return f"data:{mime_type};base64,{encoded}"

    def _replace_md(m):
        data_uri = _data_uri(m.group(2))
        return f'![{m.group(1)}]({data_uri})' if data_uri else m.group(0)

    def _replace_html(m):
        data_uri = _data_uri(m.group(2))
        return f'{m.group(1)}{data_uri}{m.group(3)}' if data_uri else m.group(0)

    text = MD_IMAGE_RE.sub(_replace_md, text)
    text = HTML_IMG_SRC_RE.sub(_replace_html, text)
    return text


# ==========================================
# DJI FLY 99-PHOTO SAVE-BLOCK OVERRIDE
# ==========================================
# Saving is blocked past the cap because DJI Fly bogs down badly and can take
# the controller with it. This lets a pilot who has reason to believe their
# setup will cope lift the block deliberately. Ticking the box only REQUESTS
# the override - it is not granted until the confirmation dialog is accepted,
# so a stray click can't quietly re-enable saving. Each caller passes its own
# `scope` so the Creator's two modes and the Editor keep independent state.

@st.dialog("Override the DJI Fly photo limit?")
def _confirm_99_override(scope, est_photos):
    st.warning(f"This mission takes {est_photos} photos - {est_photos - 99} more than the 99 DJI Fly supports.")
    st.markdown(
        "DJI Fly needs one waypoint per photo. Past 99 it is known to lag badly, "
        "and it can crash the controller **mid-flight**, taking the mission with it.\n\n"
        "Only turn this on if you have reason to believe your controller will cope, "
        "and load the mission on the controller to check it before you rely on it."
    )
    cancel_col, ok_col = st.columns(2)
    if cancel_col.button("Cancel", width='stretch'):
        st.session_state[f"{scope}_99_dialog"] = False
        st.session_state[f"{scope}_99_override_ok"] = False
        # The tick box has already been instantiated this run, so its state
        # cannot be written here - defer the reset to the next run.
        st.session_state[f"{scope}_99_reset"] = True
        st.rerun()
    if ok_col.button("Turn on override", type="primary", width='stretch'):
        st.session_state[f"{scope}_99_dialog"] = False
        st.session_state[f"{scope}_99_override_ok"] = True
        st.rerun()


def _99_override_toggled(scope):
    if st.session_state.get(f"{scope}_99_override_cb"):
        # Ask for confirmation only if it isn't already granted.
        if not st.session_state.get(f"{scope}_99_override_ok"):
            st.session_state[f"{scope}_99_dialog"] = True
    else:
        # Unticking revokes immediately - no dialog needed to become safer.
        st.session_state[f"{scope}_99_override_ok"] = False
        st.session_state[f"{scope}_99_dialog"] = False


def render_99_override(container, scope, est_photos):
    """
    Tick box (plus its confirmation dialog) that lifts the DJI Fly save block,
    rendered inside `container` directly under the limit message. Only call it
    when the mission is actually over the cap. Returns True while the override
    is active, i.e. while saving should be allowed.
    """
    if st.session_state.pop(f"{scope}_99_reset", False):
        st.session_state[f"{scope}_99_override_cb"] = False

    with container:
        st.checkbox(
            "Override the limit and let me save anyway",
            key=f"{scope}_99_override_cb",
            on_change=_99_override_toggled,
            args=(scope,),
        )

    if st.session_state.get(f"{scope}_99_dialog"):
        _confirm_99_override(scope, est_photos)

    return bool(st.session_state.get(f"{scope}_99_override_ok"))


@st.dialog("README", width="large")
def _readme_dialog():
    try:
        with open(README_PATH, encoding="utf-8") as f:
            text = f.read()
    except Exception as e:
        st.error(f"Could not read README.md: {e}")
        return

    text = _inline_local_images(text, os.path.dirname(README_PATH))

    # st.markdown only auto-generates heading-anchor ids for standalone
    # st.markdown/st.header calls, not for headings embedded in one big
    # block like this - so the README's own #anchor links (which target
    # GitHub's anchors and work fine there) silently go nowhere in-app.
    # Fix that by injecting a matching <a id="..."> right before each
    # heading here, without touching the README file itself.
    def add_anchor(m):
        return f'<a id="{_github_slug(m.group(2))}"></a>\n\n{m.group(0)}'
    text = HEADING_RE.sub(add_anchor, text)

    st.markdown(text, unsafe_allow_html=True)

    # The #anchor links in the Tabs section (and anywhere else in the
    # README) rely on the browser's native fragment-navigation scrolling the
    # nearest scrollable ancestor into view - but the README's actual
    # scroll container is a nested `.stDialog` div, not the page itself,
    # and Safari/WebKit has never reliably supported scrolling a *nested*
    # overflow container this way (Chromium/Firefox do). The link updates
    # the URL hash but the dialog just doesn't move. Bypassing native
    # fragment nav entirely and driving the scroll manually with
    # Element.scrollIntoView() - a much older, universally-supported API -
    # sidesteps that inconsistency instead of depending on it.
    #
    # components.html, not st.iframe: same reasoning as the search bar's
    # auto-close script above - st.iframe loads a URL rather than an HTML
    # string, isn't in the Streamlit version this app targets, and st.html
    # would strip the <script> tag via DOMPurify.
    components.html("""
<script>
(function() {
    const doc = window.parent.document;
    if (doc.__readmeAnchorScrollBound) return;
    doc.__readmeAnchorScrollBound = true;

    doc.addEventListener('click', function(e) {
        const link = e.target.closest('a[href^="#"]');
        if (!link) return;
        const dialog = link.closest('.stDialog');
        if (!dialog) return;  // only intercept links inside the README dialog
        const id = link.getAttribute('href').slice(1);
        const target = doc.getElementById(id);
        if (!target) return;
        e.preventDefault();
        target.scrollIntoView({block: 'start'});
    }, true);
})();
</script>
""", height=0)


with st.container(key="app_header"):
    header_title_col, header_tabs_col, header_readme_col = st.columns([1, 4, 0.6], gap="medium")
    with header_title_col:
        st.markdown("# DJI Flight Planner")
    with header_tabs_col:
        page = st.radio("Navigation", ["Creator", "Editor", "Viewer  |", "Photo Sorter", "DJI Fly Transfer"], horizontal=True, label_visibility="collapsed")
    with header_readme_col:
        if st.button("📖 README", width='stretch'):
            _readme_dialog()

# --- CREATOR MODE ---
if page == 'Creator':
    if "map_boundary" not in st.session_state:
        st.session_state.map_boundary = None

    # Preset values must land in session_state before any of the widgets
    # below are instantiated this run - Streamlit forbids setting a widget's
    # session_state key after that widget has already rendered once in the
    # same script run, so the "Load" button just stashes the preset here and
    # reruns, and this is where it actually gets applied.
    if st.session_state.get("_c_pending_preset"):
        for k, v in st.session_state.pop("_c_pending_preset").items():
            st.session_state[k] = v

    # A deleted preset's name is still sitting in the select box's state, and
    # Streamlit rejects a stored value that isn't among the widget's options.
    # Clearing it has to happen here, before the widget is rebuilt below.
    if st.session_state.pop("_c_preset_deleted", False):
        st.session_state.pop("c_preset_select", None)

    @st.dialog("Save Preset")
    def _c_save_preset_dialog(default_name):
        preset_name_input = st.text_input("Preset Name", value=default_name, key="c_preset_name_input")
        if st.button("Save", key="c_preset_save_btn"):
            if preset_name_input.strip():
                presets = load_creator_presets()
                presets[preset_name_input.strip()] = {k: st.session_state[k] for k in CREATOR_PRESET_KEYS if k in st.session_state}
                err = save_creator_presets(presets)
                if err:
                    st.error(f"Could not save the preset: {err}")
                else:
                    st.success(f"Saved preset '{preset_name_input.strip()}'")
                    st.rerun()
            else:
                st.warning("Enter a preset name.")

    @st.dialog("Delete preset?")
    def _c_delete_preset_dialog(name):
        st.warning(f"Delete the preset **{name}**?")
        st.caption("It is removed from your saved presets and cannot be recovered. "
                   "The settings currently in the sidebar are not changed.")
        cancel_col, ok_col = st.columns(2)
        if cancel_col.button("Cancel", width='stretch', key="c_preset_del_cancel"):
            st.rerun()
        if ok_col.button("Delete", type="primary", width='stretch', key="c_preset_del_ok"):
            presets = load_creator_presets()
            presets.pop(name, None)
            err = save_creator_presets(presets)
            if err:
                st.error(f"Could not delete the preset: {err}")
            else:
                st.session_state["_c_preset_deleted"] = True
                st.rerun()

    with st.sidebar:
        mapping_mode = st.checkbox(
            "Change to a mapping mission", value=False, key="mapping_mode",
            help="Draw the area you want mapped instead of a flight line. The flight "
                 "path is auto-calculated from altitude, gimbal pitch, and frontal/side "
                 "overlap, and may extend outside the drawn boundary."
        )

        st.header("1. Hardware & Payload")
        # Both platforms plan either kind of mission - mapping used to be
        # forced onto DJI Fly, which capped an area mission at 99 photos no
        # matter which aircraft was actually flying it.
        hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()), key="hw_choice", help=param_help("Drone Platform"))
        drone_enum = HARDWARE_MAP[hw_choice]["drone_enum"]
        drone_sub_enum = HARDWARE_MAP[hw_choice]["drone_sub"]
        payload_enum = HARDWARE_MAP[hw_choice]["payload_enum"]
        payload_sub_enum = HARDWARE_MAP[hw_choice]["payload_sub"]
        is_dji_fly = HARDWARE_MAP[hw_choice].get("is_dji_fly", False)

        if is_dji_fly:
            st.warning("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash saving will be disabled if you exceed this.")

        if is_dji_fly:
            cam_choice = "RGB Only"
            st.selectbox("Sensor Mode", ["RGB Only"], disabled=True,
                         help="DJI Fly aircraft are RGB only - sensor mode is not applicable.")
        else:
            cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"], key="cam_choice", help=param_help("Sensor Mode"))
        camera_type = CAM_VAL_MAP[cam_choice]
        min_photo_interval_sec = 2.0 if "narrow_band" in camera_type else 0.7

        st.header("2. Global Config")
        mission_name = sanitize_filename_component(st.text_input("Filename", "Mission_Flight", help=param_help("Filename")))

        with st.expander("💾 Parameter Presets"):
            c_presets = load_creator_presets()
            if st.session_state.get("_c_preset_load_error"):
                st.error(
                    f"Your saved presets could not be read ({st.session_state['_c_preset_load_error']}). "
                    "They are shown as empty below - saving a preset now would overwrite the existing "
                    f"file. Back up {os.path.basename(CREATOR_PRESETS_FILE)} first if you want to keep it."
                )
            c_preset_names = sorted(c_presets.keys())
            c_selected_preset = st.selectbox("Preset", ["Select a preset..."] + c_preset_names, key="c_preset_select", help=param_help("Parameter Presets"))
            c_no_preset = (c_selected_preset == "Select a preset...")
            c_pcol1, c_pcol2 = st.columns(2)
            with c_pcol1:
                if st.button("📥 Load", width='stretch', disabled=c_no_preset):
                    st.session_state["_c_pending_preset"] = c_presets[c_selected_preset]
                    st.rerun()
            with c_pcol2:
                if st.button("💾 Save", width='stretch'):
                    _c_save_preset_dialog("" if c_no_preset else c_selected_preset)
            # Delete gets its own row rather than a third column: three buttons
            # wrap badly at sidebar width, and keeping the destructive one apart
            # from Load/Save makes it harder to hit by accident. It acts on
            # whatever is selected above, and confirms before rewriting the file.
            if st.button("🗑 Delete Preset", width='stretch', disabled=c_no_preset,
                         help="Permanently delete the selected preset"):
                _c_delete_preset_dialog(c_selected_preset)

        trans_speed_mph = st.number_input("Takeoff Speed (mph)", value=22.0, step=1.0, key="trans_speed_mph", help=param_help("Takeoff Speed (mph)"))
        safe_takeoff_ft = st.number_input("Safe Takeoff Alt (ft)", value=60.0, step=1.0, key="safe_takeoff_ft", help=param_help("Safe Takeoff Alt (ft)"))

        if mapping_mode:
            st.header("3. Mapping Settings")
            st.number_input("Relative Altitude (ft)", value=100.0, key="map_alt_ft", step=1.0, help=param_help("Relative Altitude (ft)"))
        else:
            st.header("3. Waypoint Settings")
            st.number_input("Relative Altitude (ft)", value=60.0, key="alt_ft", step=1.0, on_change=sync_geometry, help=param_help("Relative Altitude (ft)"))
        st.info("❗Elevation is relative to the take off point, NOT the mission start point.")

        c_elev_source = st.selectbox("Elevation Source", ["USGS 3DEP (US High-Res)", "Open-Elevation (Global)", "Local GeoTIFF"], key="c_source", help=param_help("Elevation Source"))
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
                tif_files = [f for f in os.listdir(SURFACES_DIR) if f.lower().endswith((".tif", ".tiff"))]
                if tif_files:
                    selected_tif = st.selectbox("Select Surface File", tif_files, key="c_tif")
                    c_tif_path = os.path.join(SURFACES_DIR, selected_tif)
                    c_show_bounds = st.checkbox("Show GeoTIFF Boundaries on Map", value=True, key="c_bounds")
                else:
                    st.warning("No .tif files found in the 'surfaces' folder.")

        if mapping_mode:
            # Lower bound comes from the sensor's own FOV, not a magic number:
            # past this the footprint has no finite ground extent, so overlap
            # and line spacing can't be computed (and used to crash).
            map_pitch_min = -int(math.ceil(MIN_MAPPING_PITCH_DEG))
            # A preset saved while the old (shallower) bound was in force can
            # hold a pitch outside the new range, which Streamlit rejects
            # outright - pull any such value back in before the slider reads it.
            if "map_pitch" in st.session_state:
                st.session_state.map_pitch = int(max(-90, min(map_pitch_min, st.session_state.map_pitch)))
            st.slider("Gimbal Pitch (°)", -90, map_pitch_min, value=-90,
                      key="map_pitch", help=param_help("Gimbal Pitch (°)"))

            map_alt = safe_get_float('map_alt_ft', 100.0)
            map_pitch_val = safe_get_float('map_pitch', -90.0)
            pitch_rad = math.radians(abs(map_pitch_val))
            D_ft_c = map_alt / math.sin(pitch_rad) if pitch_rad > 0 else float('inf')
            gsd_cm = (D_ft_c * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_c != float('inf') else 0
            st.info(f"Est. Ground GSD: {gsd_cm:.2f} cm/px")

            # "parallel" aims the camera along each pass, which is the only
            # option that leaves the computed path matching the drawn area -
            # the two side aims shift alternating passes sideways to put the
            # tilted footprint back over the target. A preset saved before
            # "parallel" existed can hold a side that is no longer the default.
            if st.session_state.get("map_side") not in MAPPING_CAMERA_SIDES:
                st.session_state.map_side = "parallel"
            side = st.selectbox("Camera side of flight path", MAPPING_CAMERA_SIDES,
                                key="map_side", help=param_help("Camera side of flight path"))

            st.header("4. Coverage & Speed")
            st.number_input("Frontal Overlap (%)", min_value=0.0, max_value=95.0, value=75.0, step=1.0, key="map_front_ol")
            st.number_input("Side Overlap (%)", min_value=0.0, max_value=95.0, value=65.0, step=1.0, key="map_side_ol")

            # Off by default: the automatic bearing is the one that needs the
            # fewest passes, so a fixed bearing is opt-in and normally costs
            # passes (and photos) in exchange for lines pointing a chosen way.
            map_fix_bearing = st.checkbox("Set flight line direction", value=False, key="map_fix_bearing",
                                          help=param_help("Set flight line direction"))
            map_bearing = None
            if map_fix_bearing:
                map_bearing = float(st.slider("Flight Line Bearing (°)", 0, 179, value=90, key="map_bearing",
                                              help=param_help("Flight Line Bearing (°)")))

            map_runout = st.slider("Edge Run-out (photo intervals)", 0.0, 3.0, value=1.0, step=0.25,
                                   key="map_runout", help=param_help("Edge Run-out (photo intervals)"))

            # Experimental coverage strategy, off by default. It only ever
            # changes the path on an area whose strips are split into separate
            # pieces (a U, a ring, an H); on anything else it produces exactly
            # the same flight plan, so it is safe to leave on.
            map_decompose = st.checkbox(
                "Split passes at gaps (beta)", value=False, key="map_decompose",
                disabled=not SHAPELY_AVAILABLE,
                help=(param_help("Split passes at gaps (beta)") if SHAPELY_AVAILABLE
                      else "Needs the shapely package - run: pip install -r requirements.txt"))

            map_geom = mapping_camera_geometry(
                map_alt, map_pitch_val,
                safe_get_float('map_front_ol', 75.0), safe_get_float('map_side_ol', 65.0), side
            )
            st.info(
                f"Photo every {map_geom['interval_ft']:.0f} ft along track\n\n"
                f"Flight line spacing: {map_geom['spacing_ft']:.0f} ft\n\n"
                f"Run-out past each edge: {map_geom['interval_ft'] * map_runout:.0f} ft"
            )

            manual_mph = st.number_input("Flight Speed (mph)", min_value=2.3, step=1.0, value=4.0, key="map_speed_mph", help=param_help("Flight Speed (mph)"))
            speed_m = manual_mph * MPH_TO_MS
            max_speed_m = (map_geom['interval_ft'] * FT_TO_M) / min_photo_interval_sec
            if speed_m > max_speed_m:
                st.error(f"Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
        else:
            st.slider("Gimbal Pitch (°)", -90, 0, value= -60, key="pitch", on_change=sync_geometry, help=param_help("Gimbal Pitch (°)"))

            current_pitch = safe_get_float('pitch', -60.0)
            pitch_rad = math.radians(abs(current_pitch))
            current_alt = safe_get_float('alt_ft', 50.0)
            D_ft_c = current_alt / math.sin(pitch_rad) if pitch_rad > 0 else float('inf')
            gsd_cm = (D_ft_c * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_c != float('inf') else 0
            st.info(f"Est. Ground GSD: {gsd_cm:.2f} cm/px")

            # Overlap only means something when the frame has a bounded ground
            # footprint. Tilt the gimbal shallower than the sensor's vertical
            # half-FOV and the top of every frame points at or above the
            # horizon, so the footprint has no far edge - and any overlap
            # figure derived from it is fiction. The field is disabled rather
            # than left to produce one.
            c_overlap_ok = finite_center_footprint(current_pitch, current_alt) is not None
            if not c_overlap_ok:
                st.warning(
                    f"At {current_pitch:.0f}° the camera is tilted less than its own "
                    f"{VERT_HALF_FOV_DEG:.1f}° half field-of-view, so the top of the frame "
                    "looks at or above the horizon and the photo footprint has no far edge. "
                    "Forward Overlap can't be calculated from that, so it is disabled - set "
                    "the photo interval directly. Tilt to about -28° or steeper to use overlap."
                )

            side = st.selectbox("Side of flight path", ["right", "left"], key="side", help=param_help("Side of flight path"))

            st.header("4. Trigger & Speed")
            # DJI Fly puts one photo on every waypoint and the generator forces
            # the start index to 0, so the control does nothing there - greyed
            # out rather than left to imply an effect it can't have.
            photo_start_wp = st.number_input(
                "Start Photos at Waypoint Index", min_value=0, value=0, step=1,
                key="photo_start_wp", disabled=is_dji_fly,
                help=("Not available on DJI Fly - it shoots at every waypoint from the start."
                      if is_dji_fly else param_help("Start Photos at Waypoint Index")))
            if is_dji_fly:
                photo_start_wp = 0
            st.radio("Type", ["distance", "time"], key="trigger_type", on_change=sync_geometry, help=param_help("Type"))

            if st.session_state.get('trigger_type', 'distance') == "distance":
                st.number_input("Interval (ft)", key="t_dist_val", min_value=1.0, step=1.0, on_change=sync_dist_to_overlap, help=param_help("Interval (ft)"))
                st.number_input("Forward Overlap (%)", key="overlap_pct", min_value=0.0, max_value=99.9, step=1.0,
                                on_change=sync_overlap_to_dist, disabled=not c_overlap_ok,
                                help=(param_help("Forward Overlap (%)") if c_overlap_ok else
                                      "Unavailable at this gimbal pitch - see the notice above."))
                manual_mph = st.number_input("Flight Speed (mph)", min_value=2.3, step=1.0, value=4.0, key="manual_mph_dist", help=param_help("Flight Speed (mph)"))
                speed_m = manual_mph * MPH_TO_MS

                gap_m = max(1.0, safe_get_float('t_dist_val', 9.0) * FT_TO_M)
                max_speed_m = gap_m / min_photo_interval_sec
                if speed_m > max_speed_m:
                    st.error(f"Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
            else:
                t_val_sec = st.number_input("Interval (sec)", min_value=min_photo_interval_sec, value=max(2.0, min_photo_interval_sec), key="t_val_sec", help=param_help("Interval (sec)"))
                auto_speed = st.checkbox("Auto-Calc Speed", True, key="auto_speed")
                if auto_speed:
                    st.number_input("Target Gap (ft)", key="target_gap_ft", min_value=1.0, on_change=sync_gap_to_overlap)
                    st.number_input("Forward Overlap (%)", key="overlap_pct", min_value=0.0, max_value=99.9, step=1.0,
                                    on_change=sync_overlap_to_gap, disabled=not c_overlap_ok,
                                    help=(param_help("Forward Overlap (%)") if c_overlap_ok else
                                          "Unavailable at this gimbal pitch - see the notice above."))
                    speed_m = min(max((safe_get_float('target_gap_ft', 26.2) * FT_TO_M) / t_val_sec, 1.0), 10.0)
                    st.info(f"Auto-Calculated Speed: {speed_m * MS_TO_MPH:.1f} mph")
                else:
                    manual_mph = st.number_input("Manual Speed (mph)", min_value=2.3, value=6.0, step=1.0, key="manual_mph_time", help=param_help("Manual Speed (mph)"))
                    speed_m = manual_mph * MPH_TO_MS
                    current_gap = speed_m * M_TO_FT * t_val_sec
                    fw = finite_center_footprint(safe_get_float('pitch', -60.0), safe_get_float('alt_ft', 50.0))
                    st.info(f"Current Overlap: {format_overlap(fw, current_gap)}")

        st.header("5. Visuals")
        show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="creator_faa_toggle", help=param_help("Show FAA Airspace Restrictions"))
        if show_faa_airspace:
            st.write("#### Update restrictions of map center")
            if st.button("Update Map Center", key="btn_update_creator"):
                st.session_state.locked_creator_center = st.session_state.creator_center
                st.rerun()

    map_area = st.container(key="map_area")
    top_bar = map_area.container(key="top_bar")
    map_layer = map_area.container(key="map_layer")
    notices = map_layer.container(key="notices")
    if "c_show_search" not in st.session_state:
        st.session_state.c_show_search = False

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

    save_half, hud_half = top_bar.columns([2, 3])
    with save_half:
        existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
        save_col1, save_col2, save_col3 = st.columns([5, 0.7, 0.7])
        with save_col1:
            save_option = st.selectbox(
                "Save Destination", ["Root (missions/)"] + existing_dirs,
                key="c_save_option", on_change=_clear_c_browsed_dir
            )
        with save_col2:
            st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
            if st.button("📂", key="c_btn_browse_dir", help="Browse for a save directory", width='stretch'):
                picked = pick_folder_dialog("Select Save Directory")
                if picked:
                    st.session_state.c_browsed_dir = picked
                    st.rerun()
        with save_col3:
            st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
            if st.button("＋", key="c_btn_new_folder_popup", help="Create a new empty folder", width='stretch'):
                _c_new_folder_dialog()

        # Vestigial: "Create New Folder..." isn't an actual option in the
        # selectbox above (new folders go through the popup dialog instead),
        # so this never renders - kept only because final_dir below still
        # references new_dir_name for that dead branch.
        new_dir_name = st.text_input("New Folder Name", "New_Project", key="c_new_dir_name") if save_option == "Create New Folder..." else ""

        if st.session_state.c_browsed_dir:
            notices.caption(f"📁 Saving to custom path: {st.session_state.c_browsed_dir}")

    top_hud = hud_half.container()
    with map_layer:
        # --- Retained map view ---
        # st_folium hands the map's HTML to the component, and ANY change to
        # that HTML remounts it - which destroys every client-side Leaflet
        # layer, including a flight line drawn with the draw tools. So the view
        # handed to folium.Map must not track the live view on every rerun:
        # doing that turned each pan and zoom into "report view -> rerun -> new
        # HTML -> remount -> drawing wiped".
        #
        # Instead the view is refreshed only when something ELSE about the map
        # is already about to change - a remount is happening regardless then,
        # so it may as well land where the user is actually looking. Panning
        # and zooming on their own leave these arguments untouched, so no
        # remount happens and anything drawn survives.
        #
        # IMPORTANT: every input that changes what gets drawn onto `m` below
        # has to appear in this signature. Miss one and the map keeps building
        # at a stale view; add something that changes on every rerun and the
        # remount loop comes back.
        map_sig = (
            mapping_mode,
            bool(show_faa_airspace),
            tuple(st.session_state.locked_creator_center) if show_faa_airspace else None,
            c_tif_path if (c_elev_source == "Local GeoTIFF" and c_show_bounds) else None,
            tuple(tuple(p) for p in (st.session_state.map_boundary or ())),
            (safe_get_float('map_alt_ft', 100.0), safe_get_float('map_pitch', -90.0),
             safe_get_float('map_front_ol', 75.0), safe_get_float('map_side_ol', 65.0),
             side, map_runout, map_bearing) if mapping_mode else None,
        )
        if map_sig != st.session_state.get("creator_map_sig"):
            st.session_state.creator_map_sig = map_sig
            if st.session_state.get("creator_center"):
                st.session_state.creator_view = [
                    list(st.session_state.creator_center),
                    st.session_state.get("creator_zoom") or 17,
                ]
        view_center, view_zoom = (st.session_state.get("creator_view")
                                  or [st.session_state.locked_creator_center, 17])
        m = folium.Map(location=view_center, zoom_start=view_zoom, tiles=None)
        add_basemap(m)

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

        if mapping_mode:
            # Area-drawing tools only; the flight line is computed, not drawn.
            Draw(export=False, draw_options={
                'polyline': False,
                'polygon': {'shapeOptions': {'color': '#00ffff', 'weight': 3}},
                'rectangle': {'shapeOptions': {'color': '#00ffff', 'weight': 3}},
                'circle': False, 'circlemarker': False, 'marker': False,
            }).add_to(m)

            # Overlay the stored boundary and its computed serpentine path. The
            # boundary lives in our own session key (not just the Draw layer)
            # because re-rendering the map wipes client-side drawings.
            if st.session_state.map_boundary:
                try:
                    preview_path, _preview_info = generate_mapping_flight_path(
                        st.session_state.map_boundary,
                        safe_get_float('map_alt_ft', 100.0), safe_get_float('map_pitch', -90.0),
                        safe_get_float('map_front_ol', 75.0), safe_get_float('map_side_ol', 65.0),
                        side, map_runout, map_bearing, map_decompose
                    )
                    folium.Polygon(
                        locations=st.session_state.map_boundary, color="#00ffff", weight=3,
                        fill=True, fill_opacity=0.08, tooltip="Area to map"
                    ).add_to(m)
                    if preview_path:
                        path_line = folium.PolyLine(preview_path, color="#ff8800", weight=3, tooltip="Computed flight path").add_to(m)
                        PolyLineTextPath(path_line, '  ►  ', repeat=True, offset=7, attributes={'fill': '#000000', 'font-weight': 'bold', 'font-size': '18', 'fill-opacity': '0.4'}).add_to(m)
                except Exception as e:
                    # Previously swallowed, which made a genuine failure look
                    # identical to "no path could be drawn for this area".
                    notices.error(f"Could not compute the flight path for this area: "
                                  f"{type(e).__name__}: {e}")
        else:
            # Polyline only - a corridor mission is a single flight line, so the
            # area/point tools are disabled to avoid drawing shapes this mode
            # can't consume.
            Draw(export=False, draw_options={
                'polyline': {'shapeOptions': {'color': '#00ffff', 'weight': 5}},
                'polygon': False, 'rectangle': False,
                'circle': False, 'circlemarker': False, 'marker': False,
            }).add_to(m)

        map_data = st_folium(m, use_container_width=True, height=1000, key="creator_map")

        if mapping_mode:
            detected_boundary = extract_polygon_from_map_data(map_data)
            if detected_boundary and detected_boundary != st.session_state.map_boundary:
                st.session_state.map_boundary = detected_boundary
                st.rerun()  # re-render immediately so the computed path overlay appears

        if map_data and map_data.get("center"):
            st.session_state.creator_center = [map_data["center"]["lat"], map_data["center"]["lng"]]
            st.session_state.creator_zoom = map_data["zoom"]
            c_lat, c_lon = st.session_state.creator_center
            with st.container(key="screen_center"):
                st.markdown(f"<div>Current Screen Center: {c_lat:.6f}, {c_lon:.6f} (Click 'Update' in sidebar to update restrictions in this area)</div>", unsafe_allow_html=True)

    with st.container(key="search_toggle"):
        if st.button("📍 Jump to Address", key="c_search_toggle_btn"):
            st.session_state.c_show_search = not st.session_state.c_show_search

    if st.session_state.c_show_search:
        with st.container(key="search_bar"):
            search_col1, search_col2 = st.columns([5, 1])
            with search_col1:
                c_search_query = st.text_input("Jump to Address or Lat/Lon", key="c_search_input", placeholder="e.g. 1600 Pennsylvania Ave or 40.25, -111.64")
            with search_col2:
                st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
                if st.button("Search Location", key="c_btn_search", width='stretch'):
                    with st.spinner("Searching..."):
                        new_coords = get_coords_from_search(c_search_query)
                        if new_coords:
                            st.session_state.locked_creator_center = new_coords
                            st.session_state.creator_center = new_coords
                            # Jumping is the whole point here, so move the
                            # retained view directly - the map signature may not
                            # change (FAA off draws nothing from the centre) and
                            # then nothing else would make the map rebuild.
                            st.session_state.creator_view = [
                                list(new_coords), st.session_state.get("creator_zoom") or 17
                            ]
                            st.session_state.c_show_search = False
                            st.rerun()
                        else:
                            st.error("Location not found. Try a different query.")

    if mapping_mode:
        boundary = st.session_state.map_boundary
        if not boundary:
            notices.info("Draw the area you want to map using the polygon or rectangle tool on the map. "
                    "The flight path will be calculated automatically and may extend outside the boundary.")
        else:
            map_alt = safe_get_float('map_alt_ft', 100.0)
            map_pitch_val = safe_get_float('map_pitch', -90.0)
            map_front_ol = safe_get_float('map_front_ol', 75.0)
            map_side_ol = safe_get_float('map_side_ol', 65.0)

            path_coords, map_info = generate_mapping_flight_path(
                boundary, map_alt, map_pitch_val, map_front_ol, map_side_ol,
                side, map_runout, map_bearing, map_decompose
            )

            if path_coords:
                total_dist_ft = sum(get_haversine_dist(path_coords[i], path_coords[i+1]) for i in range(len(path_coords)-1)) * M_TO_FT
                gap_ft = map_info['interval_ft']
                # Counted the same way the generator writes them - photos on
                # the turn legs between passes are skipped, so a plain
                # distance/interval estimate would overstate the total and
                # trip the 99-photo limit earlier than the mission actually does.
                est_photos = count_mapping_photos(path_coords, gap_ft, map_info["connector_segments"], is_dji_fly)

                # The 99-photo ceiling is a DJI Fly limitation; Pilot has no
                # equivalent cap, so an area mission flown on Pilot is only
                # bounded by battery.
                save_disabled = False
                if is_dji_fly and est_photos > 99:
                    notices.error("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash please shrink the area, raise the altitude, reduce the overlaps, or switch to DJI Pilot 2.")
                    save_disabled = not render_99_override(notices, "cmap", est_photos)

                with top_hud:
                    c1, c2, c3, c4 = st.columns([1.2, 1.2, 1, 1.6])
                    c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
                    c2.metric("Estimated Photos", f"{est_photos} / 99" if is_dji_fly else f"{est_photos}")
                    # Show the bearing actually flown either way - on automatic
                    # it is the only place the chosen direction is visible, and
                    # it gives a starting value for the manual slider.
                    # When decomposition actually split something, say so - on a
                    # shape with no gaps it is a no-op and shouldn't claim credit.
                    pass_note = f"{map_info['line_bearing_deg']:.0f}° lines"
                    if map_info.get("decomposed") and map_info.get("num_cells", 1) > 1:
                        pass_note += f" · {map_info['num_cells']} regions"
                    c3.metric("Passes", f"{map_info['num_passes']}",
                              delta=pass_note, delta_color="off")
                    with c4:
                        st.markdown("<div style='margin-top: 10px;'></div>", unsafe_allow_html=True)
                        save_clicked = st.button("Save & Generate KMZ", width='stretch', disabled=save_disabled)
                        if st.button("🗑 Clear boundary", width='stretch', key="btn_clear_boundary"):
                            st.session_state.map_boundary = None
                            st.rerun()

                    if save_clicked:
                        with notices.spinner("Calculating terrain elevations and generating KMZ..."):
                            cfg = {
                                "safe_takeoff_ft": safe_takeoff_ft, "trans_speed_mph": trans_speed_mph,
                                "alt_ft": map_alt, "pitch": map_pitch_val, "side": side,
                                "trigger_type": "distance",
                                "interval_ft": gap_ft,
                                "interval_sec": 0.0,
                                "speed_m": speed_m, "photo_start_wp": 0,
                                "camera_type": camera_type, "drone_sub": drone_sub_enum, "payload_sub": payload_sub_enum,
                                "is_dji_fly": is_dji_fly,
                                "camera_yaw_mode": map_info["yaw_mode"],
                                # DJI Fly skips turns per-waypoint; Pilot needs
                                # the equivalent as one interval trigger per pass.
                                "no_photo_segments": map_info["connector_segments"],
                                "photo_index_ranges": photo_index_ranges(len(path_coords), map_info["connector_segments"]),
                            }

                            prefixed_name = f"{mission_name}_{'Fly' if is_dji_fly else 'Pilot'}"
                            suffix = f"_H{int(map_alt)}A{int(abs(map_pitch_val))}OL{int(map_front_ol)}SO{int(map_side_ol)}"
                            final_filename = f"{prefixed_name}{suffix}"

                            if st.session_state.c_browsed_dir:
                                final_dir = st.session_state.c_browsed_dir
                            elif save_option == "Root (missions/)": final_dir = MISSION_DIR
                            elif save_option == "Create New Folder...": final_dir = os.path.join(MISSION_DIR, new_dir_name)
                            else: final_dir = os.path.join(MISSION_DIR, save_option)

                            try:
                                template_kml, waylines_wpml = generate_native_kmz_contents(path_coords, cfg, c_elev_source, c_tif_path)
                            except ElevationLookupError as e:
                                notices.error(f"Save aborted: {e}")
                            else:
                                os.makedirs(final_dir, exist_ok=True)
                                final_filepath = os.path.join(final_dir, f"{final_filename}.kmz")
                                export_mission_kmz_from_strings(
                                    template_kml_str=template_kml,
                                    waylines_wpml_str=waylines_wpml,
                                    output_kmz_path=final_filepath,
                                    is_dji_fly=is_dji_fly
                                )
                                thumbnail_path = kmz_companion_path(final_filepath)
                                generate_name_thumbnail(
                                    prefixed_name, map_alt, map_pitch_val,
                                    map_front_ol, thumbnail_path, coords=path_coords, photo_count=est_photos
                                )
                                notices.success(f"Saved {final_filename}.kmz to {final_dir}/")
                                offer_kmz_download(notices, "cmap", final_filepath, f"{final_filename}.kmz")

                    # Redraws the same button on every rerun, not only the one where
                    # Save was clicked - see offer_kmz_download's docstring.
                    offer_kmz_download(notices, "cmap")
            else:
                notices.error(
                    "That shape has no area to map - its points are duplicated or fall on a "
                    "straight line. Delete it with the bin icon and draw an area with width to it."
                )

    elif map_data.get("all_drawings") and any(
        d.get('geometry', {}).get('type') == 'LineString' for d in map_data["all_drawings"]
    ):
        # Only consider drawn lines here - a polygon left over from mapping
        # mode has a nested-ring geometry this corridor flow can't consume.
        line_drawing = [d for d in map_data["all_drawings"] if d.get('geometry', {}).get('type') == 'LineString'][-1]
        coords = [(c[1], c[0]) for c in line_drawing['geometry']['coordinates']]
        total_dist_ft = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1)) * M_TO_FT

        # Read the interval from session state, not the sidebar local: t_val_sec
        # only exists on the time-trigger branch, so naming it directly here
        # would raise NameError whenever the distance trigger is selected.
        c_trigger = st.session_state.get('trigger_type', 'distance')
        gap_m = photo_gap_m(c_trigger, safe_get_float('t_dist_val', 9.0),
                            safe_get_float('t_val_sec', 2.0), speed_m)
        est_photos = count_corridor_photos(coords, gap_m, is_dji_fly, photo_start_wp)

        save_disabled = False
        if is_dji_fly and est_photos > 99:
            notices.error("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash please reduce your distance or increase the interval.")
            save_disabled = not render_99_override(notices, "cline", est_photos)

        with top_hud:
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}" + (" / 99" if is_dji_fly else ""))
            with c3:
                st.markdown("<div style='margin-top: 10px;'></div>", unsafe_allow_html=True)
                save_clicked = st.button("Save & Generate KMZ", width='stretch', disabled=save_disabled)

            if save_clicked:
                with notices.spinner("Calculating terrain elevations and generating KMZ..."):
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

                    try:
                        template_kml, waylines_wpml = generate_native_kmz_contents(coords, cfg, c_elev_source, c_tif_path)
                    except ElevationLookupError as e:
                        notices.error(f"Save aborted: {e}")
                    else:
                        os.makedirs(final_dir, exist_ok=True)
                        final_filepath = os.path.join(final_dir, f"{final_filename}.kmz")
                        export_mission_kmz_from_strings(
                            template_kml_str=template_kml,
                            waylines_wpml_str=waylines_wpml,
                            output_kmz_path=final_filepath,
                            is_dji_fly=is_dji_fly
                        )
                        thumbnail_path = kmz_companion_path(final_filepath)
                        generate_name_thumbnail(
                            prefixed_name, safe_get_float('alt_ft', 50.0), safe_get_float('pitch', -60.0),
                            safe_get_float('overlap_pct', 70.0), thumbnail_path, coords=coords, photo_count=est_photos
                        )
                        notices.success(f"Saved {final_filename}.kmz to {final_dir}/")
                        offer_kmz_download(notices, "cline", final_filepath, f"{final_filename}.kmz")

            # Redrawn on every rerun, not only the one where Save was clicked -
            # see offer_kmz_download's docstring.
            offer_kmz_download(notices, "cline")

# --- EDITOR MODE ---
elif page == 'Editor':
    if "e_browsed_dir" not in st.session_state:
        st.session_state.e_browsed_dir = None

    def _clear_e_browsed_dir():
        st.session_state.e_browsed_dir = None

    map_area = st.container(key="map_area")
    top_bar = map_area.container(key="top_bar")
    map_layer = map_area.container(key="map_layer")
    notices = map_layer.container(key="notices")
    side_panel = st.container(key="side_panel")
    if "e_show_search" not in st.session_state:
        st.session_state.e_show_search = False

    select_half, hud_half = top_bar.columns(2)
    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
    row1 = select_half.columns([3, 0.7, 3, 1.6])
    with row1[0]:
        selected_dir_name = st.selectbox("Select Folder", ["Root (missions/)"] + existing_dirs, key="edit_dir", on_change=_clear_e_browsed_dir)
    with row1[1]:
        st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
        if st.button("📂", key="e_btn_browse_dir", help="Browse for a mission directory outside missions/"):
            picked = pick_folder_dialog("Select Mission Directory")
            if picked:
                st.session_state.e_browsed_dir = picked
                st.rerun()

    if st.session_state.e_browsed_dir:
        active_dir = st.session_state.e_browsed_dir
        dir_label = active_dir
        notices.caption(f"📁 Browsing: {active_dir}")
    else:
        active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
        dir_label = selected_dir_name

    try:
        kmz_files = [f for f in os.listdir(active_dir) if is_kmz_file(f)]
    except OSError as e:
        notices.error(f"Can't read {dir_label}: {e.strerror or e}")
        kmz_files = []

    if not kmz_files:
        notices.warning(f"No missions found in {dir_label}.")
    else:
        with row1[2]:
            selected_kmz = st.selectbox("Select Mission to Edit", kmz_files)
        with row1[3]:
            st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
            make_new_file = st.checkbox("Make new file?", value=False, help=param_help("Make new file?"))

        full_path = os.path.join(active_dir, selected_kmz)

        if 'editor_kmz' not in st.session_state or st.session_state.editor_kmz != full_path:
            st.session_state.editor_kmz = full_path
            meta = parse_kmz_for_editing(full_path)
            st.session_state.meta = meta
            st.session_state.editor_key = str(datetime.now().timestamp())

            if meta['coords']:
                st.session_state.locked_editor_center = list(meta['coords'][0])
                st.session_state.editor_center = list(meta['coords'][0])

            clean_base_name = strip_flight_suffix(kmz_companion_path(selected_kmz, ""))
            st.session_state.e_name_input = f"{clean_base_name}-edited" if make_new_file else clean_base_name

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
            e_hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()), index=list(HARDWARE_MAP.keys()).index(meta.get('hardware_key', "DJI Fly (RC2 / Mini / Air Series)")), help=param_help("Drone Platform"))
            e_drone_enum = HARDWARE_MAP[e_hw_choice]["drone_enum"]
            e_drone_sub_enum = HARDWARE_MAP[e_hw_choice]["drone_sub"]
            e_payload_enum = HARDWARE_MAP[e_hw_choice]["payload_enum"]
            e_payload_sub_enum = HARDWARE_MAP[e_hw_choice]["payload_sub"]
            e_is_dji_fly = HARDWARE_MAP[e_hw_choice].get("is_dji_fly", False)
            
            if e_is_dji_fly:
                st.warning("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash saving will be disabled if you exceed this.")
            
            current_cam_display = CAM_DISPLAY_MAP.get(meta.get('camera_type', 'visible'), "RGB Only")
            if e_is_dji_fly:
                e_cam_choice = "RGB Only"
                st.selectbox("Sensor Mode", ["RGB Only"], disabled=True,
                             help="DJI Fly aircraft are RGB only - sensor mode is not applicable.")
            else:
                e_cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"], index=["RGB Only", "Multispectral Only", "RGB + Multispectral"].index(current_cam_display), help=param_help("Sensor Mode"))
            e_camera_type = CAM_VAL_MAP[e_cam_choice]
            min_photo_interval_sec = 2.0 if "narrow_band" in e_camera_type else 0.7

            edit_name = sanitize_filename_component(st.text_input("Mission Name", key="e_name_input", help=param_help("Mission Name")))
            e_preview_suffix = f"_H{int(safe_get_float('e_alt_ft', 50.0))}A{int(abs(safe_get_float('e_pitch', -60.0)))}OL{int(safe_get_float('e_overlap_pct', 70.0))}"
            e_preview_platform = "Fly" if e_is_dji_fly else "Pilot"
            st.info(f"Will save as: {edit_name}_{e_preview_platform}{e_preview_suffix}.kmz")
            st.header("2. Modify Parameters")
            e_safe = st.number_input("Safe Takeoff Alt (ft)", value=meta['safe_takeoff_ft'], help=param_help("Safe Takeoff Alt (ft)"))
            e_trans = st.number_input("Takeoff Speed (mph)", value=meta['trans_speed_mph'], help=param_help("Takeoff Speed (mph)"))
            st.number_input("Relative Altitude (ft)", value=60.0, key="e_alt_ft", step=1.0, on_change=e_sync_geometry, help=param_help("Relative Altitude (ft)"))
            st.info("❗Elevation is relative to the take off point, NOT the mission start point.")

            e_elev_source = st.selectbox("Elevation Source", ["Open-Elevation (Global)", "USGS 3DEP (US High-Res)", "Local GeoTIFF"], key="e_source", help=param_help("Elevation Source"))
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
                    tif_files = [f for f in os.listdir(SURFACES_DIR) if f.lower().endswith((".tif", ".tiff"))]
                    if tif_files:
                        selected_tif = st.selectbox("Select Surface File", tif_files, key="e_tif")
                        e_tif_path = os.path.join(SURFACES_DIR, selected_tif)
                        e_show_bounds = st.checkbox("Show GeoTIFF Boundaries on Map", value=True, key="e_bounds")
                    else:
                        st.warning("No .tif files found in the 'surfaces' folder.")

            st.slider("Gimbal Pitch (°)", -90, 0, value= -60, key="e_pitch", on_change=e_sync_geometry, help=param_help("Gimbal Pitch (°)"))

            current_e_pitch = safe_get_float('e_pitch', -60.0)
            pitch_rad_e = math.radians(abs(current_e_pitch))
            current_e_alt = safe_get_float('e_alt_ft', 50.0)
            D_ft_e = current_e_alt / math.sin(pitch_rad_e) if pitch_rad_e > 0 else float('inf')
            st.info(f"Est. Ground GSD: {(D_ft_e * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_e != float('inf') else 0:.2f} cm/px")

            # Same rule as the Creator: no bounded footprint, no meaningful
            # overlap figure, so the field is disabled instead of inventing one.
            e_overlap_ok = finite_center_footprint(current_e_pitch, current_e_alt) is not None
            if not e_overlap_ok:
                st.warning(
                    f"At {current_e_pitch:.0f}° the camera is tilted less than its own "
                    f"{VERT_HALF_FOV_DEG:.1f}° half field-of-view, so the top of the frame "
                    "looks at or above the horizon and the photo footprint has no far edge. "
                    "Forward Overlap can't be calculated from that, so it is disabled - set "
                    "the photo interval directly. Tilt to about -28° or steeper to use overlap."
                )
            
            e_side = st.selectbox("Yaw Side", ["right", "left"], help=param_help("Yaw Side"))

            st.header("3. Trigger Settings")
            # Same as the Creator: inert on DJI Fly, so don't offer it.
            e_start_wp = st.number_input(
                "Start Photos at WP", min_value=0, value=meta['photo_start_wp'], step=1,
                disabled=e_is_dji_fly,
                help=("Not available on DJI Fly - it shoots at every waypoint from the start."
                      if e_is_dji_fly else param_help("Start Photos at WP")))
            if e_is_dji_fly:
                e_start_wp = 0
            e_trigger = st.radio("Type", ["distance", "time"], key="e_trigger_type", on_change=e_sync_geometry, help=param_help("Type"))
            safe_e_speed = max(2.3, float(meta.get('speed_mph', 6.0)))

            if st.session_state.get('e_trigger_type', 'distance') == "distance":
                st.number_input("Interval (ft)", key="e_t_dist_val", min_value=1.0, step=1.0, on_change=e_sync_dist_to_overlap, help=param_help("Interval (ft)"))
                st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0,
                                on_change=e_sync_overlap_to_dist, disabled=not e_overlap_ok,
                                help=(param_help("Forward Overlap (%)") if e_overlap_ok else
                                      "Unavailable at this gimbal pitch - see the notice above."))
                e_speed_m = st.number_input("Flight Speed (mph)", min_value=2.3, step=1.0, value=safe_e_speed, help=param_help("Flight Speed (mph)")) * MPH_TO_MS

                gap_m = max(1.0, safe_get_float('e_t_dist_val', 9.0) * FT_TO_M)
                max_speed_m = gap_m / min_photo_interval_sec
                if e_speed_m > max_speed_m:
                    st.error(f"Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
            else:
                if 'e_t_time_val' not in st.session_state:
                    st.session_state.e_t_time_val = meta['t_val'] if meta['trigger_type'] == 'time' else max(2.0, min_photo_interval_sec)
                e_tval_sec = st.number_input("Interval (sec)", key="e_t_time_val", min_value=min_photo_interval_sec, help=param_help("Interval (sec)"))
                e_auto_speed = st.checkbox("Auto-Calc Speed", True)
                if e_auto_speed:
                    st.number_input("Target Gap (ft)", key="e_target_gap_ft", min_value=1.0, on_change=e_sync_gap_to_overlap)
                    st.number_input("Forward Overlap (%)", key="e_overlap_pct", min_value=0.0, max_value=99.9, step=1.0,
                                    on_change=e_sync_overlap_to_gap, disabled=not e_overlap_ok,
                                    help=(param_help("Forward Overlap (%)") if e_overlap_ok else
                                          "Unavailable at this gimbal pitch - see the notice above."))
                    e_speed_m = min(max((safe_get_float('e_target_gap_ft', 26.2) * FT_TO_M) / e_tval_sec, 1.0), 10.0)
                    st.info(f"Auto-Calculated Speed: {e_speed_m * MS_TO_MPH:.1f} mph")
                else:
                    e_speed_m = st.number_input("Manual Speed (mph)", min_value=2.3, value=safe_e_speed, step=1.0, help=param_help("Manual Speed (mph)")) * MPH_TO_MS
                    fw = finite_center_footprint(safe_get_float('e_pitch', -60.0), safe_get_float('e_alt_ft', 50.0))
                    st.info(f"Current Overlap: {format_overlap(fw, e_speed_m * M_TO_FT * e_tval_sec)}")

            st.header("4. Visuals")
            show_footprints = st.checkbox("Show Image Footprints", value=True, help=param_help("Show Image Footprints"))
            if show_footprints and abs(safe_get_float('e_pitch', -60.0)) < VERT_HALF_FOV_DEG:
                st.warning(
                    f"Gimbal pitch is shallower than {VERT_HALF_FOV_DEG:.0f}° (the camera's vertical half-FOV), "
                    "so the top of the frame looks above the horizon and the footprint has no finite size on "
                    "the ground. Footprints are hidden at this angle."
                )
            show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="editor_faa_toggle", help=param_help("Show FAA Airspace Restrictions"))
            if show_faa_airspace:
                st.write("#### Update restrictions of map center")
                if st.button("Update Map Center", key="btn_update_editor"):
                    st.session_state.locked_editor_center = st.session_state.editor_center
                    st.rerun()

        top_hud = hud_half.container()
        side_panel.write("### Fine-Tune Flight Path Coordinates")

        df = pd.DataFrame(meta['coords'], columns=['Latitude', 'Longitude'])
        edited_df = side_panel.data_editor(df, num_rows="dynamic", key=st.session_state.editor_key, width='stretch')
        current_coords = [(row['Latitude'], row['Longitude']) for _, row in edited_df.iterrows()]

        with map_layer:
            # Constant view arguments - see the note on the Creator map.
            m_edit = folium.Map(location=st.session_state.locked_editor_center, zoom_start=18, tiles=None)
            add_basemap(m_edit)
        
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

            Draw(export=False, draw_options={
                'polyline': {'shapeOptions': {'color': '#00ffff', 'weight': 5}},
                'polygon': False, 'rectangle': False,
                'circle': False, 'circlemarker': False, 'marker': False,
            }).add_to(m_edit)
            line = folium.PolyLine(current_coords, color="#00ffff", weight=5).add_to(m_edit)
            PolyLineTextPath(line, '  ►  ', repeat=True, offset=7, attributes={'fill': '#000000', 'font-weight': 'bold', 'font-size': '24', 'fill-opacity': '0.3'}).add_to(m_edit)
        
            gap_ft_preview = max(1.0, safe_get_float('e_t_dist_val', 9.0) ) if st.session_state.get('e_trigger_type', 'distance') == "distance" else e_speed_m * safe_get_float('e_t_time_val', 2.0) * M_TO_FT#* M_TO_FT

            yaws = []
            for i in range(len(current_coords) - 1):
                ref_bearing = get_bearing(current_coords[i], current_coords[i+1])
                yaws.append((ref_bearing + 90) % 360 if e_side == "right" else (ref_bearing - 90) % 360)

            cum_dist = [0.0]
            total_dist_ft = 0.0
        
            # This is only the live map preview (elevation-diff labels drawn
            # while editing) - not what actually gets saved, which computes
            # its own elevations fresh at save time and aborts on total
            # failure instead (see the "Save & Update Mission" handler
            # below). A total elevation-service outage shouldn't make the
            # Editor itself unusable, so this falls back to a flat 0 just
            # for this preview render, with a visible warning so it's clear
            # the diff labels are unreliable right now rather than silently
            # wrong.
            try:
                elevations = get_elevations_batch(current_coords, e_elev_source, e_tif_path)
            except ElevationLookupError:
                notices.warning(f"Couldn't get {e_elev_source} elevation data - elevation-diff labels below are unavailable, but the mission will still save if you can restore terrain data by save time.")
                elevations = [0.0] * len(current_coords)
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
                            if footprint:
                                folium.Polygon(locations=footprint, color="darkorange", weight=1, fill=True, fill_opacity=0.15).add_to(m_edit)
                            folium.CircleMarker([lat, lon], radius=2.5, color="yellow", fill=True).add_to(m_edit)
                            break
                    current_dist += gap_ft_preview

            map_data_edit = st_folium(m_edit, use_container_width=True, height=1000, key="editor_map")

        if map_data_edit and map_data_edit.get("center"):
            st.session_state.editor_center = [map_data_edit["center"]["lat"], map_data_edit["center"]["lng"]]
            st.session_state.editor_zoom = map_data_edit["zoom"]
            c_lat, c_lon = st.session_state.editor_center
            with st.container(key="screen_center"):
                st.markdown(f"<div>Current Screen Center: {c_lat:.6f}, {c_lon:.6f}</div>", unsafe_allow_html=True)

        with st.container(key="search_toggle"):
            if st.button("📍 Jump to Address", key="e_search_toggle_btn"):
                st.session_state.e_show_search = not st.session_state.e_show_search

        if st.session_state.e_show_search:
            with st.container(key="search_bar"):
                e_search_col1, e_search_col2 = st.columns([5, 1])
                with e_search_col1:
                    e_search_query = st.text_input("Jump to Address or Lat/Lon", key="e_search_input", placeholder="e.g. 1600 Pennsylvania Ave or 40.25, -111.64")
                with e_search_col2:
                    st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
                    if st.button("Search Location", key="e_btn_search", width='stretch'):
                        with st.spinner("Searching..."):
                            new_coords = get_coords_from_search(e_search_query)
                            if new_coords:
                                st.session_state.locked_editor_center = new_coords
                                st.session_state.editor_center = new_coords
                                st.session_state.e_show_search = False
                                st.rerun()
                            else:
                                st.error("Location not found. Try a different query.")

        final_coords = [(c[1], c[0]) for c in map_data_edit["all_drawings"][-1]['geometry']['coordinates']] if map_data_edit.get("all_drawings") and len(map_data_edit["all_drawings"]) > 0 else current_coords
        if map_data_edit.get("all_drawings") and len(map_data_edit["all_drawings"]) > 0: notices.caption("Using newly drawn line from the map.")

        with top_hud:
            total_dist_ft = sum(get_haversine_dist(final_coords[i], final_coords[i+1]) for i in range(len(final_coords)-1)) * M_TO_FT
            e_gap_m = photo_gap_m(st.session_state.get('e_trigger_type', 'distance'),
                                  safe_get_float('e_t_dist_val', 9.0),
                                  safe_get_float('e_t_time_val', 2.0), e_speed_m)
            est_photos = count_corridor_photos(final_coords, e_gap_m, e_is_dji_fly, e_start_wp)
            
            c1, c2, c3 = st.columns(3)
            c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
            c2.metric("Estimated Photos", f"{est_photos}" + (" / 99" if e_is_dji_fly else ""))
            c3.metric("Flight Speed", f"{e_speed_m * MS_TO_MPH:.1f} mph")

            save_disabled = False
            if e_is_dji_fly and est_photos > 99:
                notices.error("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash please reduce your distance or increase the interval.")
                save_disabled = not render_99_override(notices, "edit", est_photos)

            if st.button("Save & Update Mission", disabled=save_disabled):
                with notices.spinner("Calculating terrain elevations and generating KMZ..."):
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

                    try:
                        template_kml, waylines_wpml = generate_native_kmz_contents(final_coords, new_cfg, e_elev_source, e_tif_path)
                    except ElevationLookupError as e:
                        notices.error(f"Save aborted: {e}")
                    else:
                        final_filepath = os.path.join(active_dir, f"{final_filename}.kmz")
                        export_mission_kmz_from_strings(
                            template_kml_str=template_kml,
                            waylines_wpml_str=waylines_wpml,
                            output_kmz_path=final_filepath,
                            is_dji_fly=e_is_dji_fly
                        )

                        thumbnail_path = kmz_companion_path(final_filepath)
                        generate_name_thumbnail(
                            e_prefixed_name, safe_get_float('e_alt_ft', 50.0), safe_get_float('e_pitch', -60.0),
                            safe_get_float('e_overlap_pct', 70.0), thumbnail_path, coords=final_coords, photo_count=est_photos
                        )

                        # "Make new file?" unchecked means overwrite, not "keep both" -
                        # since the suffix is re-derived from the current alt/pitch/
                        # overlap (and the name is freely editable), the save path
                        # very often differs from the source file even when the user
                        # never intended to keep a second copy. Remove the original
                        # (and its paired thumbnail) once the new save has succeeded.
                        if not make_new_file and final_filepath != full_path:
                            if os.path.exists(full_path):
                                os.remove(full_path)
                            old_thumbnail = kmz_companion_path(full_path)
                            if os.path.exists(old_thumbnail):
                                os.remove(old_thumbnail)

                        notices.success(f"Successfully updated and saved as {final_filename}.kmz in {dir_label}!")
                        offer_kmz_download(notices, "edit", final_filepath, f"{final_filename}.kmz")

            # Redrawn on every rerun, not only the one where Save was clicked -
            # see offer_kmz_download's docstring.
            offer_kmz_download(notices, "edit")

# ==========================================
# VIEWER MODE
# ==========================================
elif page == 'Viewer  |':
    if "v_browsed_dir" not in st.session_state:
        st.session_state.v_browsed_dir = None

    def _clear_v_browsed_dir():
        st.session_state.v_browsed_dir = None

    map_area = st.container(key="map_area")
    top_bar = map_area.container(key="top_bar")
    map_layer = map_area.container(key="map_layer")
    notices = map_layer.container(key="notices")

    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
    col_mission, col_dir, col_browse = top_bar.columns([4, 4, 1])
    with col_dir:
        selected_dir_name = st.selectbox("Select Folder", ["Root (missions/)"] + existing_dirs, key="view_dir", on_change=_clear_v_browsed_dir)
    with col_browse:
        st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
        if st.button("📂", key="v_btn_browse_dir", help="Browse for a mission directory outside missions/"):
            picked = pick_folder_dialog("Select Mission Directory")
            if picked:
                st.session_state.v_browsed_dir = picked
                st.rerun()

    if st.session_state.v_browsed_dir:
        active_dir = st.session_state.v_browsed_dir
        dir_label = active_dir
        notices.caption(f"📁 Browsing: {active_dir}")
    else:
        active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
        dir_label = selected_dir_name

    try:
        kmz_files = [f for f in os.listdir(active_dir) if is_kmz_file(f)]
    except OSError as e:
        notices.error(f"Can't read {dir_label}: {e.strerror or e}")
        kmz_files = []

    if not kmz_files:
        notices.warning(f"No missions found in {dir_label}.")
    else:
        with col_mission:
            view_multiple = st.checkbox("View multiple?", value=False)
            if view_multiple:
                selected_kmzs = st.multiselect("Select Missions", kmz_files, default=[kmz_files[0]] if kmz_files else [])
            else:
                sel = st.selectbox("Select Mission", kmz_files)
                selected_kmzs = [sel] if sel else []
        
        with st.sidebar:
            show_footprints = st.checkbox("Show Image Footprints", value=True, help=param_help("Show Image Footprints"))
            show_faa_airspace = st.checkbox("Show FAA Airspace Restrictions", value=False, key="viewer_faa_toggle", help=param_help("Show FAA Airspace Restrictions"))
            if show_faa_airspace:
                st.write("#### Update restrictions of map center")
                if st.button("Update Map Center", key="btn_update_viewer"):
                    st.session_state.locked_viewer_center = st.session_state.viewer_center
                    st.rerun()

        with map_layer:
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
                
                    meta = {
                        "speed": 0, "pitch": -60, "mode": "None", "t_val": 0, "alt": 50.0, "safe_alt": 0,
                        "start_idx": 0, "camera_type": "visible", "drone_sub": "0", "payload_sub": "3",
                        "trans_speed": 0, "finish": "", "height_mode": ""
                    }

                    p_node = root.find('.//{*}waypointGimbalHeadingParam/{*}waypointGimbalPitchAngle')
                    if p_node is not None: meta['pitch'] = float(p_node.text)
                    speed_node = root.find('.//{*}autoFlightSpeed')
                    if speed_node is not None: meta['speed'] = float(speed_node.text)
                    safe_node = root.find('.//{*}takeOffSecurityHeight')
                    if safe_node is not None: meta['safe_alt'] = float(safe_node.text)
                    trans_node = root.find('.//{*}globalTransitionalSpeed')
                    if trans_node is not None: meta['trans_speed'] = float(trans_node.text)
                    finish_node = root.find('.//{*}finishAction')
                    if finish_node is not None: meta['finish'] = finish_node.text
                    hmode_node = root.find('.//{*}executeHeightMode')
                    if hmode_node is not None: meta['height_mode'] = hmode_node.text

                    drone_info = root_t.find('.//{*}droneInfo')
                    if drone_info is not None:
                        d_sub = drone_info.find('.//{*}droneSubEnumValue')
                        if d_sub is not None and d_sub.text: meta['drone_sub'] = d_sub.text
                    payload_info = root_t.find('.//{*}payloadInfo')
                    if payload_info is not None:
                        p_sub = payload_info.find('.//{*}payloadSubEnumValue')
                        if p_sub is not None and p_sub.text: meta['payload_sub'] = p_sub.text

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
                        if not wp_data and alt_node is not None: meta['alt'] = float(alt_node.text)

                        target_yaw = yaw
                        # Whether THIS waypoint actually fires the shutter. On a
                        # mapping mission the turns between passes are waypoints
                        # with no takePhoto action, so counting waypoints instead
                        # of photo actions overstates the total - which is what
                        # made the Viewer disagree with the Creator's estimate.
                        wp_takes_photo = False
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
                                        wp_takes_photo = True
                                        params = a.find('.//{*}actionActuatorFuncParam')
                                        if params is not None:
                                            lens = params.find('.//{*}payloadLensIndex')
                                            if lens is not None: meta['camera_type'] = lens.text
                                    elif func.text == 'rotateYaw':
                                        params = a.find('.//{*}actionActuatorFuncParam')
                                        if params is not None:
                                            heading = params.find('.//{*}aircraftHeading')
                                            if heading is not None: target_yaw = float(heading.text)
                                    elif func.text == 'gimbalRotate' and p_node is None and not wp_data:
                                        # DJI Fly waypoints never carry
                                        # waypointGimbalPitchAngle - pitch is
                                        # instead set once via the first
                                        # waypoint's gimbalRotate action (see
                                        # parse_kmz_for_editing).
                                        params = a.find('.//{*}actionActuatorFuncParam')
                                        if params is not None:
                                            p_angle = params.find('.//{*}gimbalPitchRotateAngle')
                                            if p_angle is not None and p_angle.text: meta['pitch'] = float(p_angle.text)

                        wp_data.append({'lat': float(c_raw[1]), 'lon': float(c_raw[0]), 'yaw': yaw,
                                        'target_yaw': target_yaw, 'alt': alt, 'index': idx,
                                        'photo': wp_takes_photo})

                    if wp_data:
                        if m_view is None:
                            if 'current_viewer_file' not in st.session_state or st.session_state.current_viewer_file != selected_kmzs[0]:
                                st.session_state.current_viewer_file = selected_kmzs[0]
                                st.session_state.locked_viewer_center = [wp_data[0]['lat'], wp_data[0]['lon']]

                            # Constant view arguments - see the note on the Creator map.
                            m_view = folium.Map(location=st.session_state.locked_viewer_center, zoom_start=19, tiles=None)
                            add_basemap(m_view)
                        
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
                            total_dist_m += get_haversine_dist(
                                (wp_data[i]['lat'], wp_data[i]['lon']),
                                (wp_data[i+1]['lat'], wp_data[i+1]['lon'])
                            )
                            cum_dist.append(total_dist_m)

                        # A DJI Fly mission carries one waypoint per photo, so a
                        # leg-by-leg label puts 90+ overlapping numbers on the
                        # map. Collapse them onto the waypoints that actually
                        # mean something and summarise each span between them:
                        # the distance is the length of the whole span and the
                        # elevation figure its net change, not a single leg's.
                        is_dense_mission = (len(wp_data) > 4 and meta['mode'] == "None")
                        label_idx = (major_waypoint_indices([(w['lat'], w['lon']) for w in wp_data])
                                     if is_dense_mission else list(range(len(wp_data))))
                        for a, b in zip(label_idx, label_idx[1:]):
                            span_ft = (cum_dist[b] - cum_dist[a]) * M_TO_FT
                            elev_diff_ft = wp_data[b]['alt'] - wp_data[a]['alt']
                            anchor = wp_data[(a + b) // 2]
                            folium.Marker(
                                location=[anchor['lat'], anchor['lon']],
                                icon=DivIcon(icon_size=(120, 40), icon_anchor=(60, 20), html=f'<div style="font-size: 12pt; color: #ffffff; text-shadow: 2px 2px 4px #000000, -1px -1px 0 #000, 1px -1px 0 #000, -1px 1px 0 #000, 1px 1px 0 #000; font-weight: bold; text-align: center; line-height: 1.2;">{span_ft:.1f} ft<br><span style="font-size: 10pt; color: #00ffff;">Elev Dif: {elev_diff_ft:+.1f} ft</span></div>')
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
                        if is_dense_mission:
                            for w in wp_data:
                                # Turn waypoints between mapping passes carry no
                                # takePhoto action - they are flown through, not
                                # shot - so they are neither counted nor marked.
                                if not w['photo']:
                                    continue
                                if show_footprints:
                                    yaw = w['target_yaw']
                                    footprint = get_photo_footprint(w['lat'], w['lon'], w['alt'], meta['pitch'], yaw)
                                    if footprint:
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
                                            if footprint:
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

                        # Photo spacing: stated outright by an interval trigger,
                        # and on DJI Fly recovered from the gap between the
                        # waypoints that actually shoot (the median ignores the
                        # long hop across a mapping mission's turns).
                        photo_pts = [w for w in wp_data if w['photo']]
                        if meta['mode'] == 'Distance':
                            interval_ft = meta['t_val'] * M_TO_FT
                        elif len(photo_pts) > 1:
                            gaps = sorted(
                                get_haversine_dist((photo_pts[i]['lat'], photo_pts[i]['lon']),
                                                   (photo_pts[i+1]['lat'], photo_pts[i+1]['lon'])) * M_TO_FT
                                for i in range(len(photo_pts) - 1)
                            )
                            interval_ft = gaps[len(gaps) // 2]
                        else:
                            interval_ft = 0.0

                        fw_ft = finite_center_footprint(meta['pitch'], meta['alt'] * M_TO_FT)
                        overlap_text = format_overlap(fw_ft, interval_ft)

                        # Which way the camera looked, from the angle between the
                        # gimbal yaw and the direction of travel.
                        yaw_offsets = [
                            ((wp_data[i]['target_yaw'] - get_bearing(
                                (wp_data[i]['lat'], wp_data[i]['lon']),
                                (wp_data[i+1]['lat'], wp_data[i+1]['lon'])) + 180) % 360) - 180
                            for i in range(len(wp_data) - 1)
                        ]
                        if yaw_offsets:
                            avg_off = sum(yaw_offsets) / len(yaw_offsets)
                            if sum(abs(o) for o in yaw_offsets) / len(yaw_offsets) < 45:
                                cam_side = "Parallel (along path)"
                            else:
                                cam_side = "Right of path" if avg_off > 0 else "Left of path"
                        else:
                            cam_side = "Unknown"

                        st.sidebar.write(f"Hardware Platform: {hw_key}")
                        st.sidebar.write(f"Camera Sensor: {cam_display}")
                        st.sidebar.write(f"Gimbal Pitch: {meta['pitch']}°")
                        st.sidebar.write(f"Camera Side: {cam_side}")
                        st.sidebar.write(f"Waypoint Alt: {meta['alt']*M_TO_FT:.1f} ft")
                        st.sidebar.write(f"Safe Takeoff Alt: {meta['safe_alt']*M_TO_FT:.1f} ft")
                        if meta['trans_speed']:
                            st.sidebar.write(f"Takeoff Speed: {meta['trans_speed']*MS_TO_MPH:.1f} mph")
                        if meta['speed']:
                            st.sidebar.write(f"Flight Speed: {meta['speed']*MS_TO_MPH:.1f} mph")
                        st.sidebar.write(f"Trigger: {'Dense Waypoints (DJI Fly)' if meta['mode'] == 'None' else meta['mode']} ({meta['t_val']*M_TO_FT if meta['mode']=='Distance' else meta['t_val']:.1f})")
                        if interval_ft:
                            st.sidebar.write(f"Photo Interval: {interval_ft:.1f} ft")
                        st.sidebar.write(f"Forward Overlap: {overlap_text}")
                        if meta['mode'] != 'None' and meta['start_idx']:
                            st.sidebar.write(f"Photos Start at WP: {meta['start_idx']}")
                        # Side overlap and the mapping camera aim aren't stored
                        # anywhere in the KMZ, so they come off the filename
                        # suffix the Creator writes (_H..A..OL..SO..).
                        suffix = re.search(r'_H(\d+)A(\d+)OL(\d+)(?:SO(\d+))?', current_kmz)
                        if suffix and suffix.group(4):
                            st.sidebar.write(f"Side Overlap: {suffix.group(4)}% (from filename)")
                        st.sidebar.write(f"Waypoints: {len(wp_data)}")
                        st.sidebar.write(f"Total Distance: {grand_total_dist_ft:.1f} ft")
                        st.sidebar.write(f"Calculated Photos: {grand_total_photos}")
                        if meta['finish']:
                            st.sidebar.write(f"On Finish: {meta['finish']}")
                        if show_footprints and abs(meta['pitch']) < VERT_HALF_FOV_DEG:
                            st.sidebar.warning(
                                f"This mission's {meta['pitch']}° gimbal pitch is shallower than the camera's "
                                f"{VERT_HALF_FOV_DEG:.0f}° vertical half-FOV, so its footprints have no finite "
                                "ground size and are hidden."
                            )
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
                    map_data_view = st_folium(m_view, use_container_width=True, height=1000, key="viewer_map")
            
                    if map_data_view and map_data_view.get("center"):
                        st.session_state.viewer_center = [map_data_view["center"]["lat"], map_data_view["center"]["lng"]]
                        st.session_state.viewer_zoom = map_data_view["zoom"]
                        c_lat, c_lon = st.session_state.viewer_center
                        st.sidebar.info(f"Current Screen Center: {c_lat:.6f}, {c_lon:.6f} (Click 'Update' in sidebar to update restrictions in this area)")

# ==========================================
# PHOTO SORTER MODE
# ==========================================
elif page == 'Photo Sorter':
    with st.container(key="page_body"):
        st.header("Photo Sorter")
        st.write("Automatically group drone photos into separate folders based on the time they were taken.")
        st.write("This is for specifically for drones the use DJI Fly.")

        # Initialize default paths in session state so they don't reset
        if "sorter_source" not in st.session_state:
            st.session_state.sorter_source = os.path.expanduser("~")
        if "sorter_output" not in st.session_state:
            st.session_state.sorter_output = os.path.join(os.path.expanduser("~"), "Output")
        if "sorter_groups" not in st.session_state:
            st.session_state.sorter_groups = []

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
            
            # float min_value/value/step (not int) is what makes this field
            # accept fractional minutes - e.g. 0.5 for a 30 second gap -
            # instead of rounding every entry to the nearest whole minute.
            gap_minutes = st.number_input(
                "Time Gap (minutes)", min_value=0.1, value=5.0, step=0.1, format="%.1f",
                help="If the time between two sequential photos exceeds this gap, a new folder is "
                     "created. Fractional values are allowed - 0.5 is a 30 second gap."
            )
        
        st.write("---")
        st.checkbox(
            "Name each group myself before sorting",
            key="sorter_manual_naming",
            help="After the groups are found, review each one's first photo and give it a custom "
                 "folder name before anything gets copied - instead of the automatic Group_1, "
                 "Group_2... names.",
        )
        manual_naming = st.session_state.sorter_manual_naming

        find_btn = st.button("🔍 Find Groups" if manual_naming else "🚀 Sort Photos", width='stretch')

        if find_btn:
            source_dir = st.session_state.sorter_source
            output_dir = st.session_state.sorter_output

            if not source_dir or not os.path.exists(source_dir):
                st.error("The source directory does not exist or is invalid.")
            elif not output_dir:
                st.error("Please provide an output directory.")
            else:
                with st.spinner("Scanning for photo groups..."):
                    groups = find_photo_groups(source_dir, target_date, gap_minutes)
                if manual_naming:
                    # Stashed for the naming review below rather than sorted
                    # immediately - copying only happens once the user hits
                    # "Create Folders" there.
                    st.session_state.sorter_groups = groups
                elif groups:
                    with st.spinner("Sorting photos..."):
                        copy_photo_groups(groups, output_dir)

        if manual_naming and st.session_state.sorter_groups:
            groups = st.session_state.sorter_groups
            st.write("---")
            st.subheader("Name Your Groups")
            st.caption("Each group's first photo is shown for reference. Leave a name as-is to keep the automatic one.")

            for i, group in enumerate(groups):
                thumb_col, name_col = st.columns([1, 4])
                with thumb_col:
                    try:
                        thumb = Image.open(group[0]['path'])
                        thumb.thumbnail((120, 120))
                        st.image(thumb, width=100)
                    except Exception:
                        st.caption("Preview unavailable")
                with name_col:
                    st.text_input(
                        f"Group {i + 1} name ({len(group)} photos)",
                        value=default_group_folder_name(i, group),
                        key=f"sorter_group_name_{i}",
                    )

            if st.button("✅ Create Folders", width='stretch'):
                raw_names = [st.session_state.get(f"sorter_group_name_{i}", "") for i in range(len(groups))]
                cleaned_names = [
                    _WINDOWS_ILLEGAL_FILENAME_CHARS_RE.sub('', n).strip().rstrip('.') or None
                    for n in raw_names
                ]
                final_names = [
                    cleaned_names[i] or default_group_folder_name(i, groups[i])
                    for i in range(len(groups))
                ]

                name_counts = {}
                for name in final_names:
                    name_counts[name] = name_counts.get(name, 0) + 1
                duplicates = sorted({name for name, count in name_counts.items() if count > 1})

                if duplicates:
                    st.error(f"These group names are used more than once - make each one unique: {', '.join(duplicates)}")
                else:
                    with st.spinner("Sorting photos..."):
                        copy_photo_groups(groups, st.session_state.sorter_output, final_names)
                    st.session_state.sorter_groups = []

    # ==========================================
    # BATCH TRANSFER MODE
    # ==========================================
elif page == 'DJI Fly Transfer':
    with st.container(key="page_body"):
        st.header("DJI Fly Batch Mission Transfer")
        st.write("Assign local flight plans (left) to overwrite existing missions on the RC 2 (right).")
    
        if 'rc_nests' not in st.session_state:
            st.session_state.rc_nests = {}
            st.session_state.preview_id = None
            st.session_state.rc_scan_error = None

        col1, col2 = st.columns(2)
    
        if "batch_browsed_dir" not in st.session_state:
            st.session_state.batch_browsed_dir = None

        def _clear_batch_browsed_dir():
            st.session_state.batch_browsed_dir = None

        with col1:
            st.subheader("1. Source Missions")
            existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
            dir_col, browse_col = st.columns([5, 1])
            with dir_col:
                selected_dir_name = st.selectbox("Select Local Folder", ["Root (missions/)"] + existing_dirs, key="batch_dir", on_change=_clear_batch_browsed_dir)
            with browse_col:
                st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
                if st.button("📂", key="batch_btn_browse_dir", help="Browse for a mission directory outside missions/"):
                    picked = pick_folder_dialog("Select Mission Directory")
                    if picked:
                        st.session_state.batch_browsed_dir = picked
                        st.rerun()

            if st.session_state.batch_browsed_dir:
                active_dir = st.session_state.batch_browsed_dir
                dir_label = active_dir
                st.caption(f"📁 Browsing: {active_dir}")
            else:
                active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
                dir_label = selected_dir_name

            # This page is DJI Fly-only (MTP transfer to the RC 2's dummy mission
            # slots doesn't apply to DJI Pilot missions), so Pilot-format .kmz
            # files are filtered out rather than just listed alongside Fly ones.
            try:
                kmz_files = [
                    f for f in os.listdir(active_dir)
                    if is_kmz_file(f) and is_dji_fly_kmz(os.path.join(active_dir, f))
                ]
            except OSError as e:
                st.error(f"Can't read {dir_label}: {e.strerror or e}")
                kmz_files = []

            if not kmz_files:
                st.warning(f"No DJI Fly missions found in {dir_label}.")
            else:
                st.info(f"Found {len(kmz_files)} missions ready for transfer.")
            
        with col2:
            st.subheader("2. Controller Nests")
            st.write("Connect the RC 2 via USB, power on, and close Preview and Android File Transfer.")
            if st.button("🔄 Scan RC 2 & Pull Previews", width='stretch'):
                with st.spinner("Scanning MTP and downloading thumbnails... (This takes a few seconds)"):
                    st.session_state.rc_nests, st.session_state.preview_id, st.session_state.rc_scan_error = fetch_controller_nests_and_previews()

            if st.session_state.rc_nests:
                st.success(f"Found {len(st.session_state.rc_nests)} authorized mission slots.")
            elif st.session_state.rc_scan_error:
                st.error(f"Scan failed: {st.session_state.rc_scan_error}")
                st.caption("Full details (with traceback) were printed to the terminal streamlit was launched from.")
            else:
                st.warning("No controller connected, or no dummy missions found.")
                st.warning("If controller is connected, make sure preview app is closed.")

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
                        local_jpg = os.path.join(active_dir, kmz_companion_path(loc_choice))
                        if os.path.exists(local_jpg):
                            st.image(local_jpg, width='stretch')
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
                            st.image(cached_jpg, width='stretch', caption=f"Current: {nest_choice[-8:]}")
                        else:
                            st.info("Native Dummy Mission\n\n*(Preview unreadable over USB until overridden)*")
                
                if loc_choice != "--- Select Local Mission ---" and nest_choice != "--- Select Target Nest ---":
                    transfer_map[loc_choice] = nest_choice
            
                st.write("---")
                    
            if st.button("🚀 Execute Visual Transfer", width='stretch'):
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