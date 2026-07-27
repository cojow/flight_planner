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

            local_jpg_path = local_kmz_path.replace('.kmz', '.jpg')
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
    zoom = max(3, min(20, zoom))

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
                url = f"https://mt1.google.com/vt/lyrs=m&x={tx}&y={ty}&z={zoom}"
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

def footprint_extents_ft(alt_ft, pitch, side="right"):
    """
    True ground-footprint dimensions (feet) for the mission's camera
    orientation (camera yawed 90 deg off the flight line, tilting to the
    chosen side), derived by projecting the sensor corners:
    - along_ft: extent parallel to the flight line (drives frontal overlap)
    - cross_ft: extent perpendicular to it (drives side overlap)
    - offset_ft: how far the footprint's cross-track center sits from the
      point directly below the drone (0 at nadir; grows with tilt)
    Computed with the flight line running east so along = |x|, cross = |y|.
    """
    yaw = 180.0 if side == "right" else 0.0
    pts = project_footprint_ft(alt_ft, pitch, yaw)
    if pts is None:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    along_ft = max(xs) - min(xs)
    cross_ft = max(ys) - min(ys)
    offset_ft = abs((max(ys) + min(ys)) / 2.0)
    return along_ft, cross_ft, offset_ft

def mapping_camera_geometry(alt_ft, pitch, frontal_overlap_pct, side_overlap_pct, side="right"):
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
    Pitch shallower than 20 degrees is clamped - the ground footprint
    stretches toward infinity as the camera approaches horizontal.
    """
    clamped_pitch = -min(90.0, max(20.0, abs(pitch)))
    extents = footprint_extents_ft(alt_ft, clamped_pitch, side)
    if extents is None:
        extents = footprint_extents_ft(alt_ft, -20.0, side)
    footprint_w_ft, footprint_h_ft, offset_ft = extents
    return {
        "footprint_w_ft": footprint_w_ft,
        "footprint_h_ft": footprint_h_ft,
        "offset_ft": offset_ft,
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

def generate_mapping_flight_path(boundary_coords, alt_ft, pitch, frontal_overlap_pct, side_overlap_pct, side="right"):
    """
    Builds a serpentine (lawnmower) flight path whose camera footprints
    fully cover the drawn boundary polygon, honoring altitude, gimbal
    pitch, and frontal/side overlap. The drone path itself may run outside
    the boundary: passes extend half a footprint past each end so photo
    coverage reaches the edges, and with an oblique gimbal the flight
    lines are offset sideways so the *imaged* strips (not the drone) land
    on the target area. Passes sweep parallel to the boundary's longest
    edge to minimize turns, alternating direction; because the camera
    stays on the drone's chosen side, the sideways offset flips with each
    direction change.

    Returns (path_coords, info) where path_coords is the corner-waypoint
    serpentine as (lat, lon) tuples, or (None, None) for a degenerate
    boundary.
    """
    if not boundary_coords or len(boundary_coords) < 3:
        return None, None

    geom = mapping_camera_geometry(alt_ft, pitch, frontal_overlap_pct, side_overlap_pct, side)
    fw, fh = geom["footprint_w_ft"], geom["footprint_h_ft"]
    offset, spacing = geom["offset_ft"], geom["spacing_ft"]

    # Project to a local planar frame in feet (equirectangular approximation,
    # fine at flight-area scale).
    lat0 = sum(c[0] for c in boundary_coords) / len(boundary_coords)
    lon0 = sum(c[1] for c in boundary_coords) / len(boundary_coords)
    cos_lat = math.cos(math.radians(lat0))
    pts = [((c[1] - lon0) * cos_lat * FT_PER_DEG_LAT, (c[0] - lat0) * FT_PER_DEG_LAT) for c in boundary_coords]

    # Sweep parallel to the longest boundary edge; rotate that edge horizontal.
    best_len, sweep_ang = 0.0, 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        edge_len = math.hypot(x2 - x1, y2 - y1)
        if edge_len > best_len:
            best_len, sweep_ang = edge_len, math.atan2(y2 - y1, x2 - x1)
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
    path_rot = []
    for k, c in enumerate(centers):
        x_lo, x_hi = band_x_range(c - fh / 2.0, c + fh / 2.0)
        # Extend so the end photos' footprints reach past the boundary edge.
        x_lo -= fw / 2.0
        x_hi += fw / 2.0
        eastbound = (k % 2 == 0)
        # Camera looks to the drone's chosen side of travel; place the drone
        # on the opposite side of the strip so the footprint lands on it.
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
    info["num_passes"] = len(centers)
    return path_coords, info

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
    # Along-track ground footprint (feet), used to convert between photo
    # interval and forward overlap. Uses the true projected footprint rather
    # than a center-slant estimate, so forward overlap matches the images at
    # any gimbal angle (identical to the old estimate at nadir, but the old
    # estimate under-reported the footprint - and so over-reported overlap -
    # as the camera tilted).
    if pitch == 0: return 999999.0
    extents = footprint_extents_ft(alt, pitch)
    return extents[0] if extents else 999999.0

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
def strip_flight_suffix(name):
    """
    Strips a previously-applied _Fly/_Pilot platform marker and
    _HxxAxxOLxx(SOxx) parameter suffix from a mission base name, so it can
    be cleanly regenerated from the current (possibly edited) altitude/
    pitch/overlap/platform values on save instead of stacking a second
    suffix on top of the old one.
    """
    return re.sub(r'(?:_(?:Fly|Pilot))?_H\d+A\d+OL\d+(?:SO\d+)?$', '', name)

def line_intersection_local(p_a, bearing_a, p_b, bearing_b):
    """
    Intersects two infinite lines - each given as a point plus a compass
    bearing - using a local-planar approximation valid for small areas.
    Returns None if the lines are within ~15 degrees of parallel: the true
    crossing point races toward infinity as two lines approach parallel,
    so ordinary coordinate rounding noise (KMZ coordinates are serialized
    to 8 decimal places) gets amplified by orders of magnitude there.
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
    if abs(denom) < 0.26:
        return None

    t = ((a2[0] - a1[0]) * d2[1] - (a2[1] - a1[1]) * d2[0]) / denom
    return from_local(a1[0] + t * d1[0], a1[1] + t * d1[1])

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
    if "map_boundary" not in st.session_state:
        st.session_state.map_boundary = None

    with st.sidebar:
        mapping_mode = st.checkbox(
            "Change to a mapping mission", value=False, key="mapping_mode",
            help="Draw the area you want mapped instead of a flight line. The flight "
                 "path is auto-calculated from altitude, gimbal pitch, and frontal/side "
                 "overlap, and may extend outside the drawn boundary."
        )

        st.header("1. Hardware & Payload")
        if mapping_mode:
            st.info("Mapping missions are generated as DJI Fly waypoint missions.")
            fly_hw = HARDWARE_MAP["DJI Fly (RC2 / Mini / Air Series)"]
            drone_enum = fly_hw["drone_enum"]
            drone_sub_enum = fly_hw["drone_sub"]
            payload_enum = fly_hw["payload_enum"]
            payload_sub_enum = fly_hw["payload_sub"]
            is_dji_fly = True
            st.warning("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash saving will be disabled if you exceed this.")
        else:
            hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()))
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
            cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"])
        camera_type = CAM_VAL_MAP[cam_choice]
        min_photo_interval_sec = 2.0 if "narrow_band" in camera_type else 0.7

        st.header("2. Global Config")
        mission_name = st.text_input("Filename", "Mission_Flight")
        trans_speed_mph = st.number_input("Takeoff Speed (mph)", value=22.0, step=1.0)
        safe_takeoff_ft = st.number_input("Safe Takeoff Alt (ft)", value=60.0, step=1.0)

        if mapping_mode:
            st.header("3. Mapping Settings")
            st.number_input("Relative Altitude (ft)", value=100.0, key="map_alt_ft", step=1.0)
        else:
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

        if mapping_mode:
            st.slider("Gimbal Pitch (°)", -90, -20, value=-90, key="map_pitch")

            map_alt = safe_get_float('map_alt_ft', 100.0)
            map_pitch_val = safe_get_float('map_pitch', -90.0)
            pitch_rad = math.radians(abs(map_pitch_val))
            D_ft_c = map_alt / math.sin(pitch_rad) if pitch_rad > 0 else float('inf')
            gsd_cm = (D_ft_c * FT_TO_M * SENSOR_W * 100) / (FOCAL_L * IMAGE_W) if D_ft_c != float('inf') else 0
            st.info(f"Est. Ground GSD: {gsd_cm:.2f} cm/px")

            side = st.selectbox("Camera side of flight path", ["right", "left"], key="map_side")

            st.header("4. Coverage & Speed")
            st.number_input("Frontal Overlap (%)", min_value=0.0, max_value=95.0, value=75.0, step=1.0, key="map_front_ol")
            st.number_input("Side Overlap (%)", min_value=0.0, max_value=95.0, value=65.0, step=1.0, key="map_side_ol")

            map_geom = mapping_camera_geometry(
                map_alt, map_pitch_val,
                safe_get_float('map_front_ol', 75.0), safe_get_float('map_side_ol', 65.0), side
            )
            st.info(
                f"Photo every {map_geom['interval_ft']:.0f} ft along track\n\n"
                f"Flight line spacing: {map_geom['spacing_ft']:.0f} ft"
            )

            manual_mph = st.number_input("Flight Speed (mph)", min_value=2.3, step=1.0, value=4.0, key="map_speed_mph")
            speed_m = manual_mph * MPH_TO_MS
            max_speed_m = (map_geom['interval_ft'] * FT_TO_M) / min_photo_interval_sec
            if speed_m > max_speed_m:
                st.error(f"Speed Too High! Lower your speed to {max_speed_m * MS_TO_MPH:.1f} mph.")
        else:
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
                    side
                )
                folium.Polygon(
                    locations=st.session_state.map_boundary, color="#00ffff", weight=3,
                    fill=True, fill_opacity=0.08, tooltip="Area to map"
                ).add_to(m)
                if preview_path:
                    path_line = folium.PolyLine(preview_path, color="#ff8800", weight=3, tooltip="Computed flight path").add_to(m)
                    PolyLineTextPath(path_line, '  ►  ', repeat=True, offset=7, attributes={'fill': '#000000', 'font-weight': 'bold', 'font-size': '18', 'fill-opacity': '0.4'}).add_to(m)
            except Exception:
                pass
    else:
        # Polyline only - a corridor mission is a single flight line, so the
        # area/point tools are disabled to avoid drawing shapes this mode
        # can't consume.
        Draw(export=False, draw_options={
            'polyline': {'shapeOptions': {'color': '#00ffff', 'weight': 5}},
            'polygon': False, 'rectangle': False,
            'circle': False, 'circlemarker': False, 'marker': False,
        }).add_to(m)

    map_data = st_folium(m, width=1200, height=600, key="creator_map")

    if mapping_mode:
        detected_boundary = extract_polygon_from_map_data(map_data)
        if detected_boundary and detected_boundary != st.session_state.map_boundary:
            st.session_state.map_boundary = detected_boundary
            st.rerun()  # re-render immediately so the computed path overlay appears

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

    if mapping_mode:
        boundary = st.session_state.map_boundary
        if not boundary:
            st.info("Draw the area you want to map using the polygon or rectangle tool on the map. "
                    "The flight path will be calculated automatically and may extend outside the boundary.")
        else:
            map_alt = safe_get_float('map_alt_ft', 100.0)
            map_pitch_val = safe_get_float('map_pitch', -90.0)
            map_front_ol = safe_get_float('map_front_ol', 75.0)
            map_side_ol = safe_get_float('map_side_ol', 65.0)

            path_coords, map_info = generate_mapping_flight_path(
                boundary, map_alt, map_pitch_val, map_front_ol, map_side_ol, side
            )

            if path_coords:
                total_dist_ft = sum(get_haversine_dist(path_coords[i], path_coords[i+1]) for i in range(len(path_coords)-1)) * M_TO_FT
                gap_ft = map_info['interval_ft']
                est_photos = int(total_dist_ft / gap_ft) + 1 if gap_ft > 0 else 0

                save_disabled = False
                if est_photos > 99:
                    st.error("DJI Fly greatly lags with more than 99 waypoints (photos). To prevent a crash please shrink the area, raise the altitude, or reduce the overlaps.")
                    save_disabled = True

                with top_hud:
                    c1, c2, c3, c4 = st.columns([1.2, 1.2, 1, 1.6])
                    c1.metric("Total Path Distance", f"{total_dist_ft:.1f} ft")
                    c2.metric("Estimated Photos", f"{est_photos} / 99")
                    c3.metric("Passes", f"{map_info['num_passes']}")
                    with c4:
                        st.markdown("<div style='margin-top: 10px;'></div>", unsafe_allow_html=True)
                        save_clicked = st.button("Save & Generate KMZ", use_container_width=True, disabled=save_disabled)
                        if st.button("🗑 Clear boundary", use_container_width=True, key="btn_clear_boundary"):
                            st.session_state.map_boundary = None
                            st.rerun()
                    st.write("---")

                    if save_clicked:
                        with st.spinner("Calculating terrain elevations and generating KMZ..."):
                            cfg = {
                                "safe_takeoff_ft": safe_takeoff_ft, "trans_speed_mph": trans_speed_mph,
                                "alt_ft": map_alt, "pitch": map_pitch_val, "side": side,
                                "trigger_type": "distance",
                                "interval_ft": gap_ft,
                                "interval_sec": 0.0,
                                "speed_m": speed_m, "photo_start_wp": 0,
                                "camera_type": camera_type, "drone_sub": drone_sub_enum, "payload_sub": payload_sub_enum,
                                "is_dji_fly": True
                            }

                            prefixed_name = f"{mission_name}_Fly"
                            suffix = f"_H{int(map_alt)}A{int(abs(map_pitch_val))}OL{int(map_front_ol)}SO{int(map_side_ol)}"
                            final_filename = f"{prefixed_name}{suffix}"

                            if st.session_state.c_browsed_dir:
                                final_dir = st.session_state.c_browsed_dir
                            elif save_option == "Root (missions/)": final_dir = MISSION_DIR
                            elif save_option == "Create New Folder...": final_dir = os.path.join(MISSION_DIR, new_dir_name)
                            else: final_dir = os.path.join(MISSION_DIR, save_option)

                            os.makedirs(final_dir, exist_ok=True)
                            final_filepath = os.path.join(final_dir, f"{final_filename}.kmz")

                            template_kml, waylines_wpml = generate_native_kmz_contents(path_coords, cfg, c_elev_source, c_tif_path)
                            export_mission_kmz_from_strings(
                                template_kml_str=template_kml,
                                waylines_wpml_str=waylines_wpml,
                                output_kmz_path=final_filepath,
                                is_dji_fly=True
                            )
                            thumbnail_path = final_filepath.replace('.kmz', '.jpg')
                            generate_name_thumbnail(
                                prefixed_name, map_alt, map_pitch_val,
                                map_front_ol, thumbnail_path, coords=path_coords, photo_count=est_photos
                            )

                        st.success(f"Saved {final_filename}.kmz to {final_dir}/")
                    st.divider()

    elif map_data.get("all_drawings") and any(
        d.get('geometry', {}).get('type') == 'LineString' for d in map_data["all_drawings"]
    ):
        # Only consider drawn lines here - a polygon left over from mapping
        # mode has a nested-ring geometry this corridor flow can't consume.
        line_drawing = [d for d in map_data["all_drawings"] if d.get('geometry', {}).get('type') == 'LineString'][-1]
        coords = [(c[1], c[0]) for c in line_drawing['geometry']['coordinates']]
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
                        safe_get_float('overlap_pct', 70.0), thumbnail_path, coords=coords, photo_count=est_photos
                    )

                st.success(f"Saved {final_filename}.kmz to {final_dir}/")
            st.divider()

# --- EDITOR MODE ---
elif page == 'Editor':
    if "e_browsed_dir" not in st.session_state:
        st.session_state.e_browsed_dir = None

    def _clear_e_browsed_dir():
        st.session_state.e_browsed_dir = None

    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
    col_dir, col_browse, col_file, col_new = st.columns([2, 0.6, 3, 1])
    with col_dir:
        selected_dir_name = st.selectbox("Select Folder", ["Root (missions/)"] + existing_dirs, key="edit_dir", on_change=_clear_e_browsed_dir)
    with col_browse:
        st.markdown("<div style='margin-top: 28px;'></div>", unsafe_allow_html=True)
        if st.button("📂", key="e_btn_browse_dir", help="Browse for a mission directory outside missions/"):
            picked = pick_folder_dialog("Select Mission Directory")
            if picked:
                st.session_state.e_browsed_dir = picked
                st.rerun()

    if st.session_state.e_browsed_dir:
        active_dir = st.session_state.e_browsed_dir
        dir_label = active_dir
        st.caption(f"📁 Browsing: {active_dir}")
    else:
        active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
        dir_label = selected_dir_name

    kmz_files = [f for f in os.listdir(active_dir) if f.endswith(".kmz")]

    if not kmz_files:
        st.warning(f"No missions found in {dir_label}.")
    else:
        with col_file: selected_kmz = st.selectbox("Select Mission to Edit", kmz_files)
        with col_new:
            st.markdown("<div style='margin-top: 32px;'></div>", unsafe_allow_html=True)
            make_new_file = st.checkbox("Make new file?", value=False)

        full_path = os.path.join(active_dir, selected_kmz)

        if 'editor_kmz' not in st.session_state or st.session_state.editor_kmz != full_path:
            st.session_state.editor_kmz = full_path
            meta = parse_kmz_for_editing(full_path)
            st.session_state.meta = meta
            st.session_state.editor_key = str(datetime.now().timestamp())

            if meta['coords']:
                st.session_state.locked_editor_center = list(meta['coords'][0])
                st.session_state.editor_center = list(meta['coords'][0])

            clean_base_name = strip_flight_suffix(selected_kmz.replace('.kmz', ''))
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
            e_hw_choice = st.selectbox("Drone Platform", list(HARDWARE_MAP.keys()), index=list(HARDWARE_MAP.keys()).index(meta.get('hardware_key', "DJI Fly (RC2 / Mini / Air Series)")))
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
                e_cam_choice = st.selectbox("Sensor Mode", ["RGB Only", "Multispectral Only", "RGB + Multispectral"], index=["RGB Only", "Multispectral Only", "RGB + Multispectral"].index(current_cam_display))
            e_camera_type = CAM_VAL_MAP[e_cam_choice]
            min_photo_interval_sec = 2.0 if "narrow_band" in e_camera_type else 0.7
            
            edit_name = st.text_input("Mission Name", key="e_name_input")
            e_preview_suffix = f"_H{int(safe_get_float('e_alt_ft', 50.0))}A{int(abs(safe_get_float('e_pitch', -60.0)))}OL{int(safe_get_float('e_overlap_pct', 70.0))}"
            e_preview_platform = "Fly" if e_is_dji_fly else "Pilot"
            st.info(f"Will save as: {edit_name}_{e_preview_platform}{e_preview_suffix}.kmz")
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
                        old_thumbnail = full_path.replace('.kmz', '.jpg')
                        if os.path.exists(old_thumbnail):
                            os.remove(old_thumbnail)

                st.success(f"Successfully updated and saved as {final_filename}.kmz in {dir_label}!")
            st.divider()

# ==========================================
# VIEWER MODE
# ==========================================
elif page == 'Viewer  |':
    if "v_browsed_dir" not in st.session_state:
        st.session_state.v_browsed_dir = None

    def _clear_v_browsed_dir():
        st.session_state.v_browsed_dir = None

    existing_dirs = [d for d in os.listdir(MISSION_DIR) if os.path.isdir(os.path.join(MISSION_DIR, d)) and d != ".cache"]
    col_dir, col_browse, col_file, col_multi = st.columns([2, 0.6, 3, 1])
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
        st.caption(f"📁 Browsing: {active_dir}")
    else:
        active_dir = MISSION_DIR if selected_dir_name == "Root (missions/)" else os.path.join(MISSION_DIR, selected_dir_name)
        dir_label = selected_dir_name

    kmz_files = [f for f in os.listdir(active_dir) if f.endswith(".kmz")]

    if not kmz_files:
        st.warning(f"No missions found in {dir_label}.")
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
        kmz_files = [
            f for f in os.listdir(active_dir)
            if f.endswith(".kmz") and is_dji_fly_kmz(os.path.join(active_dir, f))
        ]

        if not kmz_files:
            st.warning(f"No DJI Fly missions found in {dir_label}.")
        else:
            st.info(f"Found {len(kmz_files)} missions ready for transfer.")
            
    with col2:
        st.subheader("2. Controller Nests")
        st.write("Connect the RC 2 via USB, power on, and close Preview and Android File Transfer.")
        if st.button("🔄 Scan RC 2 & Pull Previews", use_container_width=True):
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