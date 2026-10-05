"""
DJI Fly controller transfer - standalone core, no Streamlit/folium/shapely.

The MTP/WPD layer and the two transfer entry points from app.py, so the
desktop helper can push missions to an RC 2 without dragging in the whole
planner (or, once frozen, a Python install at all):

  get_mtp_session_class()                - the backend for this OS, or None
  fetch_controller_nests_and_previews()  - scan the controller for slots
  push_mission_to_nest(kmz_path, uuid)   - write one mission into a slot

GENERATED FILE - do not hand-edit. Fix app.py, then re-run build_core.py.
`python build_core.py --check` verifies this copy still matches app.py.
"""
import os
import re
import io
import time
import shlex
import ctypes
import ctypes.util
import platform
import logging
import subprocess

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("dji_fly_transfer")

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
            client_info.SetStringValue(byref(WPD_CLIENT_NAME_KEY), "Flight Planner")
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
