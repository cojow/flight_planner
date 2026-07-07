"""
Direct ctypes bindings to libmtp, used for DJI Fly controller transfers in
ways the stock `mtp-*` CLI tools cannot do (or cannot do quickly):

- `mtp-sendfile` derives both the on-device filename and the PTP object-
  format code from the LOCAL file's name/extension, ignoring the desired
  remote name entirely. The RC 2's MTP responder also rejects the generic/
  unknown format code that .kmz/.zip/.txt files fall into, while accepting
  recognized media types. libmtp's API lets us set the destination filename
  and the object-format code independently, so a file can be labeled as an
  accepted type (e.g. JPEG) while still landing under the real name we want.

- `mtp-folders` and `mtp-files` always enumerate the ENTIRE device object
  store (every photo, video, thumbnail, cache and log file - tens of
  thousands of objects on a well-used controller), even though we only ever
  care about one small, known subtree
  (Android/data/dji.go.v5/files/waypoint). libmtp's
  `LIBMTP_Get_Files_And_Folders` lists only the direct children of one
  folder at a time, so walking down to that subtree touches a few dozen
  objects instead of the whole device - the difference between minutes and
  well under a second.

Requires libmtp's shared library to be installed (the same dependency the
`mtp-*` CLI tools already require).
"""
import ctypes
import ctypes.util
import os

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


def _load_library(candidates, friendly_name):
    last_err = None
    for cand in candidates:
        if not cand:
            continue
        try:
            return ctypes.CDLL(cand)
        except OSError as e:
            last_err = e
    raise MTPBridgeError(f"Could not load {friendly_name}. Tried: {candidates}. Last error: {last_err}")


def _find_libmtp():
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
    return _load_library(candidates, "libmtp")


def _find_libc():
    found = ctypes.util.find_library("c")
    candidates = [found, "libc.dylib", "libc.so.6", "msvcrt.dll"]
    return _load_library(candidates, "the C runtime library")


_mtp = _find_libmtp()
_libc = _find_libc()

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


def _collect_errorstack(device_ptr):
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
    operation (one scan, one transfer), instead of spawning a fresh `mtp-*`
    process - and therefore a fresh device session - for every single step.
    This avoids two problems observed with the CLI-based approach: the slow
    full-device enumeration `mtp-folders`/`mtp-files` always perform, and
    the device renumbering object IDs between separate process connections.

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
        Returns the DIRECT children of `parent_id` as a list of dicts with
        keys id/name/is_folder/size. Does not recurse and does not touch
        anything outside this one folder - the key difference from
        `mtp-files`/`mtp-folders`, which always walk the entire device.
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
        Pushes the bytes of `local_path`, landing inside `parent_folder_id`
        and named exactly `remote_filename`. The PTP object-format code is
        set to `disguise_filetype` (default: JPEG) rather than being derived
        from `remote_filename`'s extension - this is what lets non-media
        files (like .kmz) through devices that reject the generic/unknown
        format code. Returns (success: bool, message: str).
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
                errors = _collect_errorstack(self.device)
                detail = "; ".join(errors) if errors else "unknown error"
                return False, f"Send failed: {detail}"

            return True, f"Sent as '{remote_filename}' (item ID {file_struct.contents.item_id})"
        finally:
            _mtp.LIBMTP_destroy_file_t(file_struct)


def send_disguised_file(local_path, remote_filename, parent_folder_id,
                         disguise_filetype=LIBMTP_FILETYPE_JPEG, storage_id=0):
    """Convenience wrapper that opens a short-lived session for a single send. Prefer MTPSession for multi-step operations."""
    with MTPSession() as session:
        return session.send_disguised_file(local_path, remote_filename, parent_folder_id, disguise_filetype, storage_id)
