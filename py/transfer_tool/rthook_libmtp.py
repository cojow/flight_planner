"""
PyInstaller runtime hook: point libmtp lookups at the copy inside the bundle.

Runs before any application module imports, which matters because
dji_transfer_core loads libmtp at import time - by the time the app's own
code runs it is already too late to influence the search.

The core resolves the library as:

    found = ctypes.util.find_library("mtp")
    candidates = ([found] if found else []) + [hardcoded names/paths...]

so `find_library` result goes to the head of the queue. Patching that one
function therefore redirects the load with no change to the generated core
or to app.py - the hardcoded Homebrew/system paths stay as the fallback for
running from source. (On this Mac find_library("mtp") already returns None
and the hardcoded list is what works, so the patch is doing real work here,
not just reordering.)

Windows needs none of this: that backend is WPD/comtypes, and WPD ships with
the OS.
"""
import ctypes.util
import os
import sys

_original_find_library = ctypes.util.find_library

# Ordered by platform likelihood; the first one present in the bundle wins.
_BUNDLED_NAMES = {
    "mtp": ("libmtp.9.dylib", "libmtp.dylib", "libmtp.so.9", "libmtp.so"),
    "usb-1.0": ("libusb-1.0.0.dylib", "libusb-1.0.dylib", "libusb-1.0.so.0"),
}


def _bundled_find_library(name):
    bundle_dir = getattr(sys, "_MEIPASS", None)
    if bundle_dir:
        for candidate in _BUNDLED_NAMES.get(name, ()):
            path = os.path.join(bundle_dir, candidate)
            if os.path.exists(path):
                return path
    return _original_find_library(name)


ctypes.util.find_library = _bundled_find_library
