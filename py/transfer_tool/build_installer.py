"""
Freeze the transfer helper into a single download that needs no Python.

    pip install pyinstaller        # build machine only, never the student's
    python build_core.py          # refresh the copy of app.py's MTP layer
    python build_installer.py     # -> dist/

There is no cross-compiling: the Mac build has to happen on a Mac and the
Windows build on Windows. Run this once per platform you hand out.

WHAT GOES IN, PER PLATFORM

  macOS / Linux   libmtp is a real shared library that must be carried
                  inside the bundle, along with the libusb it links
                  against. Both normally live under Homebrew (or the
                  distro's lib dir) at absolute paths that will not exist
                  on a student's machine, so PyInstaller copies them in and
                  rewrites their load commands; rthook_libmtp.py then
                  points the lookup at the copies. Build on the OLDEST
                  macOS you intend to support - the bundle is not
                  backwards-compatible with older systems than the one it
                  was built on.

  Windows         Nothing to bundle. That backend is WPD through comtypes,
                  and WPD is part of Windows. comtypes generates its COM
                  wrappers at runtime, so its generated-module package is
                  forced in as a hidden import - the usual reason a frozen
                  comtypes app dies with ModuleNotFoundError on a machine
                  that never ran it from source.

SIGNING - the part that bites students, not you

  The output is unsigned. macOS Gatekeeper will refuse a plain double-click
  on a downloaded unsigned app: the first launch has to be right-click ->
  Open -> Open, once per machine. Windows SmartScreen shows "More info ->
  Run anyway". Both are avoidable only by paying for signing certificates
  (Apple Developer, ~$99/yr, plus notarisation; an Authenticode cert for
  Windows). Tell people about this in advance or the first support question
  will be "it says the app is damaged".
"""
import os
import platform
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
APP_NAME = "DJI Fly Mission Transfer"
ENTRY = os.path.join(HERE, "dji_transfer_app.py")

# The names libmtp is actually installed under, most specific first. Mirrors
# the candidate list inside app.py's own loader.
LIBMTP_CANDIDATES = [
    "/opt/homebrew/opt/libmtp/lib/libmtp.9.dylib",   # Apple silicon Homebrew
    "/usr/local/opt/libmtp/lib/libmtp.9.dylib",      # Intel Homebrew
    "/opt/homebrew/lib/libmtp.dylib",
    "/usr/local/lib/libmtp.dylib",
    "/usr/lib/x86_64-linux-gnu/libmtp.so.9",
    "/usr/lib/libmtp.so.9",
]


def find_libmtp():
    """Absolute path to a real (symlink-resolved) libmtp, or None."""
    import ctypes.util
    found = ctypes.util.find_library("mtp")
    for candidate in ([found] if found else []) + LIBMTP_CANDIDATES:
        if candidate and os.path.exists(candidate):
            return os.path.realpath(candidate)
    return None


def preflight():
    problems = []

    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        problems.append("PyInstaller is not installed. Run:  pip install pyinstaller")

    if not os.path.exists(os.path.join(HERE, "dji_transfer_core.py")):
        problems.append("dji_transfer_core.py is missing. Run:  python build_core.py")
    else:
        drifted = subprocess.run(
            [sys.executable, os.path.join(HERE, "build_core.py"), "--check"],
            capture_output=True, text=True,
        )
        if drifted.returncode != 0:
            problems.append(
                "dji_transfer_core.py has drifted from app.py - the helper would ship "
                "different transfer code than the planner. Run:  python build_core.py"
            )

    if platform.system() != "Windows" and find_libmtp() is None:
        problems.append(
            "libmtp not found, so the build would produce an app that cannot reach a "
            "controller. Install it first (macOS:  brew install libmtp)."
        )

    return problems


def build():
    problems = preflight()
    if problems:
        print("Cannot build:\n")
        for problem in problems:
            print(f"  - {problem}")
        return 1

    # Windows gets a single .exe, which is the friendliest thing to hand
    # someone there. macOS does NOT: a .app is a directory by definition, and
    # wrapping a onefile binary in one makes it unpack itself into a temp
    # directory on every launch - which is both slower and exactly the
    # behaviour Gatekeeper treats as suspicious on a downloaded app.
    # PyInstaller deprecated that combination and makes it an error in v7.
    # A .app in a .zip is the normal way to hand out a Mac app anyway.
    onefile = platform.system() == "Windows"

    args = [
        ENTRY,
        "--name", APP_NAME,
        "--onefile" if onefile else "--onedir",
        "--windowed",                       # no console window behind the UI
        "--noconfirm",
        "--clean",
        "--runtime-hook", os.path.join(HERE, "rthook_libmtp.py"),
        "--distpath", os.path.join(HERE, "dist"),
        "--workpath", os.path.join(HERE, "build"),
        "--specpath", os.path.join(HERE, "build"),
        # Nothing here needs the scientific stack; excluding it keeps the
        # download small if the build environment happens to have it.
        "--exclude-module", "streamlit",
        "--exclude-module", "folium",
        "--exclude-module", "pandas",
        "--exclude-module", "numpy",
        "--exclude-module", "matplotlib",
        "--exclude-module", "shapely",
        "--exclude-module", "rasterio",
    ]

    if platform.system() == "Windows":
        # comtypes builds its wrappers at runtime, so the generated package
        # is invisible to static analysis and has to be named explicitly.
        args += ["--hidden-import", "comtypes", "--hidden-import", "comtypes.gen"]
    else:
        libmtp = find_libmtp()
        separator = ";" if platform.system() == "Windows" else ":"
        args += ["--add-binary", f"{libmtp}{separator}."]
        print(f"bundling libmtp from: {libmtp}")
        # libusb is picked up automatically as one of libmtp's own linked
        # dependencies, but say so if it is missing, since a bundle without
        # it fails only later, on a student's machine, at device-open time.
        try:
            linked = subprocess.run(["otool", "-L", libmtp], capture_output=True, text=True).stdout
            if "usb" not in linked:
                print("  WARNING: libmtp does not appear to link libusb - check the bundle")
        except FileNotFoundError:
            pass

    print(f"\nbuilding {APP_NAME} for {platform.system()}...\n")
    # Anything in dist/ older than this came from a previous build - see the
    # listing below, which refuses to present those as this build's output.
    started_at = time.time()
    import PyInstaller.__main__
    PyInstaller.__main__.run(args)

    dist = os.path.join(HERE, "dist")

    # The zip is made here rather than left as a command to copy-paste. When
    # it was a printed instruction, the listing below still found last build's
    # zip sitting in dist/ and reported it under "built:" - so a stale app
    # looked freshly built, and got shipped. Build it, or say it isn't there.
    if platform.system() == "Darwin":
        zip_name = f"{APP_NAME}.zip"
        zip_path = os.path.join(dist, zip_name)
        # Rebuilt from scratch: zip ADDS to an existing archive rather than
        # replacing it, which would leave the previous build's files inside.
        if os.path.exists(zip_path):
            os.remove(zip_path)
        # The system zip, not shutil.make_archive, because a .app is full of
        # symlinks and executable bits that make_archive does not preserve -
        # an archive that unpacks into an app that won't launch.
        print(f"\nzipping {APP_NAME}.app (a bare .app loses its executable bit in transit)...")
        zipped = subprocess.run(["zip", "-qry", zip_name, f"{APP_NAME}.app"], cwd=dist)
        if zipped.returncode != 0:
            print(f"  WARNING: zip failed ({zipped.returncode}) - zip the .app yourself before handing it out")

    print("\nbuilt:")
    # Only what this run actually produced. Anything else in dist/ is left
    # over from an earlier build and is called out separately rather than
    # being listed as if it were new.
    fresh, stale = [], []
    for entry in sorted(os.listdir(dist)):
        path = os.path.join(dist, entry)
        size = _tree_size(path) / (1024 * 1024)
        line = f"  {entry}  ({size:.1f} MB)"
        (fresh if os.path.getmtime(path) >= started_at else stale).append(line)
    for line in fresh:
        print(line)
    if stale:
        print("\nalso in dist/, left over from an earlier build - do not hand these out:")
        for line in stale:
            print(line)
    print(
        "\nThis build is unsigned - see this script's notes. First launch on another "
        "Mac needs right-click -> Open; Windows needs 'More info -> Run anyway'."
    )
    return 0


def _tree_size(path):
    if os.path.isfile(path):
        return os.path.getsize(path)
    return sum(
        os.path.getsize(os.path.join(root, f))
        for root, _dirs, files in os.walk(path) for f in files
    )


if __name__ == "__main__":
    raise SystemExit(build())
