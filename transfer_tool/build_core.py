"""
Generates dji_transfer_core.py from app.py.

The desktop helper needs app.py's MTP/WPD layer but none of the planner, and
app.py deliberately keeps that layer inlined (see its own note on import-path
fragility) while being deployed to a live class - so the helper gets a copy
rather than app.py getting refactored underneath its users.

A copy only stays trustworthy if drift is detectable, hence this script:

    python build_core.py            # regenerate the copy
    python build_core.py --check    # fail if the copy has drifted from app.py

Run --check in CI, or any time before building an installer, so a fix that
lands in app.py's bridge can't quietly fail to reach the helper app.

Blocks are located by AST (function/class names) and by distinctive comment
text, never by raw line number, so ordinary edits to app.py shift them
harmlessly instead of silently slicing the wrong lines.
"""
import argparse
import ast
import difflib
import os
import sys

APP_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "app.py")
OUT_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dji_transfer_core.py")

HEADER = '''"""
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

'''

# Each entry is ("kind", locator). "span" runs from the line holding the
# given comment text through the end of the named definition; "def" is one
# function or class by name.
BLOCKS = [
    ("span", ("# MTP BRIDGE (direct libmtp bindings", "get_mtp_session_class")),
    ("def", "kill_macos_hijackers"),
    ("span", ("# The path, as a chain of folder names", "UUID_RE")),
    ("def", "kmz_companion_path"),
    ("def", "fetch_controller_nests_and_previews"),
    ("def", "push_mission_to_nest"),
]


def _definition_bounds(tree, name):
    """1-based inclusive (start, end) lines of a top-level def/class/assign."""
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and node.name == name:
            return node.lineno, node.end_lineno
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return node.lineno, node.end_lineno
    raise SystemExit(f"build_core.py: could not find a top-level definition named {name!r} in app.py")


def _comment_line(lines, needle):
    """1-based line number of the sole line containing `needle`."""
    hits = [i + 1 for i, line in enumerate(lines) if needle in line]
    if not hits:
        raise SystemExit(f"build_core.py: anchor comment not found in app.py: {needle!r}")
    if len(hits) > 1:
        raise SystemExit(f"build_core.py: anchor comment is ambiguous ({len(hits)} matches): {needle!r}")
    return hits[0]


def generate():
    with open(APP_PY, encoding="utf-8") as f:
        source = f.read()
    lines = source.splitlines()
    tree = ast.parse(source)

    chunks = []
    for kind, locator in BLOCKS:
        if kind == "def":
            start, end = _definition_bounds(tree, locator)
        else:
            anchor, end_name = locator
            # Back up one line to take the "# ====" banner above the comment.
            start = max(1, _comment_line(lines, anchor) - 1)
            _, end = _definition_bounds(tree, end_name)
        chunks.append("\n".join(lines[start - 1:end]))

    return HEADER + "\n\n\n".join(chunks) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="exit non-zero if the generated file has drifted from app.py")
    args = parser.parse_args()

    generated = generate()

    if args.check:
        try:
            with open(OUT_PY, encoding="utf-8") as f:
                current = f.read()
        except FileNotFoundError:
            print("dji_transfer_core.py is missing - run: python build_core.py")
            return 1
        if current != generated:
            diff = difflib.unified_diff(
                current.splitlines(keepends=True), generated.splitlines(keepends=True),
                fromfile="dji_transfer_core.py (on disk)", tofile="regenerated from app.py",
            )
            sys.stdout.writelines(diff)
            print("\ndji_transfer_core.py has drifted from app.py - run: python build_core.py")
            return 1
        print("dji_transfer_core.py is in sync with app.py")
        return 0

    with open(OUT_PY, "w", encoding="utf-8") as f:
        f.write(generated)
    print(f"wrote {OUT_PY} ({len(generated.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
