"""
Talking to the Census: the API key, and GET requests cached on disk so a
repeated run spends no API calls and gives the same answer.

Every raw input a step reads from disk - ACS answers in cache/, TIGER/Line
files in tiger/, FAA facility maps in faa/, national security areas in
nsufr/, all under data/census/ - is
recorded with `record`, so steps.py can list them in the step's inputs.json
and `python -m census publish` can archive exactly those files.

In offline mode (`offline()`, used by `python -m census reproduce`) nothing is
fetched: a missing input is an error, and the FAA map and national security
areas come from the snapshots named, so a reproduction uses the same data as
the run it reproduces.
"""
import contextlib
import hashlib
import json
import os
from pathlib import Path

import requests

from .config import REPO_ROOT

ACS_URL = "https://api.census.gov/data/{year}/acs/acs5"
TIGER_URL = "https://tigerweb.geo.census.gov/arcgis/rest/services/TIGERweb"
DATA_DIR = REPO_ROOT / "data" / "census"
CACHE_DIR = DATA_DIR / "cache"

# How data is got right now: live (the default), or offline from what is on
# disk, with the FAA map and national security areas read from saved
# snapshots. Set with `offline()`.
MODE = {"offline": False, "faa_file": None, "no_fly_file": None}
_used = set()           # the input files read since the last `start_recording`

# Where the key lives by default: a one-line text file in py/. It is
# gitignored - this repository is public, and a committed key is a published key.
KEY_FILE = Path(__file__).resolve().parents[1] / "census_api_key.txt"


class CensusError(RuntimeError):
    """Something went wrong getting Census data; the message says what."""


def api_key(key=None):
    """
    The key to use: one passed in, else the CENSUS_API_KEY environment
    variable, else the first non-blank line of census_api_key.txt.
    """
    if key:
        return key.strip()
    env = os.environ.get("CENSUS_API_KEY", "").strip()
    if env:
        return env
    if KEY_FILE.exists():
        for line in KEY_FILE.read_text().splitlines():
            if line.strip():
                return line.strip()
    raise CensusError(
        "No Census API key. Get one free at https://api.census.gov/data/key_signup.html "
        f"and paste it into {KEY_FILE}")


@contextlib.contextmanager
def offline(faa_file=None, no_fly_file=None):
    """
    Within this block nothing is downloaded; the FAA map is read from faa_file
    and the national security areas from no_fly_file.
    """
    saved = dict(MODE)
    MODE.update(offline=True, faa_file=Path(faa_file) if faa_file else None,
                no_fly_file=Path(no_fly_file) if no_fly_file else None)
    try:
        yield
    finally:
        MODE.update(saved)


def record(path):
    """Note an input file a step read (see the module docstring)."""
    _used.add(Path(path).resolve())


def start_recording():
    _used.clear()


def recorded():
    """{path relative to data/census: sha256} for every input read since start_recording."""
    root = DATA_DIR.resolve()
    return {p.relative_to(root).as_posix(): sha256(p) for p in sorted(_used) if p.is_relative_to(root)}


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def missing_offline(what):
    return CensusError(f"{what} is not on this computer, and this is an offline run. "
                       "Run `census fetch-inputs <run>` first to get the published inputs.")


def cached_get(url, params, refresh=False):
    """GET url, caching the parsed JSON on disk by request. Keys never reach the cache key."""
    safe_params = {k: v for k, v in params.items() if k != "key"}
    tag = hashlib.sha1(json.dumps([url, safe_params], sort_keys=True).encode()).hexdigest()[:20]
    path = CACHE_DIR / f"{tag}.json"
    if path.exists() and (not refresh or MODE["offline"]):
        record(path)
        return json.loads(path.read_text())
    if MODE["offline"]:
        raise missing_offline(f"The saved answer to {url} {safe_params}")

    try:
        response = requests.get(url, params=params, timeout=90)
    except requests.RequestException as exc:
        raise CensusError(f"Could not reach {url}: {exc}") from exc

    # The Census answers a missing or bad key with a redirect to an HTML page
    # and a 200 status, so a status check alone lets it through.
    final = response.url
    if "missing_key" in final:
        raise CensusError(
            "The Census API needs a key. Get one free at "
            f"https://api.census.gov/data/key_signup.html and paste it into {KEY_FILE}")
    if "invalid_key" in final:
        raise CensusError(f"The Census API rejected the key (from {KEY_FILE.name} or CENSUS_API_KEY).")
    if response.status_code != 200:
        raise CensusError(f"Census request failed ({response.status_code}): {response.text[:300].strip()}")
    try:
        data = response.json()
    except ValueError as exc:
        raise CensusError(f"Expected data, got: {response.text[:200].strip()!r}") from exc
    # TIGERweb reports a failed query as an error object with a 200 status.
    # Checked before caching: a cached error would be served as an empty
    # result on every later run, long after the cause was fixed.
    if isinstance(data, dict) and "error" in data:
        err = data["error"]
        raise CensusError(f"Boundary query failed: {err.get('message')} {err.get('details') or ''}".strip())

    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    record(path)
    return data
