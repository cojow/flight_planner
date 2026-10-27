"""
Talking to the Census: the API key, and GET requests cached on disk so a
repeated run spends no API calls and gives the same answer.
"""
import hashlib
import json
import os
from pathlib import Path

import requests

from .config import REPO_ROOT

ACS_URL = "https://api.census.gov/data/{year}/acs/acs5"
TIGER_URL = "https://tigerweb.geo.census.gov/arcgis/rest/services/TIGERweb"
CACHE_DIR = REPO_ROOT / "data" / "census" / "cache"

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


def cached_get(url, params, refresh=False):
    """GET url, caching the parsed JSON on disk by request. Keys never reach the cache key."""
    safe_params = {k: v for k, v in params.items() if k != "key"}
    tag = hashlib.sha1(json.dumps([url, safe_params], sort_keys=True).encode()).hexdigest()[:20]
    path = CACHE_DIR / f"{tag}.json"
    if path.exists() and not refresh:
        return json.loads(path.read_text())

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
    return data
