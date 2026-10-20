"""
Building age and income data from the Census, ready for clustering.

Pulls American Community Survey (ACS) 5-year estimates for the neighbourhoods
inside a city - block groups by default - so the age of a city's buildings and
the income of the people living in them can be clustered, mapped, or compared.
Every row is one area; every column is labelled with what it measures, and
whether it covers all buildings or only some of them.

Quick start
-----------
1. Get a free API key: https://api.census.gov/data/key_signup.html
2. Paste it into py/census_api_key.txt, next to this script. That file is
   gitignored; the CENSUS_API_KEY environment variable works too.
3. Run it:

    python py/building_census_data.py Provo
    python py/building_census_data.py Provo --level tract --geojson
    python py/building_census_data.py --compare Provo Lehi Logan "St. George"

Working from the sampling frame
-------------------------------
census_tool/city_block_groups.py picks the block groups that get sampled and
flown. This tool uses the same rule - an area is in the city when at least half
of it is - so `python py/building_census_data.py Provo` describes the same
block groups. Once a frame has actually been sampled from, point at the file
instead, so later boundary updates can never make the two disagree:

    python py/building_census_data.py --frame census_tool/output/provo_ut_2023_block_groups.geojson

Or from Python:

    from building_census_data import get_city_data, kmeans
    df = get_city_data("Provo")
    clustered = kmeans(df, ["median_year_built_all", "median_household_income"], k=4)

What "all buildings" means here
-------------------------------
The ACS counts housing units: houses, apartments, condos, mobile homes. It does
not cover commercial or industrial buildings at all - for those, county assessor
parcel records are the source. Within housing, columns ending in _all cover
every unit; _owner and _renter split them by who lives there, and the pct_
structure columns say what kind of building the units are in.

Reading the numbers
-------------------
- These are survey estimates. Each median comes with a margin of error (_moe)
  and a coefficient of variation (_cv). Small areas are noisy; treat a block
  group with a high _cv as uncertain rather than as different.
- Medians at the edge of the scale are capped by the Census: a median year
  built of 1939 means "1939 or earlier", and incomes top out at 250,001.
- Values the Census could not estimate come back as blank (NaN), never as the
  large negative placeholder codes the raw API uses.
"""
import argparse
import difflib
import hashlib
import json
import os
import sys
from pathlib import Path

import pandas as pd
import requests
from shapely.geometry import shape

# --------------------------------------------------------------------------
# Settings
# --------------------------------------------------------------------------

DEFAULT_YEAR = 2024          # newest ACS 5-year release when this was written
DEFAULT_LEVEL = "block group"
LEVELS = ("block group", "tract")

REPO_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_DIR = REPO_ROOT / "data" / "census"
CACHE_DIR = OUTPUT_DIR / "cache"

ACS_URL = "https://api.census.gov/data/{year}/acs/acs5"
TIGER_URL = "https://tigerweb.geo.census.gov/arcgis/rest/services/TIGERweb"
# TIGERweb layer ids: incorporated places, census tracts, census block groups.
PLACES_LAYER = "Places_CouSub_ConCity_SubMCD/MapServer/4"
AREA_LAYERS = {"tract": "Tracts_Blocks/MapServer/0", "block group": "Tracts_Blocks/MapServer/1"}

# The API rejects more than 50 variables in one request.
MAX_VARIABLES_PER_REQUEST = 49

# Which areas count as "in the city". Block groups don't nest inside city
# limits - the line cuts through them - so one counts when at least this share
# of its own area is inside. Deliberately the same rule and default as the
# sampling frame in census_tool/city_block_groups.py: the areas clustered here
# have to be the areas sampled and flown there, and on Provo the two now select
# the identical 83 block groups.
DEFAULT_MIN_OVERLAP = 0.5

STATE_FIPS = {
    "AL": "01", "AK": "02", "AZ": "04", "AR": "05", "CA": "06", "CO": "08", "CT": "09",
    "DE": "10", "DC": "11", "FL": "12", "GA": "13", "HI": "15", "ID": "16", "IL": "17",
    "IN": "18", "IA": "19", "KS": "20", "KY": "21", "LA": "22", "ME": "23", "MD": "24",
    "MA": "25", "MI": "26", "MN": "27", "MS": "28", "MO": "29", "MT": "30", "NE": "31",
    "NV": "32", "NH": "33", "NJ": "34", "NM": "35", "NY": "36", "NC": "37", "ND": "38",
    "OH": "39", "OK": "40", "OR": "41", "PA": "42", "RI": "44", "SC": "45", "SD": "46",
    "TN": "47", "TX": "48", "UT": "49", "VT": "50", "VA": "51", "WA": "53", "WV": "54",
    "WI": "55", "WY": "56", "PR": "72",
}
STATE_NAMES = {
    "alabama": "AL", "alaska": "AK", "arizona": "AZ", "arkansas": "AR", "california": "CA",
    "colorado": "CO", "connecticut": "CT", "delaware": "DE", "district of columbia": "DC",
    "florida": "FL", "georgia": "GA", "hawaii": "HI", "idaho": "ID", "illinois": "IL",
    "indiana": "IN", "iowa": "IA", "kansas": "KS", "kentucky": "KY", "louisiana": "LA",
    "maine": "ME", "maryland": "MD", "massachusetts": "MA", "michigan": "MI",
    "minnesota": "MN", "mississippi": "MS", "missouri": "MO", "montana": "MT",
    "nebraska": "NE", "nevada": "NV", "new hampshire": "NH", "new jersey": "NJ",
    "new mexico": "NM", "new york": "NY", "north carolina": "NC", "north dakota": "ND",
    "ohio": "OH", "oklahoma": "OK", "oregon": "OR", "pennsylvania": "PA",
    "rhode island": "RI", "south carolina": "SC", "south dakota": "SD", "tennessee": "TN",
    "texas": "TX", "utah": "UT", "vermont": "VT", "virginia": "VA", "washington": "WA",
    "west virginia": "WV", "wisconsin": "WI", "wyoming": "WY", "puerto rico": "PR",
}

# --------------------------------------------------------------------------
# What to pull. Census code -> column name. Codes checked against the 2024
# ACS 5-year variable list; the categories are the same back to 2020.
# --------------------------------------------------------------------------

MEDIANS = {
    "B19013_001": "median_household_income",
    "B25035_001": "median_year_built_all",       # every housing unit, vacant included
    "B25037_002": "median_year_built_owner",     # owner-occupied units
    "B25037_003": "median_year_built_renter",    # renter-occupied units
    "B25018_001": "median_rooms_all",
    "B25021_002": "median_rooms_owner",
    "B25021_003": "median_rooms_renter",
    "B25010_001": "avg_household_size",
}

# Units by the decade they were built (B25034). Shares of these describe a
# neighbourhood's age far better than one median does - a mix of 1920s houses
# and new apartments has the same median as an area built entirely in 1975.
YEAR_BUILT_TOTAL = "B25034_001"
YEAR_BUILT = {
    "B25034_002": "2020_later",
    "B25034_003": "2010_2019",
    "B25034_004": "2000_2009",
    "B25034_005": "1990_1999",
    "B25034_006": "1980_1989",
    "B25034_007": "1970_1979",
    "B25034_008": "1960_1969",
    "B25034_009": "1950_1959",
    "B25034_010": "1940_1949",
    "B25034_011": "1939_earlier",
}

# Units by the kind of building they are in (B25024).
STRUCTURE_TOTAL = "B25024_001"
STRUCTURE = {
    "B25024_002": "single_family_detached",
    "B25024_003": "single_family_attached",
    "B25024_004": "2_units",
    "B25024_005": "3_4_units",
    "B25024_006": "5_9_units",
    "B25024_007": "10_19_units",
    "B25024_008": "20_49_units",
    "B25024_009": "50_plus_units",
    "B25024_010": "mobile_home",
    "B25024_011": "boat_rv_van",
}

# Margins of error worth carrying: the two values most likely to be clustered on.
WITH_MOE = ("B19013_001", "B25035_001")

# The raw API's placeholders for "could not be estimated" are large negative
# numbers (-666666666 and friends). Anything this negative is one of them.
SENTINEL_FLOOR = -100_000_000
# -555555555 on a margin of error means "controlled - no sampling error".
MOE_NO_ERROR = -555555555


class CensusError(RuntimeError):
    """Something went wrong talking to the Census; the message says what."""


# --------------------------------------------------------------------------
# HTTP, with a disk cache so repeated runs don't spend API calls
# --------------------------------------------------------------------------

def _cached_get(url, params, refresh=False, expect="json"):
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
            "https://api.census.gov/data/key_signup.html, then: export CENSUS_API_KEY=\"...\"")
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


# Where the key lives by default: a one-line text file beside this script.
# It is gitignored - this repository is public, and a committed key is a
# published key.
KEY_FILE = Path(__file__).resolve().parent / "census_api_key.txt"


def _api_key(key=None):
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


# --------------------------------------------------------------------------
# Places and boundaries (TIGERweb - no key needed)
# --------------------------------------------------------------------------

def state_fips(state):
    """'UT', 'Utah' or '49' -> '49'."""
    s = str(state).strip()
    if s.isdigit():
        return s.zfill(2)
    abbr = STATE_NAMES.get(s.lower(), s.upper())
    if abbr not in STATE_FIPS:
        raise CensusError(f"Unknown state {state!r}.")
    return STATE_FIPS[abbr]


def find_city(city, state="UT", refresh=False):
    """
    Look a city up by its everyday name ("Provo", "St. George") and return its
    Census record and boundary. Matching on the place's code afterwards is what
    keeps a renamed or reclassified town (a "town" becoming a "city") from
    silently dropping out, as matching on full names did.
    """
    st = state_fips(state)
    data = _cached_get(f"{TIGER_URL}/{PLACES_LAYER}/query", {
        "where": f"STATE='{st}'", "outFields": "GEOID,NAME,BASENAME,PLACE",
        "returnGeometry": "true", "outSR": "4326", "f": "geojson"}, refresh=refresh)
    places = data.get("features", [])
    wanted = city.strip().lower()
    for feature in places:
        props = feature["properties"]
        if wanted in (props["BASENAME"].lower(), props["NAME"].lower()):
            return {"geoid": props["GEOID"], "name": props["NAME"], "state": st,
                    "place": props["PLACE"], "boundary": shape(feature["geometry"])}
    close = difflib.get_close_matches(city, [f["properties"]["BASENAME"] for f in places], n=5)
    hint = f" Did you mean: {', '.join(close)}?" if close else ""
    raise CensusError(f"No incorporated place called {city!r} in state {st}.{hint}")


def areas_in_city(city_record, level=DEFAULT_LEVEL, min_overlap=DEFAULT_MIN_OVERLAP, refresh=False):
    """
    The tracts or block groups with at least min_overlap of their area inside
    the city, and the share that is (share_in_city). Matches the sampling
    frame's rule - see DEFAULT_MIN_OVERLAP.
    """
    if not 0 < min_overlap <= 1:
        raise CensusError("min_overlap must be above 0 and at most 1.")
    if level not in AREA_LAYERS:
        raise CensusError(f"level must be one of {LEVELS}, not {level!r}.")
    boundary = city_record["boundary"]
    minx, miny, maxx, maxy = boundary.bounds
    data = _cached_get(f"{TIGER_URL}/{AREA_LAYERS[level]}/query", {
        "geometry": f"{minx},{miny},{maxx},{maxy}", "geometryType": "esriGeometryEnvelope",
        "inSR": "4326", "spatialRel": "esriSpatialRelIntersects",
        # Only fields the layer has: asking a tract layer for BLKGRP fails
        # the whole query.
        "outFields": "GEOID,STATE,COUNTY,TRACT,INTPTLAT,INTPTLON,AREALAND"
                     + (",BLKGRP" if level == "block group" else ""),
        "returnGeometry": "true", "outSR": "4326", "f": "geojson"}, refresh=refresh)

    rows, geometries = [], {}
    for feature in data.get("features", []):
        props = feature["properties"]
        geom = shape(feature["geometry"])
        # Area in degrees is fine for a ratio: both shapes sit at the same
        # latitude, so the distortion cancels. (The frame measures in UTM; the
        # selections agree exactly.)
        share = geom.intersection(boundary).area / geom.area if geom.area else 0.0
        if share < min_overlap:
            continue
        rows.append({
            "geoid": props["GEOID"], "county": props["COUNTY"], "tract": props["TRACT"],
            "block_group": props.get("BLKGRP") if level == "block group" else None,
            "lat": float(props["INTPTLAT"]), "lon": float(props["INTPTLON"]),
            "land_area_sq_mi": float(props["AREALAND"]) / 2_589_988.11,
            "share_in_city": round(share, 3),
        })
        geometries[props["GEOID"]] = geom
    if not rows:
        raise CensusError(f"No {level} has {min_overlap:.0%} of its area in {city_record['name']} - "
                          "try a lower min_overlap.")
    frame = pd.DataFrame(rows)
    if level == "tract":
        frame = frame.drop(columns="block_group")
    return frame, geometries


# --------------------------------------------------------------------------
# ACS data
# --------------------------------------------------------------------------

def _variable_codes():
    codes = []
    for code in MEDIANS:
        codes.append(f"{code}E")
        if code in WITH_MOE:
            codes.append(f"{code}M")
    codes += [f"{c}E" for c in (YEAR_BUILT_TOTAL, *YEAR_BUILT, STRUCTURE_TOTAL, *STRUCTURE)]
    return codes


def _fetch_acs(geography, within, year, key, refresh=False):
    """
    Raw ACS rows for one geography, as a DataFrame of strings. Splits the
    variable list across requests if it ever outgrows the API's limit.
    """
    codes = _variable_codes()
    frames = []
    for start in range(0, len(codes), MAX_VARIABLES_PER_REQUEST):
        chunk = codes[start:start + MAX_VARIABLES_PER_REQUEST]
        params = {"get": ",".join(["NAME", *chunk]), "for": geography, "key": key}
        if within:
            params["in"] = within
        data = _cached_get(ACS_URL.format(year=year), params, refresh=refresh)
        frames.append(pd.DataFrame(data[1:], columns=data[0]))
    merged = frames[0]
    geo_cols = [c for c in merged.columns if c not in codes and c != "NAME"]
    for extra in frames[1:]:
        merged = merged.merge(extra.drop(columns="NAME"), on=geo_cols)
    return merged


def _clean(raw, year):
    """Raw ACS strings -> labelled numeric columns, placeholders blanked, shares derived."""
    out = pd.DataFrame(index=raw.index)
    num = {c: pd.to_numeric(raw[c], errors="coerce") for c in raw.columns if c[:1] in "BC"}
    for code in num:
        values = num[code]
        if code.endswith("M"):
            # "No sampling error" is a real zero; the other codes mean unknown.
            values = values.mask(values == MOE_NO_ERROR, 0)
        num[code] = values.mask(values <= SENTINEL_FLOOR)

    for code, name in MEDIANS.items():
        out[name] = num[f"{code}E"]
        if code in WITH_MOE:
            out[f"{name}_moe"] = num[f"{code}M"]
            # Coefficient of variation: the standard error as a share of the
            # estimate. Under ~0.12 is solid; over ~0.40 is shaky.
            out[f"{name}_cv"] = (num[f"{code}M"] / 1.645 / num[f"{code}E"]).round(3)

    # Ages are relative to the survey, not today, so they don't drift every January.
    out["median_age_years_all"] = year - out["median_year_built_all"]
    out["median_age_years_owner"] = year - out["median_year_built_owner"]
    out["median_age_years_renter"] = year - out["median_year_built_renter"]

    total_built = num[f"{YEAR_BUILT_TOTAL}E"]
    out["housing_units"] = total_built
    for code, era in YEAR_BUILT.items():
        out[f"pct_built_{era}"] = (num[f"{code}E"] / total_built * 100).round(1)
    out["pct_built_since_2000"] = out[["pct_built_2020_later", "pct_built_2010_2019",
                                       "pct_built_2000_2009"]].sum(axis=1, min_count=1)
    out["pct_built_before_1980"] = out[[f"pct_built_{e}" for e in
                                        ("1970_1979", "1960_1969", "1950_1959",
                                         "1940_1949", "1939_earlier")]].sum(axis=1, min_count=1)

    total_struct = num[f"{STRUCTURE_TOTAL}E"]
    for code, kind in STRUCTURE.items():
        out[f"pct_{kind}"] = (num[f"{code}E"] / total_struct * 100).round(1)
    # The house / apartment split most analyses want, in two columns.
    out["pct_single_family"] = out[["pct_single_family_detached",
                                    "pct_single_family_attached"]].sum(axis=1, min_count=1)
    out["pct_multi_family"] = out[["pct_2_units", "pct_3_4_units", "pct_5_9_units",
                                   "pct_10_19_units", "pct_20_49_units",
                                   "pct_50_plus_units"]].sum(axis=1, min_count=1)
    return out


def get_city_data(city, state="UT", level=DEFAULT_LEVEL, year=DEFAULT_YEAR,
                  min_overlap=DEFAULT_MIN_OVERLAP, key=None, refresh=False, with_geometry=False):
    """
    One row per tract or block group inside a city, with building age, building
    type, rooms and income columns. See the module docstring for what they mean.

    with_geometry=True also returns {geoid: shapely polygon}, for mapping.
    """
    key = _api_key(key)
    city_record = find_city(city, state, refresh=refresh)
    areas, geometries = areas_in_city(city_record, level, min_overlap, refresh=refresh)
    df = _attach_acs(areas, city_record["state"], level, year, key, refresh)
    df.insert(1, "city", city_record["name"])
    return (df, geometries) if with_geometry else df


def _attach_acs(areas, state, level, year, key, refresh=False):
    """ACS columns joined onto a table of areas with geoid and county columns."""
    pieces = []
    for county in sorted(areas["county"].unique()):
        if level == "block group":
            raw = _fetch_acs("block group:*", f"state:{state} county:{county} tract:*",
                             year, key, refresh)
            raw["geoid"] = raw["state"] + raw["county"] + raw["tract"] + raw["block group"]
        else:
            raw = _fetch_acs("tract:*", f"state:{state} county:{county}",
                             year, key, refresh)
            raw["geoid"] = raw["state"] + raw["county"] + raw["tract"]
        cleaned = _clean(raw, year)
        cleaned.insert(0, "geoid", raw["geoid"].values)
        cleaned.insert(1, "census_name", raw["NAME"].values)
        pieces.append(cleaned)
    acs = pd.concat(pieces, ignore_index=True)

    df = areas.merge(acs, on="geoid", how="left")
    # Expected for special land-use tracts (codes starting 98/99 - campuses,
    # airports, parks), which the ACS mostly leaves unestimated.
    missing = df["census_name"].isna().sum()
    if missing:
        print(f"Note: {missing} {level}(s) have no ACS estimates for {year} "
              "(often special land-use tracts, codes 98xx/99xx).", file=sys.stderr)
    df["year"] = year
    return df


def get_frame_data(frame_path, year=DEFAULT_YEAR, key=None, refresh=False, with_geometry=False):
    """
    ACS columns for exactly the areas in an existing frame GeoJSON - the
    output of census_tool/city_block_groups.py, say. Use this when the frame
    has already been sampled from: it guarantees the data describes those
    areas and no others, whatever later boundary updates do.
    """
    features = json.loads(Path(frame_path).read_text()).get("features", [])
    if not features:
        raise CensusError(f"No areas in {frame_path}.")
    rows, geometries = [], {}
    for feature in features:
        props = feature["properties"]
        geoid = str(props.get("GEOID") or props.get("geoid") or "")
        if len(geoid) not in (11, 12):
            raise CensusError(f"{frame_path}: expected 11- or 12-digit GEOIDs, found {geoid!r}.")
        rows.append({"geoid": geoid, "county": geoid[2:5], "tract": geoid[5:11],
                     "block_group": geoid[11:] or None})
        if feature.get("geometry"):
            geometries[geoid] = shape(feature["geometry"])
    lengths = {len(r["geoid"]) for r in rows}
    if len(lengths) > 1:
        raise CensusError(f"{frame_path} mixes tracts and block groups.")
    level = "block group" if lengths == {12} else "tract"
    areas = pd.DataFrame(rows)
    if level == "tract":
        areas = areas.drop(columns="block_group")
    df = _attach_acs(areas, rows[0]["geoid"][:2], level, year, _api_key(key), refresh)
    df.insert(1, "frame", Path(frame_path).stem)
    return (df, geometries) if with_geometry else df


def get_cities_data(cities, state="UT", year=DEFAULT_YEAR, key=None, refresh=False):
    """
    One row per city, same columns as get_city_data - the city-to-city comparison
    the original script did, but matched on place codes rather than names.
    """
    key = _api_key(key)
    records = [find_city(c, state, refresh=refresh) for c in cities]
    st = records[0]["state"]
    raw = _fetch_acs(f"place:{','.join(r['place'] for r in records)}", f"state:{st}",
                     year, key, refresh)
    cleaned = _clean(raw, year)
    cleaned.insert(0, "geoid", (raw["state"] + raw["place"]).values)
    cleaned.insert(1, "city", raw["NAME"].values)
    cleaned["year"] = year
    order = {r["geoid"]: i for i, r in enumerate(records)}
    return cleaned.sort_values("geoid", key=lambda s: s.map(order)).reset_index(drop=True)


# --------------------------------------------------------------------------
# Clustering helpers
# --------------------------------------------------------------------------

def cluster_features(df, columns, standardize=True):
    """
    The rows and columns k-means should see: areas missing any of `columns`
    dropped, and each column rescaled to mean 0 / standard deviation 1.

    Rescaling is not optional for k-means. It measures plain distance, so an
    income in dollars (tens of thousands) would swamp a year built (a few
    decades of spread) and the "clusters" would just be income bands.
    """
    missing = [c for c in columns if c not in df.columns]
    if missing:
        raise KeyError(f"Not columns in this data: {missing}")
    usable = df.dropna(subset=list(columns))
    features = usable[list(columns)].astype(float)
    if standardize:
        features = (features - features.mean()) / features.std(ddof=0)
    dropped = len(df) - len(usable)
    return features, usable.index, dropped


def kmeans(df, columns, k, random_state=0):
    """
    df with a `cluster` column added (blank for areas missing a feature).
    Needs scikit-learn.
    """
    from sklearn.cluster import KMeans
    features, index, dropped = cluster_features(df, columns)
    if dropped:
        print(f"Note: {dropped} area(s) left out for missing {columns}.", file=sys.stderr)
    labels = KMeans(n_clusters=k, n_init=10, random_state=random_state).fit_predict(features)
    out = df.copy()
    out["cluster"] = pd.Series(labels, index=index).astype("Int64")
    return out


def choose_k(df, columns, k_values=range(2, 9), random_state=0):
    """
    Silhouette score for each k - higher means tighter, better-separated
    clusters. A starting point for picking k, not a verdict.
    """
    from sklearn.cluster import KMeans
    from sklearn.metrics import silhouette_score
    features, _, _ = cluster_features(df, columns)
    scores = {}
    for k in k_values:
        if k >= len(features):
            break
        labels = KMeans(n_clusters=k, n_init=10, random_state=random_state).fit_predict(features)
        scores[k] = round(float(silhouette_score(features, labels)), 3)
    return pd.Series(scores, name="silhouette")


# --------------------------------------------------------------------------
# Saving and the command line
# --------------------------------------------------------------------------

def save_geojson(df, geometries, path):
    """Write the areas with every column as properties - opens in QGIS, ArcGIS, geojson.io."""
    features = []
    for _, row in df.iterrows():
        geom = geometries.get(row["geoid"])
        if geom is None:
            continue
        props = {k: (None if pd.isna(v) else v) for k, v in row.items()}
        features.append({"type": "Feature", "geometry": geom.__geo_interface__, "properties": props})
    Path(path).write_text(json.dumps({"type": "FeatureCollection", "features": features}, default=str))


def _slug(text):
    return "".join(ch if ch.isalnum() else "_" for ch in text.lower()).strip("_")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Census building-age and income data for the areas inside a city.",
        epilog="Needs a free key (https://api.census.gov/data/key_signup.html) "
               "in py/census_api_key.txt or CENSUS_API_KEY.")
    parser.add_argument("city", nargs="?", help='city name, e.g. Provo or "St. George"')
    parser.add_argument("--state", default="UT", help="state abbreviation, name or FIPS (default UT)")
    parser.add_argument("--level", default=DEFAULT_LEVEL, choices=LEVELS,
                        help=f"size of area per row (default {DEFAULT_LEVEL})")
    parser.add_argument("--year", type=int, default=DEFAULT_YEAR, help=f"ACS 5-year release (default {DEFAULT_YEAR})")
    parser.add_argument("--min-overlap", type=float, default=DEFAULT_MIN_OVERLAP,
                        help="share of an area that must be inside the city, 0-1 "
                             f"(default {DEFAULT_MIN_OVERLAP}, same as census_tool's frame)")
    parser.add_argument("--frame", metavar="GEOJSON",
                        help="use exactly the areas in this frame file, e.g. "
                             "census_tool/output/provo_ut_2023_block_groups.geojson")
    parser.add_argument("--compare", nargs="+", metavar="CITY",
                        help="one row per city instead of per area, for comparing cities")
    parser.add_argument("--out", help="output CSV path (default data/census/...)")
    parser.add_argument("--geojson", action="store_true", help="also write boundaries for mapping")
    parser.add_argument("--refresh", action="store_true", help="ignore the cache and download again")
    args = parser.parse_args(argv)

    if not (args.city or args.compare or args.frame):
        parser.error("give a city, --frame with a frame GeoJSON, or --compare with a list of cities")

    try:
        if args.frame:
            df, geometries = get_frame_data(args.frame, args.year, refresh=args.refresh,
                                            with_geometry=True)
            default_name = f"{Path(args.frame).stem}_acs_{args.year}.csv"
        elif args.compare:
            df = get_cities_data(args.compare, args.state, args.year, refresh=args.refresh)
            default_name = f"compare_{len(args.compare)}_cities_{args.year}.csv"
            geometries = None
        else:
            df, geometries = get_city_data(args.city, args.state, args.level, args.year,
                                           args.min_overlap, refresh=args.refresh,
                                           with_geometry=True)
            default_name = f"{_slug(args.city)}_{_slug(args.level)}_{args.year}.csv"
    except CensusError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    out = Path(args.out) if args.out else OUTPUT_DIR / default_name
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)
    print(f"Saved {len(df)} rows to {out}")

    if args.geojson and geometries:
        geo_path = out.with_suffix(".geojson")
        save_geojson(df, geometries, geo_path)
        print(f"Saved boundaries to {geo_path}")

    if not args.compare:
        noisy = (df["median_household_income_cv"] > 0.40).sum()
        if noisy:
            print(f"{noisy} area(s) have a shaky income estimate (median_household_income_cv > 0.40).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
