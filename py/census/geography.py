"""
Where things are: the cities in a county, the block groups in each city, and
the roads, all from the Census TIGER/Line files of the same year as the ACS
numbers (2024 boundaries for the 2020-2024 ACS).

The files are downloaded once into data/census/tiger/ and never change after
that, so a run's areas and streets can be repeated exactly; `sources` records
each file's SHA-256 so a run says which files it used.

Block groups don't nest inside cities - a city limit cuts straight through
them - so "the block groups in a city" needs a rule. Here a block group is in
a city when at least `min_overlap` of its own area lies inside the city
boundary (default 0.5, half).
"""
import difflib
import functools
import hashlib
import os
import time

import geopandas as gpd
import pandas as pd
import requests
import shapely

from .config import REPO_ROOT
from .sources import CensusError

TIGER_DIR = REPO_ROOT / "data" / "census" / "tiger"
TIGER_FILE_URL = "https://www2.census.gov/geo/tiger/TIGER{year}/{layer}/{name}"
SQ_M_PER_SQ_MI = 2_589_988.11
DOWNLOAD_TRIES = 3
TIMEOUT_S = 60

# A city counts as "in the county" when at least this share of it is.
PLACE_IN_COUNTY_SHARE = 0.01


def _file_name(kind, year, state, county):
    return {"place": f"tl_{year}_{state}_place.zip",
            "bg": f"tl_{year}_{state}_bg.zip",
            "roads": f"tl_{year}_{state}{county}_roads.zip"}[kind]


def tiger_file(kind, year, state, county=None):
    """Path to a TIGER/Line zip ("place", "bg" or "roads"), downloading it on first use."""
    name = _file_name(kind, year, state, county)
    path = TIGER_DIR / name
    if path.exists():
        return path
    url = TIGER_FILE_URL.format(year=year, layer={"place": "PLACE", "bg": "BG", "roads": "ROADS"}[kind], name=name)
    TIGER_DIR.mkdir(parents=True, exist_ok=True)
    # Download to a temporary name and rename, so an interrupted download can't
    # leave a truncated zip that later runs would trust. The Census server
    # sometimes stalls mid-file, so each try gives up after TIMEOUT_S of silence.
    partial = path.with_suffix(".part")
    problem = ""
    for attempt in range(DOWNLOAD_TRIES):
        try:
            with requests.get(url, stream=True, timeout=TIMEOUT_S) as response:
                if response.status_code == 404:
                    raise CensusError(f"The Census has no {url} - is {year} a published TIGER/Line year?")
                response.raise_for_status()
                expected = int(response.headers.get("content-length", 0))
                with open(partial, "wb") as f:
                    for chunk in response.iter_content(1 << 16):
                        f.write(chunk)
            if expected and partial.stat().st_size != expected:
                raise requests.RequestException(f"got {partial.stat().st_size} of {expected} bytes")
            os.replace(partial, path)
            return path
        except requests.RequestException as exc:
            problem = str(exc)
        finally:
            if partial.exists():
                partial.unlink()
    raise CensusError(f"Could not download {url} after {DOWNLOAD_TRIES} tries ({problem}).")


def sources(year, state, county, kinds=("place", "bg", "roads")):
    """{file name: {sha256, downloaded}} for the TIGER/Line files a run uses."""
    out = {}
    for kind in kinds:
        path = tiger_file(kind, year, state, county)
        out[path.name] = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                          "downloaded": time.strftime("%Y-%m-%d", time.localtime(path.stat().st_mtime))}
    return out


@functools.lru_cache(maxsize=4)
def _places(year, state):
    """Every incorporated city and town in a state (CDPs left out), in lat/lon."""
    gdf = gpd.read_file(tiger_file("place", year, state))
    return gdf[gdf["CLASSFP"].str.startswith("C")].to_crs(4326).reset_index(drop=True)


@functools.lru_cache(maxsize=4)
def _block_groups(year, state):
    return gpd.read_file(tiger_file("bg", year, state)).to_crs(4326)


@functools.lru_cache(maxsize=4)
def _county_area(year, state, county):
    bgs = _block_groups(year, state)
    return shapely.union_all(bgs[bgs["COUNTYFP"] == county].geometry.values)


def county_places(state="49", county="049", year=2024):
    """
    The incorporated cities and towns in a county: a DataFrame with geoid,
    name ("Provo") and full_name ("Provo city"), sorted by name.
    """
    area = _county_area(year, state, county)
    rows = []
    for _, p in _places(year, state).iterrows():
        g = p.geometry
        if g.area and g.intersection(area).area / g.area >= PLACE_IN_COUNTY_SHARE:
            rows.append({"geoid": p["GEOID"], "name": p["NAME"], "full_name": p["NAMELSAD"]})
    return pd.DataFrame(rows).sort_values("name").reset_index(drop=True)


def place(state, geoid=None, name=None, year=2024):
    """One city's record: {geoid, name, full_name, boundary}, found by GEOID or by name."""
    places = _places(year, state)
    if geoid:
        hits = places[places["GEOID"] == geoid]
    else:
        wanted = (name or "").strip().lower()
        hits = places[(places["NAME"].str.lower() == wanted) | (places["NAMELSAD"].str.lower() == wanted)]
    if len(hits) == 1:
        p = hits.iloc[0]
        return {"geoid": p["GEOID"], "name": p["NAME"], "full_name": p["NAMELSAD"], "boundary": p.geometry}
    if len(hits) > 1:
        raise CensusError(f"{name!r} matches more than one place; pick by GEOID: {', '.join(hits['GEOID'])}")
    close = difflib.get_close_matches(name or "", list(places["NAME"]), n=5)
    hint = f" Did you mean: {', '.join(close)}?" if close else ""
    raise CensusError(f"No city or town {geoid or name!r} in state {state} in {year}.{hint}")


def block_groups_in_place(city, min_overlap=0.5, year=2024, state="49"):
    """
    The block groups with at least min_overlap of their area inside a city:
    (DataFrame with geoid, city, share_in_city, land area and interior point;
     {geoid: shapely polygon}).
    """
    if not 0 < min_overlap <= 1:
        raise CensusError("min_overlap must be above 0 and at most 1.")
    boundary = city["boundary"]
    bgs = _block_groups(year, state)
    near = bgs[bgs.intersects(boundary)]

    rows, geometries = [], {}
    for _, bg in near.iterrows():
        geom = bg.geometry
        # Area in degrees is fine for a ratio: both shapes sit at the same
        # latitude, so the distortion cancels.
        share = geom.intersection(boundary).area / geom.area if geom.area else 0.0
        if share < min_overlap:
            continue
        rows.append({"geoid": bg["GEOID"], "city": city["full_name"], "county": bg["COUNTYFP"],
                     "share_in_city": round(share, 3),
                     "land_area_sq_mi": float(bg["ALAND"]) / SQ_M_PER_SQ_MI,
                     "lat": float(bg["INTPTLAT"]), "lon": float(bg["INTPTLON"])})
        geometries[bg["GEOID"]] = geom
    return pd.DataFrame(rows), geometries


def block_groups_in_places(state, places, min_overlap=0.5, year=2024):
    """
    The block groups in any of several cities, each listed once (a block group
    can't be more than half in two cities). places is [{"geoid", "name"}, ...].
    Returns (DataFrame, {geoid: polygon}).
    """
    frames, geometries = [], {}
    for chosen in places:
        city = place(state, geoid=chosen.get("geoid"), name=chosen.get("name"), year=year)
        frame, geoms = block_groups_in_place(city, min_overlap, year, state)
        if frame.empty:
            raise CensusError(f"No block group has {min_overlap:.0%} of its area in {city['full_name']} - "
                              "try a lower min_overlap.")
        frames.append(frame)
        geometries.update(geoms)
    table = pd.concat(frames, ignore_index=True).drop_duplicates("geoid").reset_index(drop=True)
    return table, geometries


def block_groups_near(shape, year=2024, state="49"):
    """The state's block groups that touch a shape (TIGER/Line columns)."""
    bgs = _block_groups(year, state)
    return bgs[bgs.intersects(shape)]


@functools.lru_cache(maxsize=8)
def roads(year, state, county):
    """Every road in a county: GeoDataFrame with FULLNAME, MTFCC, geometry (lat/lon)."""
    return gpd.read_file(tiger_file("roads", year, state, county)).to_crs(4326)
