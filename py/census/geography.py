"""
Where things are: the cities in a county, and the block groups in each city.

Block groups don't nest inside cities - a city limit cuts straight through
them - so "the block groups in a city" needs a rule. Here a block group is in
a city when at least `min_overlap` of its own area lies inside the city
boundary (default 0.5, half).
"""
import difflib

import pandas as pd
import shapely
from shapely.geometry import shape

from .sources import TIGER_URL, CensusError, cached_get

# TIGERweb layer ids: incorporated places, block groups, counties.
PLACES_LAYER = "Places_CouSub_ConCity_SubMCD/MapServer/4"
BLOCK_GROUP_LAYER = "Tracts_Blocks/MapServer/1"
COUNTY_LAYER = "State_County/MapServer/1"
SQ_M_PER_SQ_MI = 2_589_988.11

# A city counts as "in the county" when at least this share of it is.
PLACE_IN_COUNTY_SHARE = 0.01


def _state_places(state, refresh=False):
    """Every incorporated place in a state, as GeoJSON features."""
    data = cached_get(f"{TIGER_URL}/{PLACES_LAYER}/query", {
        "where": f"STATE='{state}'", "outFields": "GEOID,NAME,BASENAME,PLACE",
        "returnGeometry": "true", "outSR": "4326", "f": "geojson"}, refresh=refresh)
    return data.get("features", [])


def county_boundary(state, county, refresh=False):
    data = cached_get(f"{TIGER_URL}/{COUNTY_LAYER}/query", {
        "where": f"STATE='{state}' AND COUNTY='{county}'", "outFields": "GEOID,NAME",
        "returnGeometry": "true", "outSR": "4326", "f": "geojson"}, refresh=refresh)
    features = data.get("features", [])
    if not features:
        raise CensusError(f"No county {county} in state {state}.")
    return shape(features[0]["geometry"])


def county_places(state="49", county="049", year=None, refresh=False):
    """
    The incorporated cities and towns in a county: a DataFrame with geoid,
    name ("Provo") and full_name ("Provo city"), sorted by name.
    `year` is accepted so callers needn't care which boundary vintage is used.
    """
    area = county_boundary(state, county, refresh)
    rows = []
    for feature in _state_places(state, refresh):
        props = feature["properties"]
        boundary = shape(feature["geometry"])
        if boundary.area and boundary.intersection(area).area / boundary.area >= PLACE_IN_COUNTY_SHARE:
            rows.append({"geoid": props["GEOID"], "name": props["BASENAME"], "full_name": props["NAME"]})
    return pd.DataFrame(rows).sort_values("name").reset_index(drop=True)


def place(state, geoid=None, name=None, refresh=False):
    """One city's record: {geoid, name, full_name, boundary}, found by GEOID or by name."""
    features = _state_places(state, refresh)
    for feature in features:
        props = feature["properties"]
        if (geoid and props["GEOID"] == geoid) or \
           (not geoid and name and name.strip().lower() in (props["BASENAME"].lower(), props["NAME"].lower())):
            return {"geoid": props["GEOID"], "name": props["BASENAME"], "full_name": props["NAME"],
                    "boundary": shape(feature["geometry"])}
    close = difflib.get_close_matches(name or "", [f["properties"]["BASENAME"] for f in features], n=5)
    hint = f" Did you mean: {', '.join(close)}?" if close else ""
    raise CensusError(f"No incorporated place {geoid or name!r} in state {state}.{hint}")


def block_groups_in_place(city, min_overlap=0.5, refresh=False):
    """
    The block groups with at least min_overlap of their area inside a city:
    (DataFrame with geoid, city, share_in_city, land area and interior point;
     {geoid: shapely polygon}).
    """
    if not 0 < min_overlap <= 1:
        raise CensusError("min_overlap must be above 0 and at most 1.")
    boundary = city["boundary"]
    minx, miny, maxx, maxy = boundary.bounds
    data = cached_get(f"{TIGER_URL}/{BLOCK_GROUP_LAYER}/query", {
        "geometry": f"{minx},{miny},{maxx},{maxy}", "geometryType": "esriGeometryEnvelope",
        "inSR": "4326", "spatialRel": "esriSpatialRelIntersects",
        "outFields": "GEOID,STATE,COUNTY,TRACT,INTPTLAT,INTPTLON,AREALAND,BLKGRP",
        "returnGeometry": "true", "outSR": "4326", "f": "geojson"}, refresh=refresh)

    rows, geometries = [], {}
    for feature in data.get("features", []):
        props = feature["properties"]
        geom = shape(feature["geometry"])
        # Area in degrees is fine for a ratio: both shapes sit at the same
        # latitude, so the distortion cancels.
        share = geom.intersection(boundary).area / geom.area if geom.area else 0.0
        if share < min_overlap:
            continue
        rows.append({"geoid": props["GEOID"], "city": city["full_name"], "county": props["COUNTY"],
                     "share_in_city": round(share, 3),
                     "land_area_sq_mi": float(props["AREALAND"]) / SQ_M_PER_SQ_MI,
                     "lat": float(props["INTPTLAT"]), "lon": float(props["INTPTLON"])})
        geometries[props["GEOID"]] = geom
    return pd.DataFrame(rows), geometries


def block_groups_in_places(state, places, min_overlap=0.5, refresh=False):
    """
    The block groups in any of several cities, each listed once (a block group
    can't be more than half in two cities). places is [{"geoid", "name"}, ...].
    Returns (DataFrame, {geoid: polygon}).
    """
    frames, geometries = [], {}
    for chosen in places:
        city = place(state, geoid=chosen.get("geoid"), name=chosen.get("name"), refresh=refresh)
        frame, geoms = block_groups_in_place(city, min_overlap, refresh)
        if frame.empty:
            raise CensusError(f"No block group has {min_overlap:.0%} of its area in {city['full_name']} - "
                              "try a lower min_overlap.")
        frames.append(frame)
        geometries.update(geoms)
    table = pd.concat(frames, ignore_index=True).drop_duplicates("geoid").reset_index(drop=True)
    return table, geometries


def union(geometries):
    return shapely.union_all(list(geometries))
