"""
The streets to fly inside each picked block group.

Only Census secondary roads and local streets count by default; highways,
ramps, alleys, private roads, parking lots and trails are left out.
"""
from collections import namedtuple

import pandas as pd
import shapely
from shapely.geometry import mapping, shape
from shapely.ops import transform

from .geo_math import METERS_PER_MILE, to_degrees, to_metres
from . import geography
from .sources import CensusError

# Census road codes (MTFCC) -> what they are.
ROAD_TYPES = {"S1200": "secondary road", "S1400": "local street"}

DIRECTIONS = {"N": "North", "S": "South", "E": "East", "W": "West"}
Roads = namedtuple("Roads", "geometries types names tree")


def full_name(name):
    """
    The Census names the same grid street two ways in different stretches
    ("N 475 E" and "N 475 East St"). Spell the short one out, so the street is
    listed once: "N 475 E" -> "N 475 East St", "E 1000 N" -> "E 1000 North St".
    """
    parts = name.split()
    if len(parts) == 3 and parts[0] in DIRECTIONS and parts[1].isdigit() and parts[2] in DIRECTIONS:
        return f"{parts[0]} {parts[1]} {DIRECTIONS[parts[2]]} St"
    return name


def load_roads(bounds, street_types, year, state, county):
    """
    Every street segment of the chosen types inside a bounding box (degrees),
    as shapes in metres with a spatial index, from the county's TIGER/Line
    roads file. Each block group's streets are then found locally.
    """
    for code in street_types:
        if code not in ROAD_TYPES:
            raise CensusError(f"Unknown street type {code!r}; known: {', '.join(ROAD_TYPES)}.")
    all_roads = geography.roads(year, state, county)
    box = shapely.box(*bounds)
    geometries, types, names = [], [], []
    for code in street_types:
        chosen = all_roads[(all_roads["MTFCC"] == code) & all_roads.intersects(box)]
        for name, geom in zip(chosen["FULLNAME"], chosen.geometry):
            geometries.append(transform(to_metres, geom))
            types.append(ROAD_TYPES[code])
            names.append((name or "").strip())
    return Roads(geometries, types, names, shapely.STRtree(geometries))


def streets_in(geoid, polygon, roads, buffer_m, min_segment_m, low_zone_m=None):
    """
    Every street segment inside one block group (plus the buffer_m border):
    (rows for the table, GeoJSON features), one segment per stretch between junctions.
    low_zone_m is the FAA squares with a ceiling under the high flight, in metres,
    so each street can be marked as inside restricted airspace.
    """
    area = transform(to_metres, polygon).buffer(buffer_m)
    # The Census lists a street once per name it goes by ("E 1000 N" and
    # "E 1000 North St"), all on the same line. Segments are merged by shape so
    # each stretch of road is counted once, under its longest name.
    found = {}
    for i in sorted(roads.tree.query(area, predicate="intersects")):
        inside = roads.geometries[i].intersection(area)
        if inside.is_empty or inside.length < min_segment_m:
            continue
        key = shapely.normalize(shapely.set_precision(inside, 0.5)).wkb
        entry = found.setdefault(key, {"names": set(), "road_type": roads.types[i], "geometry": inside})
        if roads.names[i]:
            entry["names"].add(roads.names[i])

    rows, features = [], []
    for entry in found.values():
        names = sorted(entry["names"], key=lambda n: (-len(n), n))
        name = full_name(names[0]) if names else "(unnamed)"
        aliases = "; ".join(n for n in names if n != name)
        length = entry["geometry"].length
        # How much of this stretch lies in an FAA square with a ceiling under the high flight.
        restricted = (entry["geometry"].intersection(low_zone_m).length
                      if low_zone_m is not None else 0.0)
        rows.append({"geoid": geoid, "street": name, "road_type": entry["road_type"],
                     "miles": length / METERS_PER_MILE, "aliases": aliases,
                     "restricted_miles": restricted / METERS_PER_MILE})
        features.append({"type": "Feature",
                         "geometry": mapping(transform(to_degrees, entry["geometry"])),
                         "properties": {"geoid": geoid, "street": name,
                                        "road_type": entry["road_type"], "aliases": aliases,
                                        "restricted": bool(restricted >= 0.5 * length)}})
    return rows, features


def find_streets(picked, boundaries, cells, settings, areas):
    """
    The streets in every picked block group: (table with one row per street per
    block group, GeoJSON features). settings and areas are the run's sample
    and areas config sections.
    """
    low_squares = [g for c, g in (cells or []) if c < settings["high_flight_ft"]]
    low_zone_m = transform(to_metres, shapely.union_all(low_squares)) if low_squares else None
    minx, miny, maxx, maxy = shapely.union_all([boundaries[g] for g in picked["geoid"]]).bounds
    margin = 0.005                                  # degrees; far more than the buffer
    roads = load_roads((minx - margin, miny - margin, maxx + margin, maxy + margin), settings["street_types"],
                       areas["year"], areas["state"], areas["county"])

    rows, features = [], []
    for geoid in picked["geoid"]:
        block_rows, block_features = streets_in(geoid, boundaries[geoid], roads, settings["buffer_m"],
                                                settings["min_segment_m"], low_zone_m)
        rows += block_rows
        features += block_features

    streets = (pd.DataFrame(rows)
               .groupby(["geoid", "street", "road_type"], as_index=False)
               .agg(miles=("miles", "sum"), segments=("miles", "size"),
                    restricted_miles=("restricted_miles", "sum"),
                    aliases=("aliases", lambda a: "; ".join(sorted({x for s in a for x in s.split("; ") if x})))))
    # A street is marked restricted when at least half of it is in restricted airspace.
    streets["restricted"] = streets["restricted_miles"] >= 0.5 * streets["miles"]
    streets["miles"] = streets["miles"].round(2)
    streets["restricted_miles"] = streets["restricted_miles"].round(2)
    return streets.merge(picked[["geoid", "city", "cluster", "pick"]], on="geoid"), features
