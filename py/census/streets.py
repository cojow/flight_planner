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

# Census road codes (MTFCC) that can be flown -> what they are. The default is
# secondary roads and local streets (config: sample.street_types).
ROAD_TYPES = {
    "S1100": "primary road",        # interstates and other limited-access highways
    "S1200": "secondary road",      # state and county routes
    "S1400": "local street",
    "S1500": "vehicular trail",     # 4WD
    "S1630": "ramp",
    "S1640": "service drive",
    "S1730": "alley",
    "S1740": "private road",
}

# Joining street pieces: coordinates are snapped to this grid, and a piece
# counts as running along a stretch of road when it is within SAME_LINE_M of it.
GRID_M = 0.5
SAME_LINE_M = 2.0
MIN_SPLIT_M = 1.0                   # bits of street this short, left by the airspace split, are dropped

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


def road_counties(picked, boundaries, areas, margin=0.005):
    """
    The counties whose roads files are needed: every county with a block group
    near the picks (TIGER/Line roads come one file per county, and a pick's
    buffer can reach across a county line).
    """
    box = shapely.box(*shapely.union_all([boundaries[g] for g in picked["geoid"]]).buffer(margin).bounds)
    bgs = geography.block_groups_near(box, areas["year"], areas["state"])
    return sorted(set(bgs["COUNTYFP"]) | {areas["county"]})


def load_roads(bounds, street_types, year, state, counties):
    """
    Every street segment of the chosen types inside a bounding box (degrees),
    as shapes in metres with a spatial index, from the counties' TIGER/Line
    roads files. Each block group's streets are then found locally.
    """
    for code in street_types:
        if code not in ROAD_TYPES:
            raise CensusError(f"Unknown street type {code!r}; known: {', '.join(ROAD_TYPES)}.")
    all_roads = pd.concat([geography.roads(year, state, c) for c in counties], ignore_index=True)
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
    (rows for the table, GeoJSON features). low_zone_m is the FAA squares with a
    ceiling under the high flight, in metres: streets are cut at its edge, and
    the stretches inside it are marked restricted.
    """
    area = transform(to_metres, polygon).buffer(buffer_m)
    pieces = []                     # (shape, name, road type), clipped to the block group
    for i in sorted(roads.tree.query(area, predicate="intersects")):
        inside = roads.geometries[i].intersection(area)
        if not inside.is_empty and inside.length >= min_segment_m:
            pieces.append((inside, roads.names[i], roads.types[i]))
    if not pieces:
        return [], []

    # The Census lists a stretch of road once per name it goes by ("State Rte
    # 114" and "N Geneva Rd"; "950 N" and "W 950 North St"), and where two
    # names share only part of a road the shapes differ. Joining every piece
    # into one network counts each stretch of pavement once; each stretch then
    # takes the names of the pieces that run along it, and is listed under the
    # longest of them with the rest as aliases.
    network = shapely.union_all([g for g, _, _ in pieces], grid_size=GRID_M)
    corridors = [g.buffer(SAME_LINE_M) for g, _, _ in pieces]
    shapely.prepare(corridors)
    found = {}
    for part in getattr(network, "geoms", [network]):
        if part.length == 0:
            continue
        along = [j for j, corridor in enumerate(corridors)
                 if corridor.intersects(part) and part.intersection(corridor).length >= 0.9 * part.length]
        names = frozenset(pieces[j][1] for j in along if pieces[j][1])
        road_type = pieces[along[0]][2] if along else pieces[0][2]
        found.setdefault((names, road_type), []).append(part)

    # No length filter here: every piece already passed min_segment_m, and a
    # short stretch left over from joining two names is still pavement.
    entries = [{"names": set(names), "road_type": road_type, "geometry": shapely.line_merge(shapely.union_all(parts))}
               for (names, road_type), parts in found.items()]

    rows, features = [], []
    for entry in entries:
        names = sorted(entry["names"], key=lambda n: (-len(n), n))
        name = full_name(names[0]) if names else "(unnamed)"
        aliases = "; ".join(n for n in names if n != name)
        # Split at the edge of FAA squares with a ceiling under the high flight,
        # so the stretch inside one is listed (and skipped) on its own.
        if low_zone_m is None:
            split = [(entry["geometry"], False)]
        else:
            split = [(entry["geometry"].difference(low_zone_m), False),
                     (entry["geometry"].intersection(low_zone_m), True)]
        for geometry, restricted in split:
            geometry = _lines(geometry)
            if geometry.length < MIN_SPLIT_M:
                continue
            rows.append({"geoid": geoid, "street": name, "road_type": entry["road_type"],
                         "restricted": restricted, "miles": geometry.length / METERS_PER_MILE,
                         "aliases": aliases})
            features.append({"type": "Feature",
                             "geometry": mapping(transform(to_degrees, geometry)),
                             "properties": {"geoid": geoid, "street": name, "road_type": entry["road_type"],
                                            "aliases": aliases, "restricted": restricted}})
    return rows, features


def _lines(geometry):
    """Just the line parts of a shape (cutting a line with a polygon can leave stray points)."""
    parts = [g for g in getattr(geometry, "geoms", [geometry]) if g.geom_type in ("LineString", "MultiLineString")]
    return shapely.line_merge(shapely.union_all(parts)) if parts else shapely.LineString()


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
                       areas["year"], areas["state"], road_counties(picked, boundaries, areas, margin))

    rows, features = [], []
    for geoid in picked["geoid"]:
        block_rows, block_features = streets_in(geoid, boundaries[geoid], roads, settings["buffer_m"],
                                                settings["min_segment_m"], low_zone_m)
        rows += block_rows
        features += block_features

    # One row per street per block group, and per side of a restricted-airspace edge.
    streets = (pd.DataFrame(rows)
               .groupby(["geoid", "street", "road_type", "restricted"], as_index=False)
               .agg(miles=("miles", "sum"), segments=("miles", "size"),
                    aliases=("aliases", lambda a: "; ".join(sorted({x for s in a for x in s.split("; ") if x})))))
    return streets.merge(picked[["geoid", "city", "cluster", "pick"]], on="geoid"), features
