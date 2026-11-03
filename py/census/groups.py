"""
Split each picked block group's streets into a few groups of about equal
length, so the flying can be shared out or spread over several days.

The pieces that get grouped are stretches of street from one corner to the
next (from one cross street to the next), never part of a block. Each one is
named like a mission in the flight standards: Street1_Street2, where Street1
is the street flown and Street2 the cross street at its south or west end
("END" for a dead end, "Edge" where the street leaves the block group).

Groups are made by cutting the block group in two along its longer direction
so the street miles split in proportion, then cutting each half again, until
there are as many groups as asked for. That keeps each group in one compact
area, with miles as even as the corner-to-corner pieces allow.
"""
import re
from collections import defaultdict

import numpy as np
import shapely
from shapely.geometry import LineString, mapping, shape
from shapely.ops import transform

from .geo_math import METERS_PER_MILE, to_degrees, to_metres

GRID_M = 0.5            # the street network is snapped to this grid, so corners meet exactly
ALONG_M = 1.0           # a network piece belongs to a street when it runs within this of it
EDGE_M = 2.0            # a street end this close to the block group's buffered edge leaves it

DIRECTION_WORDS = {"North": "N", "South": "S", "East": "E", "West": "W"}
STREET_TYPES = {"St", "Ave", "Rd", "Dr", "Ln", "Cir", "Ct", "Way", "Pl", "Blvd", "Pkwy", "Ter", "Trl",
                "Loop", "Hwy", "Cv", "Row", "Plz", "Sq", "Aly"}


def short_name(street):
    """
    A street as the flight standards write it in mission names:
    "N 1200 East St" -> "1200E", "E 100 North St" -> "100N", "Elm Ave" -> "Elm",
    "N Geneva Rd" -> "Geneva", "(unnamed)" -> "Unnamed".
    """
    if not street or street == "(unnamed)":
        return "Unnamed"
    words = street.split()
    grid = re.fullmatch(r"(?:[NSEW] )?(\d+) (North|South|East|West|[NSEW])(?: St)?", street)
    if grid:
        return f"{grid.group(1)}{DIRECTION_WORDS.get(grid.group(2), grid.group(2))}"
    if len(words) > 1 and words[0] in "NSEW":
        words = words[1:]
    if len(words) > 1 and words[-1] in STREET_TYPES:
        words = words[:-1]
    return "".join(w[:1].upper() + w[1:] for w in words)


def _key(x, y):
    return (round(x, 3), round(y, 3))


def corner_to_corner(features, polygon, buffer_m):
    """
    The flyable streets of one block group cut into corner-to-corner pieces.

    features is the block group's street GeoJSON features (with street and
    restricted properties). Stretches in restricted airspace are cut the same
    way and marked restricted, so they can be drawn differently (they need
    FAA authorization to fly). Returns a list of dicts: street, from, to (the cross
    streets at each end), mission, miles, restricted, geometry (metres), x, y
    (its middle).
    """
    lines = [transform(to_metres, shape(f["geometry"])) for f in features]
    names = [f["properties"]["street"] for f in features]
    restricted = [bool(f["properties"].get("restricted")) for f in features]
    if not lines:
        return []

    # Node the whole network: every crossing becomes a shared vertex, so each
    # piece runs between two nodes.
    network = shapely.union_all(lines, grid_size=GRID_M)
    edges = [g for g in getattr(network, "geoms", [network]) if g.length > 0]
    corridors = [g.buffer(ALONG_M) for g in lines]
    shapely.prepare(corridors)

    degree = defaultdict(int)
    streets_at = defaultdict(set)           # node -> names of the streets that meet there
    by_street = defaultdict(list)           # feature index -> its edges
    for edge in edges:
        owner = next((i for i, c in enumerate(corridors)
                      if c.intersects(edge) and edge.intersection(c).length >= 0.9 * edge.length), None)
        for x, y in (edge.coords[0], edge.coords[-1]):
            degree[_key(x, y)] += 1
            if owner is not None:
                streets_at[_key(x, y)].add((names[owner], restricted[owner]))
        if owner is not None:
            by_street[owner].append(edge)

    edge_zone = transform(to_metres, polygon).buffer(buffer_m).exterior.buffer(EDGE_M)
    pieces = []
    for i, own in by_street.items():
        merged = shapely.line_merge(shapely.union_all(own))
        for line in getattr(merged, "geoms", [merged]):
            # Cut the street at every corner it passes through (3 or more pieces meet there).
            run = [line.coords[0]]
            for x, y in line.coords[1:]:
                run.append((x, y))
                if degree[_key(x, y)] >= 3 and len(run) > 1:
                    pieces.append(_piece(LineString(run), names[i], restricted[i], streets_at, edge_zone))
                    run = [(x, y)]
            if len(run) > 1:
                pieces.append(_piece(LineString(run), names[i], restricted[i], streets_at, edge_zone))
    return [p for p in pieces if p["miles"] > 0]


def _piece(line, street, restricted, streets_at, edge_zone):
    def cross(x, y):
        others = sorted(n for n, r in streets_at[_key(x, y)] if n != street)
        if others:
            return others[0]
        if any(n == street and r != restricted for n, r in streets_at[_key(x, y)]):
            return "Airspace"               # the street goes on, into or out of restricted airspace
        return "Edge" if edge_zone.contains(shapely.Point(x, y)) else "END"

    (x0, y0), (x1, y1) = line.coords[0], line.coords[-1]
    a, b = cross(x0, y0), cross(x1, y1)
    # Street2 is the cross street at the south end of a north-south street, or
    # the west end of an east-west one.
    runs_north_south = abs(y1 - y0) >= abs(x1 - x0)
    first_is_start = (y0 <= y1) if runs_north_south else (x0 <= x1)
    start, end = (a, b) if first_is_start else (b, a)
    middle = line.interpolate(0.5, normalized=True)
    return {"street": street, "from": start, "to": end, "restricted": restricted,
            "mission": f"{short_name(street)}_{short_name(start) if start not in ('END', 'Edge', 'Airspace') else start.upper()}",
            "miles": line.length / METERS_PER_MILE, "geometry": line, "x": middle.x, "y": middle.y}


def split_into_groups(pieces, n):
    """
    Divide pieces into n compact groups of about equal miles (fewer if there
    are fewer pieces). Returns a group number (1..) for each piece, in order.
    """
    groups = [0] * len(pieces)
    n = max(1, min(n, len(pieces)))

    def cut(index, n_groups, first):
        if n_groups == 1:
            for i in index:
                groups[i] = first
            return
        pts = np.array([(pieces[i]["x"], pieces[i]["y"]) for i in index])
        miles = np.array([pieces[i]["miles"] for i in index])
        # Cut across the longer direction of the area (its main axis, weighted by miles).
        centre = np.average(pts, axis=0, weights=miles)
        _, _, axes = np.linalg.svd((pts - centre) * np.sqrt(miles)[:, None], full_matrices=False)
        order = [index[j] for j in np.argsort((pts - centre) @ axes[0], kind="stable")]
        left_groups = n_groups // 2
        target = sum(miles) * left_groups / n_groups
        cum = np.cumsum([pieces[i]["miles"] for i in order])
        # Where to cut: as close to the target as possible, leaving at least one piece per group.
        lo, hi = left_groups, len(order) - (n_groups - left_groups)
        split = lo + int(np.argmin(np.abs(cum[lo - 1:hi] - target)))
        cut(order[:split], left_groups, first)
        cut(order[split:], n_groups - left_groups, first + left_groups)

    cut(list(range(len(pieces))), n, 1)
    return groups


def group_block_group(features, polygon, buffer_m, n):
    """
    Corner-to-corner pieces of one block group, each with its group number.
    Pieces in restricted airspace are grouped like the rest (they are still
    flown, with FAA authorization) and keep restricted = True.
    """
    pieces = corner_to_corner(features, polygon, buffer_m)
    for piece, group in zip(pieces, split_into_groups(pieces, n)):
        piece["group"] = group
    return pieces


def to_feature(piece, **props):
    """A piece as a GeoJSON feature in degrees."""
    return {"type": "Feature", "geometry": mapping(transform(to_degrees, piece["geometry"])),
            "properties": {**props, "group": piece["group"], "group_name": piece.get("group_name", ""),
                           "restricted": piece["restricted"], "street": piece["street"],
                           "from": piece["from"], "to": piece["to"], "mission": piece["mission"],
                           "miles": round(piece["miles"], 3)}}
