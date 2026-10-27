"""
Which block groups to fly: the airspace check, the draw from each cluster,
each pick's weight, and the order to fly them in.
"""
import json
import time
from pathlib import Path

import numpy as np
import requests
import shapely
from shapely.geometry import shape

from .geo_math import to_metres
from .sources import CensusError

FAA_URL = ("https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/"
           "FAA_UAS_FacilityMap_Data/FeatureServer/0/query")      # the same map the flight planner uses


# --------------------------------------------------------------------------
# Airspace
# --------------------------------------------------------------------------

def faa_cells(bounds, save_to=None, tries=4, wait_s=20):
    """
    The FAA UAS Facility Map squares over a bounding box, as [(max LAANC
    altitude in feet, shape)]. A ceiling of 0 means no LAANC approval is given
    there at all. The raw answer is saved to `save_to`, so the run records the
    airspace it was drawn under.

    The FAA's server limits how often it can be asked (it answers "too many
    requests"), so a failed ask is retried. If it still fails, or returns only
    part of the squares, this raises CensusError: drawing a sample with unknown
    airspace would make every block group look flyable.
    """
    minx, miny, maxx, maxy = bounds
    params = {"where": "1=1", "geometry": f"{minx},{miny},{maxx},{maxy}",
              "geometryType": "esriGeometryEnvelope", "inSR": "4326",
              "spatialRel": "esriSpatialRelIntersects", "outFields": "CEILING",
              "returnGeometry": "true", "outSR": "4326", "f": "geojson"}
    problem = "no answer"
    for attempt in range(tries):
        if attempt:
            time.sleep(wait_s * attempt)
        try:
            data = requests.get(FAA_URL, params=params, headers={"User-Agent": "Mozilla/5.0"}, timeout=90).json()
        except (requests.RequestException, ValueError) as exc:
            problem = str(exc)
            continue
        if "features" not in data:
            problem = (data.get("error") or {}).get("message", "an answer without map squares")
            continue
        if data.get("exceededTransferLimit"):
            raise CensusError("The FAA map returned only part of its squares for this area, so airspace "
                              "can't be checked. Sample fewer or smaller cities.")
        if save_to:
            Path(save_to).write_text(json.dumps(data))
        return [(f["properties"]["CEILING"], shape(f["geometry"]).buffer(0)) for f in data["features"]]
    raise CensusError(f"Could not get the FAA facility map after {tries} tries ({problem}). "
                      "Wait a minute and run the sample step again.")


def airspace_status(polygon, cells, high_flight_ft, restricted_share):
    """
    (status, lowest ceiling in feet, share of the block group that is restricted):
      clear       not inside any FAA facility map square - no LAANC needed
      laanc       LAANC needed; every ceiling allows the high flight
      partial     a small part (under restricted_share) has a ceiling below the high
                  flight: fly the rest, and leave out the streets in the restricted squares
      restricted  a large part has a ceiling below the high flight (a ceiling of 0 needs
                  FAA authorization, not LAANC): not eligible to be picked
    Squares that touch only a sliver (under 1%) of the block group are ignored.
    """
    touching = [(c, g) for c, g in cells
                if g.intersects(polygon) and g.intersection(polygon).area > 0.01 * polygon.area]
    if not touching:
        return "clear", np.nan, 0.0
    lowest = min(c for c, _ in touching)
    low_squares = [g for c, g in touching if c < high_flight_ft]
    share = (polygon.intersection(shapely.union_all(low_squares)).area / polygon.area
             if low_squares else 0.0)
    if share >= restricted_share:
        return "restricted", lowest, round(share, 2)
    return ("partial" if low_squares else "laanc"), lowest, round(share, 2)


# --------------------------------------------------------------------------
# The draw
# --------------------------------------------------------------------------

def draw(df, per_cluster, seed, weights="legacy"):
    """
    Pick `per_cluster` eligible block groups from each cluster. Adds to df:
      pick     1 for a cluster's first pick, 2 for its second, 0 if not picked
      sampled  whether it was picked
      weight   how many block groups of its cluster a pick stands for
    Each cluster's eligible block groups are shuffled once and the first
    `per_cluster` are taken, so a smaller sample is the start of a larger one.
    Returns a list of warnings (clusters with too few eligible block groups).
    """
    rng = np.random.default_rng(seed)
    warnings = []
    df["pick"] = 0
    for cluster, group in df.groupby("cluster"):
        eligible = group.index[group["airspace"] != "restricted"].to_numpy()
        chosen = rng.permutation(eligible)[:per_cluster]
        df.loc[chosen, "pick"] = np.arange(1, len(chosen) + 1)
        if len(chosen) < per_cluster:
            warnings.append(f"Cluster {cluster} has only {len(chosen)} eligible block group(s).")
    df["sampled"] = df["pick"] > 0
    sizes = df["cluster"].value_counts().sort_index()
    df["weight"] = df["cluster"].map(sizes / df[df["sampled"]].groupby("cluster").size())
    return warnings


def nearest_first(points, start):
    """Visit order for {id: (x, y)} in metres: the nearest to `start`, then the nearest to that, and so on."""
    left, order, here = dict(points), [], start
    while left:
        nxt = min(left, key=lambda k: (left[k][0] - here[0]) ** 2 + (left[k][1] - here[1]) ** 2)
        order.append(nxt)
        here = left.pop(nxt)
    return order


def plan_order(picked, boundaries, start_lat, start_lon):
    """
    Add fly_order (and lat, lon): first picks first, each pick next to the last,
    starting from (start_lat, start_lon). If you only fly one per cluster, these
    are the first ones.
    """
    centres = {g: boundaries[g].centroid for g in picked["geoid"]}
    picked["lat"] = [round(centres[g].y, 5) for g in picked["geoid"]]
    picked["lon"] = [round(centres[g].x, 5) for g in picked["geoid"]]
    xy = {g: tuple(float(v) for v in to_metres(c.x, c.y)) for g, c in centres.items()}
    here = tuple(float(v) for v in to_metres(start_lon, start_lat))

    order = []
    for rank in sorted(picked["pick"].unique()):
        visit = nearest_first({g: xy[g] for g in picked.loc[picked["pick"] == rank, "geoid"]}, here)
        order += visit
        here = xy[visit[-1]]
    picked["fly_order"] = picked["geoid"].map({g: i for i, g in enumerate(order, start=1)})
    return picked
