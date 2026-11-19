"""
Which block groups to fly: the airspace check, the draw from each cluster,
each pick's weight, and the order to fly them in.
"""
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import requests
import shapely
from shapely.geometry import shape

from .geo_math import to_metres
from .sources import DATA_DIR, MODE, CensusError, missing_offline, record

FAA_DIR = DATA_DIR / "faa"      # every FAA answer a run used, named by its content's hash
NO_FLY_DIR = DATA_DIR / "nsufr"     # the same for the national security areas

FAA_URL = ("https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/"
           "FAA_UAS_FacilityMap_Data/FeatureServer/0/query")      # the same map the flight planner uses
# National Security UAS Flight Restrictions (14 CFR 99.7), on the same FAA
# server: military bases and the like, where no drone may fly, LAANC or not.
# Part-time ones are only active when a NOTAM says so, but that can come at
# short notice, so they are left out too.
NO_FLY_URLS = {
    "full time": ("https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/"
                  "DoD_Mar_13/FeatureServer/0/query"),
    "part time": ("https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/"
                  "Part_Time_National_Security_UAS_Flight_Restrictions/FeatureServer/0/query"),
}

SLIVER = 0.01           # airspace covering less than this share of a block group is ignored


# --------------------------------------------------------------------------
# Airspace
# --------------------------------------------------------------------------

def faa_cells(bounds, save_to=None, tries=4, wait_s=20):
    """
    The FAA UAS Facility Map squares over a bounding box, as [(max LAANC
    altitude in feet, shape)]. A ceiling of 0 means no LAANC approval is given
    there at all. The raw answer is saved to `save_to`, so the run records the
    airspace it was drawn under, and to data/census/faa/<hash>.geojson, the
    snapshot a published run archives.

    The map is live and the FAA updates it, so in offline mode (a
    reproduction) it is read from the snapshot the original run used instead.

    The FAA's server limits how often it can be asked (it answers "too many
    requests"), so a failed ask is retried. If it still fails, or returns only
    part of the squares, this raises CensusError: drawing a sample with unknown
    airspace would make every block group look flyable.
    """
    if MODE["faa_file"]:
        data = _read_snapshot(MODE["faa_file"], save_to)
    elif MODE["offline"]:
        raise missing_offline("The FAA facility map snapshot")
    else:
        data = _ask(FAA_URL, _envelope(bounds, "CEILING"), "the FAA facility map", tries, wait_s)
        _save_snapshot(data, FAA_DIR, save_to)
    return [(f["properties"]["CEILING"], shape(f["geometry"]).buffer(0)) for f in data["features"]]


def no_fly_zones(bounds, save_to=None, tries=4, wait_s=20):
    """
    The National Security UAS Flight Restrictions over a bounding box, full
    and part time, as one shape (None if there are none). Never flown: the
    parts of block groups inside them are off-limits whatever
    sample.fly_restricted says, and their streets aren't listed at all.

    Saved and read back like the facility map (see faa_cells), to `save_to`
    and to data/census/nsufr/<hash>.geojson.
    """
    if MODE["no_fly_file"]:
        data = _read_snapshot(MODE["no_fly_file"], save_to)
    elif MODE["offline"]:
        raise missing_offline("The national security areas snapshot")
    else:
        features = []
        for schedule, url in NO_FLY_URLS.items():
            answer = _ask(url, _envelope(bounds, "Facility,Base"), "the national security areas", tries, wait_s)
            for f in answer["features"]:
                f["properties"]["schedule"] = schedule
                features.append(f)
        data = {"type": "FeatureCollection", "features": features}
        _save_snapshot(data, NO_FLY_DIR, save_to)
    shapes = [shape(f["geometry"]).buffer(0) for f in data["features"]]
    return shapely.union_all(shapes) if shapes else None


def _envelope(bounds, fields):
    minx, miny, maxx, maxy = bounds
    return {"where": "1=1", "geometry": f"{minx},{miny},{maxx},{maxy}",
            "geometryType": "esriGeometryEnvelope", "inSR": "4326",
            "spatialRel": "esriSpatialRelIntersects", "outFields": fields,
            "returnGeometry": "true", "outSR": "4326", "f": "geojson"}


def _ask(url, params, what, tries, wait_s):
    """An FAA ArcGIS query's GeoJSON answer, retried; raises CensusError if it can't be had whole."""
    problem = "no answer"
    for attempt in range(tries):
        if attempt:
            time.sleep(wait_s * attempt)
        try:
            data = requests.get(url, params=params, headers={"User-Agent": "Mozilla/5.0"}, timeout=90).json()
        except (requests.RequestException, ValueError) as exc:
            problem = str(exc)
            continue
        if "features" not in data:
            problem = (data.get("error") or {}).get("message", "an answer without map shapes")
            continue
        if data.get("exceededTransferLimit"):
            raise CensusError(f"{what[0].upper() + what[1:]} returned only part of its shapes for this area, "
                              "so airspace can't be checked. Sample fewer or smaller cities.")
        return data
    raise CensusError(f"Could not get {what} after {tries} tries ({problem}). "
                      "Wait a minute and run the sample step again.")


def _read_snapshot(path, save_to):
    data = json.loads(Path(path).read_text())
    record(path)
    if save_to:
        Path(save_to).write_text(json.dumps(data))
    return data


def _save_snapshot(data, folder, save_to):
    text = json.dumps(data)
    snapshot = folder / f"{hashlib.sha256(text.encode()).hexdigest()[:16]}.geojson"
    folder.mkdir(parents=True, exist_ok=True)
    snapshot.write_text(text)
    record(snapshot)
    if save_to:
        Path(save_to).write_text(text)


def needs_authorization(ceiling, high_flight_ft):
    """
    Whether flying the high flight in an FAA square with this ceiling needs
    authorization beyond LAANC's instant approval: a LAANC further-coordination
    request made ahead of time (0 ft squares included). Streets in these
    squares are marked restricted = True.
    """
    return ceiling < high_flight_ft


def off_limits(ceiling, high_flight_ft, fly_restricted="no"):
    """
    Whether an FAA square with this ceiling is left unflown (sample.fly_restricted):
      "no"     every square that needs authorization (see needs_authorization)
      "laanc"  none: those squares are flown with LAANC authorization
    National security areas are always off-limits (see no_fly_zones).
    """
    return fly_restricted == "no" and needs_authorization(ceiling, high_flight_ft)


def airspace_status(polygon, cells, high_flight_ft, restricted_share, fly_restricted="no", no_fly=None):
    """
    (status, lowest ceiling in feet, share of the block group that is
    off-limits, share inside a national security area):
      clear       not inside any FAA facility map square - no LAANC needed
      laanc       LAANC needed; every ceiling allows the high flight
      approval    some ceilings are under the high flight (0 ft included), and
                  fly_restricted = "laanc" flies them: request LAANC authorization
                  ahead of time (further coordination)
      partial     a small part (under restricted_share) is off-limits: fly the
                  rest, and leave out the streets there
      restricted  a large part is off-limits: not eligible to be picked
    Off-limits is the FAA squares `off_limits` leaves unflown, plus any part in
    `no_fly` (the national security areas), which is never flown. With
    fly_restricted="no" (the default) "approval" never comes up; with "laanc"
    only a national security area makes "partial" or "restricted".
    Squares and areas covering only a sliver (under 1%) of the block group are
    ignored.
    """
    touching = [(c, g) for c, g in cells
                if g.intersects(polygon) and g.intersection(polygon).area > SLIVER * polygon.area]
    blocked = [g for c, g in touching if off_limits(c, high_flight_ft, fly_restricted)]
    no_fly_share = (polygon.intersection(no_fly).area / polygon.area
                    if no_fly is not None and no_fly.intersects(polygon) else 0.0)
    if no_fly_share > SLIVER:
        blocked.append(no_fly)
    else:
        no_fly_share = 0.0
    share = (polygon.intersection(shapely.union_all(blocked)).area / polygon.area
             if blocked else 0.0)
    lowest = min(c for c, _ in touching) if touching else np.nan
    if share >= restricted_share:
        return "restricted", lowest, round(share, 2), round(no_fly_share, 2)
    if blocked:
        return "partial", lowest, round(share, 2), round(no_fly_share, 2)
    if not touching:
        return "clear", lowest, 0.0, 0.0
    return ("approval" if lowest < high_flight_ft else "laanc"), lowest, round(share, 2), 0.0


# --------------------------------------------------------------------------
# The draw
# --------------------------------------------------------------------------

def draw(df, per_cluster, seed, weights="eligible"):
    """
    Pick `per_cluster` eligible block groups from each cluster. Adds to df:
      pick     1 for a cluster's first pick, 2 for its second, 0 if not picked
      sampled  whether it was picked
      weight   how many block groups of its cluster a pick stands for
    With weights="eligible" (the default) a pick stands for the block groups
    that could have been picked: its cluster's block groups outside restricted
    airspace. Restricted ones had no chance of being drawn, so the sample can
    only describe the flyable part of the cities. weights="legacy" counts the
    whole cluster, as sampling_draft/ did.
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
    pool = df if weights == "legacy" else df[df["airspace"] != "restricted"]
    sizes = pool["cluster"].value_counts().sort_index()
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
