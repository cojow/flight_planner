"""
Which block groups to fly, and which streets inside them: Provo and Orem.

Follows Connor's plan: k-means cluster the block groups (income + house age),
then pull one or two block groups from each cluster to fly. Every street inside
each chosen block group is flown.

    python py/sample_block_groups.py                      # 3 clusters, 2 block groups each
    python py/sample_block_groups.py --per-cluster 1      # one from each cluster
    python py/sample_block_groups.py --k 4                # 4 clusters instead of 3

How a block group is picked:
  - Only block groups that can be flown are eligible: ones mostly inside FAA
    airspace with a ceiling under the 200 ft high flight (checked against the FAA
    UAS Facility Map) are left out, since they could not be flown as planned.
  - Each cluster's eligible block groups are put in a random order (seeded, so
    the draw can be repeated) and the first one or two are taken. The first pick
    is the same whether you take one or two per cluster, so choosing one later
    is just the first pick of each.

Each pick gets a weight - how many block groups of its cluster it stands for -
so results from the flown block groups can be scaled up to the whole city.

Run provo_orem_vs_utah_county.py first - it saves the table this reads.

Writes to data/census/provo_orem_vs_county/sample_k<k>_<n>per/:
    sample_block_groups.csv    the chosen block groups in flying order
    sample_streets.csv         one row per street per block group: what to fly
    sample_streets.geojson     the street segments, for QGIS or the flight planner map
    sample_block_groups.png    the picks among all block groups, by income and house age
    sample_map.png             the clusters and picks on a map
    sample_streets_map.png     the streets to fly in the chosen block groups
"""
import argparse
import json
from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import requests
import shapely
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch, Polygon as MplPolygon
from shapely.geometry import mapping, shape
from shapely.ops import transform

import building_census_data as bcd
from provo_orem_common import (COLORS, COLUMNS, DATA_DIR, K, LATITUDE, cluster_block_groups,
                               load_provo_orem, save_figure, style_income_age_axes)

CITIES = ["Provo", "Orem"]
PER_CLUSTER = 2                     # block groups to pick from each cluster (Connor: one or two)
SEED = 2026

# Which Census road types count as streets to fly. Highways, ramps, alleys,
# private roads, parking lots and trails are left out.
STREET_TYPES = {"S1200": "secondary road", "S1400": "local street"}
# TIGERweb layers holding those types: secondary roads and local roads.
ROAD_LAYERS = {"secondary road": 6, "local street": 8}
ROAD_URL = f"{bcd.TIGER_URL}/Transportation/MapServer/{{layer}}/query"
# Block group borders usually run down a street. A small buffer keeps those
# border streets, and so a street on the edge of two block groups appears in both.
BUFFER_M = 15
MIN_SEGMENT_M = 10                  # clipped slivers shorter than this are noise
METERS_PER_MILE = 1609.344

# Airspace and order.
HIGH_FLIGHT_FT = 200                # the high (nadir) flight height in the methodology
RESTRICTED_SHARE = 0.25             # not eligible when this much of it is under a ceiling below 200 ft
START_LAT, START_LON = 40.2338, -111.6585   # BYU: where each trip starts
FAA_URL = ("https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/"
           "FAA_UAS_FacilityMap_Data/FeatureServer/0/query")      # the same map the flight planner uses

# Lengths and buffers need metres, not degrees. Around Provo one degree of
# latitude is ~111 km and one of longitude ~84 km; this flat-earth conversion is
# off by well under 1% across a city, and needs no projection library.
LAT0, LON0 = 40.25, -111.67
M_PER_DEG_LAT = 111_132.0
M_PER_DEG_LON = 111_320.0 * np.cos(np.radians(LAT0))

DIRECTIONS = {"N": "North", "S": "South", "E": "East", "W": "West"}
Roads = namedtuple("Roads", "geometries types names tree")


def to_metres(x, y, z=None):
    return (np.asarray(x) - LON0) * M_PER_DEG_LON, (np.asarray(y) - LAT0) * M_PER_DEG_LAT


def to_degrees(x, y, z=None):
    return np.asarray(x) / M_PER_DEG_LON + LON0, np.asarray(y) / M_PER_DEG_LAT + LAT0


# --------------------------------------------------------------------------
# Airspace
# --------------------------------------------------------------------------

def faa_cells(bounds):
    """
    The FAA UAS Facility Map squares over a bounding box, as [(max LAANC
    altitude in feet, shape)]; None if the FAA's server can't be reached.
    A ceiling of 0 means no LAANC approval is given there at all.
    """
    minx, miny, maxx, maxy = bounds
    try:
        data = requests.get(FAA_URL, params={
            "where": "1=1", "geometry": f"{minx},{miny},{maxx},{maxy}",
            "geometryType": "esriGeometryEnvelope", "inSR": "4326",
            "spatialRel": "esriSpatialRelIntersects", "outFields": "CEILING",
            "returnGeometry": "true", "outSR": "4326", "f": "geojson"},
            headers={"User-Agent": "Mozilla/5.0"}, timeout=90).json()
    except (requests.RequestException, ValueError):
        return None
    if "features" not in data:
        return None
    if data.get("exceededTransferLimit"):
        print("Warning: the FAA map returned only part of the squares; airspace flags may be incomplete.")
    return [(f["properties"]["CEILING"], shape(f["geometry"]).buffer(0)) for f in data["features"]]


def airspace_status(polygon, cells):
    """
    (status, lowest ceiling in feet, share of the block group that is restricted):
      clear       not inside any FAA facility map square - no LAANC needed
      laanc       LAANC needed; every ceiling allows the 200 ft high flight
      partial     a small part (under RESTRICTED_SHARE) has a ceiling below 200 ft:
                  fly the rest, and leave out the streets in the restricted squares
      restricted  a large part has a ceiling below 200 ft (a ceiling of 0 needs FAA
                  authorization, not LAANC): not eligible to be picked
      unknown     the FAA map could not be reached
    Squares that touch only a sliver (under 1%) of the block group are ignored.
    """
    if cells is None:
        return "unknown", np.nan, np.nan
    touching = [(c, g) for c, g in cells
                if g.intersects(polygon) and g.intersection(polygon).area > 0.01 * polygon.area]
    if not touching:
        return "clear", np.nan, 0.0
    lowest = min(c for c, _ in touching)
    low_squares = [g for c, g in touching if c < HIGH_FLIGHT_FT]
    share = (polygon.intersection(shapely.union_all(low_squares)).area / polygon.area
             if low_squares else 0.0)
    if share >= RESTRICTED_SHARE:
        return "restricted", lowest, round(share, 2)
    return ("partial" if low_squares else "laanc"), lowest, round(share, 2)


# --------------------------------------------------------------------------
# The picks
# --------------------------------------------------------------------------

def draw_sample(df, per_cluster):
    """
    Pick `per_cluster` eligible block groups from each cluster. Adds to df:
      pick     1 for a cluster's first pick, 2 for its second, 0 if not picked
      sampled  whether it was picked
      weight   how many block groups of its cluster a pick stands for
    Each cluster's eligible block groups are shuffled once, and the first
    `per_cluster` are taken, so one per cluster is the first of the two.
    """
    rng = np.random.default_rng(SEED)
    df["pick"] = 0
    for stratum, group in df.groupby("stratum"):
        eligible = group.index[group["airspace"] != "restricted"].to_numpy()
        chosen = rng.permutation(eligible)[:per_cluster]
        df.loc[chosen, "pick"] = np.arange(1, len(chosen) + 1)
        if len(chosen) < per_cluster:
            print(f"Warning: cluster {stratum} has only {len(chosen)} eligible block group(s).")
    df["sampled"] = df["pick"] > 0
    sizes = df["stratum"].value_counts().sort_index()
    df["weight"] = df["stratum"].map(sizes / df[df["sampled"]].groupby("stratum").size())
    return sizes


def nearest_first(points, start):
    """Visit order for {id: (x, y)} in metres: the nearest to `start`, then the nearest to that, and so on."""
    left, order, here = dict(points), [], start
    while left:
        nxt = min(left, key=lambda k: (left[k][0] - here[0]) ** 2 + (left[k][1] - here[1]) ** 2)
        order.append(nxt)
        here = left.pop(nxt)
    return order


def plan_order(picked, boundaries):
    """
    Add fly_order (and lat, lon): first picks first, each pick next to the last,
    starting from BYU. If you only fly one per cluster, these are the first ones.
    """
    centres = {g: boundaries[g].centroid for g in picked["geoid"]}
    picked["lat"] = [round(centres[g].y, 5) for g in picked["geoid"]]
    picked["lon"] = [round(centres[g].x, 5) for g in picked["geoid"]]
    xy = {g: tuple(float(v) for v in to_metres(c.x, c.y)) for g, c in centres.items()}
    here = tuple(float(v) for v in to_metres(START_LON, START_LAT))

    order = []
    for rank in sorted(picked["pick"].unique()):
        visit = nearest_first({g: xy[g] for g in picked.loc[picked["pick"] == rank, "geoid"]}, here)
        order += visit
        here = xy[visit[-1]]
    picked["fly_order"] = picked["geoid"].map({g: i for i, g in enumerate(order, start=1)})
    return picked


# --------------------------------------------------------------------------
# The streets
# --------------------------------------------------------------------------

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


def load_roads(bounds):
    """
    Every street segment inside a bounding box (degrees), as shapes in metres
    with a spatial index. One download per road type, then each block group's
    streets are found locally - not one request per block group.
    """
    minx, miny, maxx, maxy = bounds
    geometries, types, names = [], [], []
    for road_type, layer in ROAD_LAYERS.items():
        data = bcd._cached_get(ROAD_URL.format(layer=layer), {
            "geometry": f"{minx},{miny},{maxx},{maxy}", "geometryType": "esriGeometryEnvelope",
            "inSR": "4326", "spatialRel": "esriSpatialRelIntersects",
            "outFields": "NAME,MTFCC", "returnGeometry": "true", "outSR": "4326", "f": "geojson"})
        if data.get("exceededTransferLimit"):
            raise bcd.CensusError("The Census returned only part of the roads; sample fewer or smaller areas.")
        for feature in data.get("features", []):
            props = feature["properties"]
            if STREET_TYPES.get(props.get("MTFCC")) != road_type:
                continue
            geometries.append(transform(to_metres, shape(feature["geometry"])))
            types.append(road_type)
            names.append((props.get("NAME") or "").strip())
    return Roads(geometries, types, names, shapely.STRtree(geometries))


def streets_in(geoid, polygon, roads, low_zone_m=None):
    """
    Every street segment inside one block group (plus the BUFFER_M border):
    (rows for the table, GeoJSON features), one segment per stretch between junctions.
    low_zone_m is the FAA squares with a ceiling under HIGH_FLIGHT_FT, in metres, so
    each street can be marked as inside restricted airspace.
    """
    area = transform(to_metres, polygon).buffer(BUFFER_M)
    # The Census lists a street once per name it goes by ("E 1000 N" and
    # "E 1000 North St"), all on the same line. Segments are merged by shape so
    # each stretch of road is counted once, under its longest name.
    found = {}
    for i in sorted(roads.tree.query(area, predicate="intersects")):
        inside = roads.geometries[i].intersection(area)
        if inside.is_empty or inside.length < MIN_SEGMENT_M:
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


def find_streets(picked, boundaries, cells):
    """The streets in every picked block group: (table with one row per street per block group, GeoJSON features)."""
    low_squares = [g for c, g in (cells or []) if c < HIGH_FLIGHT_FT]
    low_zone_m = transform(to_metres, shapely.union_all(low_squares)) if low_squares else None
    minx, miny, maxx, maxy = shapely.union_all([boundaries[g] for g in picked["geoid"]]).bounds
    margin = 0.005                                  # degrees; far more than BUFFER_M
    roads = load_roads((minx - margin, miny - margin, maxx + margin, maxy + margin))

    rows, features = [], []
    for geoid in picked["geoid"]:
        block_rows, block_features = streets_in(geoid, boundaries[geoid], roads, low_zone_m)
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
    return streets.merge(picked[["geoid", "city", "stratum", "pick"]], on="geoid"), features


# --------------------------------------------------------------------------
# Report and save
# --------------------------------------------------------------------------

def report(df, picked, streets, sizes, k, per_cluster):
    summary = df.groupby("stratum").agg(
        block_groups=("geoid", "size"),
        eligible=("airspace", lambda s: int((s != "restricted").sum())),
        picked=("sampled", "sum"),
        income=("median_household_income", "mean"),
        house_age=("median_age_years_all", "mean"),
    ).round(1)
    print(f"\n{len(df)} Provo + Orem block groups in {k} clusters; {per_cluster} per cluster = {len(picked)} to fly\n")
    print(summary.to_string())
    print("\nPicked, in flying order:")
    for _, r in picked.sort_values("fly_order").iterrows():
        print(f"  {r['fly_order']}. {r['geoid']}  {r['city']:<11} cluster {r['stratum']}  "
              f"pick {r['pick']}  {r['street_miles']:>5.1f} street miles  airspace: {r['airspace']}")
    print(f"\nStreets to fly: {len(streets)} (street, block group) pairs, "
          f"{streets['miles'].sum():,.1f} miles of street. "
          f"Flown twice (high and low) that is {2 * streets['miles'].sum():,.1f} miles of flight.")
    print(f"{int(streets['restricted'].sum())} of those streets "
          f"({streets.loc[streets['restricted'], 'miles'].sum():,.1f} miles) "
          f"are mostly in FAA squares with a ceiling under {HIGH_FLIGHT_FT} ft: skip them without authorization.")

    # With so few block groups the picks can sit far from the city average; show how far.
    print("\nPicks against the whole of Provo + Orem (weighted by cluster size):")
    for col in COLUMNS:
        weighted = np.average(picked[col], weights=picked["weight"])
        print(f"  {col:<26} all {df[col].mean():>9,.1f}   picks {weighted:>9,.1f}")


def save_tables(out_dir, picked, streets, street_features):
    keep = ["fly_order", "pick", "geoid", "city", "stratum", "airspace", "faa_ceiling_ft",
            "share_restricted", "streets", "street_miles", "median_household_income",
            "median_age_years_all", "housing_units", "weight", "lat", "lon"]
    picked[keep].sort_values("fly_order").to_csv(out_dir / "sample_block_groups.csv", index=False)
    streets.sort_values(["fly_order", "street"]).to_csv(out_dir / "sample_streets.csv", index=False)
    (out_dir / "sample_streets.geojson").write_text(
        json.dumps({"type": "FeatureCollection", "features": street_features}))


# --------------------------------------------------------------------------
# Charts
# --------------------------------------------------------------------------

def rings(geometry):
    """The outer edge of each part of a (multi)polygon, as arrays of x, y."""
    return [np.array(part.exterior.coords) for part in getattr(geometry, "geoms", [geometry])]


def plot_picks(df, path):
    """Every block group by income and house age, with the picks outlined and numbered."""
    fig, ax = plt.subplots(figsize=(9, 6))
    for stratum, group in df.groupby("stratum"):
        color = COLORS[stratum % len(COLORS)]
        ax.scatter(group["median_age_years_all"], group["median_household_income"],
                   s=30, alpha=0.35, color=color, edgecolors="none")
        chosen = group[group["sampled"]]
        ax.scatter(chosen["median_age_years_all"], chosen["median_household_income"],
                   s=90, color=color, edgecolors="black", linewidths=1.2,
                   label=f"Cluster {stratum}: {len(chosen)} of {len(group)} picked")
    style_income_age_axes(ax, "Picked block groups (outlined) among all Provo + Orem block groups")
    ax.legend(frameon=False, loc="upper right", fontsize=9)
    save_figure(fig, path)


def plot_map(df, boundaries, path):
    """The clusters on a map, with picks solid and numbered by fly order; hatched = not eligible (airspace)."""
    fig, ax = plt.subplots(figsize=(9, 9))
    for _, row in df.iterrows():
        color = COLORS[row["stratum"] % len(COLORS)]
        if row["sampled"]:
            style = dict(facecolor=color, alpha=1.0, edgecolor="black", linewidth=1.8)
        elif row["airspace"] == "restricted":
            style = dict(facecolor="white", alpha=1.0, edgecolor="#d1495b", linewidth=0.6, hatch="////")
        else:
            style = dict(facecolor=color, alpha=0.3, edgecolor="white", linewidth=0.5)
        for ring in rings(boundaries[row["geoid"]]):
            ax.add_patch(MplPolygon(ring, closed=True, **style))
    for _, row in df[df["sampled"]].iterrows():
        c = boundaries[row["geoid"]].centroid
        ax.text(c.x, c.y, str(int(row["fly_order"])), fontsize=9, fontweight="bold",
                ha="center", va="center", color="white", zorder=6,
                bbox=dict(boxstyle="circle,pad=0.3", facecolor="black", edgecolor="none"))
    ax.autoscale_view()
    ax.set_aspect(1 / np.cos(np.radians(LATITUDE)))
    ax.axis("off")
    ax.set_title("Clusters and picks (number = fly order)")
    handles = [Patch(facecolor=COLORS[s % len(COLORS)], label=f"Cluster {s}") for s in sorted(df["stratum"].unique())]
    handles.append(Patch(facecolor="white", edgecolor="#d1495b", hatch="////", label="Not eligible: restricted airspace"))
    ax.legend(handles=handles, loc="lower left", frameon=False, fontsize=9)
    save_figure(fig, path)


def plot_street_map(picked, boundaries, street_features, path):
    """One panel per picked block group, in flying order, with every street to fly; restricted streets in grey, dashed."""
    picked = picked.sort_values("fly_order")
    cols = min(3, len(picked))
    rows = -(-len(picked) // cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows), squeeze=False)
    for ax in axes.flat:
        ax.axis("off")
    for ax, (_, row) in zip(axes.flat, picked.iterrows()):
        color = COLORS[row["stratum"] % len(COLORS)]
        ax.add_collection(LineCollection(rings(boundaries[row["geoid"]]), colors="black", linewidths=1.2))
        for restricted, style in ((False, "solid"), (True, "dashed")):
            lines = []
            for feature in street_features:
                if feature["properties"]["geoid"] != row["geoid"] or feature["properties"]["restricted"] != restricted:
                    continue
                geom = shape(feature["geometry"])
                lines += [np.array(part.coords) for part in getattr(geom, "geoms", [geom])
                          if part.geom_type == "LineString"]
            ax.add_collection(LineCollection(lines, colors="#8c8c8c" if restricted else color,
                                             linewidths=1.0, linestyles=style))
        minx, miny, maxx, maxy = boundaries[row["geoid"]].bounds
        pad = 0.05 * max(maxx - minx, maxy - miny)
        ax.set_xlim(minx - pad, maxx + pad)
        ax.set_ylim(miny - pad, maxy + pad)
        ax.set_aspect(1 / np.cos(np.radians(LATITUDE)))
        ax.set_title(f"{int(row['fly_order'])}. {row['city'].replace(' city', '')}, cluster {row['stratum']}, "
                     f"{row['street_miles']:.1f} mi", fontsize=11)
    fig.suptitle("Streets to fly, in flying order (grey dashed = restricted airspace)", fontsize=13)
    save_figure(fig, path)


# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Pick block groups from each cluster, and list the streets to fly.")
    parser.add_argument("--k", type=int, default=K, help=f"number of clusters (default {K})")
    parser.add_argument("--per-cluster", type=int, default=PER_CLUSTER, choices=(1, 2),
                        help=f"block groups to pick from each cluster (default {PER_CLUSTER})")
    args = parser.parse_args()
    out_dir = DATA_DIR / f"sample_k{args.k}_{args.per_cluster}per"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Provo and Orem, clustered with the same k-means as kmeans_provo_orem.py.
    df = load_provo_orem()
    df["stratum"], _, _ = cluster_block_groups(df, args.k)

    print("Looking up boundaries and airspace...")
    boundaries = {}
    for city in CITIES:
        boundaries.update(bcd.get_city_data(city, "UT", with_geometry=True)[1])
    cells = faa_cells(shapely.union_all([boundaries[g] for g in df["geoid"]]).bounds)
    if cells is None:
        print("Warning: could not reach the FAA facility map; airspace is 'unknown'. "
              "Check it in the flight planner (Show FAA Airspace Restrictions).")
    status = [airspace_status(boundaries[g], cells) for g in df["geoid"]]
    df["airspace"] = [s for s, _, _ in status]
    df["faa_ceiling_ft"] = [c for _, c, _ in status]
    df["share_restricted"] = [r for _, _, r in status]

    sizes = draw_sample(df, args.per_cluster)
    picked = df[df["sampled"]].copy()

    print("Looking up streets (the first run downloads them; later runs use the cache)...")
    streets, street_features = find_streets(picked, boundaries, cells)
    miles = streets.groupby("geoid")["miles"].agg(street_miles="sum", streets="size")
    picked = picked.merge(miles, left_on="geoid", right_index=True, how="left").fillna(
        {"street_miles": 0, "streets": 0})
    picked["street_miles"] = picked["street_miles"].round(2)
    picked = plan_order(picked, boundaries)
    streets = streets.merge(picked[["geoid", "fly_order"]], on="geoid")
    df = df.merge(picked[["geoid", "fly_order"]], on="geoid", how="left")

    report(df, picked, streets, sizes, args.k, args.per_cluster)
    save_tables(out_dir, picked, streets, street_features)
    plot_picks(df, out_dir / "sample_block_groups.png")
    plot_map(df, boundaries, out_dir / "sample_map.png")
    plot_street_map(picked, boundaries, street_features, out_dir / "sample_streets_map.png")
    print(f"\nSaved in {out_dir}")


if __name__ == "__main__":
    main()
