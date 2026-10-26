"""
Which block groups to fly, and which streets inside them: a stratified random
sample of Provo and Orem.

Follows BYU_research_documentation/methodology.qmd:
  1. The sampling frame is the Census block groups of the study cities.
  2. The k-means clusters (income + house age) are the strata. Block groups are
     drawn at random from each stratum, so every kind of neighborhood is flown,
     each stratum getting a share of the sample that matches its share of the city.
  3. Every street inside each chosen block group is flown. This script lists
     those streets, with their length, from the Census Bureau's road data.
  4. It then sets the order to fly them in:
       - pilot: one block group per stratum, flown first to time the work and
         test the method. The most typical one (by street miles) in clear airspace.
       - test areas: a random share of each stratum, held back to test how well
         the results extrapolate to areas that were not flown (methodology, section 2).
       - airspace: each block group, and each street, is checked against the FAA's
         UAS Facility Map, so you know which need LAANC, which cannot be flown at
         200 ft, and which streets to skip.
       - waves: after the pilot, groups of WAVE_SIZE block groups, each wave
         holding every stratum in proportion, so stopping early still leaves a
         balanced sample. Inside a wave they are ordered so each is next to the
         last. Block groups that are mostly in restricted airspace go last.

Run provo_orem_vs_utah_county.py first - it saves the table this reads.

    python py/sample_block_groups.py

The draw is seeded, so running it again gives the same sample. Change SEED for
a different draw, and keep the files it writes: they are the sample.

Writes to data/census/provo_orem_vs_county/:
    sample_block_groups.csv      the chosen block groups in flying order
    sample_streets.csv           one row per street per block group: what to fly
    sample_streets.geojson       the street segments, for QGIS or the flight planner map
    sample_block_groups.png      the sample among all block groups
    sample_streets_map.png       a map of the chosen block groups and their streets
    sample_fly_order_map.png     a map of the flying order, with airspace holds
"""
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
from provo_orem_common import (COLORS, COLUMNS, DATA_DIR, LATITUDE, cluster_block_groups,
                               load_provo_orem, save_figure, style_income_age_axes)

CITIES = ["Provo", "Orem"]
N_BLOCK_GROUPS = 100                # block groups to fly; change to match what you can fly
MIN_PER_STRATUM = 2                 # fewer than 2 and a stratum's spread can't be measured
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

# Flying order.
HIGH_FLIGHT_FT = 200                # the high (nadir) flight height in the methodology
TEST_SHARE = 0.2                    # share of each stratum held back to test extrapolation (to decide)
WAVE_SIZE = 10                      # block groups per wave after the pilot
RESTRICTED_HOLD_SHARE = 0.25        # hold a block group when this much of it is below 200 ft ceiling
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
# 1-2. The sample
# --------------------------------------------------------------------------

def allocate(sizes, total, minimum):
    """
    How many block groups to draw from each stratum: proportional to its size,
    never below `minimum` (or the stratum's own size), adding up to `total`.
    """
    sizes = pd.Series(sizes)
    ideal = sizes / sizes.sum() * total
    counts = np.maximum(np.floor(ideal), minimum).clip(upper=sizes).astype(int)
    while counts.sum() < total:                     # hand out what rounding left over
        room = counts[counts < sizes]
        counts[(ideal - counts)[room.index].idxmax()] += 1
    while counts.sum() > total:                     # or take back what the minimum added
        room = counts[counts > minimum]
        counts[(ideal - counts)[room.index].idxmin()] -= 1
    return counts


def draw_sample(df):
    """
    Mark `sampled` on a simple random draw inside each stratum, and give each
    block group a `weight`: how many block groups of its stratum it stands for.
    Use the weight when scaling flown results up to the whole city.
    Returns (stratum sizes, number drawn).
    """
    n_groups = min(N_BLOCK_GROUPS, len(df))
    sizes = df["stratum"].value_counts().sort_index()
    counts = allocate(sizes, n_groups, MIN_PER_STRATUM)

    rng = np.random.default_rng(SEED)
    chosen = []
    for stratum, n in counts.items():
        members = df.index[df["stratum"] == stratum]
        chosen += list(rng.choice(members, size=n, replace=False))
    df["sampled"] = df.index.isin(chosen)
    df["weight"] = df["stratum"].map(sizes / counts)
    return sizes, n_groups


# --------------------------------------------------------------------------
# 3. The streets
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


def find_streets(picked, boundaries, bounds, cells):
    """The streets in every sampled block group: (table with one row per street per block group, GeoJSON features)."""
    low_squares = [g for c, g in (cells or []) if c < HIGH_FLIGHT_FT]
    low_zone_m = transform(to_metres, shapely.union_all(low_squares)) if low_squares else None
    margin = 0.005                                  # degrees; far more than BUFFER_M
    minx, miny, maxx, maxy = bounds
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
    return streets.merge(picked[["geoid", "city", "stratum"]], on="geoid"), features


# --------------------------------------------------------------------------
# 4. Airspace, pilot, test areas and the flying order
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
      partial     a small part (under RESTRICTED_HOLD_SHARE) has a ceiling below 200 ft:
                  fly the rest, and leave out the streets in the restricted squares
      restricted  a large part has a ceiling below 200 ft (a ceiling of 0 needs FAA
                  authorization, not LAANC): held until approval or a lower flight
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
    if share >= RESTRICTED_HOLD_SHARE:
        return "restricted", lowest, round(share, 2)
    return ("partial" if low_squares else "laanc"), lowest, round(share, 2)


def nearest_first(points, start):
    """Visit order for {id: (x, y)} in metres: the nearest to `start`, then the nearest to that, and so on."""
    left, order, here = dict(points), [], start
    while left:
        nxt = min(left, key=lambda k: (left[k][0] - here[0]) ** 2 + (left[k][1] - here[1]) ** 2)
        order.append(nxt)
        here = left.pop(nxt)
    return order


def plan_flights(picked, boundaries, cells):
    """
    Add airspace, pilot, test_area, wave and fly_order to the sampled block groups.
    Returns (the table, the number of the last wave - the held block groups).
    """
    status = [airspace_status(boundaries[g], cells) for g in picked["geoid"]]
    picked["airspace"] = [s for s, _, _ in status]
    picked["faa_ceiling_ft"] = [c for _, c, _ in status]
    picked["share_restricted"] = [r for _, _, r in status]
    picked["hold"] = picked["airspace"] == "restricted"                    # not flyable as planned

    centres = {g: boundaries[g].centroid for g in picked["geoid"]}
    picked["lat"] = [round(centres[g].y, 5) for g in picked["geoid"]]
    picked["lon"] = [round(centres[g].x, 5) for g in picked["geoid"]]
    xy = {g: tuple(float(v) for v in to_metres(c.x, c.y)) for g, c in centres.items()}
    start = tuple(float(v) for v in to_metres(START_LON, START_LAT))

    # Pilot: in each stratum, the block group whose street miles are closest to the
    # stratum's typical one (so its timing is representative), in clear airspace.
    picked["pilot"] = False
    for stratum, group in picked.groupby("stratum"):
        usable = group[group["airspace"] == "clear"]
        if usable.empty:
            usable = group[~group["hold"]]
        if usable.empty:
            continue
        typical = group["street_miles"].median()
        picked.loc[(usable["street_miles"] - typical).abs().idxmin(), "pilot"] = True

    # Test areas: random within each stratum, never the pilot.
    test_rng = np.random.default_rng(SEED + 1)
    picked["test_area"] = False
    for stratum, group in picked.groupby("stratum"):
        candidates = group[~group["pilot"]]
        n_test = min(max(1, round(TEST_SHARE * len(group))), len(candidates))
        picked.loc[test_rng.choice(candidates.index, size=n_test, replace=False), "test_area"] = True

    # Order: pilots first (wave 0); then waves of WAVE_SIZE with every stratum in
    # proportion (each block group's place in its stratum's random order, as a
    # fraction, is what the waves are cut on); held block groups last.
    picked["wave"] = 0
    order = nearest_first({g: xy[g] for g in picked.loc[picked["pilot"], "geoid"]}, start)
    here = xy[order[-1]] if order else start

    ready = picked[~picked["hold"] & ~picked["pilot"]].sample(frac=1, random_state=SEED)
    ready["progress"] = ((ready.groupby("stratum").cumcount() + 0.5)
                         / ready.groupby("stratum")["geoid"].transform("size"))
    ready = ready.sort_values(["progress", "stratum"])
    ready["wave"] = 1 + np.arange(len(ready)) // WAVE_SIZE
    for wave, group in ready.groupby("wave"):
        visit = nearest_first({g: xy[g] for g in group["geoid"]}, here)
        order += visit
        here = xy[visit[-1]]
        picked.loc[group.index, "wave"] = wave

    held = picked[picked["hold"]]
    held_wave = int(ready["wave"].max() + 1) if len(ready) else 1
    order += nearest_first({g: xy[g] for g in held["geoid"]}, here)
    picked.loc[held.index, "wave"] = held_wave
    picked["fly_order"] = picked["geoid"].map({g: i for i, g in enumerate(order, start=1)})
    return picked, held_wave


# --------------------------------------------------------------------------
# 5-6. Report and save
# --------------------------------------------------------------------------

def report(df, picked, streets, sizes, n_groups):
    summary = df.groupby("stratum").agg(
        block_groups=("geoid", "size"),
        sampled=("sampled", "sum"),
        income=("median_household_income", "mean"),
        house_age=("median_age_years_all", "mean"),
        street_miles=("street_miles", "sum"),
    ).round(1)
    print(f"\n{len(df)} Provo + Orem block groups, {len(sizes)} strata, {n_groups} sampled\n")
    print(summary.to_string())
    print(f"\nStreets to fly: {len(streets)} (street, block group) pairs, "
          f"{streets['miles'].sum():,.0f} miles of street. "
          f"Flown twice (high and low) that is {2 * streets['miles'].sum():,.0f} miles of flight.")
    print(f"{int(streets['restricted'].sum())} of those streets "
          f"({streets.loc[streets['restricted'], 'miles'].sum():,.0f} miles) "
          f"are mostly in FAA squares with a ceiling under {HIGH_FLIGHT_FT} ft.")

    # A sample that matches the city should have a similar average to the whole city.
    print("\nSample against the whole of Provo + Orem (weighted sample should be close):")
    for col in COLUMNS:
        weighted = np.average(picked[col], weights=picked["weight"])
        print(f"  {col:<26} all {df[col].mean():>9,.1f}   sample {weighted:>9,.1f}")
    print("  Provo / Orem share of the sample:",
          picked["city"].str.replace(" city", "").value_counts().to_dict())

    held = picked[picked["hold"]]
    waves = picked.loc[~picked["hold"] & ~picked["pilot"], "wave"].nunique()
    print(f"\nFlying order: {int(picked['pilot'].sum())} pilot block groups, then "
          f"{waves} waves of up to {WAVE_SIZE}, then {len(held)} held for airspace.")
    for _, row in picked[picked["pilot"]].sort_values("fly_order").iterrows():
        print(f"  pilot #{row['fly_order']}: {row['geoid']} ({row['city']}, stratum {row['stratum']}, "
              f"{row['street_miles']:.1f} street miles, airspace: {row['airspace']})")
    print("Airspace:", picked["airspace"].value_counts().to_dict())
    print("Test areas held back per stratum:", picked[picked["test_area"]].groupby("stratum").size().to_dict())
    if len(held):
        print(f"Held (mostly under a ceiling below {HIGH_FLIGHT_FT} ft): {len(held)} block groups, "
              f"{held['street_miles'].sum():.0f} street miles")


def save_tables(picked, streets, street_features):
    keep = ["fly_order", "wave", "pilot", "test_area", "geoid", "city", "stratum", "airspace",
            "faa_ceiling_ft", "share_restricted", "streets", "street_miles", "median_household_income",
            "median_age_years_all", "housing_units", "weight", "lat", "lon"]
    picked[keep].sort_values("fly_order").to_csv(DATA_DIR / "sample_block_groups.csv", index=False)
    streets.to_csv(DATA_DIR / "sample_streets.csv", index=False)
    (DATA_DIR / "sample_streets.geojson").write_text(
        json.dumps({"type": "FeatureCollection", "features": street_features}))


# --------------------------------------------------------------------------
# Charts
# --------------------------------------------------------------------------

def rings(geometry):
    """The outer edge of each part of a (multi)polygon, as arrays of x, y."""
    return [np.array(part.exterior.coords) for part in getattr(geometry, "geoms", [geometry])]


def plot_sample(df, path):
    """Every block group by income and house age, with the sampled ones outlined."""
    fig, ax = plt.subplots(figsize=(9, 6))
    for stratum, group in df.groupby("stratum"):
        color = COLORS[stratum % len(COLORS)]
        ax.scatter(group["median_age_years_all"], group["median_household_income"],
                   s=30, alpha=0.35, color=color, edgecolors="none")
        chosen = group[group["sampled"]]
        ax.scatter(chosen["median_age_years_all"], chosen["median_household_income"],
                   s=70, color=color, edgecolors="black", linewidths=1.2,
                   label=f"Stratum {stratum}: {len(chosen)} of {len(group)} sampled")
    style_income_age_axes(ax, "Sampled block groups (outlined) among all Provo + Orem block groups")
    ax.legend(frameon=False, loc="upper right", fontsize=9)
    save_figure(fig, path)


def plot_street_map(df, picked, boundaries, street_features, path):
    """Each sampled block group's outline and streets in its stratum's color; grey = not sampled."""
    stratum_of = picked.set_index("geoid")["stratum"]
    fig, ax = plt.subplots(figsize=(9, 9))
    unsampled = [ring for geoid in df.loc[~df["sampled"], "geoid"] if geoid in boundaries
                 for ring in rings(boundaries[geoid])]
    ax.add_collection(LineCollection(unsampled, colors="#cccccc", linewidths=0.5))

    outlines, outline_colors = [], []
    for geoid, stratum in stratum_of.items():
        for ring in rings(boundaries[geoid]):
            outlines.append(ring)
            outline_colors.append(COLORS[stratum % len(COLORS)])
    ax.add_collection(LineCollection(outlines, colors=outline_colors, linewidths=0.8))

    lines, line_colors = [], []
    for feature in street_features:
        color = COLORS[stratum_of[feature["properties"]["geoid"]] % len(COLORS)]
        geom = shape(feature["geometry"])
        for line in getattr(geom, "geoms", [geom]):
            if line.geom_type == "LineString":
                lines.append(np.array(line.coords))
                line_colors.append(color)
    ax.add_collection(LineCollection(lines, colors=line_colors, linewidths=0.7))

    ax.autoscale_view()
    ax.set_aspect(1 / np.cos(np.radians(LATITUDE)))     # keep north-south and east-west to scale
    ax.set_title("Streets to fly in the sampled block groups (color = stratum; grey = not sampled)")
    ax.axis("off")
    save_figure(fig, path)


def plot_fly_order_map(df, picked, boundaries, held_wave, path):
    """
    Block groups colored by wave and numbered by fly order. Held block groups
    (airspace) are hatched red; the pilots carry a star.
    """
    cmap = plt.get_cmap("viridis", max(held_wave, 2))
    fig, ax = plt.subplots(figsize=(9, 9))
    unsampled = [ring for geoid in df.loc[~df["sampled"], "geoid"] if geoid in boundaries
                 for ring in rings(boundaries[geoid])]
    ax.add_collection(LineCollection(unsampled, colors="#cccccc", linewidths=0.5))
    for _, row in picked.iterrows():
        for ring in rings(boundaries[row["geoid"]]):
            ax.add_patch(MplPolygon(
                ring, closed=True,
                facecolor="white" if row["hold"] else cmap(row["wave"] - 1 if row["wave"] else 0),
                edgecolor="#d1495b" if row["hold"] else "white", linewidth=1.2 if row["hold"] else 0.6,
                hatch="////" if row["hold"] else None, alpha=0.9))
        c = boundaries[row["geoid"]].centroid
        ax.text(c.x, c.y, str(row["fly_order"]), fontsize=5.5, ha="center", va="center",
                color="black" if row["hold"] else "white")
        if row["pilot"]:
            ax.plot(c.x, c.y + 0.0025, marker="*", color="#d1495b", markeredgecolor="black",
                    markersize=11, zorder=5)
    ax.autoscale_view()
    ax.set_aspect(1 / np.cos(np.radians(LATITUDE)))
    ax.axis("off")
    ax.set_title("Order to fly the sampled block groups (number = fly order, star = pilot)")
    ax.legend(handles=[Patch(facecolor=cmap(0), label="Wave 1 (flown first, after the pilots)"),
                       Patch(facecolor=cmap(max(held_wave - 2, 0)), label=f"Wave {max(held_wave - 1, 1)} (flown last)"),
                       Patch(facecolor="white", edgecolor="#d1495b", hatch="////",
                             label="Held: mostly in restricted airspace")],
              loc="lower left", frameon=False, fontsize=8)
    save_figure(fig, path)


# --------------------------------------------------------------------------

def main():
    # Provo and Orem, sorted into strata with the same k-means as kmeans_provo_orem.py.
    df = load_provo_orem()
    df["stratum"], _, _ = cluster_block_groups(df)
    sizes, n_groups = draw_sample(df)

    print("Looking up streets (the first run downloads them; later runs use the cache)...")
    boundaries = {}
    for city in CITIES:
        boundaries.update(bcd.get_city_data(city, "UT", with_geometry=True)[1])
    picked = df[df["sampled"]].copy()

    # The FAA facility map is fetched first so each street can be checked against it.
    bounds = shapely.union_all([boundaries[g] for g in picked["geoid"]]).bounds
    cells = faa_cells(bounds)
    if cells is None:
        print("Warning: could not reach the FAA facility map; airspace is 'unknown'. "
              "Check it in the flight planner (Show FAA Airspace Restrictions).")

    streets, street_features = find_streets(picked, boundaries, bounds, cells)
    miles = streets.groupby("geoid")["miles"].agg(street_miles="sum", streets="size")
    picked = picked.merge(miles, left_on="geoid", right_index=True, how="left").fillna(
        {"street_miles": 0, "streets": 0})
    picked["street_miles"] = picked["street_miles"].round(2)
    df = df.merge(picked[["geoid", "street_miles"]], on="geoid", how="left")

    picked, held_wave = plan_flights(picked, boundaries, cells)
    streets = streets.merge(picked[["geoid", "fly_order", "wave", "pilot", "test_area", "airspace"]], on="geoid")
    streets = streets.sort_values(["fly_order", "street"])

    report(df, picked, streets, sizes, n_groups)
    save_tables(picked, streets, street_features)
    plot_sample(df, DATA_DIR / "sample_block_groups.png")
    plot_street_map(df, picked, boundaries, street_features, DATA_DIR / "sample_streets_map.png")
    plot_fly_order_map(df, picked, boundaries, held_wave, DATA_DIR / "sample_fly_order_map.png")
    print(f"\nSaved in {DATA_DIR}:\n  sample_block_groups.csv, sample_streets.csv, sample_streets.geojson, "
          "sample_block_groups.png, sample_streets_map.png, sample_fly_order_map.png")


if __name__ == "__main__":
    main()
