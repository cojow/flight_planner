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

Run provo_orem_vs_utah_county.py first - it saves the table this reads.

    python py/sample_block_groups.py

The draw is seeded, so running it again gives the same sample. Change SEED for
a different draw, and keep the files it writes: they are the sample.

Writes to data/census/provo_orem_vs_county/:
    sample_block_groups.csv      the chosen block groups, with weight and street miles
    sample_streets.csv           one row per street per block group: what to fly
    sample_streets.geojson       the street segments, for QGIS or the flight planner map
    sample_block_groups.png      the sample among all block groups
    sample_streets_map.png       a map of the chosen block groups and their streets
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")               # draw to a file; no window to close
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import shapely
from matplotlib.ticker import FuncFormatter
from shapely.geometry import mapping, shape
from shapely.ops import transform
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler

import building_census_data as bcd

CITIES = ["Provo", "Orem"]
N_BLOCK_GROUPS = 100                # block groups to fly; change to match what you can fly
K = 3                               # strata: the number of k-means clusters
MIN_PER_STRATUM = 2                 # fewer than 2 and a stratum's spread can't be measured
SEED = 2026
COLUMNS = ["median_household_income", "median_age_years_all"]

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

DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "census" / "provo_orem_vs_county"
TABLE = DATA_DIR / "utah_county_block_groups.csv"

# Lengths and buffers need metres, not degrees. Around Provo one degree of
# latitude is ~111 km and one of longitude ~84 km; this flat-earth conversion is
# off by well under 1% across a city, and needs no projection library.
LAT0, LON0 = 40.25, -111.67
M_PER_DEG_LAT = 111_132.0
M_PER_DEG_LON = 111_320.0 * np.cos(np.radians(LAT0))


def to_metres(x, y, z=None):
    return (np.asarray(x) - LON0) * M_PER_DEG_LON, (np.asarray(y) - LAT0) * M_PER_DEG_LAT


def to_degrees(x, y, z=None):
    return np.asarray(x) / M_PER_DEG_LON + LON0, np.asarray(y) / M_PER_DEG_LAT + LAT0


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


def streets_in(geoid, polygon):
    """
    Every street segment inside one block group (plus the BUFFER_M border):
    (rows for the table, GeoJSON features), one segment per stretch between junctions.
    """
    area = transform(to_metres, polygon).buffer(BUFFER_M)
    minx, miny, maxx, maxy = transform(to_degrees, area).bounds
    # The Census lists a street once per name it goes by ("E 1000 N" and
    # "E 1000 North St"), all on the same line. Segments are merged by shape so
    # each stretch of road is counted once, under its longest name.
    found = {}
    for road_type, layer in ROAD_LAYERS.items():
        data = bcd._cached_get(ROAD_URL.format(layer=layer), {
            "geometry": f"{minx},{miny},{maxx},{maxy}", "geometryType": "esriGeometryEnvelope",
            "inSR": "4326", "spatialRel": "esriSpatialRelIntersects",
            "outFields": "NAME,MTFCC", "returnGeometry": "true", "outSR": "4326", "f": "geojson"})
        for feature in data.get("features", []):
            props = feature["properties"]
            if STREET_TYPES.get(props.get("MTFCC")) != road_type:
                continue
            inside = transform(to_metres, shape(feature["geometry"])).intersection(area)
            if inside.is_empty or inside.length < MIN_SEGMENT_M:
                continue
            key = shapely.normalize(shapely.set_precision(inside, 0.5)).wkb
            entry = found.setdefault(key, {"names": set(), "road_type": road_type, "geometry": inside})
            if (props.get("NAME") or "").strip():
                entry["names"].add(props["NAME"].strip())

    rows, features = [], []
    for entry in found.values():
        names = sorted(entry["names"], key=lambda n: (-len(n), n))
        name = names[0] if names else "(unnamed)"
        aliases = "; ".join(names[1:])
        rows.append({"geoid": geoid, "street": name, "road_type": entry["road_type"],
                     "miles": entry["geometry"].length / METERS_PER_MILE, "aliases": aliases})
        features.append({"type": "Feature",
                         "geometry": mapping(transform(to_degrees, entry["geometry"])),
                         "properties": {"geoid": geoid, "street": name,
                                        "road_type": entry["road_type"], "aliases": aliases}})
    return rows, features


# 1. Load Provo and Orem, and sort them into strata with the same k-means as before.
if not TABLE.exists():
    raise SystemExit(f"{TABLE} not found. Run py/provo_orem_vs_utah_county.py first.")
df = pd.read_csv(TABLE, dtype={"geoid": str})
df = df[df["in_provo_orem"]].dropna(subset=COLUMNS).reset_index(drop=True)

X = StandardScaler().fit_transform(df[COLUMNS])
df["stratum"] = KMeans(n_clusters=K, n_init=10, random_state=0).fit_predict(X)

# 2. Draw the sample: simple random inside each stratum.
n_groups = min(N_BLOCK_GROUPS, len(df))
sizes = df["stratum"].value_counts().sort_index()
counts = allocate(sizes, n_groups, MIN_PER_STRATUM)

rng = np.random.default_rng(SEED)
chosen = []
for stratum, n in counts.items():
    members = df.index[df["stratum"] == stratum]
    chosen += list(rng.choice(members, size=n, replace=False))
df["sampled"] = df.index.isin(chosen)

# Each sampled block group stands for this many block groups in its stratum.
# Use it when scaling flown results up to the whole city.
df["weight"] = df["stratum"].map(sizes / counts)

# 3. The streets inside every chosen block group.
print("Looking up streets (the first run downloads them; later runs use the cache)...")
boundaries = {}
for city in CITIES:
    boundaries.update(bcd.get_city_data(city, "UT", with_geometry=True)[1])
picked = df[df["sampled"]].copy()

street_rows, street_features = [], []
for i, geoid in enumerate(picked["geoid"], start=1):
    rows, features = streets_in(geoid, boundaries[geoid])
    street_rows += rows
    street_features += features
    if i % 20 == 0 or i == len(picked):
        print(f"  {i}/{len(picked)} block groups done")

streets = (pd.DataFrame(street_rows)
           .groupby(["geoid", "street", "road_type"], as_index=False)
           .agg(miles=("miles", "sum"), segments=("miles", "size"),
                aliases=("aliases", lambda a: "; ".join(sorted({x for s in a for x in s.split("; ") if x})))))
streets["miles"] = streets["miles"].round(2)
streets = streets.merge(picked[["geoid", "city", "stratum"]], on="geoid")
streets = streets.sort_values(["stratum", "geoid", "street"])

miles = streets.groupby("geoid")["miles"].agg(street_miles="sum", streets="size")
picked = picked.merge(miles, left_on="geoid", right_index=True, how="left").fillna(
    {"street_miles": 0, "streets": 0})
picked["street_miles"] = picked["street_miles"].round(2)
df = df.merge(picked[["geoid", "street_miles"]], on="geoid", how="left")

# 4. Report.
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

# A sample that matches the city should have a similar average to the whole city.
print("\nSample against the whole of Provo + Orem (weighted sample should be close):")
for col in COLUMNS:
    weighted = np.average(picked[col], weights=picked["weight"])
    print(f"  {col:<26} all {df[col].mean():>9,.1f}   sample {weighted:>9,.1f}")
print("  Provo / Orem share of the sample:",
      picked["city"].str.replace(" city", "").value_counts().to_dict())

# 5. Save the sample, the streets, and two charts.
keep = ["geoid", "city", "stratum", "median_household_income", "median_age_years_all",
        "housing_units", "weight", "streets", "street_miles"]
picked[keep].sort_values(["stratum", "geoid"]).to_csv(DATA_DIR / "sample_block_groups.csv", index=False)
streets.to_csv(DATA_DIR / "sample_streets.csv", index=False)
(DATA_DIR / "sample_streets.geojson").write_text(
    json.dumps({"type": "FeatureCollection", "features": street_features}))

colors = ["#d1495b", "#2e7d9a", "#edae49", "#66a182", "#8d6a9f", "#8c8c8c"]

fig, ax = plt.subplots(figsize=(9, 6))
for stratum, group in df.groupby("stratum"):
    color = colors[stratum % len(colors)]
    ax.scatter(group["median_age_years_all"], group["median_household_income"],
               s=30, alpha=0.35, color=color, edgecolors="none")
    chosen_here = group[group["sampled"]]
    ax.scatter(chosen_here["median_age_years_all"], chosen_here["median_household_income"],
               s=70, color=color, edgecolors="black", linewidths=1.2,
               label=f"Stratum {stratum}: {len(chosen_here)} of {len(group)} sampled")
ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"${v / 1000:,.0f}K"))
ax.set_xlabel("Median house age (years)")
ax.set_ylabel("Median household income")
ax.set_title("Sampled block groups (outlined) among all Provo + Orem block groups")
ax.legend(frameon=False, loc="upper right", fontsize=9)
ax.grid(alpha=0.25)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
fig.tight_layout()
fig.savefig(DATA_DIR / "sample_block_groups.png", dpi=200)
plt.close(fig)

# The map: each sampled block group's outline, with its streets drawn in the stratum's color.
stratum_of = picked.set_index("geoid")["stratum"]
fig, ax = plt.subplots(figsize=(9, 9))
for geoid in df.loc[~df["sampled"], "geoid"]:
    if geoid in boundaries:
        x, y = boundaries[geoid].exterior.xy if boundaries[geoid].geom_type == "Polygon" \
            else boundaries[geoid].geoms[0].exterior.xy
        ax.plot(x, y, color="#cccccc", linewidth=0.5)
for geoid, stratum in stratum_of.items():
    poly = boundaries[geoid]
    x, y = poly.exterior.xy if poly.geom_type == "Polygon" else poly.geoms[0].exterior.xy
    ax.plot(x, y, color=colors[stratum % len(colors)], linewidth=0.8)
for feature in street_features:
    stratum = stratum_of[feature["properties"]["geoid"]]
    geom = shape(feature["geometry"])
    for line in getattr(geom, "geoms", [geom]):
        if line.geom_type == "LineString":
            x, y = line.xy
            ax.plot(x, y, color=colors[stratum % len(colors)], linewidth=0.7)
ax.set_aspect(1 / np.cos(np.radians(40.25)))        # keep north-south and east-west to scale
ax.set_title("Streets to fly in the sampled block groups (color = stratum; grey = not sampled)")
ax.axis("off")
fig.tight_layout()
fig.savefig(DATA_DIR / "sample_streets_map.png", dpi=200)
plt.close(fig)

print(f"\nSaved in {DATA_DIR}:\n  sample_block_groups.csv, sample_streets.csv, "
      "sample_streets.geojson, sample_block_groups.png, sample_streets_map.png")
