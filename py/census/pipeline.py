"""
What each step does, from settings to files. steps.py wraps these with the
bookkeeping (checking earlier steps, recording inputs); call those, not these.

Each function takes the run's config and the step's output folder, and returns
({label: path}, report text).
"""
import io
import json

import numpy as np
import pandas as pd
import shapely
from shapely.geometry import mapping, shape

from . import acs, clustering, geography, plots, sampling, streets
from .config import STEPS


def _read_table(folder, step, name):
    return pd.read_csv(folder / STEPS[step]["folder"] / name, dtype={"geoid": str})


def _read_boundaries(folder):
    data = json.loads((folder / STEPS["areas"]["folder"] / "block_groups.geojson").read_text())
    return {f["properties"]["geoid"]: shape(f["geometry"]) for f in data["features"]}


def _dropped_note(dropped):
    if not dropped:
        return ""
    return (f"{len(dropped)} used block group(s) left out for a blank value in a clustering column "
            f"(the Census had no estimate): {', '.join(dropped)}\n")


def _clustered_rows(folder, config):
    """The block groups step 1 kept, minus any missing a clustering column."""
    table = _read_table(folder, "areas", "block_groups.csv")
    kept, dropped = clustering.usable(table[table["used"]], config["features"]["columns"])
    return kept, dropped


# --------------------------------------------------------------------------
# 1. Areas
# --------------------------------------------------------------------------

def areas(config, out):
    a = config["areas"]
    county = acs.county_block_groups(a["state"], a["county"], a["year"])
    members, boundaries = geography.block_groups_in_places(a["state"], a["places"], a["min_overlap"], a["year"])

    # Keep the county table's order: it is the order k-means sees the rows in.
    table = county.merge(members.drop(columns="county"), on="geoid", how="inner")
    missing = members[~members["geoid"].isin(county["geoid"])].drop(columns="county")
    table = pd.concat([table, missing], ignore_index=True)

    reasons = pd.Series("", index=table.index)
    reasons[table["housing_units"].fillna(0) <= 0] = "no housing units"
    reasons[table["census_name"].isna()] = "no ACS estimate"
    reasons[table["geoid"].isin(a["exclude"])] = "excluded by hand"
    table.insert(1, "used", reasons == "")
    table.insert(2, "reason", reasons)
    first = ["geoid", "used", "reason", "city", "share_in_city", "census_name"]
    table = table[first + [c for c in table.columns if c not in first]]

    county["in_cities"] = county["geoid"].isin(table.loc[table["used"], "geoid"])
    files = {
        "Block groups (table)": out / "block_groups.csv",
        "Block groups (GeoJSON)": out / "block_groups.geojson",
        "Whole county (table)": out / "county_block_groups.csv",
        "Map": out / "map.png",
        "Sources (JSON)": out / "sources.json",
    }
    files["Sources (JSON)"].write_text(json.dumps(
        {"acs": f"ACS 5-year {a['year']}", "tiger": geography.sources(a["year"], a["state"], a["county"], ("place", "bg"))},
        indent=2) + "\n")
    table.to_csv(files["Block groups (table)"], index=False)
    county.to_csv(files["Whole county (table)"], index=False)
    features = [{"type": "Feature", "geometry": mapping(boundaries[g]),
                 "properties": {"geoid": g, "city": c, "used": bool(u)}}
                for g, c, u in zip(table["geoid"], table["city"], table["used"]) if g in boundaries]
    files["Block groups (GeoJSON)"].write_text(json.dumps({"type": "FeatureCollection", "features": features}))
    plots.areas_map(table, boundaries, files["Map"])

    report = io.StringIO()
    cities = ", ".join(p.get("name", p.get("geoid")) for p in a["places"])
    report.write(f"{cities}: {len(table)} block groups with at least {a['min_overlap']:.0%} of their area "
                 f"in a city; {int(table['used'].sum())} used.\n")
    for reason, n in table.loc[~table["used"], "reason"].value_counts().items():
        report.write(f"  {n} left out: {reason}\n")
    report.write(f"\nACS 5-year {a['year']}, county {a['state']}{a['county']} ({len(county)} block groups).\n")
    report.write("\nChosen cities against the rest of the county (used block groups only):\n")
    report.write(compare_with_county(county).to_string() + "\n")
    return files, report.getvalue()


def compare_with_county(county, columns=("median_household_income", "median_age_years_all")):
    """Mean and median of each column: the chosen cities, the whole county, and the rest of it."""
    usable = county[county["housing_units"].fillna(0) > 0].dropna(subset=list(columns))
    groups = {"Chosen cities": usable[usable["in_cities"]], "Whole county": usable,
              "Rest of county": usable[~usable["in_cities"]]}
    rows = {}
    for name, df in groups.items():
        row = {"block groups": len(df)}
        for col in columns:
            row[f"{col} (mean)"] = df[col].mean()
            row[f"{col} (median)"] = df[col].median()
        rows[name] = row
    return pd.DataFrame(rows).round(1)


# --------------------------------------------------------------------------
# 2. Choose k
# --------------------------------------------------------------------------

def choose_k(config, folder, out):
    f, c = config["features"], config["choose_k"]
    df, dropped = _clustered_rows(folder, config)
    X, scaler = clustering.scale(df, f["columns"])
    scores = clustering.score_k(X, range(c["k_min"], c["k_max"] + 1), f["seed"], f["n_init"])

    files = {"Scores (table)": out / "k_scores.csv", "Scores (chart)": out / "k_scores.png"}
    scores.to_csv(files["Scores (table)"], index=False)
    plots.k_scores(scores, files["Scores (chart)"])

    previews, summaries = [], []
    for k in c["preview"]:
        if not 2 <= k < len(df):
            continue
        frame = df.copy()
        frame["cluster"] = clustering.kmeans(X, k, f["seed"], f["n_init"]).labels_
        previews.append((k, frame))
        summaries.append(clustering.summary(frame, f["columns"], k))
        files[f"Clusters, k={k}"] = plots.clusters(frame, X, scaler, f["columns"], k, out / f"clusters_k{k}.png")
    if previews:
        files["Preview k side by side"] = plots.clusters_compare(previews, X, scaler, f["columns"],
                                                                 out / "clusters_compare.png")
        files["Preview summary (table)"] = out / "preview_summary.csv"
        pd.concat(summaries, ignore_index=True).to_csv(files["Preview summary (table)"], index=False)

    report = io.StringIO()
    report.write(f"{len(df)} block groups clustered on {', '.join(f['columns'])}.\n{_dropped_note(dropped)}\n")
    report.write(scores.to_string(index=False) + "\n")
    if len(scores):
        best = scores.loc[scores["silhouette"].idxmax()]
        report.write(f"\nHighest silhouette: k = {int(best['k'])}. Check the elbow chart too; the choice is yours.\n")
    if summaries:
        report.write("\nClusters for each preview k:\n" + pd.concat(summaries, ignore_index=True).to_string(index=False) + "\n")
    return files, report.getvalue()


# --------------------------------------------------------------------------
# 3. Cluster
# --------------------------------------------------------------------------

def cluster(config, folder, out):
    f, k = config["features"], config["cluster"]["k"]
    df, dropped = _clustered_rows(folder, config)
    if k >= len(df):
        raise ValueError(f"k = {k} needs more than {len(df)} block groups.")
    X, scaler = clustering.scale(df, f["columns"])
    df["cluster"] = clustering.kmeans(X, k, f["seed"], f["n_init"]).labels_
    table = df[["geoid", "cluster"] + [c for c in df.columns if c not in ("geoid", "cluster")]]
    summary = clustering.summary(df, f["columns"])

    files = {"Clusters (table)": out / "clusters.csv", "Summary (table)": out / "cluster_summary.csv",
             "Chart": out / "clusters.png", "Map": out / "clusters_map.png"}
    table.to_csv(files["Clusters (table)"], index=False)
    summary.to_csv(files["Summary (table)"], index=False)
    plots.clusters(df, X, scaler, f["columns"], k, files["Chart"])
    plots.clusters_map(df, _read_boundaries(folder), files["Map"], f"{k} clusters")

    report = (f"{len(df)} block groups in {k} clusters on {', '.join(f['columns'])}.\n{_dropped_note(dropped)}\n"
              + summary.to_string(index=False) + "\n")
    return files, report


# --------------------------------------------------------------------------
# 4. Sample
# --------------------------------------------------------------------------

def sample(config, folder, out):
    s, columns = config["sample"], config["features"]["columns"]
    df = _read_table(folder, "cluster", "clusters.csv")
    boundaries = _read_boundaries(folder)
    k = config["cluster"]["k"]

    notes = []
    cells = sampling.faa_cells(shapely.union_all([boundaries[g] for g in df["geoid"]]).bounds,
                               save_to=out / "faa_facility_map.geojson")
    status =[sampling.airspace_status(boundaries[g], cells, s["high_flight_ft"], s["restricted_share"])
              for g in df["geoid"]]
    df["airspace"] = [x for x, _, _ in status]
    df["faa_ceiling_ft"] = [c for _, c, _ in status]
    df["share_restricted"] = [r for _, _, r in status]

    notes += sampling.draw(df, s["per_cluster"], s["seed"], s["weights"])
    picked = df[df["sampled"]].copy()

    street_table, street_features = streets.find_streets(picked, boundaries, cells, s, config["areas"])
    flyable = street_table[~street_table["restricted"]]
    miles = flyable.groupby("geoid")["miles"].agg(street_miles="sum", streets="size")
    picked = picked.merge(miles, left_on="geoid", right_index=True, how="left").fillna(
        {"street_miles": 0, "streets": 0})
    picked["street_miles"] = picked["street_miles"].round(2)
    picked = sampling.plan_order(picked, boundaries, s["start_lat"], s["start_lon"])
    street_table = street_table.merge(picked[["geoid", "fly_order"]], on="geoid")
    df = df.merge(picked[["geoid", "fly_order"]], on="geoid", how="left")

    files = {
        "Picked block groups (table)": out / "sample_block_groups.csv",
        "Streets (table)": out / "sample_streets.csv",
        "Streets (GeoJSON)": out / "sample_streets.geojson",
        "Picks chart": out / "sample_block_groups.png",
        "Map": out / "sample_map.png",
        "Street maps": out / "sample_streets_map.png",
        "FAA airspace used (GeoJSON)": out / "faa_facility_map.geojson",
        "Sources (JSON)": out / "sources.json",
    }
    a = config["areas"]
    files["Sources (JSON)"].write_text(json.dumps(
        {"tiger": geography.sources(a["year"], a["state"], a["county"], ("roads",)),
         "faa": {"url": sampling.FAA_URL, "fetched": pd.Timestamp.now().strftime("%Y-%m-%d %H:%M")}},
        indent=2) + "\n")
    keep = ["fly_order", "pick", "geoid", "city", "cluster", "airspace", "faa_ceiling_ft",
            "share_restricted", "streets", "street_miles", *columns, "housing_units", "weight", "lat", "lon"]
    keep = list(dict.fromkeys(keep))
    picked[keep].sort_values("fly_order").to_csv(files["Picked block groups (table)"], index=False)
    street_table.sort_values(["fly_order", "street"]).to_csv(files["Streets (table)"], index=False)
    files["Streets (GeoJSON)"].write_text(json.dumps({"type": "FeatureCollection", "features": street_features}))
    plots.picks(df, columns, files["Picks chart"])
    plots.sample_map(df, boundaries, files["Map"])
    plots.street_map(picked, boundaries, street_features, files["Street maps"])

    return files, _sample_report(df, picked, street_table, k, s, columns, notes)


def _sample_report(df, picked, street_table, k, s, columns, notes):
    report = io.StringIO()
    for note in notes:
        report.write(note + "\n")
    summary = df.groupby("cluster").agg(
        block_groups=("geoid", "size"),
        eligible=("airspace", lambda a: int((a != "restricted").sum())),
        picked=("sampled", "sum"),
        **{c: (c, "mean") for c in columns},
    ).round(1)
    report.write(f"\n{len(df)} block groups in {k} clusters; {s['per_cluster']} per cluster = {len(picked)} to fly\n\n")
    report.write(summary.to_string() + "\n\nPicked, in flying order:\n")
    for _, r in picked.sort_values("fly_order").iterrows():
        report.write(f"  {r['fly_order']}. {r['geoid']}  {r['city']:<11} cluster {r['cluster']}  "
                     f"pick {r['pick']}  {r['street_miles']:>5.1f} street miles  airspace: {r['airspace']}\n")
    flyable = street_table[~street_table["restricted"]]
    total = flyable["miles"].sum()
    report.write(f"\nStreets to fly: {len(flyable)} (street, block group) rows, {total:,.1f} miles of street. "
                 f"Flown twice (high and low) that is {2 * total:,.1f} miles of flight.\n")
    restricted = street_table[street_table["restricted"]]
    report.write(f"Not counted: {len(restricted)} stretches ({restricted['miles'].sum():,.2f} miles) inside FAA "
                 f"squares with a ceiling under {s['high_flight_ft']} ft (restricted = True). "
                 "Skip them without FAA authorization.\n")

    # What the weights describe, and how far the picks sit from it.
    frame = df if s["weights"] == "legacy" else df[df["airspace"] != "restricted"]
    if s["weights"] == "legacy":
        report.write("\nWeights: legacy - each pick stands for its whole cluster, restricted block groups included.\n")
    else:
        report.write(f"\nWeights: eligible - estimates describe the {len(frame)} block groups outside restricted "
                     f"airspace, not the {len(df) - len(frame)} that could not be drawn.\n")
    report.write("Picks (weighted) against the block groups they stand for:\n")
    for col in columns:
        weighted = np.average(picked[col], weights=picked["weight"])
        report.write(f"  {col:<26} those {frame[col].mean():>9,.1f}   picks {weighted:>9,.1f}\n")
    return report.getvalue()
