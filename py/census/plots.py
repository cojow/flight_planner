"""
The charts and maps each step saves, as PNG files.
"""
import matplotlib
matplotlib.use("Agg")               # draw to files; no window to close
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Ellipse, Patch, Polygon as MplPolygon
from matplotlib.ticker import FuncFormatter
from shapely.geometry import shape

COLORS = ["#d1495b", "#2e7d9a", "#edae49", "#66a182", "#8d6a9f", "#8c8c8c",
          "#e07b39", "#3f88c5", "#a4508b", "#5c946e"]
LATITUDE = 40.25                    # Utah County; sets the map aspect so distances look right
CIRCLE_SHARE = 0.90                 # share of a cluster's dots its circle must contain

LABELS = {"median_household_income": "Median household income",
          "median_age_years_all": "Median house age (years)"}


def color(cluster):
    return COLORS[int(cluster) % len(COLORS)]


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def axes_for(columns):
    """(x column, y column) for a scatter: house age across and income up when both are used."""
    if len(columns) == 1:
        return columns[0], None
    if "median_household_income" in columns and "median_age_years_all" in columns:
        return "median_age_years_all", "median_household_income"
    return columns[1], columns[0]


def style_axes(ax, x, y, title):
    for col, axis, setter in ((x, ax.xaxis, ax.set_xlabel), (y, ax.yaxis, ax.set_ylabel)):
        if col is None:
            continue
        setter(LABELS.get(col, col))
        if "income" in col:
            axis.set_major_formatter(FuncFormatter(lambda v, _: f"${v / 1000:,.0f}K"))
    ax.set_title(title)
    ax.grid(alpha=0.25)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def rings(geometry):
    """The outer edge of each part of a (multi)polygon, as arrays of x, y."""
    return [np.array(part.exterior.coords) for part in getattr(geometry, "geoms", [geometry])]


def _map_axes(ax):
    ax.autoscale_view()
    ax.set_aspect(1 / np.cos(np.radians(LATITUDE)))
    ax.axis("off")


# --------------------------------------------------------------------------
# Step 1: areas
# --------------------------------------------------------------------------

def areas_map(table, boundaries, path):
    """The block groups in the chosen cities: one color per city, hatched = not used."""
    fig, ax = plt.subplots(figsize=(9, 9))
    cities = sorted(table["city"].dropna().unique())
    for _, row in table.iterrows():
        if row["geoid"] not in boundaries:
            continue
        c = COLORS[cities.index(row["city"]) % len(COLORS)]
        style = (dict(facecolor=c, alpha=0.45, edgecolor="white", linewidth=0.5) if row["used"] else
                 dict(facecolor="white", edgecolor="#555555", linewidth=0.6, hatch="////"))
        for ring in rings(boundaries[row["geoid"]]):
            ax.add_patch(MplPolygon(ring, closed=True, **style))
    _map_axes(ax)
    handles = [Patch(facecolor=COLORS[i % len(COLORS)], alpha=0.45, label=c) for i, c in enumerate(cities)]
    handles.append(Patch(facecolor="white", edgecolor="#555555", hatch="////", label="Not used (see reason)"))
    ax.legend(handles=handles, loc="lower left", frameon=False, fontsize=9)
    ax.set_title(f"{int(table['used'].sum())} block groups used of {len(table)} in the chosen cities")
    return save(fig, path)


# --------------------------------------------------------------------------
# Steps 2 and 3: choosing k, and the clusters
# --------------------------------------------------------------------------

def k_scores(scores, path):
    fig, (left, right) = plt.subplots(1, 2, figsize=(10, 4))
    left.plot(scores["k"], scores["inertia"], marker="o")
    left.set_title("Elbow: inertia (look for the bend)")
    right.plot(scores["k"], scores["silhouette"], marker="o", color=COLORS[0])
    right.set_title("Silhouette (higher is better)")
    for ax in (left, right):
        ax.set_xlabel("k (number of clusters)")
        ax.grid(alpha=0.25)
    return save(fig, path)


def draw_clusters(ax, df, X, scaler, columns, k):
    """One color per cluster, a star at each cluster's average, and a circle around its dots."""
    x, y = axes_for(columns)
    for cluster, group in df.groupby("cluster"):
        c = color(cluster)
        mx = group[x].mean()
        my = group[y].mean() if y else 0.0
        if y and len(columns) == 2:
            # K-means works in the scaled space, where "close to the center" is a
            # plain circle. Drawn on the real axes it stretches into an oval: its
            # width and height are the circle's radius times each column's
            # spread. It holds CIRCLE_SHARE of the cluster's dots, so a few
            # strays don't blow it up.
            scaled = X[(df["cluster"] == cluster).to_numpy()]
            radius = np.quantile(np.linalg.norm(scaled - scaled.mean(axis=0), axis=1), CIRCLE_SHARE)
            spread = dict(zip(columns, scaler.scale_))
            ax.add_patch(Ellipse((mx, my), width=2 * radius * spread[x], height=2 * radius * spread[y],
                                 facecolor=c, alpha=0.12, edgecolor=c, linewidth=2, zorder=1))
        means = ", ".join(f"{LABELS.get(col, col).split(' (')[0]} {group[col].mean():,.0f}" for col in columns[:2])
        ax.scatter(group[x], group[y] if y else np.zeros(len(group)), s=34, alpha=0.7, color=c,
                   edgecolors="none", label=f"Cluster {cluster}: {len(group)} block groups ({means})")
        ax.scatter(mx, my, s=300, marker="*", color=c, edgecolors="black", linewidths=1, zorder=5)
    note = f"circles hold {CIRCLE_SHARE:.0%} of its dots" if y and len(columns) == 2 else \
        f"showing 2 of {len(columns)} columns" if len(columns) > 2 else "one column"
    style_axes(ax, x, y, f"{k} clusters\nstars = cluster average; {note}")
    ax.legend(frameon=False, loc="upper right", fontsize=7)


def clusters(df, X, scaler, columns, k, path):
    fig, ax = plt.subplots(figsize=(9, 6))
    draw_clusters(ax, df, X, scaler, columns, k)
    return save(fig, path)


def clusters_compare(frames, X, scaler, columns, path):
    """frames is [(k, df with its cluster column)]: the clusters for each k side by side."""
    fig, axes = plt.subplots(1, len(frames), figsize=(7 * len(frames), 6.5), squeeze=False)
    for ax, (k, df) in zip(axes[0], frames):
        draw_clusters(ax, df, X, scaler, columns, k)
    return save(fig, path)


def clusters_map(df, boundaries, path, title="Clusters"):
    fig, ax = plt.subplots(figsize=(9, 9))
    for _, row in df.iterrows():
        for ring in rings(boundaries[row["geoid"]]):
            ax.add_patch(MplPolygon(ring, closed=True, facecolor=color(row["cluster"]), alpha=0.6,
                                    edgecolor="white", linewidth=0.5))
    _map_axes(ax)
    handles = [Patch(facecolor=color(c), alpha=0.6, label=f"Cluster {c} ({n})")
               for c, n in df["cluster"].value_counts().sort_index().items()]
    ax.legend(handles=handles, loc="lower left", frameon=False, fontsize=9)
    ax.set_title(title)
    return save(fig, path)


# --------------------------------------------------------------------------
# Step 4: the sample
# --------------------------------------------------------------------------

def picks(df, columns, path):
    """Every block group on the cluster chart, with the picks outlined."""
    x, y = axes_for(columns)
    fig, ax = plt.subplots(figsize=(9, 6))
    for cluster, group in df.groupby("cluster"):
        c = color(cluster)
        ax.scatter(group[x], group[y] if y else np.zeros(len(group)), s=30, alpha=0.35, color=c, edgecolors="none")
        chosen = group[group["sampled"]]
        ax.scatter(chosen[x], chosen[y] if y else np.zeros(len(chosen)), s=90, color=c,
                   edgecolors="black", linewidths=1.2,
                   label=f"Cluster {cluster}: {len(chosen)} of {len(group)} picked")
    style_axes(ax, x, y, "Picked block groups (outlined) among all block groups")
    ax.legend(frameon=False, loc="upper right", fontsize=9)
    return save(fig, path)


def sample_map(df, boundaries, path):
    """The clusters on a map, with picks solid and numbered by fly order; hatched = not eligible (airspace)."""
    fig, ax = plt.subplots(figsize=(9, 9))
    for _, row in df.iterrows():
        c = color(row["cluster"])
        if row["sampled"]:
            style = dict(facecolor=c, alpha=1.0, edgecolor="black", linewidth=1.8)
        elif row["airspace"] == "restricted":
            style = dict(facecolor="white", alpha=1.0, edgecolor="#d1495b", linewidth=0.6, hatch="////")
        else:
            style = dict(facecolor=c, alpha=0.3, edgecolor="white", linewidth=0.5)
        for ring in rings(boundaries[row["geoid"]]):
            ax.add_patch(MplPolygon(ring, closed=True, **style))
    for _, row in df[df["sampled"]].iterrows():
        centre = boundaries[row["geoid"]].centroid
        ax.text(centre.x, centre.y, str(int(row["fly_order"])), fontsize=9, fontweight="bold",
                ha="center", va="center", color="white", zorder=6,
                bbox=dict(boxstyle="circle,pad=0.3", facecolor="black", edgecolor="none"))
    _map_axes(ax)
    ax.set_title("Clusters and picks (number = fly order)")
    handles = [Patch(facecolor=color(s), label=f"Cluster {s}") for s in sorted(df["cluster"].unique())]
    handles.append(Patch(facecolor="white", edgecolor="#d1495b", hatch="////", label="Not eligible: restricted airspace"))
    ax.legend(handles=handles, loc="lower left", frameon=False, fontsize=9)
    return save(fig, path)


def street_map(picked, boundaries, street_features, path):
    """One panel per picked block group, in flying order, with every street to fly; restricted streets grey, dashed."""
    picked = picked.sort_values("fly_order")
    cols = min(3, len(picked))
    nrows = -(-len(picked) // cols)
    fig, axes = plt.subplots(nrows, cols, figsize=(5 * cols, 5 * nrows), squeeze=False)
    for ax in axes.flat:
        ax.axis("off")
    for ax, (_, row) in zip(axes.flat, picked.iterrows()):
        c = color(row["cluster"])
        ax.add_collection(LineCollection(rings(boundaries[row["geoid"]]), colors="black", linewidths=1.2))
        for restricted, style in ((False, "solid"), (True, "dashed")):
            lines = []
            for feature in street_features:
                if feature["properties"]["geoid"] != row["geoid"] or feature["properties"]["restricted"] != restricted:
                    continue
                geom = shape(feature["geometry"])
                lines += [np.array(part.coords) for part in getattr(geom, "geoms", [geom])
                          if part.geom_type == "LineString"]
            ax.add_collection(LineCollection(lines, colors="#8c8c8c" if restricted else c,
                                             linewidths=1.0, linestyles=style))
        minx, miny, maxx, maxy = boundaries[row["geoid"]].bounds
        pad = 0.05 * max(maxx - minx, maxy - miny)
        ax.set_xlim(minx - pad, maxx + pad)
        ax.set_ylim(miny - pad, maxy + pad)
        ax.set_aspect(1 / np.cos(np.radians(LATITUDE)))
        ax.set_title(f"{int(row['fly_order'])}. {row['city'].replace(' city', '')}, cluster {row['cluster']}, "
                     f"{row['street_miles']:.1f} mi", fontsize=11)
    fig.suptitle("Streets to fly, in flying order (grey dashed = restricted airspace)", fontsize=13)
    return save(fig, path)
