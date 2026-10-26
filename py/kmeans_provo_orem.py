"""
K-means on Provo-Orem block groups: house age and median income.

Helps decide how many clusters to use. It
  1. tries k = 2..8 and prints two scores for each (inertia and silhouette) and
     saves a chart of them. Look for the "elbow" where inertia stops falling fast,
     and for a high silhouette (clusters that are tight and well separated);
  2. for each k in SHOW_K (3, 4, 5), prints and saves what the clusters look
     like - size, average income and house age - with a chart of each, and one
     chart putting them side by side.

Run provo_orem_vs_utah_county.py first - it saves the table this reads.

    python py/kmeans_provo_orem.py

Writes to data/census/provo_orem_vs_county/:
    kmeans_choose_k.png           inertia and silhouette for k = 2..8
    kmeans_clusters_k<k>.png      the clusters for one k
    kmeans_clusters_compare.png   the clusters for every k in SHOW_K, side by side
    kmeans_cluster_summary.csv    the cluster sizes and averages for every k in SHOW_K
"""
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Ellipse
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score

from provo_orem_common import (COLORS, DATA_DIR, cluster_block_groups, load_provo_orem,
                               save_figure, style_income_age_axes)

SHOW_K = (3, 4, 5)                  # the cluster counts to compare
CIRCLE_SHARE = 0.90                 # share of a cluster's dots its circle must contain


def score_k_values(X, ks):
    """(inertia, silhouette) for each k: how tight, and how well separated, the clusters are."""
    inertias, silhouettes = [], []
    print(" k   inertia   silhouette")
    for k in ks:
        model = KMeans(n_clusters=k, n_init=10, random_state=0).fit(X)
        inertias.append(model.inertia_)
        silhouettes.append(silhouette_score(X, model.labels_))
        print(f"{k:>2}  {inertias[-1]:>8.1f}  {silhouettes[-1]:>9.3f}")
    return inertias, silhouettes


def plot_k_scores(ks, inertias, silhouettes, path):
    fig, (left, right) = plt.subplots(1, 2, figsize=(10, 4))
    left.plot(ks, inertias, marker="o")
    left.set_title("Elbow: inertia (look for the bend)")
    right.plot(ks, silhouettes, marker="o", color=COLORS[0])
    right.set_title("Silhouette (higher is better)")
    for ax in (left, right):
        ax.set_xlabel("k (number of clusters)")
        ax.grid(alpha=0.25)
    save_figure(fig, path)


def draw_clusters(ax, df, X, scaler, k):
    """One color per cluster, a star at each cluster's average, and a circle around its dots."""
    # K-means works in the scaled space, where "close to the center" means a plain
    # circle. Drawn back on the chart's real axes (years across, dollars up) that
    # circle is stretched into an oval, so the oval's width and height are the
    # circle's radius times each column's own spread. The circle holds CIRCLE_SHARE
    # of the cluster's dots; the rest sit just outside, so a few strays don't
    # blow it up.
    income_spread, age_spread = scaler.scale_       # same order as COLUMNS
    for cluster, group in df.groupby("cluster"):
        color = COLORS[cluster % len(COLORS)]
        scaled = X[(df["cluster"] == cluster).to_numpy()]
        radius = np.quantile(np.linalg.norm(scaled - scaled.mean(axis=0), axis=1), CIRCLE_SHARE)
        age, income = group["median_age_years_all"].mean(), group["median_household_income"].mean()
        ax.add_patch(Ellipse((age, income), width=2 * radius * age_spread, height=2 * radius * income_spread,
                             facecolor=color, alpha=0.12, edgecolor=color, linewidth=2, zorder=1))
        label = f"Cluster {cluster}: {len(group)} block groups, ${income / 1000:,.0f}K, {age:.0f} yr houses"
        ax.scatter(group["median_age_years_all"], group["median_household_income"],
                   s=34, alpha=0.7, color=color, edgecolors="none", label=label)
        ax.scatter(age, income, s=300, marker="*", color=color, edgecolors="black", linewidths=1, zorder=5)
    style_income_age_axes(ax, f"{k} clusters\nstars = cluster average; circles hold {CIRCLE_SHARE:.0%} of its dots")
    ax.legend(frameon=False, loc="upper right", fontsize=8)


def cluster_summary(df, k):
    """One row per cluster: how many block groups, and their average income and house age."""
    summary = df.groupby("cluster").agg(
        block_groups=("geoid", "size"),
        income=("median_household_income", "mean"),
        house_age_years=("median_age_years_all", "mean"),
    ).round(1).reset_index()
    summary.insert(0, "k", k)
    return summary


def main():
    df = load_provo_orem()

    ks = list(range(2, 9))
    _, X, scaler = cluster_block_groups(df)
    inertias, silhouettes = score_k_values(X, ks)
    plot_k_scores(ks, inertias, silhouettes, DATA_DIR / "kmeans_choose_k.png")

    summaries = []
    compare, compare_axes = plt.subplots(1, len(SHOW_K), figsize=(7 * len(SHOW_K), 6.5))
    for ax_compare, k in zip(np.atleast_1d(compare_axes), SHOW_K):
        df["cluster"], _, _ = cluster_block_groups(df, k)
        summaries.append(cluster_summary(df, k))
        draw_clusters(ax_compare, df, X, scaler, k)
        fig, ax = plt.subplots(figsize=(9, 6))
        draw_clusters(ax, df, X, scaler, k)
        save_figure(fig, DATA_DIR / f"kmeans_clusters_k{k}.png")
    save_figure(compare, DATA_DIR / "kmeans_clusters_compare.png")

    summary = pd.concat(summaries, ignore_index=True)
    summary.to_csv(DATA_DIR / "kmeans_cluster_summary.csv", index=False)
    print("\nClusters for each k (income = average median household income, house age in years):")
    print(summary.to_string(index=False))
    print(f"\nCharts and summary saved in {DATA_DIR}")


if __name__ == "__main__":
    main()
