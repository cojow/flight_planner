"""
K-means on Provo-Orem block groups: house age and median income.

Two steps, both in one run:
  1. Try k = 2..8 and print two scores for each, to help pick k:
       - inertia: how spread out the groups are (always falls as k grows;
         look for the "elbow" where it stops falling fast)
       - silhouette: how well separated the groups are (higher is better)
     A chart of both is saved next to the data.
  2. Fit the k you chose (K in provo_orem_common.py), print what each cluster
     looks like, and save a scatter plot of the clusters.

Run provo_orem_vs_utah_county.py first - it saves the table this reads.

    python py/kmeans_provo_orem.py
"""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score

from provo_orem_common import (COLORS, COLUMNS, DATA_DIR, K, cluster_block_groups,
                               load_provo_orem, save_figure, style_income_age_axes)

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


def plot_clusters(df, X, scaler, path):
    """One color per cluster, a star at each cluster's average, and a circle around its dots."""
    # K-means works in the scaled space, where "close to the center" means a plain
    # circle. Drawn back on the chart's real axes (years across, dollars up) that
    # circle is stretched into an oval, so the oval's width and height are the
    # circle's radius times each column's own spread. The circle holds CIRCLE_SHARE
    # of the cluster's dots; the rest sit just outside, so a few strays don't
    # blow it up.
    income_spread, age_spread = scaler.scale_       # same order as COLUMNS
    fig, ax = plt.subplots(figsize=(9, 6))
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
    style_income_age_axes(ax, f"K-means clusters of Provo-Orem block groups (k = {K})\n"
                              f"stars mark each cluster's average; circles hold {CIRCLE_SHARE:.0%} of its dots")
    ax.legend(frameon=False, loc="upper right", fontsize=9)
    save_figure(fig, path)


def main():
    df = load_provo_orem()
    df["cluster"], X, scaler = cluster_block_groups(df)

    ks = list(range(2, 9))
    inertias, silhouettes = score_k_values(X, ks)
    chart = DATA_DIR / "kmeans_choose_k.png"
    plot_k_scores(ks, inertias, silhouettes, chart)
    print(f"\nChart saved to {chart}")

    print(f"\nClusters with k = {K}:")
    print(df.groupby("cluster")[COLUMNS].agg(["count", "mean"]).round(1))
    cluster_chart = DATA_DIR / f"kmeans_clusters_k{K}.png"
    plot_clusters(df, X, scaler, cluster_chart)
    print(f"\nCluster chart saved to {cluster_chart}")


if __name__ == "__main__":
    main()
