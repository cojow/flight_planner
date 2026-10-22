"""
K-means on Provo-Orem block groups: house age and median income.

Two steps, both in one run:
  1. Try k = 2..8 and print two scores for each, to help pick k:
       - inertia: how spread out the groups are (always falls as k grows;
         look for the "elbow" where it stops falling fast)
       - silhouette: how well separated the groups are (higher is better)
     A chart of both is saved next to the data.
  2. Fit the k you chose (K below), print what each cluster looks like, and
     save a scatter plot of the clusters.

Run provo_orem_vs_utah_county.py first - it saves the table this reads.

    python py/kmeans_provo_orem.py
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")               # draw to a file; no window to close
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
from sklearn.preprocessing import StandardScaler

K = 3                               # the number of clusters to fit; change it to try others
COLUMNS = ["median_household_income", "median_age_years_all"]

DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "census" / "provo_orem_vs_county"
TABLE = DATA_DIR / "utah_county_block_groups.csv"

# 1. Load the saved table, keep Provo-Orem, and drop areas missing either column.
if not TABLE.exists():
    raise SystemExit(f"{TABLE} not found. Run py/provo_orem_vs_utah_county.py first.")
df = pd.read_csv(TABLE)
df = df[df["in_provo_orem"]].dropna(subset=COLUMNS).copy()

# 2. Put both columns on the same scale. Without this, income (tens of thousands)
#    would drown out house age (tens), and the clusters would just be income bands.
scaler = StandardScaler()
X = scaler.fit_transform(df[COLUMNS])

# 3. Try several values of k and score each one.
ks = list(range(2, 9))
inertias, silhouettes = [], []
print(" k   inertia   silhouette")
for k in ks:
    model = KMeans(n_clusters=k, n_init=10, random_state=0).fit(X)
    inertias.append(model.inertia_)
    silhouettes.append(silhouette_score(X, model.labels_))
    print(f"{k:>2}  {inertias[-1]:>8.1f}  {silhouettes[-1]:>9.3f}")

fig, (left, right) = plt.subplots(1, 2, figsize=(10, 4))
left.plot(ks, inertias, marker="o")
left.set_title("Elbow: inertia (look for the bend)")
right.plot(ks, silhouettes, marker="o", color="#d1495b")
right.set_title("Silhouette (higher is better)")
for ax in (left, right):
    ax.set_xlabel("k (number of clusters)")
    ax.grid(alpha=0.25)
fig.tight_layout()
chart = DATA_DIR / "kmeans_choose_k.png"
fig.savefig(chart, dpi=200)
plt.close(fig)
print(f"\nChart saved to {chart}")

# 4. Fit with the chosen k and look at what each cluster is like.
df["cluster"] = KMeans(n_clusters=K, n_init=10, random_state=0).fit_predict(X)
print(f"\nClusters with k = {K}:")
print(df.groupby("cluster")[COLUMNS].agg(["count", "mean"]).round(1))

# 5. Chart the clusters: one color per cluster, a star at each cluster's average.
from matplotlib.ticker import FuncFormatter

from matplotlib.patches import Ellipse

# The circle around each cluster. K-means works in the scaled space from step 2,
# where "close to the center" means a plain circle. Drawn back on the chart's real
# axes (years across, dollars up) that circle is stretched into an oval, so the
# oval's width and height are the circle's radius times each column's own spread.
# CIRCLE_SHARE is the share of a cluster's points the circle must contain; the
# rest sit just outside, so a few stray block groups don't blow the circle up.
CIRCLE_SHARE = 0.90
income_spread, age_spread = scaler.scale_       # same order as COLUMNS

colors = ["#d1495b", "#2e7d9a", "#edae49", "#66a182", "#8d6a9f", "#8c8c8c"]
fig, ax = plt.subplots(figsize=(9, 6))
for cluster, group in df.groupby("cluster"):
    color = colors[cluster % len(colors)]
    scaled = X[(df["cluster"] == cluster).to_numpy()]
    radius = np.quantile(np.linalg.norm(scaled - scaled.mean(axis=0), axis=1), CIRCLE_SHARE)
    ax.add_patch(Ellipse(
        (group["median_age_years_all"].mean(), group["median_household_income"].mean()),
        width=2 * radius * age_spread, height=2 * radius * income_spread,
        facecolor=color, alpha=0.12, edgecolor=color, linewidth=2, zorder=1))
    label = (f"Cluster {cluster}: {len(group)} block groups, "
             f"${group['median_household_income'].mean() / 1000:,.0f}K, "
             f"{group['median_age_years_all'].mean():.0f} yr houses")
    ax.scatter(group["median_age_years_all"], group["median_household_income"],
               s=34, alpha=0.7, color=color, edgecolors="none", label=label)
    ax.scatter(group["median_age_years_all"].mean(), group["median_household_income"].mean(),
               s=300, marker="*", color=color, edgecolors="black", linewidths=1, zorder=5)

ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"${v / 1000:,.0f}K"))
ax.set_xlabel("Median house age (years)")
ax.set_ylabel("Median household income")
ax.set_title(f"K-means clusters of Provo-Orem block groups (k = {K})\n"
             f"stars mark each cluster's average; circles hold {CIRCLE_SHARE:.0%} of its dots")
ax.legend(frameon=False, loc="upper right", fontsize=9)
ax.grid(alpha=0.25)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
fig.tight_layout()
cluster_chart = DATA_DIR / f"kmeans_clusters_k{K}.png"
fig.savefig(cluster_chart, dpi=200)
plt.close(fig)
print(f"\nCluster chart saved to {cluster_chart}")
