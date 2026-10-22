"""
K-means on Provo-Orem block groups: house age and median income.

Two steps, both in one run:
  1. Try k = 2..8 and print two scores for each, to help pick k:
       - inertia: how spread out the groups are (always falls as k grows;
         look for the "elbow" where it stops falling fast)
       - silhouette: how well separated the groups are (higher is better)
     A chart of both is saved next to the data.
  2. Fit the k you chose (K below) and print what each cluster looks like.

Run provo_orem_vs_utah_county.py first - it saves the table this reads.

    python py/kmeans_provo_orem.py
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")               # draw to a file; no window to close
import matplotlib.pyplot as plt
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
X = StandardScaler().fit_transform(df[COLUMNS])

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
