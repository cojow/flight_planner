"""
Settings and helpers shared by the Provo-Orem scripts.

kmeans_provo_orem.py and sample_block_groups.py both start from the table that
provo_orem_vs_utah_county.py saves and cluster it the same way. Keeping that here
means they cannot drift apart: the clusters in the k-means charts are exactly
the strata the sample is drawn from.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")               # draw to files; no window to close
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.ticker import FuncFormatter
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler

DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "census" / "provo_orem_vs_county"
TABLE = DATA_DIR / "utah_county_block_groups.csv"

# The two numbers everything is compared and clustered on.
COLUMNS = ["median_household_income", "median_age_years_all"]
K = 3                               # clusters (strata) for Provo-Orem
COLORS = ["#d1495b", "#2e7d9a", "#edae49", "#66a182", "#8d6a9f", "#8c8c8c"]
LATITUDE = 40.25                    # Provo-Orem; sets the map aspect so distances look right


def load_provo_orem():
    """The Provo and Orem block groups from the table provo_orem_vs_utah_county.py saves."""
    if not TABLE.exists():
        raise SystemExit(f"{TABLE} not found. Run py/provo_orem_vs_utah_county.py first.")
    df = pd.read_csv(TABLE, dtype={"geoid": str})
    return df[df["in_provo_orem"]].dropna(subset=COLUMNS).reset_index(drop=True)


def cluster_block_groups(df, k=K):
    """
    K-means on income and house age. Returns (cluster of each row, scaled
    features, the scaler).

    Both columns are put on the same scale first. K-means measures plain
    distance, so income in dollars (tens of thousands) would otherwise drown
    out house age (tens of years) and the clusters would just be income bands.
    """
    scaler = StandardScaler()
    X = scaler.fit_transform(df[COLUMNS])
    labels = KMeans(n_clusters=k, n_init=10, random_state=0).fit_predict(X)
    return labels, X, scaler


def style_income_age_axes(ax, title):
    """Labels, dollar ticks and a light grid for a chart of income against house age."""
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"${v / 1000:,.0f}K"))
    ax.set_xlabel("Median house age (years)")
    ax.set_ylabel("Median household income")
    ax.set_title(title)
    ax.grid(alpha=0.25)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def save_figure(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
