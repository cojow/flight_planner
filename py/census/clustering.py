"""
K-means on block groups.

Every column is put on the same scale first (mean 0, standard deviation 1).
K-means measures plain distance, so income in dollars (tens of thousands)
would otherwise drown out house age (tens of years) and the clusters would
just be income bands.
"""
import pandas as pd
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
from sklearn.preprocessing import StandardScaler


def usable(df, columns):
    """(rows that have every column, the GEOIDs of rows dropped for a blank)."""
    blank = df[list(columns)].isna().any(axis=1)
    return df[~blank].reset_index(drop=True), list(df.loc[blank, "geoid"])


def scale(df, columns):
    """(scaled features as an array, the fitted scaler)."""
    scaler = StandardScaler()
    return scaler.fit_transform(df[list(columns)]), scaler


def kmeans(X, k, seed=0, n_init=10):
    """The fitted model: .labels_ is each row's cluster, .inertia_ its spread."""
    return KMeans(n_clusters=k, n_init=n_init, random_state=seed).fit(X)


def score_k(X, ks, seed=0, n_init=10):
    """
    Inertia and silhouette for each k. Look for the "elbow" where inertia stops
    falling fast, and a high silhouette (tight, well separated clusters).
    Guidance for choosing k, not a verdict.
    """
    rows = []
    for k in ks:
        if k >= len(X):
            break
        model = kmeans(X, k, seed, n_init)
        rows.append({"k": k, "inertia": round(float(model.inertia_), 3),
                     "silhouette": round(float(silhouette_score(X, model.labels_)), 4)})
    return pd.DataFrame(rows, columns=["k", "inertia", "silhouette"])


def summary(df, columns, k=None):
    """One row per cluster: how many block groups, and the average of each column."""
    table = df.groupby("cluster").agg(block_groups=("geoid", "size"),
                                      **{c: (c, "mean") for c in columns}).round(1).reset_index()
    if k is not None:
        table.insert(0, "k", k)
    return table
