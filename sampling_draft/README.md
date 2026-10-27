# Sampling draft: Provo and Orem

**This is a draft.** It shows the block groups to fly if we use **3 clusters and 2 block groups per cluster**.
The number of clusters, and whether to fly one or two per cluster, is still to be decided.

The block groups are picked from k-means clusters of Provo and Orem (139 Census block groups, clustered on
median household income and the age of the houses). Only block groups that can be flown are eligible: ones mostly
inside FAA airspace with a ceiling under 200 ft are left out. The first three picks (one per cluster) are the
one-per-cluster option.

## Files

| File | What it holds |
|---|---|
| `sample_block_groups.csv` | The six picked block groups in flying order (`fly_order`), with cluster, airspace, weight, street miles and location |
| `sample_streets.csv` | Every street to fly in each block group: name, other names, road type, miles, and whether it is in restricted airspace |
| `sample_streets.geojson` | The street segments, for QGIS or a map layer |
| `sample_map.png` | The clusters on a map, with the picks numbered in flying order |
| `sample_streets_map.png` | The streets to fly in each pick (grey dashed = restricted airspace) |
| `sample_block_groups.png` | The picks among all block groups, by income and house age |
| `kmeans_clusters_compare.png` | The clusters for 3, 4 and 5 clusters side by side |
| `kmeans_cluster_summary.csv` | The size, average income and house age of each cluster for 3, 4 and 5 clusters |

In `sample_streets.csv`, `restricted = True` marks a street that is mostly in airspace with a ceiling under 200 ft:
skip it without FAA authorization.

## Regenerating

## This draft is now the old baseline

These files were made by the old scripts, which have been replaced by the Census Sampling app
(`streamlit run census_app.py`, or `python -m census` from `py/`; see the main README). They are kept as they are, as a
record. The app finds three problems in them:

- **Street miles are too high.** The same stretch of road was often counted two or three times under different names
  ("State Rte 114" and "N Geneva Rd"), so the 43.05 miles here are about 36.3 flyable miles.
- **Some restricted street was listed as flyable.** Streets were flagged restricted only when at least half of them
  was under a ceiling below 200 ft; about 0.25 more miles in pick 3 is also in 0 ft airspace.
- **The weights count block groups that could not be picked.** The `weight` column here counts the whole cluster,
  including the 21 block groups under restricted airspace. The app's default counts only the 118 that could be drawn.

The clusters, picks and fly order still match. To reproduce them, from `py/`:

```
python -m census new regress_draft --cities Provo Orem
python -m census set regress_draft sample.weights=legacy
python -m census run regress_draft
python -m census.check_draft regress_draft
```
