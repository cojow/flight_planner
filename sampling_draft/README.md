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

The draw is seeded, so the same settings give the same block groups. From the repository root, with the Census API
key in `py/census_api_key.txt`:

```
python py/provo_orem_vs_utah_county.py
python py/kmeans_provo_orem.py
python py/sample_block_groups.py --k 3 --per-cluster 2
```

The results are written to `data/census/provo_orem_vs_county/`, which is not tracked by git. This folder is a copy of
that output, so it will not update by itself: rerun the scripts and copy the files here when the settings change.
