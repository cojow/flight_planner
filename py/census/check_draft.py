"""
Does a run reproduce sampling_draft/? The draft was made by the old scripts
(3 clusters, 2 per cluster, Provo + Orem), and the package matched it exactly
when it replaced them (commit 7a39345).

    python -m census new regress_draft --cities Provo Orem
    python -m census set regress_draft sample.weights=legacy
    python -m census run regress_draft
    python -m census.check_draft regress_draft

Since then the street fixes (one count per stretch of road, cuts at restricted
airspace) and the 2024 TIGER/Line boundaries have changed the street files on
purpose, so "streets" and "street shapes" now differ. The clusters, picks and
legacy weights should still match; if they don't, something else moved. The
FAA map is a live service, so a difference in the picks can also mean the
airspace changed since the draft.
"""
import json
import sys

import numpy as np
import pandas as pd

from . import config as cfg

DRAFT = cfg.REPO_ROOT / "sampling_draft"


def _csv(path):
    return pd.read_csv(path, dtype={"geoid": str}).rename(columns={"stratum": "cluster"})


def _same(a, b):
    """Equal values, ignoring int/float storage (587 vs 587.0)."""
    if list(a.columns) != list(b.columns) or len(a) != len(b):
        return False
    for col in a.columns:
        x, y = a[col], b[col]
        if pd.api.types.is_numeric_dtype(x) and pd.api.types.is_numeric_dtype(y):
            if not np.allclose(x.astype(float), y.astype(float), equal_nan=True, rtol=0, atol=1e-9):
                return False
        elif not x.astype(str).equals(y.astype(str)):
            return False
    return True


def check(name):
    run = cfg.run_dir(name)
    results = {}
    draft = _csv(DRAFT / "sample_block_groups.csv")
    results["picked block groups"] = _same(draft, _csv(run / "4_sample" / "sample_block_groups.csv")[draft.columns])
    draft = _csv(DRAFT / "sample_streets.csv")
    results["streets"] = _same(draft, _csv(run / "4_sample" / "sample_streets.csv")[draft.columns])
    results["street shapes"] = (json.loads((DRAFT / "sample_streets.geojson").read_text())
                                == json.loads((run / "4_sample" / "sample_streets.geojson").read_text()))
    draft = pd.read_csv(DRAFT / "kmeans_cluster_summary.csv")
    preview = pd.read_csv(run / "2_choose_k" / "preview_summary.csv")
    results["k = 3, 4, 5 clusters"] = np.allclose(draft.values, preview.values)
    for label, ok in results.items():
        print(f"  {'same' if ok else 'DIFFERENT':<9} {label}")
    return all(results.values())


if __name__ == "__main__":
    sys.exit(0 if check(sys.argv[1] if len(sys.argv) > 1 else "regress_draft") else 1)
