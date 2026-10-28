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
purpose, so the street files are only summarised. The clusters, picks and
legacy weights should still match; if they don't, something else moved. The
FAA map is a live service, so a difference in the picks can also mean the
airspace changed since the draft.
"""
import json
import sys

import numpy as np
import pandas as pd

from . import config as cfg
from . import steps

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
    # Street counts and miles changed on purpose with the street fixes; the rest should not.
    same_cols = [c for c in draft.columns if c not in ("streets", "street_miles")]
    files = steps.outputs(run, "sample")
    picks = _csv(files["Picked block groups (table)"])
    results["picks, fly order and weights"] = _same(draft[same_cols], picks[same_cols])
    draft = pd.read_csv(DRAFT / "kmeans_cluster_summary.csv")
    preview = pd.read_csv(run / "2_choose_k" / "preview_summary.csv")
    results["k = 3, 4, 5 clusters"] = np.allclose(draft.values, preview.values)
    for label, ok in results.items():
        print(f"  {'same' if ok else 'DIFFERENT':<9} {label}")
    old = _csv(DRAFT / "sample_streets.csv")
    new = _csv(files["Streets (table)"])
    flyable = new[~new["restricted"]] if "restricted" in new else new
    print(f"  (streets: draft {old['miles'].sum():.2f} mi in {len(old)} rows; "
          f"this run {flyable['miles'].sum():.2f} flyable mi in {len(flyable)} rows)")
    return all(results.values())


if __name__ == "__main__":
    sys.exit(0 if check(sys.argv[1] if len(sys.argv) > 1 else "regress_draft") else 1)
