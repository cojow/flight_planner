"""
Hand a finished sample to the flight planner (app.py).

A sample is saved under a name as census_samples/<name>.geojson at the top of
the repository: one feature per picked block group, with its GEOID, city,
cluster, fly order and street miles. app.py lists that folder and draws the
chosen sample on its maps. The folder is kept in git (unlike data/), so a
committed sample reaches everyone who plans flights.

    python -m census export provo_orem provo_orem_fall_2026
"""
import json
import re
import time

import pandas as pd

from . import config as cfg
from . import steps

SAMPLES_DIR = cfg.REPO_ROOT / "census_samples"

# Columns of sample_block_groups.csv carried into each feature, in order.
FIELDS = ["geoid", "fly_order", "pick", "city", "cluster", "airspace", "faa_ceiling_ft",
          "street_miles", "streets", "weight", "lat", "lon"]


def sample_path(name):
    if not name or not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise cfg.ConfigError("A sample name may use only letters, digits, - and _.")
    return SAMPLES_DIR / f"{name}.geojson"


def list_samples():
    return sorted(p.stem for p in SAMPLES_DIR.glob("*.geojson"))


def export_sample(folder, name, overwrite=False):
    """
    Save a run's picked block groups as census_samples/<name>.geojson and
    return the path. The run's sample step must be current, so the file never
    mixes picks with settings they weren't drawn under.
    """
    path = sample_path(name)
    if path.exists() and not overwrite:
        raise cfg.ConfigError(f"A sample called {name!r} already exists.")
    if steps.status(folder)["sample"] != "current":
        raise steps.StepError("Run step 4 (sample) with the current settings first.")
    config = cfg.load(folder)

    sample_dir = steps.step_dir(folder, "sample")
    picked = pd.read_csv(sample_dir / "sample_block_groups.csv", dtype={"geoid": str}).sort_values("fly_order")
    areas = pd.read_csv(steps.step_dir(folder, "areas") / "block_groups.csv", dtype={"geoid": str})
    names = dict(zip(areas["geoid"], areas["census_name"]))
    shapes = {f["properties"]["geoid"]: f["geometry"] for f in json.loads(
        (steps.step_dir(folder, "areas") / "block_groups.geojson").read_text())["features"]}

    features = []
    for _, row in picked.iterrows():
        props = {c: (None if pd.isna(row[c]) else row[c].item() if hasattr(row[c], "item") else row[c])
                 for c in FIELDS if c in row}
        props["census_name"] = names.get(row["geoid"])
        features.append({"type": "Feature", "geometry": shapes[row["geoid"]], "properties": props})

    s, a = config["sample"], config["areas"]
    collection = {
        "type": "FeatureCollection",
        # What app.py shows about the sample as a whole.
        "sample": {
            "name": name,
            "run": config["name"],
            "saved": time.strftime("%Y-%m-%d %H:%M"),
            "cities": [p["name"] for p in a["places"]],
            "acs_year": a["year"],
            "k": config["cluster"]["k"],
            "per_cluster": s["per_cluster"],
            "seed": s["seed"],
            "weights": s["weights"],
        },
        "features": features,
    }
    SAMPLES_DIR.mkdir(exist_ok=True)
    path.write_text(json.dumps(collection) + "\n")
    return path
