"""
The settings for one run, saved as config.json in the run's folder.

A run is one go through the four steps - areas, choose k, cluster, sample - for
one set of cities. Every input any step uses lives here, so a run can be
checked, changed and repeated, and the GUI and the command line read the same
file. Each step records a fingerprint of the settings it used (see
`fingerprint`), so a later step can tell when an earlier one is out of date.
"""
import copy
import hashlib
import json
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNS_DIR = REPO_ROOT / "data" / "census" / "runs"

# The steps in order, the folder each writes to, and the config sections each
# one depends on (its own and everything upstream of it).
STEPS = {
    "areas":    {"folder": "1_areas",    "sections": ["areas"]},
    "choose_k": {"folder": "2_choose_k", "sections": ["areas", "features", "choose_k"]},
    "cluster":  {"folder": "3_cluster",  "sections": ["areas", "features", "cluster"]},
    "sample":   {"folder": "4_sample",   "sections": ["areas", "features", "cluster", "sample"]},
}

DEFAULTS = {
    "areas": {
        "state": "49",                  # Utah
        "county": "049",                # Utah County
        "year": 2024,                   # ACS 5-year release (2020-2024)
        "places": [],                   # [{"geoid": "4962470", "name": "Provo"}, ...]
        "min_overlap": 0.5,             # share of a block group's area inside a city for it to count
        "exclude": [],                  # block group GEOIDs left out by hand
    },
    "features": {
        # The columns k-means groups block groups on. Each is rescaled to
        # mean 0 / sd 1 first, so dollars don't drown out years.
        "columns": ["median_household_income", "median_age_years_all"],
        "seed": 0,                      # k-means starting points
        "n_init": 10,                   # k-means restarts; the best is kept
    },
    "choose_k": {
        "k_min": 2,
        "k_max": 8,
        "preview": [3, 4, 5],           # cluster charts drawn for these k
    },
    "cluster": {
        "k": 3,
    },
    "sample": {
        "per_cluster": 2,               # block groups picked from each cluster
        "seed": 2026,                   # the draw
        "weights": "legacy",            # "legacy": cluster size / picks
        "high_flight_ft": 200,          # the high (nadir) flight height
        "restricted_share": 0.25,       # not eligible when this much is under a ceiling below high_flight_ft
        "buffer_m": 15,                 # streets this close outside a block group's edge count as in it
        "min_segment_m": 10,            # clipped street slivers shorter than this are dropped
        "street_types": ["S1200", "S1400"],   # Census road codes: secondary roads, local streets
        "start_lat": 40.2338,           # where each trip starts (BYU)
        "start_lon": -111.6585,
    },
}


# The ways a pick's weight can be worked out (sample.weights).
WEIGHTS = ("legacy",)


class ConfigError(ValueError):
    """A setting is missing or out of range; the message says which and why."""


def default_config(name=""):
    config = copy.deepcopy(DEFAULTS)
    config["name"] = name
    return config


def run_dir(name):
    """The folder for a run name. Names are kept to letters, digits, - and _."""
    if not name or not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise ConfigError("A run name may use only letters, digits, - and _.")
    return RUNS_DIR / name


def list_runs():
    return sorted(p.name for p in RUNS_DIR.glob("*") if (p / "config.json").exists())


def load(folder):
    """The config in a run folder, with any setting it lacks filled from DEFAULTS."""
    path = Path(folder) / "config.json"
    if not path.exists():
        raise ConfigError(f"No config.json in {folder}.")
    saved = json.loads(path.read_text())
    config = default_config(saved.get("name", Path(folder).name))
    for section, values in saved.items():
        if isinstance(values, dict) and section in config:
            config[section].update(values)
        else:
            config[section] = values
    return config


def save(folder, config):
    validate(config)
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "config.json").write_text(json.dumps(config, indent=2) + "\n")


def validate(config):
    """Raise ConfigError for the first setting that can't work."""
    a, f, c, k, s = (config[x] for x in ("areas", "features", "cluster", "choose_k", "sample"))
    if not a["places"]:
        raise ConfigError("Pick at least one city.")
    if not 0 < a["min_overlap"] <= 1:
        raise ConfigError("min_overlap must be above 0 and at most 1.")
    if not f["columns"]:
        raise ConfigError("Pick at least one column to cluster on.")
    if not 2 <= k["k_min"] <= k["k_max"]:
        raise ConfigError("The k range must start at 2 or more and not run backwards.")
    if c["k"] < 2:
        raise ConfigError("k must be at least 2.")
    if s["per_cluster"] < 1:
        raise ConfigError("Pick at least one block group per cluster.")
    if s["weights"] not in WEIGHTS:
        raise ConfigError(f"Unknown weights setting {s['weights']!r}.")
    if not 0 < s["restricted_share"] <= 1:
        raise ConfigError("restricted_share must be above 0 and at most 1.")


def fingerprint(config, step):
    """A short hash of the settings `step` depends on. Equal hashes, equal inputs."""
    used = {section: config[section] for section in STEPS[step]["sections"]}
    return hashlib.sha256(json.dumps(used, sort_keys=True).encode()).hexdigest()[:16]
