"""
The four steps of a run, as plain functions. The GUI (census_app.py) and the
command line (python -m census) both call these and nothing else, so a result
never depends on which one ran it.

Every step:
  - reads the run's config.json (see config.py) from the run folder,
  - refuses to run if a step it depends on hasn't run, or ran with different
    settings (StepError says which),
  - writes its files to its own subfolder, plus inputs.json recording the
    settings fingerprint, the time and the package versions,
  - returns a StepResult: the files it wrote and a plain-text report.

    folder = config.run_dir("provo_orem")
    config.save(folder, cfg)
    run_areas(folder); run_choose_k(folder); run_cluster(folder); run_sample(folder)
"""
import json
import platform
import shutil
import time
from dataclasses import dataclass, field
from importlib import metadata
from pathlib import Path

from . import config as cfg

ORDER = list(cfg.STEPS)
UPSTREAM = {"areas": [], "choose_k": ["areas"], "cluster": ["areas"], "sample": ["areas", "cluster"]}


class StepError(RuntimeError):
    """A step can't run: an earlier step is missing or out of date, or the data won't allow it."""


@dataclass
class StepResult:
    step: str
    folder: Path
    files: dict = field(default_factory=dict)     # label -> Path, in display order
    report: str = ""                              # what the old scripts printed


def step_dir(folder, step):
    return Path(folder) / cfg.STEPS[step]["folder"]


def status(folder):
    """
    {step: state} for every step, where state is
      "not run"   no output yet
      "current"   ran with the settings now in config.json
      "stale"     ran, but a setting it depends on has changed since
    """
    config = cfg.load(folder)
    out = {}
    for step in ORDER:
        inputs = step_dir(folder, step) / "inputs.json"
        if not inputs.exists():
            out[step] = "not run"
        else:
            saved = json.loads(inputs.read_text()).get("fingerprint")
            out[step] = "current" if saved == cfg.fingerprint(config, step) else "stale"
    return out


def outputs(folder, step):
    """The files a finished step wrote, label -> Path (empty if it hasn't run)."""
    index = step_dir(folder, step) / "inputs.json"
    if not index.exists():
        return {}
    return {label: step_dir(folder, step) / name
            for label, name in json.loads(index.read_text()).get("files", {}).items()}


def _check_upstream(folder, step):
    states = status(folder)
    for before in UPSTREAM[step]:
        if states[before] != "current":
            raise StepError(f"Run '{before}' first: it is {states[before]}.")


def _versions():
    names = ["pandas", "numpy", "scikit-learn", "shapely", "matplotlib", "requests"]
    found = {"python": platform.python_version()}
    for name in names:
        try:
            found[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            pass
    return found


def _finish(folder, step, config, files, report, extra=None):
    """Write inputs.json for a step and build its StepResult."""
    out = step_dir(folder, step)
    record = {
        "step": step,
        "fingerprint": cfg.fingerprint(config, step),
        "settings": {s: config[s] for s in cfg.STEPS[step]["sections"]},
        "finished": time.strftime("%Y-%m-%d %H:%M:%S"),
        "versions": _versions(),
        "files": {label: Path(p).name for label, p in files.items()},
    }
    record.update(extra or {})
    (out / "inputs.json").write_text(json.dumps(record, indent=2) + "\n")
    (out / "report.txt").write_text(report)
    return StepResult(step, out, {label: Path(p) for label, p in files.items()}, report)


def _start(folder, step):
    config = cfg.load(folder)
    cfg.validate(config)
    _check_upstream(folder, step)
    out = step_dir(folder, step)
    out.mkdir(parents=True, exist_ok=True)
    for old in out.iterdir():           # a rerun replaces the step's files
        if old.is_dir():
            shutil.rmtree(old)
        else:
            old.unlink()
    # Anything downstream that used this step's files is now invalid: clear it,
    # so it shows as "not run" and its old files can't be mistaken for current.
    for later in ORDER[ORDER.index(step) + 1:]:
        if step in UPSTREAM[later]:
            for old in step_dir(folder, later).glob("*"):
                if old.is_file():
                    old.unlink()
    return config, out


# --------------------------------------------------------------------------
# The steps
# --------------------------------------------------------------------------

def _run(folder, step, work):
    """Start a step, do its work, record it. Data problems come back as StepError."""
    from .sources import CensusError
    config, out = _start(folder, step)
    try:
        files, report = work(config, out)
    except (CensusError, ValueError) as exc:
        raise StepError(str(exc)) from exc
    return _finish(folder, step, config, files, report)


def run_areas(folder):
    """
    1. Areas. Every block group in the chosen cities, with its Census numbers.

    Files: block_groups.csv (one row per block group in the cities; `used` is
    False with a `reason` for ones left out: no ACS estimate, no housing, or
    excluded by hand), block_groups.geojson, county_block_groups.csv (every
    block group in the county, flagged in_cities), map.png.
    """
    from . import pipeline
    return _run(folder, "areas", pipeline.areas)


def run_choose_k(folder):
    """
    2. Choose k. Inertia and silhouette for each k in the range, and cluster
    charts for the preview k values, to help pick k. Nothing is decided here.

    Files: k_scores.csv, k_scores.png, clusters_k<k>.png for each preview k,
    clusters_compare.png, preview_summary.csv.
    """
    from . import pipeline
    return _run(folder, "choose_k", lambda config, out: pipeline.choose_k(config, folder, out))


def run_cluster(folder):
    """
    3. Cluster. K-means with the chosen k on the used block groups.

    Files: clusters.csv (one row per used block group, with its cluster),
    cluster_summary.csv, clusters.png, clusters_map.png.
    """
    from . import pipeline
    return _run(folder, "cluster", lambda config, out: pipeline.cluster(config, folder, out))


def run_sample(folder):
    """
    4. Sample. Airspace check, the draw from each cluster, fly order and the
    streets to fly. Reads the clusters saved by step 3.

    Files, each named with the draw's seed (e.g. sample_block_groups_seed2026.csv):
    sample_block_groups, sample_streets (.csv and .geojson), sample_block_groups.png,
    sample_map.png, sample_streets_map.png. Also faa_facility_map.geojson (the
    airspace the draw used) and sources.json.
    """
    from . import pipeline
    return _run(folder, "sample", lambda config, out: pipeline.sample(config, folder, out))


RUNNERS = {"areas": run_areas, "choose_k": run_choose_k, "cluster": run_cluster, "sample": run_sample}
