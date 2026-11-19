"""
Publishing a run so anyone can reproduce it, and reproducing one.

    census publish provo_2026 [--url URL]           # the author, once every step is current
    census fetch-inputs provo_2026 [--from ZIP]     # anyone: get the archived raw inputs
    census reproduce provo_2026 [--as NAME]         # rerun every step offline, then verify
    census verify provo_2026 [--run NAME]           # compare a run's results with the published ones

A published run is a folder census_runs/<name>/, committed to git:
  config.json     the run's settings
  manifest.json   what made it: the code version, the package versions, every raw
                  input file with its SHA-256, the inputs archive (file, SHA-256,
                  URL), the FAA map and national security area snapshots, and the
                  SHA-256 of every result file
  expected/       the result tables (CSV), to compare a reproduction against

The raw inputs - the ACS answers, the TIGER/Line files, and the FAA facility map
and national security areas the draw was made under - are too big for git, so `publish` packs exactly the
files the run read into data/census/archives/<name>_inputs.zip. Upload that
file somewhere public (Zenodo gives it a DOI; a GitHub release works too) and
put its URL in the manifest with `publish --url`.

A reproduction never goes online: it reads only those archived files, so the
FAA maps (which the FAA keeps updating) and the Census numbers are the ones the
published run used, and a missing file is an error rather than a fresh download.
"""
import json
import time
import zipfile
from pathlib import Path, PurePosixPath

import numpy as np
import pandas as pd
import requests

from . import config as cfg
from . import sources, steps
from .sources import DATA_DIR, sha256

PUBLISHED_DIR = cfg.REPO_ROOT / "census_runs"
ARCHIVE_DIR = DATA_DIR / "archives"
RESULT_SUFFIXES = (".csv", ".geojson")      # compared by verify; charts and reports are not
ZIP_DATE = (1980, 1, 1, 0, 0, 0)            # fixed, so the same inputs always make the same archive


def published_dir(name):
    cfg.run_dir(name)                       # the same naming rule as runs
    return PUBLISHED_DIR / name


def load_manifest(name):
    path = published_dir(name) / "manifest.json"
    if not path.exists():
        known = sorted(p.parent.name for p in PUBLISHED_DIR.glob("*/manifest.json"))
        raise cfg.ConfigError(f"No published run {name!r} in {PUBLISHED_DIR.name}/. "
                              f"Published: {', '.join(known) or 'none yet'}.")
    return json.loads(path.read_text())


def _input_path(rel):
    """A manifest path as a file under data/census/, refusing anything that would land outside it."""
    parts = PurePosixPath(rel)
    if parts.is_absolute() or ".." in parts.parts:
        raise cfg.ConfigError(f"Bad input path in the manifest: {rel!r}")
    return DATA_DIR.joinpath(*parts.parts)


def missing_inputs(manifest):
    """The manifest's input files that are absent here or have different contents."""
    return [rel for rel, info in manifest["inputs"].items()
            if not _input_path(rel).exists() or sha256(_input_path(rel)) != info["sha256"]]


# --------------------------------------------------------------------------
# Publish
# --------------------------------------------------------------------------

def publish(name, url=None, overwrite=False):
    """
    Write census_runs/<name>/ and data/census/archives/<name>_inputs.zip from
    the run of the same name. Every step must be current (run with the
    settings and code there now). Returns a short report.
    """
    folder = cfg.run_dir(name)
    states = steps.status(folder)
    behind = [f"{step} ({state})" for step, state in states.items() if state != "current"]
    if behind:
        raise steps.StepError("Every step must be run with the current settings and code before publishing; "
                              f"not current: {', '.join(behind)}.")
    out = published_dir(name)
    old = json.loads((out / "manifest.json").read_text()) if (out / "manifest.json").exists() else None
    if old and not overwrite:
        raise cfg.ConfigError(f"{name!r} is already published; use --overwrite to replace it.")

    records = {step: json.loads((steps.step_dir(folder, step) / "inputs.json").read_text()) for step in steps.ORDER}
    inputs = {}
    for record in records.values():
        if "inputs" not in record:
            raise steps.StepError("These steps ran before inputs were recorded; run them again, then publish.")
        inputs.update(record["inputs"])
    for rel, digest in inputs.items():
        path = _input_path(rel)
        if not path.exists() or sha256(path) != digest:
            raise steps.StepError(f"data/census/{rel} has changed or gone since the run read it; "
                                  "run the steps again, then publish.")
    snapshots = {}
    for folder_name, what in (("faa", "FAA map"), ("nsufr", "national security areas")):
        found = sorted(rel for rel in inputs if rel.startswith(f"{folder_name}/"))
        if len(found) != 1:
            raise steps.StepError(f"Expected one {what} snapshot among the inputs, found {len(found)}; "
                                  "run the sample step again.")
        snapshots[folder_name] = found[0]

    results = {}
    for step in steps.ORDER:
        for path in steps.outputs(folder, step).values():
            if path.suffix in RESULT_SUFFIXES:
                results[f"{path.parent.name}/{path.name}"] = sha256(path)

    archive = ARCHIVE_DIR / f"{name}_inputs.zip"
    _write_archive(archive, sorted(inputs))
    archive_sha = sha256(archive)
    if url is None and old and old["archive"]["sha256"] == archive_sha:
        url = old["archive"].get("url", "")     # same inputs as before: the uploaded copy still fits

    out.mkdir(parents=True, exist_ok=True)
    for stale in (out / "expected").rglob("*.csv"):     # only what publish wrote; anything else is kept
        stale.unlink()
    for rel in results:
        if rel.endswith(".csv"):
            dest = out / "expected" / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes((folder / rel).read_bytes())
    (out / "config.json").write_text((folder / "config.json").read_text())

    sample_sources = json.loads((steps.step_dir(folder, "sample") / "sources.json").read_text())
    manifest = {
        "run": name,
        "published": time.strftime("%Y-%m-%d %H:%M"),
        "code": cfg.code_version(),
        "versions": records[steps.ORDER[-1]]["versions"],
        "faa": {"snapshot": snapshots["faa"], "fetched": sample_sources.get("faa", {}).get("fetched")},
        "national_security": {"snapshot": snapshots["nsufr"],
                              "fetched": sample_sources.get("national_security", {}).get("fetched")},
        "archive": {"file": archive.name, "sha256": archive_sha, "bytes": archive.stat().st_size, "url": url or ""},
        "inputs": {rel: {"sha256": inputs[rel], "bytes": _input_path(rel).stat().st_size} for rel in sorted(inputs)},
        "results": results,
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    lines = [f"Published {out.relative_to(cfg.REPO_ROOT)}/ ({len(results)} result files, {len(inputs)} inputs).",
             f"Inputs archive: {archive.relative_to(cfg.REPO_ROOT)} ({archive.stat().st_size / 1e6:.1f} MB).",
             f"Commit census_runs/{name}/ (and pyproject.toml / uv.lock if they changed)."]
    if url:
        lines.append(f"Archive URL: {url}")
    else:
        lines.append(f"Next: upload {archive.name} (Zenodo or a GitHub release), then run "
                     f"`census publish {name} --overwrite --url <its download URL>`.")
    return "\n".join(lines)


def _write_archive(path, rels):
    """A zip of data/census/<rel> files, byte-for-byte the same every time for the same files."""
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_suffix(".part")
    with zipfile.ZipFile(partial, "w") as z:
        for rel in rels:
            info = zipfile.ZipInfo(rel, date_time=ZIP_DATE)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            z.writestr(info, _input_path(rel).read_bytes(), compresslevel=6)
    partial.replace(path)


# --------------------------------------------------------------------------
# Fetch the inputs
# --------------------------------------------------------------------------

def fetch_inputs(name, from_zip=None):
    """
    Put a published run's raw inputs into data/census/, from the archive at the
    manifest's URL or a zip you already have. Each file is checked against its
    SHA-256. Returns a short report.
    """
    manifest = load_manifest(name)
    missing = missing_inputs(manifest)
    if not missing:
        return f"All {len(manifest['inputs'])} inputs for {name!r} are already here."

    expected = manifest["archive"]
    if from_zip:
        archive = Path(from_zip)
        if not archive.exists():
            raise cfg.ConfigError(f"No file {archive}.")
    else:
        archive = ARCHIVE_DIR / expected["file"]
        if not (archive.exists() and sha256(archive) == expected["sha256"]):
            if not expected.get("url"):
                raise cfg.ConfigError(f"The manifest for {name!r} has no archive URL yet. Ask the author for "
                                      f"{expected['file']} and run `census fetch-inputs {name} --from <the zip>`.")
            _download(expected["url"], archive, expected.get("bytes"))
    if sha256(archive) != expected["sha256"]:
        raise cfg.ConfigError(f"{archive} is not the published archive for {name!r} (its SHA-256 differs).")

    with zipfile.ZipFile(archive) as z:
        names = set(z.namelist())
        for rel in missing:
            if rel not in names:
                raise cfg.ConfigError(f"The archive has no {rel}.")
            data = z.read(rel)
            dest = _input_path(rel)
            dest.parent.mkdir(parents=True, exist_ok=True)
            partial = dest.with_name(dest.name + ".part")
            partial.write_bytes(data)
            if sha256(partial) != manifest["inputs"][rel]["sha256"]:
                partial.unlink()
                raise cfg.ConfigError(f"{rel} in the archive doesn't match the manifest.")
            partial.replace(dest)
    return f"Put {len(missing)} input file(s) for {name!r} in data/census/ ({len(manifest['inputs'])} in all), all checked."


def _download(url, dest, size=None):
    dest.parent.mkdir(parents=True, exist_ok=True)
    partial = dest.with_suffix(".part")
    print(f"Downloading {url}" + (f" ({size / 1e6:.1f} MB)" if size else "") + " ...")
    try:
        with requests.get(url, stream=True, timeout=60) as response:
            response.raise_for_status()
            with open(partial, "wb") as f:
                for chunk in response.iter_content(1 << 20):
                    f.write(chunk)
    except requests.RequestException as exc:
        if partial.exists():
            partial.unlink()
        raise sources.CensusError(f"Could not download {url}: {exc}") from exc
    partial.replace(dest)


# --------------------------------------------------------------------------
# Reproduce and verify
# --------------------------------------------------------------------------

def reproduce(name, as_name=None):
    """
    Run every step of a published run offline from its archived inputs, into
    data/census/runs/<as_name or name>/, then verify. Returns (all match, report).
    """
    manifest = load_manifest(name)
    missing = missing_inputs(manifest)
    if missing:
        raise cfg.ConfigError(f"{len(missing)} of the published inputs aren't here; "
                              f"run `census fetch-inputs {name}` first.")

    published = cfg.load(published_dir(name))
    target = as_name or name
    folder = cfg.run_dir(target)
    if (folder / "config.json").exists():
        mine = cfg.load(folder)
        if any(mine[s] != published[s] for s in published if isinstance(published[s], dict)):
            raise cfg.ConfigError(f"A run called {target!r} already exists with other settings. "
                                  f"Reproduce into a new one: `census reproduce {name} --as {name}_check`.")
    published["name"] = target
    cfg.save(folder, published)

    if cfg.code_version() != manifest["code"]:
        print("Note: the census code here differs from the code that made the published run, "
              "so results may differ (git log py/census shows what changed).")
    from .steps import _versions
    differ = {k: (v, manifest["versions"].get(k)) for k, v in _versions().items()
              if manifest["versions"].get(k) not in (None, v)}
    if differ:
        print("Note: package versions differ from the published run's (here vs published): "
              + ", ".join(f"{k} {a} vs {b}" for k, (a, b) in differ.items())
              + ". Use `uv sync --frozen` for the locked versions.")

    with sources.offline(faa_file=_input_path(manifest["faa"]["snapshot"]),
                         no_fly_file=_input_path(manifest["national_security"]["snapshot"])):
        for step in steps.ORDER:
            print(f"Running {step} (offline) ...", flush=True)
            steps.RUNNERS[step](folder)
    return verify(name, target)


def verify(name, run=None):
    """
    Compare a run's result files with a published run's. Returns (all match, report).
    Identical bytes match; CSVs that differ in bytes match when every value is
    the same (numbers within a millionth, for float printing).
    """
    manifest = load_manifest(name)
    folder = cfg.run_dir(run or name)
    lines = []
    states = steps.status(folder)
    if any(state != "current" for state in states.values()):
        lines.append("Warning: not every step is current here: "
                     + ", ".join(f"{s} {st}" for s, st in states.items() if st != "current"))
    bad = 0
    for rel, digest in manifest["results"].items():
        path = folder / rel
        if not path.exists():
            verdict = "MISSING"
        elif sha256(path) == digest:
            verdict = "identical"
        elif path.suffix == ".csv":
            problem = compare_csv(path, published_dir(name) / "expected" / rel)
            verdict = "same values" if problem is None else f"DIFFERS: {problem}"
        else:
            verdict = "DIFFERS (bytes; compare the tables)"
        bad += verdict == "MISSING" or verdict.startswith("DIFFERS")
        lines.append(f"  {rel:<58} {verdict}")
    total = len(manifest["results"])
    lines.append(f"\nReproduced: all {total} result files match the published run {name!r}." if not bad
                 else f"\n{bad} of {total} result files don't match the published run {name!r}.")
    return not bad, "\n".join(lines)


def compare_csv(path, expected):
    """None when two CSVs hold the same values, else where they first differ."""
    if not expected.exists():
        return f"no published copy at {expected.relative_to(cfg.REPO_ROOT)}"
    a = pd.read_csv(path, dtype=str, keep_default_na=False)
    b = pd.read_csv(expected, dtype=str, keep_default_na=False)
    if list(a.columns) != list(b.columns):
        return f"columns differ: {sorted(set(a.columns) ^ set(b.columns)) or 'order'}"
    if len(a) != len(b):
        return f"{len(a)} rows, published {len(b)}"
    for col in a.columns:
        same = a[col] == b[col]
        if same.all():
            continue
        x, y = pd.to_numeric(a[col], errors="coerce"), pd.to_numeric(b[col], errors="coerce")
        numeric = (x.notna() | (a[col] == "")) & (y.notna() | (b[col] == ""))
        if numeric.all() and np.allclose(x, y, rtol=1e-6, atol=1e-9, equal_nan=True):
            continue
        row = int(np.flatnonzero(~(same | (numeric & np.isclose(x, y, rtol=1e-6, atol=1e-9, equal_nan=True))))[0])
        return f"column {col!r}, row {row + 1}: {a[col][row]!r}, published {b[col][row]!r}"
    return None
