# Census sampling

This package picks the Census block groups, and the streets in them, that the drones fly for the research
[methodology](https://cojow.github.io/flight_planner/BYU_research_documentation/methodology.html). It takes the
cities you choose in Utah County and clusters their block groups with k-means on ACS numbers (median household income
and house age by default). It then draws a stratified sample from each cluster, checks the FAA airspace, and lists the
streets to fly, split into groups.

You can use it two ways, and both run the same code:

- **The app:** `uv run streamlit run census_app.py` (in the repository root)
- **The command line:** `uv run census ...`

There are two jobs here:

1. [Reproducing a published run](#reproduce-a-published-run): for anyone checking the results.
2. [Making and publishing a run](#make-and-publish-a-run): for whoever draws the sample.

## Setup (once)

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), a Python package manager. It installs the
exact Python packages this project was run with, and the right Python too.

macOS or Linux:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
Windows (PowerShell):
```bash
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

Then get the code and its environment:
```bash
git clone https://github.com/cojow/flight_planner.git
```
```bash
cd flight_planner
```
```bash
uv sync --frozen
```

`uv sync --frozen` installs the versions pinned in `uv.lock` into `.venv/`. `--frozen` means "use the lock as it
is": it never upgrades anything.

## Reproduce a published run

Published runs are kept in `census_runs/<name>/`. To list them, run `ls census_runs`. For a run called
`provo_2026`, these are the commands:

```bash
uv run census fetch-inputs provo_2026
```
```bash
uv run census reproduce provo_2026
```

1. **`fetch-inputs`** downloads the run's archived raw inputs and checks every file against the SHA-256 recorded
   in the manifest. Those inputs are the Census ACS answers, the TIGER/Line boundary and road files, and the FAA
   airspace map, and they go into `data/census/`. If you were given the zip file directly, run this instead:
   `uv run census fetch-inputs provo_2026 --from path/to/provo_2026_inputs.zip`.
2. **`reproduce`** runs every step the run published **offline** from those files, then compares every result table with the
   published one. The output ends with one of these lines:

   ```
   Reproduced: all 14 result files match the published run 'provo_2026'.
   ```
   ```
   1 of 14 result files don't match the published run 'provo_2026'.
   ```

   If a file doesn't match, the output names it and shows the first value that differs.

Your outputs are in `data/census/runs/provo_2026/`, one folder per step. You can also open the run in the app to see
its reports, charts and maps.

**You don't need a Census API key to reproduce a run.** The ACS answers are in the archive.

To check a run again later without re-running it:
```bash
uv run census verify provo_2026
```

If you already have a run with that name and different settings, reproduce it under a new name with
`uv run census reproduce provo_2026 --as provo_2026_check`, then check it with
`uv run census verify provo_2026 --run provo_2026_check`.

### Why the inputs are archived

Four of the inputs come from online services, and reproducing a run means using the same copies the run used:

- **FAA UAS Facility Map:** live, and the FAA keeps updating it. A different airspace map can change which block
  groups are eligible, and with that the picks. A reproduction reads the snapshot the original run saved.
- **National Security UAS Flight Restrictions:** live too, from the same FAA server, and saved and read back the
  same way.
- **ACS 5-year estimates:** fetched from the Census API, which needs a key.
- **TIGER/Line files:** fixed for a given year, but large, and census.gov is sometimes slow.

In offline mode a missing input is an error. It is never quietly downloaded fresh.

### What "match" means

`verify` checks the result files: every CSV and GeoJSON a step writes. It doesn't check the charts (PNG), the
reports, or the `sources.json` and `inputs.json` records, because those hold dates and timestamps.

- A file whose bytes are the same as the published one is reported **identical**.
- A CSV whose bytes differ but whose values are all the same is reported **same values**. Numbers count as the same
  when they are within a millionth of each other, which allows for how floats are printed.

If a file doesn't match even though you used `uv sync --frozen`, `reproduce` prints any package versions or census
code that differ from the published run's. Code changes on GitHub after a run was published are the usual cause:
check out the commit the run was published from (`git log census_runs/<name>`) and run it again.

## Make and publish a run

You need a free [Census API key](https://api.census.gov/data/key_signup.html), pasted on one line into
`py/census_api_key.txt`. That file is gitignored. You can use the environment variable `CENSUS_API_KEY` instead.

### 1. Make the run

Either in the app:
```bash
uv run streamlit run census_app.py
```
Make a run in the sidebar, then work through tabs 1 to 5, and 6 if you want training segments. Or from the command
line:
```bash
uv run census new provo_2026 --cities Provo Orem
```
```bash
uv run census set provo_2026 cluster.k=4 sample.per_cluster=2
```
```bash
uv run census run provo_2026
```

| Command | What it does |
|---|---|
| `uv run census places` | List the cities in Utah County |
| `uv run census new NAME --cities A B` | Make a run with the default settings |
| `uv run census show NAME` | Print the run's settings and each step's status |
| `uv run census set NAME section.key=value ...` | Change settings (see `py/census/config.py` for every one) |
| `uv run census run NAME [step ...]` | Run every step, or just the ones named: `areas choose_k cluster sample groups training` |
| `uv run census compare NAME` | Optional report: the chosen cities against the rest of the county |
| `uv run census export NAME SAMPLE` | Save the picks as `census_samples/SAMPLE.geojson` for the flight planner |

Each run keeps everything in `data/census/runs/<name>/`. That includes `config.json` with every setting, and one
folder per step with its files, a `report.txt`, and an `inputs.json`. The `inputs.json` records:

- the settings and code fingerprint;
- the package versions;
- every raw input file the step read, with its SHA-256.

**A step is out of date ("stale")** when one of these has changed since it ran:

- a setting it depends on;
- **the census code** (every module in `py/census` except the command line, `export.py`, `compare.py`,
  `check_draft.py` and `publish.py`).

Run it again to bring it up to date.

### 2. Publish it

Once every step is current, made with `uv run`, so the locked package versions were used:
```bash
uv run census publish provo_2026
```

This writes two things:

- **`census_runs/provo_2026/`**, to commit:
  - `config.json`: the settings.
  - `manifest.json`: the code version, the package versions, every input file with its SHA-256, the FAA and
    national security snapshots used and when they were fetched, the inputs archive, and the SHA-256 of every result file.
  - `expected/`: the result tables, for `verify` to compare against.
- **`data/census/archives/provo_2026_inputs.zip`**, to upload. It holds exactly the input files the run read. Files
  are stored in a fixed order with fixed dates, so the same inputs always make the same zip.

### 3. Upload the inputs and record their URL

Upload the zip somewhere public. Either:

- **Zenodo** ([zenodo.org](https://zenodo.org)), which gives it a DOI to cite in a paper; or
- **a GitHub release** of this repository (*Releases → Draft a new release*, then attach the zip).

Then record its direct download URL and commit:
```bash
uv run census publish provo_2026 --overwrite --url https://zenodo.org/records/<id>/files/provo_2026_inputs.zip
```
```bash
git add census_runs/provo_2026 pyproject.toml uv.lock && git commit -m "Publish the provo_2026 census run"
```

Republishing the same run makes the same zip. If you republish without `--url`, the URL already in the manifest is
kept.

### 4. Check it as a stranger would

Before announcing it, reproduce it under another name. That runs everything offline from the archived inputs only:
```bash
uv run census reproduce provo_2026 --as provo_2026_check
```

## Restricted airspace

At step 4 (Sample), *Fly restricted airspace?* decides what happens with FAA facility map squares whose ceiling is
under the high (nadir) flight height (200 ft by default). The command-line setting is `sample.fly_restricted`.

LAANC approves a flight instantly only up to a square's ceiling. Flying higher than that, including anywhere in a
0 ft square, needs a LAANC authorization requested ahead of time (further coordination). Allow about a week: in
practice, a request for the 0 ft squares near Provo airport was granted for a set time window about a week later.

| Setting | What it does |
|---|---|
| `no` (default) | Those squares are not flown. Block groups with at least `restricted_share` of their area under them can't be drawn, and streets in them are left out of the street miles. |
| `laanc` | Those squares are flown, 0 ft ones included, with LAANC authorization requested ahead of time. Every block group can be drawn and every street counts. |

In both modes, streets inside those squares are marked `restricted = True` and drawn dashed, in the step 4 and 5
files, maps and saved samples. That tells you which stretches fall under the authorization's time window.

With `laanc`, a pick that has such squares is marked `airspace = approval`, with its lowest ceiling in
`faa_ceiling_ft`, and the sample report lists those picks. Request authorization for them before the flying days.

The other airspace values are:

- `clear`: no FAA square.
- `laanc`: every ceiling allows the high flight, so LAANC approves it instantly.
- `partial`: a small part is off-limits; fly the rest. With `laanc`, only a national security area makes this.
- `restricted`: mostly off-limits; not eligible to be drawn. With `laanc`, only a national security area makes this.

With `eligible` weights, a pick stands for the block groups in its cluster that could have been drawn. So with
`fly_restricted = laanc`, the estimates describe every block group outside national security areas (below).

### National security areas

National Security UAS Flight Restrictions (14 CFR 99.7) are military bases and similar sites, such as Camp Williams
and the Utah Data Center, where no drone may fly with or without LAANC. They are never flown, whatever
`fly_restricted` says. Part-time ones are left out too, since a NOTAM can activate them at short notice.

- The part of a block group inside one counts as off-limits, in both modes. Its share is `share_no_fly`, and it adds
  to `share_restricted`. A block group with at least `restricted_share` off-limits can't be drawn, and one with less
  is `partial`.
- Streets inside them aren't listed at all, as if they were outside the block group. A street cut off at one ends
  at `NOFLY` in its mission name, for example `Unnamed_NOFLY`.
- The sample report says how many block groups are partly inside one. The areas used are saved as
  `national_security_areas.geojson` in step 4's folder and in `data/census/nsufr/`, so a reproduction uses the
  same ones.

So with `laanc`, `partial` and `restricted` mean a national security area. A pick that is `partial` but also has low
FAA squares is still listed in the report as needing authorization, by its `faa_ceiling_ft`.

## Training segments

Step 6 (*Training segments*, `training` on the command line) draws a number of street segments at random from every
picked block group, for example to collect object-detection training images from each one. A segment is one of step
5's corner-to-corner stretches, with its mission name (`Street1_Street2`). The step is optional: nothing else needs
it, and `publish` leaves it out when it hasn't run.

| Setting | Default | What it does |
|---|---|---|
| `training.per_block_group` | 5 | Segments drawn from each block group. A block group with fewer takes all of them, and the report says so |
| `training.seed` | 2026 | The draw. The app's *New random seed* button picks one at random |
| `training.min_length_m` | 30 | Shorter segments aren't drawn. About 3 in 10 segments are under 20 m, mostly stubs where a cross street pokes into the buffer around the block group (`Edge`). 0 allows every segment |

```bash
uv run census set provo_2026 training.per_block_group=10 training.seed=7
```
```bash
uv run census run provo_2026 training
```

- **The same seed always draws the same segments.** A larger number keeps every segment a smaller one drew, so
  going from 5 to 10 per block group adds 5 more without changing the first 5.
- **Restricted airspace follows step 4.** Segments in restricted airspace can be drawn only when
  `sample.fly_restricted = laanc`; they are listed with `restricted = True`.
- **Files:** `training_segments_seed<seed>.csv` and `.geojson` in `6_training/`, one row per drawn segment with its
  `training_pick` (draw order), its `segment` number (its place among the block group's stretches in step 5's
  GeoJSON), group, mission name, cross streets and miles. There is also a map per block group with the drawn segments
  numbered.
- **Saved samples:** when step 6 is current, *Save sample* (in tab 5 or 6) adds `segment` and `training_pick` to
  every street in `census_samples/<name>.geojson`. `training_pick` is empty for streets that weren't drawn.

## Where things are

| Path | What | In git? |
|---|---|---|
| `py/census/` | The package. `config.py` lists every setting, `steps.py` says what each step writes, `pipeline.py` does the work | yes |
| `census_app.py` | The app (settings and results only; no analysis) | yes |
| `census_runs/<name>/` | Published runs | yes |
| `census_samples/<name>.geojson` | Samples saved for the flight planner (`app.py`) | yes |
| `pyproject.toml`, `uv.lock`, `.python-version` | The environment. `requirements.txt` lists the same packages for pip users | yes |
| `data/census/runs/<name>/` | Your runs: settings and every step's outputs | no |
| `data/census/cache/`, `tiger/`, `faa/`, `nsufr/` | Raw inputs: ACS answers, TIGER/Line files, FAA map and national security area snapshots | no |
| `data/census/archives/` | Input archives made by `publish` or downloaded by `fetch-inputs` | no |
| `py/census_api_key.txt` | Your Census API key | **never** |

The package works without uv as well: `pip install -r requirements.txt`, then `python -m census ...` from the `py/`
folder. That installs whatever versions are newest, though, so use uv for anything you publish or reproduce.
