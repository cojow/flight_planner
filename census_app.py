"""
Census sampling: a simple front end for the py/census package.

Pick (or make) a run in the sidebar, then work through the four steps in
order: 1 Areas, 2 Choose k, 3 Cluster, 4 Sample. Each tab shows that step's
settings, prefilled from the run's config.json. "Save + run" writes the edited
settings back to config.json and runs the step; its outputs (report, tables,
charts and downloads) show underneath.

All the analysis lives in py/census; this file only shows settings and
results. The same steps can be run without it from the command line.

Run with:
    streamlit run census_app.py
"""
import sys
from pathlib import Path

import pandas as pd
import streamlit as st

sys.path.insert(0, str(Path(__file__).parent / "py"))
from census import acs, config as cfg, export, steps, streets  # noqa: E402

st.set_page_config(page_title="Census sampling", layout="wide")

TITLES = {"areas": "1 Areas", "choose_k": "2 Choose k", "cluster": "3 Cluster", "sample": "4 Sample"}
MARKERS = {"not run": "⚪ not run", "current": "🟢 current", "stale": "🟠 stale (settings changed since it ran)"}

# The choices the package accepts.
ROAD_TYPES = streets.ROAD_TYPES     # Census road code -> what it is
WEIGHTS = cfg.WEIGHTS


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

@st.cache_data(show_spinner="Loading the county's cities...")
def _county_places(state, county, year):
    from census import geography
    return geography.county_places(state, county, year)


def county_places(state, county, year):
    """
    The places (cities and towns) in the county: geoid -> name. If the list
    can't be loaded, the error is shown and only the cities already saved in
    the run are offered.
    """
    try:
        places = _county_places(state, county, year)
        return dict(zip(places["geoid"].astype(str), places["name"]))
    except (ImportError, AttributeError):
        st.info("The county's city list isn't available yet (census.geography.county_places).")
    except Exception as e:  # network trouble and the like
        st.warning(f"Couldn't load the county's cities: {e}")
    return {}


def read_csv(path):
    """A CSV with any GEOID column kept as text, so IDs match exactly."""
    return pd.read_csv(path, dtype={"geoid": str, "GEOID": str})


def geoid_column(table):
    for name in ("geoid", "GEOID"):
        if name in table.columns:
            return name
    return None


def show_outputs(folder, step, skip=()):
    """The report, tables, charts and download buttons for a finished step."""
    files = steps.outputs(folder, step)
    if not files:
        st.caption("No outputs yet.")
        return
    if steps.status(folder)[step] == "stale":
        st.warning("These outputs were made with older settings. Run the step again to update them.")

    report = steps.step_dir(folder, step) / "report.txt"
    if report.exists() and report.read_text().strip():
        st.code(report.read_text(), language=None)

    for label, path in files.items():
        if path.name in skip or not path.exists():
            continue
        if path.suffix == ".csv":
            st.markdown(f"**{label}**")
            st.dataframe(read_csv(path), hide_index=True)
        elif path.suffix == ".png":
            st.image(str(path), caption=label)

    st.markdown("**Downloads**")
    cols = st.columns(3)
    for i, (label, path) in enumerate(files.items()):
        if path.exists():
            cols[i % 3].download_button(
                f"{label} ({path.name})", path.read_bytes(), file_name=path.name,
                key=f"dl:{folder.name}:{step}:{label}")


def save_and_run(folder, step, changes):
    """
    The "Save + run" button for a step. `changes` maps config sections to their
    edited values; they replace what's saved, then the step runs.
    """
    config = cfg.load(folder)
    unsaved = any(config[section] != values for section, values in changes.items())
    if unsaved:
        st.caption("You have unsaved changes on this tab.")
    clicked = st.button(f"Save + run {TITLES[step]}", type="primary", key=f"run:{folder.name}:{step}")

    # The outcome of the last click, kept across the page reload below.
    flash = st.session_state.pop(f"flash:{folder.name}:{step}", None)
    if flash:
        kind, message = flash
        if kind == "success":
            st.success(message)
        elif kind == "error":
            st.error(message)
        else:
            st.error(f"{TITLES[step]} failed.")
            st.exception(message)
    if not clicked:
        return

    config.update(changes)
    try:
        cfg.save(folder, config)
    except cfg.ConfigError as e:
        st.error(f"Settings not saved: {e}")
        return
    try:
        with st.spinner(f"Running {TITLES[step]}..."):
            steps.RUNNERS[step](folder)
        flash = ("success", f"{TITLES[step]} finished.")
    except steps.StepError as e:
        flash = ("error", str(e))
    except Exception as e:
        flash = ("exception", e)
    st.session_state[f"flash:{folder.name}:{step}"] = flash
    st.rerun()      # the settings were saved either way, so redraw status and outputs


# --------------------------------------------------------------------------
# The four steps
# --------------------------------------------------------------------------

def tab_areas(folder, config):
    a = config["areas"]
    key = folder.name
    st.write("Which block groups are in the chosen cities, with their Census numbers.")

    year = st.number_input("ACS 5-year release (last year of the survey)", 2013, 2030,
                           int(a["year"]), step=1, key=f"{key}:year")
    places = county_places(a["state"], a["county"], year)
    for p in a["places"]:               # keep saved cities even if the list didn't load
        places.setdefault(p["geoid"], p["name"])
    chosen = st.multiselect("Cities", list(places), default=[p["geoid"] for p in a["places"]],
                            format_func=lambda g: f"{places[g]} ({g})", key=f"{key}:places")
    min_overlap = st.number_input(
        "Share of a block group's area that must be inside a city for it to count",
        0.01, 1.0, float(a["min_overlap"]), step=0.05, key=f"{key}:overlap")

    # After a run, every block group can be left in or out by hand.
    exclude = list(a["exclude"])
    table_path = steps.step_dir(folder, "areas") / "block_groups.csv"
    if table_path.exists():
        table = read_csv(table_path)
        gid = geoid_column(table)
        if gid:
            st.markdown("**Block groups** - untick *include* to leave one out of clustering and sampling.")
            st.caption("Block groups missing a value in a clustering column are also left out at steps 2 and 3; "
                       "their reports list them.")
            table.insert(0, "include", ~table[gid].isin(a["exclude"]))
            edited = st.data_editor(
                table, hide_index=True, disabled=[c for c in table.columns if c != "include"],
                key=f"{key}:exclude:{table_path.stat().st_mtime}")
            exclude = sorted(edited.loc[~edited["include"], gid])

    changes = {"areas": dict(a, year=int(year), min_overlap=float(min_overlap), exclude=exclude,
                             places=[{"geoid": g, "name": places[g]} for g in chosen])}
    save_and_run(folder, "areas", changes)
    show_outputs(folder, "areas", skip=("block_groups.csv",) if table_path.exists() else ())


def tab_choose_k(folder, config):
    f, k = config["features"], config["choose_k"]
    key = folder.name
    st.write("Scores for each k to help you pick one. Nothing is decided here; you set k in step 3.")

    # Any of the Census columns from step 1 can be clustered on.
    options = list(f["columns"]) + [c for c in acs.numeric_columns() if c not in f["columns"]]
    columns = st.multiselect("Columns to cluster on (each rescaled to mean 0, sd 1)",
                             options, default=f["columns"], key=f"{key}:columns")

    c1, c2, c3, c4 = st.columns(4)
    seed = c1.number_input("k-means seed", 0, None, int(f["seed"]), step=1, key=f"{key}:kseed")
    n_init = c2.number_input("k-means restarts", 1, 100, int(f["n_init"]), step=1, key=f"{key}:ninit")
    k_min = c3.number_input("Smallest k", 2, 20, int(k["k_min"]), step=1, key=f"{key}:kmin")
    k_max = c4.number_input("Largest k", 2, 20, int(k["k_max"]), step=1, key=f"{key}:kmax")
    preview = st.multiselect("Draw cluster charts for these k", list(range(2, 21)),
                             default=k["preview"], key=f"{key}:preview")

    changes = {
        "features": dict(f, columns=columns, seed=int(seed), n_init=int(n_init)),
        "choose_k": dict(k, k_min=int(k_min), k_max=int(k_max), preview=sorted(preview)),
    }
    save_and_run(folder, "choose_k", changes)
    show_outputs(folder, "choose_k")


def tab_cluster(folder, config):
    c = config["cluster"]
    key = folder.name
    st.write("K-means with your chosen k on the block groups kept in step 1. "
             "The columns, seed and restarts are set in step 2.")

    left, right = st.columns([1, 2])
    k = left.number_input("k (number of clusters)", 2, 20, int(c["k"]), step=1, key=f"{key}:k")
    scores = steps.step_dir(folder, "choose_k") / "k_scores.csv"
    if scores.exists() and steps.status(folder)["choose_k"] == "stale":
        right.warning("Step 2's scores below were made with older settings; run step 2 again.")
    if scores.exists():
        right.markdown("**Scores from step 2** (higher silhouette = better separated clusters)")
        right.dataframe(read_csv(scores), hide_index=True)
    else:
        right.caption("Run step 2 to see scores for each k here.")

    save_and_run(folder, "cluster", {"cluster": dict(c, k=int(k))})
    show_outputs(folder, "cluster")


def tab_sample(folder, config):
    s = config["sample"]
    key = folder.name
    st.write("Airspace check, the draw from each cluster, fly order and the streets to fly. "
             "Uses the clusters saved in step 3.")

    c1, c2, c3 = st.columns(3)
    per_cluster = c1.number_input("Block groups per cluster", 1, 20, int(s["per_cluster"]),
                                  step=1, key=f"{key}:per")
    seed = c2.number_input("Draw seed", 0, None, int(s["seed"]), step=1, key=f"{key}:sseed")
    weights = c3.selectbox("Weights", WEIGHTS,
                           index=WEIGHTS.index(s["weights"]) if s["weights"] in WEIGHTS else 0,
                           key=f"{key}:weights")

    c1, c2 = st.columns(2)
    high_flight_ft = c1.number_input("High (nadir) flight height, ft", 1, 400, int(s["high_flight_ft"]),
                                     step=10, key=f"{key}:high")
    restricted_share = c2.number_input(
        "Not eligible when this share is under a ceiling below that height",
        0.01, 1.0, float(s["restricted_share"]), step=0.05, key=f"{key}:share")

    c1, c2 = st.columns(2)
    buffer_m = c1.number_input("Count streets this close outside the edge, m", 0, 500, int(s["buffer_m"]),
                               step=5, key=f"{key}:buffer")
    min_segment_m = c2.number_input("Drop street pieces shorter than, m", 0, 500, int(s["min_segment_m"]),
                                    step=5, key=f"{key}:minseg")

    road_types = dict(ROAD_TYPES)
    for code in s["street_types"]:
        road_types.setdefault(code, code)
    street_types = st.multiselect("Street types", list(road_types), default=s["street_types"],
                                  format_func=lambda code: f"{code}: {road_types[code]}",
                                  help="Census road codes. The default is secondary roads and local streets.",
                                  key=f"{key}:streets")

    c1, c2 = st.columns(2)
    start_lat = c1.number_input("Start latitude", -90.0, 90.0, float(s["start_lat"]), step=0.0001,
                                format="%.4f", key=f"{key}:lat")
    start_lon = c2.number_input("Start longitude", -180.0, 180.0, float(s["start_lon"]), step=0.0001,
                                format="%.4f", key=f"{key}:lon")

    changes = {"sample": dict(
        s, per_cluster=int(per_cluster), seed=int(seed), weights=weights,
        high_flight_ft=int(high_flight_ft), restricted_share=float(restricted_share),
        buffer_m=int(buffer_m), min_segment_m=int(min_segment_m), street_types=street_types,
        start_lat=float(start_lat), start_lon=float(start_lon))}
    save_and_run(folder, "sample", changes)
    show_outputs(folder, "sample")
    save_for_planner(folder)


def save_for_planner(folder):
    """Name the sample and save its block groups where the flight planner (app.py) can show them."""
    if steps.status(folder)["sample"] != "current":
        return
    st.divider()
    st.markdown("**Save for the flight planner**")
    st.caption(f"Saves the picked block groups as census_samples/<name>.geojson. In the flight planner, "
               "tick *Show Sampled Block Groups* and pick this name to see them on the map.")
    key = folder.name
    name = st.text_input("Sample name (letters, digits, - and _)", value=folder.name, key=f"{key}:sample_name")
    exists = name in export.list_samples()
    overwrite = st.checkbox(f"Replace the saved sample {name!r}", key=f"{key}:overwrite") if exists else False
    if st.button("Save sample", key=f"{key}:save_sample", disabled=exists and not overwrite):
        try:
            path = export.export_sample(folder, name, overwrite=overwrite)
            st.success(f"Saved {path.relative_to(cfg.REPO_ROOT)}")
        except (cfg.ConfigError, steps.StepError) as e:
            st.error(str(e))
    if export.list_samples():
        st.caption("Saved samples: " + ", ".join(export.list_samples()))


# --------------------------------------------------------------------------
# Page
# --------------------------------------------------------------------------

def new_run_form():
    """Make a run: a name and its cities (a run needs at least one city to save)."""
    with st.sidebar.expander("New run", expanded=not cfg.list_runs()):
        name = st.text_input("Name (letters, digits, - and _)", key="new:name")
        d = cfg.DEFAULTS["areas"]
        places = county_places(d["state"], d["county"], d["year"])
        chosen = st.multiselect("Cities", list(places), format_func=lambda g: places[g], key="new:places")
        if st.button("Create run"):
            try:
                folder = cfg.run_dir(name)
                if (folder / "config.json").exists():
                    raise cfg.ConfigError(f"A run called {name!r} already exists.")
                config = cfg.default_config(name)
                config["areas"]["places"] = [{"geoid": g, "name": places[g]} for g in chosen]
                cfg.save(folder, config)
            except cfg.ConfigError as e:
                st.error(str(e))
                return
            st.session_state["pending_run"] = name
            st.rerun()


def main():
    st.title("Census sampling")

    # A run just made in the form becomes the selected one (this has to happen
    # before the select box is drawn).
    if "pending_run" in st.session_state:
        st.session_state["run"] = st.session_state.pop("pending_run")

    runs = cfg.list_runs()
    st.sidebar.header("Run")
    run = st.sidebar.selectbox("Open a run", runs, key="run") if runs else None
    new_run_form()
    if not run:
        st.info("Make a run in the sidebar to start: give it a name and pick its cities.")
        return

    folder = cfg.run_dir(run)
    try:
        config = cfg.load(folder)
        states = steps.status(folder)
    except (cfg.ConfigError, ValueError, KeyError) as e:
        st.error(f"Can't read this run's config.json: {e}")
        return

    st.sidebar.markdown("**Steps**")
    for step, title in TITLES.items():
        st.sidebar.write(f"{title}: {MARKERS[states[step]]}")
    st.sidebar.caption(f"Folder: {folder}")
    st.caption("Cities: " + ", ".join(p["name"] for p in config["areas"]["places"]))

    tabs = st.tabs(list(TITLES.values()), key="tab")
    for tab, show in zip(tabs, (tab_areas, tab_choose_k, tab_cluster, tab_sample)):
        with tab:
            show(folder, config)


main()
