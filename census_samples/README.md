# Census samples

Block groups picked for flights, saved by the Census Sampling app (`streamlit run census_app.py`, step 4, *Save sample*)
or by `python -m census export <run> <name>` from `py/`. Each `<name>.geojson` holds one outlined area per picked block
group, with its GEOID, city, cluster, fly order and street miles; the file's `sample` entry records the cities and the
settings it was drawn with.

The flight planner (`app.py`) lists this folder: tick *Show Sampled Block Groups* under Visuals in the Creator, Editor
or Viewer tab and pick a sample to see its areas on the map. Commit a sample to share it with everyone planning flights.
