"""
City Block Groups - pull the Census block groups that make up one city and
show them on a map.

Block groups don't nest inside cities (a city limit cuts straight through
them), so "the block groups in a city" needs a rule. The rule here is area
overlap: a block group is in the city when at least --min-overlap of its own
area lies inside the city boundary (default 0.5). Block groups that overlap
less than that are kept on the map as a dashed layer, so the cut-off is
visible instead of silent.

Boundaries come from the Census Bureau's TIGER/Line files, downloaded once
per state and cached. The block-group GEOID in the output is the key for
joining ACS tables later.

Usage:
    python city_block_groups.py --city Provo --state UT
"""
import argparse
import difflib
import html
import os
import re
import urllib.error
import urllib.request
import webbrowser

import folium
import geopandas as gpd

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CACHE_DIR = os.path.join(HERE, "data")
DEFAULT_OUT_DIR = os.path.join(HERE, "output")

# TIGER vintage. ACS 5-year estimates use the boundaries of their final year,
# so 2023 pairs with the 2019-2023 ACS. Pass --year to match a different one.
DEFAULT_YEAR = 2023
DEFAULT_MIN_OVERLAP = 0.5

# Block groups that only touch the city line, or overlap it by a sliver left
# over from the two layers being digitised separately, are noise, not
# candidates. Anything under this share of a block group's area is dropped
# before the cut-off is applied.
MIN_SHARE_SHOWN = 0.001

TIGER_URL = "https://www2.census.gov/geo/tiger/TIGER{year}/{layer}/tl_{year}_{fips}_{suffix}.zip"
SQM_PER_SQMI = 2_589_988.11

# Same imagery source as the Flight Planner's own maps (see add_basemap in
# app.py), so a block group looks the same here as it will when you plan over it.
ESRI_TILE_BASE = "https://server.arcgisonline.com/ArcGIS/rest/services"
ESRI_IMAGERY_URL = f"{ESRI_TILE_BASE}/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}"
ESRI_TRANSPORT_URL = f"{ESRI_TILE_BASE}/Reference/World_Transportation/MapServer/tile/{{z}}/{{y}}/{{x}}"
ESRI_ATTR = ("Tiles &copy; Esri &mdash; Source: Esri, Maxar, Earthstar Geographics, "
             "and the GIS User Community")

STATE_FIPS = {
    "AL": "01", "AK": "02", "AZ": "04", "AR": "05", "CA": "06", "CO": "08",
    "CT": "09", "DE": "10", "DC": "11", "FL": "12", "GA": "13", "HI": "15",
    "ID": "16", "IL": "17", "IN": "18", "IA": "19", "KS": "20", "KY": "21",
    "LA": "22", "ME": "23", "MD": "24", "MA": "25", "MI": "26", "MN": "27",
    "MS": "28", "MO": "29", "MT": "30", "NE": "31", "NV": "32", "NH": "33",
    "NJ": "34", "NM": "35", "NY": "36", "NC": "37", "ND": "38", "OH": "39",
    "OK": "40", "OR": "41", "PA": "42", "RI": "44", "SC": "45", "SD": "46",
    "TN": "47", "TX": "48", "UT": "49", "VT": "50", "VA": "51", "WA": "53",
    "WV": "54", "WI": "55", "WY": "56", "PR": "72",
}


class CensusDataError(Exception):
    """A lookup or download failed in a way the user can act on."""


def state_fips(state):
    """Two-letter abbreviation (or an already-numeric FIPS code) -> FIPS code."""
    key = state.strip().upper()
    if key in STATE_FIPS:
        return STATE_FIPS[key]
    if key in STATE_FIPS.values():
        return key
    raise CensusDataError(f"Unknown state '{state}'. Use the two-letter abbreviation, e.g. UT.")


def tiger_zip(layer, suffix, fips, year, cache_dir):
    """Path to a state's TIGER/Line zip, downloading it on first use."""
    path = os.path.join(cache_dir, f"tl_{year}_{fips}_{suffix}.zip")
    if os.path.exists(path):
        return path

    url = TIGER_URL.format(year=year, layer=layer, fips=fips, suffix=suffix)
    os.makedirs(cache_dir, exist_ok=True)
    print(f"Downloading {url}")
    # Download to a temp name and rename, so an interrupted download can't
    # leave a truncated zip that later runs would trust as cached.
    partial = path + ".part"
    try:
        urllib.request.urlretrieve(url, partial)
        os.replace(partial, path)
    except urllib.error.HTTPError as e:
        raise CensusDataError(
            f"Census server answered {e.code} for {url} - is {year} a published TIGER year?"
        ) from e
    finally:
        if os.path.exists(partial):
            os.remove(partial)
    return path


def load_places(fips, year, cache_dir):
    return gpd.read_file(tiger_zip("PLACE", "place", fips, year, cache_dir))


def load_block_groups(fips, year, cache_dir):
    return gpd.read_file(tiger_zip("BG", "bg", fips, year, cache_dir))


def find_place(places, city, geoid=None):
    """
    The one-row GeoDataFrame for a city. Matches on NAME ("Provo") or
    NAMELSAD ("Provo city"), ignoring case.

    Names repeat within a state (a city and a CDP, or two towns with the same
    name), so more than one hit is an error that lists the GEOIDs rather than
    a guess; pass geoid to pick one.
    """
    if geoid:
        hit = places[places["GEOID"] == geoid]
        if hit.empty:
            raise CensusDataError(f"No place with GEOID {geoid} in this state.")
        return hit.iloc[[0]]

    wanted = city.strip().casefold()
    names = places["NAME"].str.casefold()
    hits = places[(names == wanted) | (places["NAMELSAD"].str.casefold() == wanted)]
    if len(hits) == 1:
        return hits.iloc[[0]]

    if hits.empty:
        original = dict(zip(names, places["NAME"]))
        close = [original[n] for n in difflib.get_close_matches(wanted, list(original), n=5, cutoff=0.6)]
        hint = f" Did you mean: {', '.join(close)}?" if close else ""
        raise CensusDataError(f"No place named '{city}' in this state.{hint}")

    rows = "\n".join(f"  {r.GEOID}  {r.NAMELSAD}" for r in hits.itertuples())
    raise CensusDataError(f"'{city}' matches more than one place - pick one with --place-geoid:\n{rows}")


def select_block_groups(block_groups, place, min_overlap=DEFAULT_MIN_OVERLAP):
    """
    Split the block groups that touch the city into (inside, partial).

    inside:  at least min_overlap of the block group's area is in the city.
    partial: overlaps the city, but by less than that.
    Both carry pct_in_city and come back in WGS84 for mapping.
    """
    # Overlap shares are ratios of areas, so they need a projection that
    # measures area in metres; the TIGER files are in lat/lon degrees. UTM
    # zone for the city is plenty accurate at city scale.
    crs = place.estimate_utm_crs()
    city = place.to_crs(crs).geometry.iloc[0]
    projected = block_groups.to_crs(crs)
    projected = projected[projected.intersects(city)]

    share = projected.intersection(city).area / projected.area
    candidates = block_groups.loc[projected.index].copy()
    candidates["pct_in_city"] = (share * 100).round(1)

    shown = share > MIN_SHARE_SHOWN
    candidates, share = candidates[shown], share[shown]
    inside = candidates[share >= min_overlap].to_crs(4326)
    partial = candidates[share < min_overlap].to_crs(4326)
    return inside, partial


def build_map(place, inside, partial, title, min_overlap):
    """Folium map: block groups in the city, near-misses dashed, city outline on top."""
    m = folium.Map(tiles=None, control_scale=True)
    folium.TileLayer(tiles=ESRI_IMAGERY_URL, attr=ESRI_ATTR, name="Satellite", max_zoom=19).add_to(m)
    # show=False: otherwise Leaflet opens on whichever base layer was added last.
    folium.TileLayer("OpenStreetMap", name="Street map", show=False).add_to(m)
    folium.TileLayer(tiles=ESRI_TRANSPORT_URL, attr=ESRI_ATTR, name="Street labels",
                     overlay=True, max_zoom=19).add_to(m)

    # One per layer: a folium element has a single parent, so sharing one
    # tooltip between two GeoJson layers leaves the first layer's tooltip
    # bound to the second layer's variable and the page script dies on load.
    def tooltip():
        return folium.GeoJsonTooltip(
            fields=["GEOID", "TRACTCE", "BLKGRPCE", "pct_in_city", "land_sq_mi"],
            aliases=["GEOID", "Tract", "Block group", "% of area in city", "Land (sq mi)"],
            sticky=True,
        )

    def with_land_area(gdf):
        return gdf.assign(land_sq_mi=(gdf["ALAND"] / SQM_PER_SQMI).round(2))

    if len(partial):
        folium.GeoJson(
            with_land_area(partial),
            name=f"Partly in city, excluded ({len(partial)})",
            style_function=lambda _: {"color": "#ffffff", "weight": 1.5, "dashArray": "5,5",
                                      "fillColor": "#ffffff", "fillOpacity": 0.04},
            highlight_function=lambda _: {"weight": 3, "fillOpacity": 0.2},
            tooltip=tooltip(),
        ).add_to(m)

    if len(inside):
        folium.GeoJson(
            with_land_area(inside),
            name=f"Block groups in city ({len(inside)})",
            style_function=lambda _: {"color": "#00e5ff", "weight": 2,
                                      "fillColor": "#00e5ff", "fillOpacity": 0.12},
            highlight_function=lambda _: {"weight": 4, "fillOpacity": 0.35},
            tooltip=tooltip(),
        ).add_to(m)

    # Last, so the city line is drawn over the block-group edges it cuts through.
    folium.GeoJson(
        place.to_crs(4326)[["NAMELSAD", "geometry"]],
        name="City boundary",
        style_function=lambda _: {"color": "#ff8800", "weight": 4, "fill": False},
    ).add_to(m)

    boxes = [place.to_crs(4326).total_bounds]
    if len(inside):
        boxes.append(inside.total_bounds)
    m.fit_bounds([[min(b[1] for b in boxes), min(b[0] for b in boxes)],
                  [max(b[3] for b in boxes), max(b[2] for b in boxes)]])
    folium.LayerControl(collapsed=False).add_to(m)

    legend = f"""
    <div style="position: fixed; bottom: 28px; left: 12px; z-index: 9999;
                background: rgba(255,255,255,.92); padding: 8px 12px; border-radius: 6px;
                font: 13px/1.6 sans-serif; box-shadow: 0 1px 4px rgba(0,0,0,.3);">
      <b>{html.escape(title)}</b><br>
      <span style="color:#00b8cc">&#9632;</span> {len(inside)} block groups with
      &ge;{min_overlap:.0%} of their area in the city<br>
      <span style="color:#888">&#9632;</span> {len(partial)} partly overlapping (excluded)<br>
      <span style="color:#ff8800">&#9632;</span> city boundary
    </div>"""
    m.get_root().html.add_child(folium.Element(legend))
    return m


def city_block_groups(city, state, year=DEFAULT_YEAR, min_overlap=DEFAULT_MIN_OVERLAP,
                      place_geoid=None, cache_dir=DEFAULT_CACHE_DIR):
    """Load the state's data and return (place, inside, partial)."""
    if not 0 < min_overlap <= 1:
        raise CensusDataError("--min-overlap must be above 0 and at most 1.")
    fips = state_fips(state)
    place = find_place(load_places(fips, year, cache_dir), city, place_geoid)
    inside, partial = select_block_groups(load_block_groups(fips, year, cache_dir), place, min_overlap)
    if inside.empty:
        raise CensusDataError(
            f"No block group has {min_overlap:.0%} of its area in {place['NAMELSAD'].iloc[0]}. "
            f"{len(partial)} overlap it less than that - try a lower --min-overlap."
        )
    return place, inside, partial


def main(argv=None):
    ap = argparse.ArgumentParser(description="Map the Census block groups in one city.")
    ap.add_argument("--city", required=True, help='City name, e.g. "Provo" or "Salt Lake City".')
    ap.add_argument("--state", required=True, help="Two-letter state abbreviation, e.g. UT.")
    ap.add_argument("--year", type=int, default=DEFAULT_YEAR,
                    help=f"TIGER/Line vintage (default {DEFAULT_YEAR}).")
    ap.add_argument("--min-overlap", type=float, default=DEFAULT_MIN_OVERLAP,
                    help="Share of a block group's area that must be inside the city for it to "
                         f"count as in the city, 0-1 (default {DEFAULT_MIN_OVERLAP}).")
    ap.add_argument("--place-geoid", help="Pick a place by GEOID when the name matches more than one.")
    ap.add_argument("--cache-dir", default=DEFAULT_CACHE_DIR, help="Where downloaded TIGER zips are kept.")
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR, help="Where the map and GeoJSON are written.")
    ap.add_argument("--no-open", action="store_true", help="Write the map but don't open it in a browser.")
    args = ap.parse_args(argv)

    try:
        place, inside, partial = city_block_groups(
            args.city, args.state, args.year, args.min_overlap, args.place_geoid, args.cache_dir)
    except CensusDataError as e:
        raise SystemExit(str(e))

    name = place["NAMELSAD"].iloc[0]
    title = f"{name}, {args.state.upper()}"
    print(f"{title} (GEOID {place['GEOID'].iloc[0]}): {len(inside)} block groups with "
          f">={args.min_overlap:.0%} of their area in the city; {len(partial)} more overlap it less than that.")

    os.makedirs(args.out_dir, exist_ok=True)
    stem = re.sub(r"[^a-z0-9]+", "_", f"{place['NAME'].iloc[0]}_{args.state}_{args.year}".lower()).strip("_")
    geojson_path = os.path.join(args.out_dir, f"{stem}_block_groups.geojson")
    map_path = os.path.join(args.out_dir, f"{stem}_block_groups.html")

    inside.to_file(geojson_path, driver="GeoJSON")
    build_map(place, inside, partial, title, args.min_overlap).save(map_path)
    print(f"Wrote {geojson_path}\nWrote {map_path}")

    if not args.no_open:
        webbrowser.open("file://" + os.path.abspath(map_path))


if __name__ == "__main__":
    main()
