# City Block Groups

Pick a city, get the Census block groups in it, and see them on a map. This is
the sampling frame from the methodology: the areas we stratify, sample and fly.

```bash
pip install -r requirements.txt
python city_block_groups.py --city Provo --state UT
```

It opens an interactive map (satellite by default, hover a block group for its
GEOID, tract, and land area) and writes two files to `output/`:

- `provo_ut_2023_block_groups.html` - the map
- `provo_ut_2023_block_groups.geojson` - the block groups in the city, one row each

The first run for a state downloads its TIGER/Line boundary files from the
Census Bureau (about 10 MB for Utah) into `data/`. Later runs use the cache.

## What counts as "in the city"

Block groups do not nest inside city limits - the city line cuts through them.
So a block group counts as in the city when at least `--min-overlap` of its own
area lies inside the city boundary (default `0.5`). The ones that overlap less
are drawn as dashed white outlines and left out of the extract, so you can see
exactly where the cut-off fell.

| Want | Use |
|---|---|
| Only block groups that are mostly in the city | `--min-overlap 0.5` (default) |
| Anything meaningfully overlapping the city | `--min-overlap 0.1` |
| Only block groups entirely inside the city | `--min-overlap 0.99` |

Areas are measured on the polygons as the Census publishes them, which include
water. A block group that is mostly lake can therefore land on either side of
the cut-off; `ALAND` and `AWATER` are in the GeoJSON if you want to filter those.

## Options

| Flag | Meaning |
|---|---|
| `--city`, `--state` | City name and two-letter state abbreviation (required) |
| `--year` | TIGER vintage, default 2023 (matches the 2019-2023 ACS) |
| `--min-overlap` | Share of a block group's area that must be in the city, 0-1 |
| `--place-geoid` | Pick a place by GEOID when a name matches more than one |
| `--no-open` | Write the files without opening the map |

## Joining ACS data

The GeoJSON's `GEOID` is the 12-digit block-group code (state + county + tract +
block group), the same key the ACS tables use. Join on it as a string, not a
number - leading zeros matter.

Tract codes starting with `98` (or `99`) are Census special-land-use and water
tracts - campuses, airports, large parks. Provo's extract has four of them
(`490499803001` to `490499806001`). They are real block groups in the frame, but
the ACS generally publishes no estimates for them, so expect gaps when you join.

From a notebook you can also call `city_block_groups(city, state)` directly; it
returns `(place, inside, partial)` as GeoDataFrames.
