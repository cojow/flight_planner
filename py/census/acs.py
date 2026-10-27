"""
The American Community Survey (ACS) 5-year numbers for every block group in a
county: income, the age and kind of the housing, rooms and household size.

What the columns mean
---------------------
The ACS counts housing units: houses, apartments, condos, mobile homes - not
commercial buildings. Columns ending in _all cover every unit; _owner and
_renter split them by who lives there; the pct_ columns say when the units
were built and what kind of building they are in.

- These are survey estimates. Income and year built come with a margin of
  error (_moe) and a coefficient of variation (_cv): under ~0.12 is solid,
  over ~0.40 is shaky.
- median_age_years_* is the survey year minus the median year built, so 40
  means a typical unit there is 40 years old.
- Medians at the edge of the scale are capped by the Census: a median year
  built of 1939 means "1939 or earlier", and incomes top out at 250,001.
- Values the Census could not estimate are blank (NaN), never the large
  negative placeholder codes the raw API uses.
"""
import pandas as pd

from .sources import ACS_URL, api_key, cached_get

MEDIANS = {
    "B19013_001": "median_household_income",
    "B25035_001": "median_year_built_all",       # every housing unit, vacant included
    "B25037_002": "median_year_built_owner",     # owner-occupied units
    "B25037_003": "median_year_built_renter",    # renter-occupied units
    "B25018_001": "median_rooms_all",
    "B25021_002": "median_rooms_owner",
    "B25021_003": "median_rooms_renter",
    "B25010_001": "avg_household_size",
}

# Units by the decade they were built (B25034). Shares of these describe a
# neighbourhood's age far better than one median does - a mix of 1920s houses
# and new apartments has the same median as an area built entirely in 1975.
YEAR_BUILT_TOTAL = "B25034_001"
YEAR_BUILT = {
    "B25034_002": "2020_later",
    "B25034_003": "2010_2019",
    "B25034_004": "2000_2009",
    "B25034_005": "1990_1999",
    "B25034_006": "1980_1989",
    "B25034_007": "1970_1979",
    "B25034_008": "1960_1969",
    "B25034_009": "1950_1959",
    "B25034_010": "1940_1949",
    "B25034_011": "1939_earlier",
}

# Units by the kind of building they are in (B25024).
STRUCTURE_TOTAL = "B25024_001"
STRUCTURE = {
    "B25024_002": "single_family_detached",
    "B25024_003": "single_family_attached",
    "B25024_004": "2_units",
    "B25024_005": "3_4_units",
    "B25024_006": "5_9_units",
    "B25024_007": "10_19_units",
    "B25024_008": "20_49_units",
    "B25024_009": "50_plus_units",
    "B25024_010": "mobile_home",
    "B25024_011": "boat_rv_van",
}

# Margins of error worth carrying: the two values most likely to be clustered on.
WITH_MOE = ("B19013_001", "B25035_001")

# The raw API's placeholders for "could not be estimated" are large negative
# numbers (-666666666 and friends). Anything this negative is one of them.
SENTINEL_FLOOR = -100_000_000
# -555555555 on a margin of error means "controlled - no sampling error".
MOE_NO_ERROR = -555555555

# The API rejects more than 50 variables in one request.
MAX_VARIABLES_PER_REQUEST = 49


def _variable_codes():
    codes = []
    for code in MEDIANS:
        codes.append(f"{code}E")
        if code in WITH_MOE:
            codes.append(f"{code}M")
    codes += [f"{c}E" for c in (YEAR_BUILT_TOTAL, *YEAR_BUILT, STRUCTURE_TOTAL, *STRUCTURE)]
    return codes


def _fetch(geography, within, year, key, refresh=False):
    """Raw ACS rows for one geography, as a DataFrame of strings."""
    codes = _variable_codes()
    frames = []
    for start in range(0, len(codes), MAX_VARIABLES_PER_REQUEST):
        chunk = codes[start:start + MAX_VARIABLES_PER_REQUEST]
        params = {"get": ",".join(["NAME", *chunk]), "for": geography, "key": key}
        if within:
            params["in"] = within
        data = cached_get(ACS_URL.format(year=year), params, refresh=refresh)
        frames.append(pd.DataFrame(data[1:], columns=data[0]))
    merged = frames[0]
    geo_cols = [c for c in merged.columns if c not in codes and c != "NAME"]
    for extra in frames[1:]:
        merged = merged.merge(extra.drop(columns="NAME"), on=geo_cols)
    return merged


def _clean(raw, year):
    """Raw ACS strings -> labelled numeric columns, placeholders blanked, shares derived."""
    out = pd.DataFrame(index=raw.index)
    num = {c: pd.to_numeric(raw[c], errors="coerce") for c in raw.columns if c[:1] in "BC"}
    for code in num:
        values = num[code]
        if code.endswith("M"):
            # "No sampling error" is a real zero; the other codes mean unknown.
            values = values.mask(values == MOE_NO_ERROR, 0)
        num[code] = values.mask(values <= SENTINEL_FLOOR)

    for code, name in MEDIANS.items():
        out[name] = num[f"{code}E"]
        if code in WITH_MOE:
            out[f"{name}_moe"] = num[f"{code}M"]
            # Coefficient of variation: the standard error as a share of the estimate.
            out[f"{name}_cv"] = (num[f"{code}M"] / 1.645 / num[f"{code}E"]).round(3)

    # Ages are relative to the survey, not today, so they don't drift every January.
    out["median_age_years_all"] = year - out["median_year_built_all"]
    out["median_age_years_owner"] = year - out["median_year_built_owner"]
    out["median_age_years_renter"] = year - out["median_year_built_renter"]

    total_built = num[f"{YEAR_BUILT_TOTAL}E"]
    out["housing_units"] = total_built
    for code, era in YEAR_BUILT.items():
        out[f"pct_built_{era}"] = (num[f"{code}E"] / total_built * 100).round(1)
    out["pct_built_since_2000"] = out[["pct_built_2020_later", "pct_built_2010_2019",
                                       "pct_built_2000_2009"]].sum(axis=1, min_count=1)
    out["pct_built_before_1980"] = out[[f"pct_built_{e}" for e in
                                        ("1970_1979", "1960_1969", "1950_1959",
                                         "1940_1949", "1939_earlier")]].sum(axis=1, min_count=1)

    total_struct = num[f"{STRUCTURE_TOTAL}E"]
    for code, kind in STRUCTURE.items():
        out[f"pct_{kind}"] = (num[f"{code}E"] / total_struct * 100).round(1)
    # The house / apartment split most analyses want, in two columns.
    out["pct_single_family"] = out[["pct_single_family_detached",
                                    "pct_single_family_attached"]].sum(axis=1, min_count=1)
    out["pct_multi_family"] = out[["pct_2_units", "pct_3_4_units", "pct_5_9_units",
                                   "pct_10_19_units", "pct_20_49_units",
                                   "pct_50_plus_units"]].sum(axis=1, min_count=1)
    return out


def county_block_groups(state, county, year, key=None, refresh=False):
    """One row per block group in a county: geoid, census_name, then the ACS columns."""
    raw = _fetch("block group:*", f"state:{state} county:{county} tract:*", year, api_key(key), refresh)
    table = _clean(raw, year)
    table.insert(0, "geoid", (raw["state"] + raw["county"] + raw["tract"] + raw["block group"]).values)
    table.insert(1, "census_name", raw["NAME"].values)
    return table


def numeric_columns():
    """The ACS columns that can be clustered on, in table order."""
    names = []
    for code, name in MEDIANS.items():
        names.append(name)
        if code in WITH_MOE:
            names += [f"{name}_moe", f"{name}_cv"]
    names += ["median_age_years_all", "median_age_years_owner", "median_age_years_renter", "housing_units"]
    names += [f"pct_built_{e}" for e in YEAR_BUILT.values()] + ["pct_built_since_2000", "pct_built_before_1980"]
    names += [f"pct_{k}" for k in STRUCTURE.values()] + ["pct_single_family", "pct_multi_family"]
    return names
