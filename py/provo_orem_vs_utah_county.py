"""
Provo-Orem versus all of Utah County: do they look like the same place?

Pulls Census block-group data (ACS 5-year) and compares the two areas on
median household income and the age of the houses (median_age_years_all: the
survey year minus the median year built, so 40 means a typical house there
is 40 years old). Then it asks the same question two more ways:

  1. Clustering (k-means). Groups block groups that look alike on income and
     house age, separately for Provo-Orem and for the whole county, and shows
     how the groups differ.
  2. Random forest. Learns to predict a block group's median income from its
     housing traits (house age, building type, rooms, household size), once on
     Provo-Orem and once on the whole county. If the two models lean on
     different traits, or the Provo-Orem model does badly on the rest of the
     county, the places work differently.

Run it:

    python py/provo_orem_vs_utah_county.py

Needs the Census API key in py/census_api_key.txt (see building_census_data.py)
and:  pip install pandas requests shapely scikit-learn matplotlib

The scatter plot of income against house age is saved as
data/census/provo_orem_vs_county/income_vs_house_age.png, and a second one with
only Provo and Orem as income_vs_house_age_provo_orem.png.
"""
import sys

import matplotlib.pyplot as plt
import pandas as pd
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import cross_val_score

import building_census_data as bcd
from provo_orem_common import COLORS, COLUMNS, DATA_DIR, save_figure, style_income_age_axes

STATE = "UT"
COUNTY = "049"                      # Utah County
CITIES = ["Provo", "Orem"]

# What the forest may use to predict income. House age is one of them: the
# forest shows how much it matters on its own.
PREDICTORS = ["median_age_years_all", "avg_household_size", "median_rooms_all",
              "pct_single_family", "pct_multi_family",
              "pct_built_since_2000", "pct_built_before_1980"]


# --------------------------------------------------------------------------
# Getting the data
# --------------------------------------------------------------------------

def load(year=bcd.DEFAULT_YEAR):
    """
    (county, provo_orem): one row per block group. provo_orem is a subset of
    county - the block groups that are mostly inside Provo or Orem.
    """
    key = bcd._api_key()

    # Every block group in the county. _fetch_acs/_clean are the same steps
    # get_city_data uses, so the columns match exactly.
    raw = bcd._fetch_acs("block group:*", f"state:{bcd.state_fips(STATE)} county:{COUNTY} tract:*",
                         year, key)
    county = bcd._clean(raw, year)
    county.insert(0, "geoid", (raw["state"] + raw["county"] + raw["tract"] + raw["block group"]).values)

    # Which of those belong to Provo and Orem (same rule as the sampling frame).
    cities = [bcd.get_city_data(c, STATE, year=year, key=key)[["geoid", "city"]] for c in CITIES]
    membership = pd.concat(cities).drop_duplicates("geoid")
    county = county.merge(membership, on="geoid", how="left")
    county["in_provo_orem"] = county["city"].notna()

    # Block groups with no income or no houses (campuses, parks, airports)
    # carry no signal, so they are dropped from every comparison.
    county = county.dropna(subset=COLUMNS)
    county = county[county["housing_units"] > 0]
    return county, county[county["in_provo_orem"]]


# --------------------------------------------------------------------------
# 1. Plain comparison
# --------------------------------------------------------------------------

def compare(county, provo_orem):
    rest = county[~county["in_provo_orem"]]
    groups = {"Provo-Orem": provo_orem, "Whole county": county, "Rest of county": rest}
    rows = {}
    for name, df in groups.items():
        row = {"block groups": len(df)}
        for col in COLUMNS:
            row[f"{col} (mean)"] = df[col].mean()
            row[f"{col} (median)"] = df[col].median()
        rows[name] = row
    return pd.DataFrame(rows).T.round(1)


# --------------------------------------------------------------------------
# 2. Clustering
# --------------------------------------------------------------------------

def cluster_summary(df, label):
    """
    k-means on income + house age, with k picked by silhouette score; one row
    per cluster, with its averages.
    """
    k = int(bcd.choose_k(df, COLUMNS, range(2, 7)).idxmax())
    summary = bcd.kmeans(df, COLUMNS, k).groupby("cluster").agg(
        block_groups=("geoid", "size"),
        income=("median_household_income", "mean"),
        house_age_years=("median_age_years_all", "mean"),
    ).round(1)
    summary.insert(0, "model", f"{label} (k={k})")
    return summary


# --------------------------------------------------------------------------
# 3. Random forest
# --------------------------------------------------------------------------

def forest(county, provo_orem, target="median_household_income"):
    """
    Two forests predicting `target`: one trained on Provo-Orem, one on the whole
    county. Returns (importances table, scores table).

    Scores are R^2 (1.0 = perfect, 0 = no better than guessing the average).
    The Provo-Orem forest is also scored on the rest of the county, which it
    never saw - the real test of "is it different".
    """
    rest = county[~county["in_provo_orem"]]
    use = [c for c in PREDICTORS if c in county.columns]

    def fit(df):
        # One thread: with a few hundred rows, starting worker threads costs more than it saves.
        df = df.dropna(subset=[target, *use])
        return RandomForestRegressor(n_estimators=300, min_samples_leaf=3, random_state=0, n_jobs=1), df

    po_model, po = fit(provo_orem)
    co_model, co = fit(county)
    rest = rest.dropna(subset=[target, *use])

    po_model.fit(po[use], po[target])
    co_model.fit(co[use], co[target])

    importances = pd.DataFrame({"Provo-Orem": po_model.feature_importances_,
                                "Whole county": co_model.feature_importances_},
                               index=use).sort_values("Whole county", ascending=False).round(3)

    # Cross-validated, so a forest is never scored on block groups it trained on.
    scores = {
        "Provo-Orem model, on Provo-Orem (5-fold CV)":
            cross_val_score(po_model, po[use], po[target], cv=5, scoring="r2").mean(),
        "County model, on whole county (5-fold CV)":
            cross_val_score(co_model, co[use], co[target], cv=5, scoring="r2").mean(),
        "Provo-Orem model, on rest of county (never seen)":
            po_model.score(rest[use], rest[target]),
    }
    return importances, pd.Series(scores, name="R^2").round(3)


# --------------------------------------------------------------------------
# 4. Scatter plot
# --------------------------------------------------------------------------

def scatter_plot(groups, title, path):
    """
    One dot per block group: house age across, income up. `groups` is a list of
    (label, block groups, color); each gets a star at its median.
    """
    fig, ax = plt.subplots(figsize=(9, 6))
    for label, df, color in groups:
        ax.scatter(df["median_age_years_all"], df["median_household_income"],
                   s=30, alpha=0.65, color=color, edgecolors="none", label=f"{label} ({len(df)})")
        ax.scatter(df["median_age_years_all"].median(), df["median_household_income"].median(),
                   s=260, marker="*", color=color, edgecolors="black", linewidths=1, zorder=5)
    style_income_age_axes(ax, title)
    ax.legend(frameon=False)
    save_figure(fig, path)


# --------------------------------------------------------------------------

def main():
    try:
        county, provo_orem = load()
    except bcd.CensusError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    pd.set_option("display.width", 200)
    pd.set_option("display.max_columns", 20)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    county.to_csv(DATA_DIR / "utah_county_block_groups.csv", index=False)

    print(f"\n{len(provo_orem)} Provo-Orem block groups, {len(county)} in Utah County overall.\n")

    print("=== Income and house age: Provo-Orem vs county ===")
    print(compare(county, provo_orem).T.to_string(), "\n")

    scatter_plot([("Rest of Utah County", county[~county["in_provo_orem"]], "#9aa0a6"),
                  ("Provo-Orem", provo_orem, COLORS[0])],
                 "Provo-Orem vs the rest of Utah County\none dot per block group, stars mark each group's median",
                 DATA_DIR / "income_vs_house_age.png")
    # The Census names the cities "Provo city" and "Orem city".
    scatter_plot([(city, provo_orem[provo_orem["city"].str.startswith(city)], COLORS[i])
                  for i, city in enumerate(CITIES)],
                 "Provo and Orem block groups\none dot per block group, stars mark each city's median",
                 DATA_DIR / "income_vs_house_age_provo_orem.png")
    print("Scatter plots saved.\n")

    print("=== Clusters (income + house age), k chosen by silhouette ===")
    summaries = [cluster_summary(df, label)
                 for label, df in (("Provo-Orem", provo_orem), ("Whole county", county))]
    print(pd.concat(summaries).to_string(), "\n")

    importances, scores = forest(county, provo_orem)
    print("=== Random forest predicting median household income ===")
    print("How much each trait mattered (importances sum to 1):")
    print(importances.to_string(), "\n")
    print("How well each model predicts (R^2):")
    print(scores.to_string(), "\n")
    print(f"County table and plots saved in {DATA_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
