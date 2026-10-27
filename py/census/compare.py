"""
An optional report: do the chosen cities look like the rest of the county?

Reads step 1's county table and asks it three ways - averages, a scatter of
income against house age, and two random forests that predict a block
group's income from its housing (one trained on the chosen cities, one on the
whole county). If the forests lean on different traits, or the cities' forest
does badly on the rest of the county, the places work differently.

    python -m census compare provo_orem

Writes to the run's 1_areas/compare/ folder. Not needed by any other step.
"""
import pandas as pd
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import cross_val_score

from . import plots
from .config import STEPS
from .pipeline import compare_with_county

# What the forests may use to predict income. House age is one of them: the
# forest shows how much it matters on its own.
PREDICTORS = ["median_age_years_all", "avg_household_size", "median_rooms_all",
              "pct_single_family", "pct_multi_family",
              "pct_built_since_2000", "pct_built_before_1980"]
TARGET = "median_household_income"


def forests(county):
    """
    (importances table, R^2 scores). R^2 is 1.0 for a perfect prediction and 0
    for no better than guessing the average. The cities' forest is also scored
    on the rest of the county, which it never saw - the real test of "is it
    different".
    """
    county = county[county["housing_units"].fillna(0) > 0].dropna(subset=[TARGET, *PREDICTORS])
    cities, rest = county[county["in_cities"]], county[~county["in_cities"]]

    def model():
        # One thread: with a few hundred rows, starting worker threads costs more than it saves.
        return RandomForestRegressor(n_estimators=300, min_samples_leaf=3, random_state=0, n_jobs=1)

    city_model = model().fit(cities[PREDICTORS], cities[TARGET])
    county_model = model().fit(county[PREDICTORS], county[TARGET])
    importances = pd.DataFrame({"Chosen cities": city_model.feature_importances_,
                                "Whole county": county_model.feature_importances_},
                               index=PREDICTORS).sort_values("Whole county", ascending=False).round(3)
    scores = pd.Series({
        "Cities model, on the cities (5-fold CV)":
            cross_val_score(model(), cities[PREDICTORS], cities[TARGET], cv=5, scoring="r2").mean(),
        "County model, on the whole county (5-fold CV)":
            cross_val_score(model(), county[PREDICTORS], county[TARGET], cv=5, scoring="r2").mean(),
        "Cities model, on the rest of the county (never seen)":
            city_model.score(rest[PREDICTORS], rest[TARGET]),
    }, name="R^2").round(3)
    return importances, scores


def scatter(county, path):
    """One dot per block group, the chosen cities against the rest of the county."""
    usable = county[county["housing_units"].fillna(0) > 0].dropna(
        subset=["median_age_years_all", "median_household_income"])
    fig, ax = plots.plt.subplots(figsize=(9, 6))
    for label, df, c in (("Rest of the county", usable[~usable["in_cities"]], "#9aa0a6"),
                         ("Chosen cities", usable[usable["in_cities"]], plots.COLORS[0])):
        ax.scatter(df["median_age_years_all"], df["median_household_income"], s=30, alpha=0.65,
                   color=c, edgecolors="none", label=f"{label} ({len(df)})")
        ax.scatter(df["median_age_years_all"].median(), df["median_household_income"].median(),
                   s=260, marker="*", color=c, edgecolors="black", linewidths=1, zorder=5)
    plots.style_axes(ax, "median_age_years_all", "median_household_income",
                     "Chosen cities vs the rest of the county\none dot per block group, stars mark each median")
    ax.legend(frameon=False)
    return plots.save(fig, path)


def report(folder):
    """Write the comparison into 1_areas/compare/ and return its text."""
    areas = folder / STEPS["areas"]["folder"]
    county = pd.read_csv(areas / "county_block_groups.csv", dtype={"geoid": str})
    out = areas / "compare"
    out.mkdir(exist_ok=True)
    averages = compare_with_county(county)
    importances, scores = forests(county)
    averages.to_csv(out / "averages.csv")
    importances.to_csv(out / "forest_importances.csv")
    scores.to_csv(out / "forest_scores.csv")
    scatter(county, out / "income_vs_house_age.png")
    return ("Income and house age:\n" + averages.to_string() +
            "\n\nRandom forest predicting median household income - how much each trait mattered:\n"
            + importances.to_string() + "\n\nHow well each forest predicts (R^2):\n" + scores.to_string()
            + f"\n\nSaved in {out}\n")
