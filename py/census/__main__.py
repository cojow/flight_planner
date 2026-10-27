"""
The command line. Run from the py/ folder (or with py/ on PYTHONPATH):

    python -m census places                              # cities in Utah County
    python -m census new provo_orem --cities Provo Orem  # make a run with default settings
    python -m census show provo_orem                     # its settings and step status
    python -m census set provo_orem cluster.k=4 sample.per_cluster=1
    python -m census run provo_orem                      # every step, in order
    python -m census run provo_orem cluster sample       # just these steps

Runs live in data/census/runs/<name>/; config.json there holds every setting
and can also be edited by hand.
"""
import argparse
import json
import sys

from . import config as cfg
from . import geography, steps
from .sources import CensusError


def _set(config, assignment):
    """Apply "section.key=value"; value is read as JSON when it can be (numbers, lists), else as text."""
    path, _, raw = assignment.partition("=")
    section, _, key = path.partition(".")
    if not raw or section not in config or not isinstance(config[section], dict) or key not in config[section]:
        raise cfg.ConfigError(f"Not a setting: {path!r}. Use section.key=value; see 'show'.")
    try:
        value = json.loads(raw)
    except ValueError:
        value = raw
    config[section][key] = value


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m census", description="Census block-group sampling runs.")
    sub = ap.add_subparsers(dest="command", required=True)
    p = sub.add_parser("places", help="list the cities in the county")
    p.add_argument("--state", default=cfg.DEFAULTS["areas"]["state"])
    p.add_argument("--county", default=cfg.DEFAULTS["areas"]["county"])
    p = sub.add_parser("new", help="make a run")
    p.add_argument("name")
    p.add_argument("--cities", nargs="+", required=True, help='city names, e.g. Provo Orem "American Fork"')
    p = sub.add_parser("show", help="print a run's settings and step status")
    p.add_argument("name")
    p = sub.add_parser("set", help="change settings: section.key=value ...")
    p.add_argument("name")
    p.add_argument("assignments", nargs="+")
    p = sub.add_parser("run", help="run steps (default: all)")
    p.add_argument("name")
    p.add_argument("steps", nargs="*", help=f"any of {', '.join(steps.ORDER)}")
    args = ap.parse_args(argv)

    try:
        if args.command == "places":
            print(geography.county_places(args.state, args.county).to_string(index=False))
            return 0

        folder = cfg.run_dir(args.name)
        if args.command == "new":
            if (folder / "config.json").exists():
                raise cfg.ConfigError(f"A run called {args.name!r} already exists.")
            config = cfg.default_config(args.name)
            a = config["areas"]
            config["areas"]["places"] = [
                {"geoid": c["geoid"], "name": c["name"]}
                for c in (geography.place(a["state"], name=n) for n in args.cities)]
            cfg.save(folder, config)
            print(f"Made {folder}/config.json")
        elif args.command == "set":
            config = cfg.load(folder)
            for assignment in args.assignments:
                _set(config, assignment)
            cfg.save(folder, config)
        elif args.command == "show":
            print(json.dumps(cfg.load(folder), indent=2))
        elif args.command == "run":
            unknown = [s for s in args.steps if s not in steps.ORDER]
            if unknown:
                raise cfg.ConfigError(f"Unknown step(s) {unknown}; steps are {', '.join(steps.ORDER)}.")
            for step in args.steps or steps.ORDER:
                print(f"\n=== {step} ===")
                result = steps.RUNNERS[step](folder)
                print(result.report)
                print(f"Saved in {result.folder}")
        for step, state in steps.status(folder).items():
            print(f"  {step:<9} {state}")
    except (cfg.ConfigError, steps.StepError, CensusError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
