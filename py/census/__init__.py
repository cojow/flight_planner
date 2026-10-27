"""
Census block groups for drone sampling: pick cities, cluster their block
groups on Census numbers, and draw the sample of block groups and streets to fly.

    python -m census new provo_orem --cities Provo Orem
    python -m census run provo_orem            # all four steps
    streamlit run census_app.py                # the same, with a GUI

See config.py for every setting and steps.py for what each step writes.
"""
