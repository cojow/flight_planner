# Published census runs

Each folder here is one Census sampling run, published with `uv run census publish <name>` so that anyone can rerun
it and check they get the same results:

```bash
uv sync --frozen
```
```bash
uv run census fetch-inputs <name>
```
```bash
uv run census reproduce <name>
```

Each folder holds:

- **`config.json`**: the run's settings.
- **`manifest.json`**: the code and package versions, every raw input with its SHA-256, where the inputs archive is,
  and the SHA-256 of each result file.
- **`expected/`**: the result tables to compare against.

See [py/census/README.md](../py/census/README.md) for the details.
