# Luxembourg Cropping Pipeline

A set of scripts to spatially and temporally crop the LuST (Luxembourg SUMO
Traffic) scenario into a network folder directly usable by RouteRL / URB
(COeXISTENCE project). Includes a path-availability filter and a JanuX hang
detector so the path generation step never runs forever.

The pipeline produces a URB-compatible network folder:

```
<network>/
├── <network>.net.xml
├── <network>.edg.xml
├── <network>.nod.xml
├── <network>.con.xml
├── <network>.tll.xml
├── <network>.typ.xml            (optional, derived)
├── <network>.netccfg
├── <network>.rou.xml            ← copy of <network>.net.xml (URB convention)
├── human_demand.rou.xml         ← actual vehicle demand
├── background_buses.rou.xml
├── agents.csv                   id,start_time,origin,destination,kind
├── demand_routes.csv            agent_id,source_id,route_id,probability,free_flow_time,edges
├── od_<network>.txt             {'origins': [...], 'destinations': [...]}
└── metadata.json
```

`<network>.rou.xml` is NOT a demand file — it is a byte-for-byte copy of
`<network>.net.xml`, as required by JanuX (URB's path generator). The human
demand lives in `human_demand.rou.xml`.

---

## 1. Requirements

### Software

- Python >= 3.10
- SUMO >= 1.20 (`SUMO_HOME` must be set)
- LuST scenario (full, including `lust.net.xml`)

### Python dependencies

```
networkx>=2.8
pandas>=1.5
matplotlib>=3.5
janux
```

Install with:

```bash
pip install -r requirements.txt
```

### LuST data files required

Place your LuST scenario under `LuSTScenario/scenario/`:

| File                                        | Role                                          |
| ------------------------------------------- | --------------------------------------------- |
| `lust.net.xml`                              | **Source network for the crop**               |
| `DUERoutes/local.static.{0,1,2}.rou.xml`    | Human demand                                  |
| `transit.rou.xml`                           | Transit vehicles (merged into the agent pool) |
| `buslines.rou.xml`, `busses.rou.xml`        | Background traffic                            |
| `vtypes.add.xml`                            | Vehicle types                                 |
| `busstops.add.xml`                          | Bus stops                                     |
| `tll.static.xml`                            | Static traffic-light programs                 |
| `due.static.vehroute.until9.unfinished.xml` | **Reference exit-times (recommended)**        |
| `due.static.until9.tripinfo.xml`            | Reference trip durations (recommended)        |

If the two reference files are missing, the pipeline still works but falls
back to free-flow × congestion-factor estimation (less accurate for entry
times at the crop boundary).

---

## 2. Directory layout

````
project/
├── build_lux_crop.py
├── check_janux_od.py
├── apply_bad_od_filter.py
├── try_crops.py
├── README.md                    ← this file
├── requirements.txt
├── .gitignore
├── my_windows.jsonl             ← example for try_crops.py
├── LuSTScenario/
│   └── scenario/
│       ├── lust.net.xml
│       ├── DUERoutes/local.static.*.rou.xml
│       ├── transit.rou.xml
│       ├── buslines.rou.xml, busses.rou.xml, trams.rou.xml, trains.rou.xml
│       ├── vtypes.add.xml
│       ├── busstops.add.xml
│       ├── tll.static.xml
│       ├── due.static.vehroute.until9.unfinished.xml
│       └── due.static.until9.tripinfo.xml

---

## 3. Workflow

The typical workflow is:

1. **(Optional) pick a window** with `try_crops.py --visualize-only` or add a few windows
2. **Build the crop** with `build_lux_crop.py` or with try_crops.py (detailed later)
3. **Detect JanuX bugs** with `check_janux_od.py`
4. **Filter the bad ODs** with `apply_bad_od_filter.py`
5. Copy the folder to `URB/networks/<network>/`
6. Run your URB algorithm

---

## 4. `build_lux_crop.py` — main pipeline

Spatially and temporally crops LuST into a URB-compatible network folder.

### Basic usage

```bash
python build_lux_crop.py \
    --geo 7700 8600 4600 5500 \
    --t0 7.75 --t1 8.50 \
    --network-name luxembourg_south_bonnevoie \
    --run-sumo
````

### What it does

1. Loads human demand from `LuSTScenario/scenario/` between
   `t0 - preload-hours` and `t1`.
2. Sanitizes the source `.net.xml`: strip the anomalous `--` prefix from
   edge IDs (a LuST-specific artefact of multi-step OSM → SUMO conversion).
   Edge IDs like `--30694#2` become `30694#2`, matching the standard
   convention used by other URB networks (ingolstadt, nangis, etc.).
3. Spatially crops the network with `netconvert --keep-edges.input-file`.
4. Optionally bakes static TLS programs into the cropped `.net.xml`.
5. Segments each human trajectory into contiguous runs over the cropped
   edges, and estimates the entry time in the crop using this chain:
   `vehroute` (exact exit-times) → `prop_ff` (tripinfo-based proportional)
   → `ff_global` (free-flow × K_GLOBAL) → `ff` (pure free-flow).
6. Optionally filters out OD pairs that do not have enough distinct paths
   in the cropped network (see §4.4).
7. Exports the URB files (`agents.csv`, `demand_routes.csv`, `od_*.txt`,
   `<network>.rou.xml` as a copy of the network, `human_demand.rou.xml`,
   `background_buses.rou.xml`).
8. Runs SUMO headless (optional) and prints a validation summary.

### 4.1 Spatial and temporal parameters

| Parameter                   | Default               | Description                           |
| --------------------------- | --------------------- | ------------------------------------- |
| `--geo XMIN XMAX YMIN YMAX` | `6500 9500 4200 7100` | Geographic window (SUMO coordinates)  |
| `--t0`                      | `7.0`                 | Window start (hours)                  |
| `--t1`                      | `9.0`                 | Window end (hours)                    |
| `--preload-hours`           | `1.0`                 | Load demand from `t0 - preload_hours` |

**Preload**: by default it loads 1 hour before `t0` so that vehicles
that started their LuST journey before `t0` but enter the crop after `t0`
are still captured. The default `--preload-hours 1.0` is safe for the
default `t0=7h` (loads from 6h).

### 4.2 Entry-time estimators

| `--entry-estimator`  | Coverage | Accuracy      | Notes                                                        |
| -------------------- | -------- | ------------- | ------------------------------------------------------------ |
| `vehroute` (default) | ~92 %    | Exact         | Uses exit-times from the reference simulation(vehroute file) |
| `prop_ff`            | ~73 %    | ±4 s median   | Uses tripinfo durations (tripinfo file)                      |
| `ff_global`          | 100 %    | ±14 s median  | Free-flow × K_GLOBAL                                         |
| `ff`                 | 100 %    | −106 s median | Free-flow only (baseline, not recommended)                   |

The chain is automatic: if a vehicle is not in the primary source, the
pipeline falls back to the next. You only need to set the highest-priority
estimator you want to use. (Vehroute by default)

### 4.3 Insertion presets for vehicles entering from outside

| `--outside-insertion`                                        | Attributes                                                 | Notes                   |
| ------------------------------------------------------------ | ---------------------------------------------------------- | ----------------------- |
| `base_desired_best` (default)                                | `departLane=best`, `departPos=base`, `departSpeed=desired` | Best measured trade-off |
| `randfree_0_best`                                            | `departPos=random_free`, `departSpeed=0.00`                | Robust alternative      |
| `last_desired_best`                                          | `departPos=last`, `departSpeed=desired`                    | Older behavior          |
| `free_desired_best`, `randfree_desired_best`, `last_desired` | Variants                                                   | For comparison          |

This presets are written in the sumo .cfg file that is generated by the pipeline, but this cfg file is not used by RouteRl, so this presets will be overwritten when the network and demand are used in Urb or Routerl. (It was used to test how we can lower the number of teleports)

### 4.4 Path availability filter

JanuX generates a fixed number of paths per OD pair. When an OD pair has
fewer distinct paths than the requested number, JanuX can hang. The pipeline
uses `networkx` to pre-check how many distinct paths exist per OD pair, and
drops the ones below a threshold.

| Parameter            | Default | Description                                             |
| -------------------- | ------- | ------------------------------------------------------- |
| `--min-paths`        | `6`     | Minimum distinct paths per OD pair                      |
| `--max-paths-probe`  | `12`    | Max paths to probe before giving up                     |
| `--path-similarity`  | `0.85`  | Jaccard similarity above which two paths are "the same" |
| `--path-timeout`     | `2.0`   | Max seconds probing a single OD pair                    |
| `--skip-path-filter` | `False` | Disable the filter                                      |

The filter works on the cropped network's `.con.xml`, so it matches what
JanuX sees. If it drops too many agents (> 30 %), you can lower
`--min-paths` to 3-4, or switch to clustered routes (see §9).

### 4.5 Traffic lights

| `--tls-mode`       | Behavior                                                                                                              |
| ------------------ | --------------------------------------------------------------------------------------------------------------------- |
| `static` (default) | `tll.static.xml` programs baked **into** the cropped `.net.xml`, but only when the number of controlled links matches |
| `original`         | Keep the actuated programs produced by netconvert                                                                     |

### 4.6 URB post-processing

| Parameter           | Default | Description                                              |
| ------------------- | ------- | -------------------------------------------------------- |
| `--vtypes-mode`     | `merge` | Inline vTypes into demand files, delete `vtypes.add.xml` |
| `--busstops-mode`   | `strip` | Remove `<stop>` from buses, delete `busstops.add.xml`    |
| `--extract-typ`     | `True`  | Extract `<type>` definitions into `<network>.typ.xml`    |
| `--write-metadata`  | `True`  | Write `metadata.json`                                    |
| `--cleanup-outputs` | `True`  | Move SUMO outputs to `_validation/`                      |
| `--keep-sumocfg`    | `False` | Keep `custom.sumocfg` in the main folder                 |

### 4.7 SUMO validation

| Parameter           | Effect                                                |
| ------------------- | ----------------------------------------------------- |
| `--run-sumo`        | Run SUMO headless, print warnings + teleports summary |
| `--sumo-gui`        | Launch `sumo-gui` on the generated config (blocking)  |
| `--skip-validation` | Skip the post-generation file checks                  |

### 4.8 Output files

**Network** (used by RouteRL and JanuX):

- `<network>.net.xml` — cropped network with baked TLS
- `<network>.edg.xml`, `<network>.nod.xml`, `<network>.con.xml`, `<network>.tll.xml` — plain-XML outputs of netconvert, required by JanuX
- `<network>.typ.xml` — edge-type definitions (derived)
- `<network>.netccfg` — netconvert config

**Demand** (RouteRL conventions):

- `<network>.rou.xml` — **copy of `<network>.net.xml`**, required by JanuX
- `human_demand.rou.xml` — actual human vehicle demand (vTypes inlined)
- `background_buses.rou.xml` — filtered buses (background only)
- `agents.csv` — `id,start_time,origin,destination,kind` (id is 0..N-1, start_time is integer)
- `demand_routes.csv` — `agent_id,source_id,route_id,probability,free_flow_time,edges`
- `od_<network>.txt` — pretty-printed JSON with origins and destinations
- `metadata.json` — summary of the crop

**Validation** (in `_validation/` if `--cleanup-outputs`):

- `custom.sumocfg`, `stats.xml`, `summary.xml`, `tripinfo.xml`, `sumo_*.log`

---

## 5. `check_janux_od.py` — detect JanuX hangs

JanuX sometimes hangs when an OD pair has fewer distinct paths than the
requested `number_of_paths`. This script probes each used OD pair with a
wall-clock timeout and reports the ones that block.

### Usage

```bash
# Probe the current network with the test config values
python check_janux_od.py \
    --network-dir "URB/networks/luxembourg_south_bonnevoie" \
    --num-paths 4 --num-samples 10 \
    --timeout 5
```

### What it does

1. Reads `agents.csv` to extract the OD pairs actually used.
2. Pre-filters with `networkx.has_path` (drops disconnected pairs).
3. For each remaining pair, runs `jx.extended_generator` in a subprocess
   with a `--timeout` seconds budget.
4. Writes `bad_od.json` with the list of pairs that timed out or errored.

### Output

```
=== Summary ===
  Pairs from agents.csv  : 477
  Skipped (no path)      : 0
  Skipped (missing node) : 0
  Sent to JanuX          : 477
    ok                   : 456
    timeout              : 21
    error                : 0
  Total time             : 1251.0s
```

`bad_od.json` structure:

```json
{
  "bad_od_pairs": [["-30292#0", "-30292#1"]],
  "timeout_od_pairs": [],
  "error_od_pairs": [],
  "nopath_od_pairs": [],
  "missing_node_od_pairs": []
}
```

### When to use it

Run it once after `build_lux_crop.py` if the path generation in URB takes
more than 10-15 minutes, or hangs forever at 0 %. If the number of timed-out
OD pairs is small (< 10 %), filter them with `apply_bad_od_filter.py` and
rerun. If it is large (> 30 %), consider switching to **clustered routes**
(`iql_arh.py` with `--route-set`, as ingolstadt21 does).

---

## 6. `apply_bad_od_filter.py` — remove problematic OD pairs

Takes `bad_od.json` produced by `check_janux_od.py` and rewrites
`agents.csv`, `demand_routes.csv`, and `od_<network>.txt` to drop all agents
whose (origin, destination) pair failed in JanuX. Renumbers remaining agents
from 0 to N-1 so `agent_id` stays clean.

### Usage

```bash
# Dry-run: shows how many agents would be dropped, without touching files
python apply_bad_od_filter.py --network-dir "URB/networks/luxembourg_south_bonnevoie"

# Apply: overwrites files (backups saved as *.bak)
python apply_bad_od_filter.py --network-dir "URB/networks/luxembourg_south_bonnevoie" --apply
```

### What it does

1. Loads `bad_od.json`.
2. Reads `agents.csv` and maps `origin` / `destination` indices to edge IDs
   via `od_<network>.txt`.
3. Drops every agent whose (origin, destination) is in the bad list.
4. Rewrites `agents.csv`, `demand_routes.csv`, and `od_<network>.txt` with
   consistent renumbering.

### Output

```
=== Filter report ===
  Agents before    : 2959
  Agents dropped   : 87
  Agents remaining : 2872
  Drop ratio       : 2.9%

  Drop breakdown by bad pair (top 10):
    -30528#24 -> -30528#30: 8 agents
```

If the drop ratio is < 10 %, apply directly. If it is > 30 %, consider
switching to clustered routes instead (see §9).

---

## 7. `try_crops.py` — visualize and compare candidate windows

Draws several candidate crop windows on the full LuST map (with traffic
lights and highway edges highlighted) and, optionally, runs the whole
pipeline for each window to get the actual agent count.

### Usage

```bash
# Only draw the candidate windows on the map
python try_crops.py --visualize-only

# Run the full pipeline for each window, plus headless SUMO validation
python try_crops.py --run-pipeline --run-sumo --clean-builds

# Custom windows from a .jsonl file
python try_crops.py --run-pipeline --windows-file my_windows.jsonl
```

### Custom windows

Create `my_windows.jsonl` (one JSON object per line):

```jsonl
{"name": "center_A", "xmin": 7800, "xmax": 8600, "ymin": 5400, "ymax": 6200, "t0": 7.75, "t1": 8.5}
{"name": "east_B",   "xmin": 9000, "xmax": 9800, "ymin": 7000, "ymax": 7800, "description": "highway + urban"}
{"name": "south_C",  "xmin": 5300, "xmax": 10500, "ymin": 1500, "ymax": 4200, "preload_hours": 0.5}
```

Fields: `name`, `xmin`, `xmax`, `ymin`, `ymax` (required); `t0`, `t1`,
`preload_hours`, `description` (optional).

### Outputs

- `crop_tests/_overview.png` — all windows on the full map
- `crop_tests/<NN>_<name>.png` — 2-panel view per window (full + zoom)
- `crop_tests/<name>.log` — full pipeline stdout for that window
- `luxembourg_custom_<coords>/` — one folder per window (when `--run-pipeline`)

### Adding a window to the defaults

Edit `default_candidates()` in `try_crops.py`. Each entry:

```python
CropCandidate(
    name="my_zone",
    xmin=7800, xmax=8600, ymin=5400, ymax=6200,
    t0=7.75, t1=8.5,
    description="short description",
)
```

---

## 8. Typical end-to-end example

```bash
# 1. Pick a window
python try_crops.py --visualize-only
# → look at crop_tests/_overview.png and pick an area

# 2. Build the crop
python build_lux_crop.py \
    --geo 7700 8600 4600 5500 \
    --t0 7.75 --t1 8.50 \
    --network-name luxembourg_south_bonnevoie \
    --run-sumo
# → produces luxembourg_south_bonnevoie/ with ~2900 agents

# 3. (Recommended) detect JanuX hangs
python check_janux_od.py \
    --network-dir "luxembourg_south_bonnevoie" \
    --num-paths 4 --num-samples 10 --timeout 5
# → produces bad_od.json

# 4. (Recommended) filter the bad ODs
python apply_bad_od_filter.py --network-dir "luxembourg_south_bonnevoie"
# → dry-run report: how many agents would be dropped
python apply_bad_od_filter.py --network-dir "luxembourg_south_bonnevoie" --apply
# → overwrites agents.csv, demand_routes.csv, od_*.txt

# 5. Copy to URB
cp -r luxembourg_south_bonnevoie URB/networks/

# 6. Run URB
cd URB/scripts
python iql.py --id example --env-conf example --alg-conf example \
              --task-conf example --net luxembourg_south_bonnevoie \
              --skip-metrics
```

---

## 9. Important: `vehroute.until9.unfinished.xml`

The reference simulation runs from t=0 to t=9h (32400 s). Its output files
are `due.static.vehroute.until9.unfinished.xml` and
`due.static.until9.tripinfo.xml`:

| File                             | Contents                                                                                | Coverage                                    |
| -------------------------------- | --------------------------------------------------------------------------------------- | ------------------------------------------- |
| `tripinfo.until9.xml`            | Vehicles that **arrived** before 9h                                                     | ~73 % of the demand loaded for a 7h-9h crop |
| `vehroute.until9.unfinished.xml` | Vehicles that were **inserted** before 9h, including those still running (`-1` markers) | ~92 % of the demand loaded for a 7h-9h crop |

**Why not 100 %?** ~5 000 vehicles were never inserted (SUMO
`max-depart-delay=600` queueing). They appear in neither file.

**Is the crop window well covered?** For a crop window of 7h45-8h30, yes:
all vehicles entering the crop between 7h45 and 8h30 entered the reference
simulation before 9h, so their entry edge is recorded in `vehroute`. The
coverage drops slightly towards 8h45-9h (down to ~88 %) because those
vehicles had less time to reach the crop entry edge before the reference
ended.

**If you need a crop window that ends after 9h**, regenerate the reference
with `--end 36000` (10h) or higher. The filenames can stay the same (they
are just conventions); what matters is the `<end>` value in the sumocfg
that produced them.

---

## 10. Troubleshooting

### "Path generation takes forever"

Run `check_janux_od.py`. If more than 30 % of OD pairs time out, switch to
**clustered routes**. If only a few pairs (< 10 %) time out, filter them
with `apply_bad_od_filter.py` and rerun.

### "AssertionError: Origin X is not in the network" (JanuX)

The `<network>.rou.xml` must be a copy of `<network>.net.xml`, not a
demand file. Check that `build_lux_crop.py` produced it correctly (look at
`postprocess_for_urb` in the code, section "URB convention").

### "Connection closed by SUMO" in URB

You launched a GUI SUMO which was closed manually. Set
`"sumo_type": "sumo"` in the script and relaunch.

### "Simulation hangs at 0 %"

Check `simulation_timesteps` in the URB script. It must be at least
`max_start_time + 60` where `max_start_time` is the last `start_time` in
`agents.csv`.

### "Agents have start_time as float"

The pipeline now rounds `start_time` to integer. If you have an old
`agents.csv`, round the column manually with `pandas`:

```python
import pandas as pd
df = pd.read_csv("agents.csv")
df["start_time"] = df["start_time"].round().astype(int)
df.to_csv("agents.csv", index=False)
```

### "No buslines.rou.xml in the output"

That means the background filter dropped all buses (none intersect the
crop). Not an error, just an empty background.

---

## 11. What `build_lux_crop.py` deliberately does NOT do

- No matrix of `run_experiment` variants (this is a pipeline, not a benchmark).
- No insertion presets that were already rejected (only the surviving ones).
- No automatic `sumo-gui` launch (`--sumo-gui` is opt-in).
- No silent deletion of the output directory (unless `--skip-keep`).
- No generation of `tll.static.crop.xml` as an additional file — the
  `--tls-mode static` path replaces programs _inside_ the `.net.xml`,
  which is the only method that was validated to work.

---

## 12. FAQ

**Q: Can I run the crop on a different rectangle?**
Yes — `--geo XMIN XMAX YMIN YMAX`. The output folder name changes
accordingly (`luxembourg_custom_<coords>`), so successive runs do not
overwrite each other.

**Q: Why does `agents.csv` include transit vehicles?**
On a different crop, transit vehicles are legitimate traffic that RouteRL
may mutate into AVs. They are merged into the human pool and appear in
`agents.csv` with the same `source` field.

**Q: Why are buses excluded from `agents.csv`?**
Buses are background traffic and not part of the AV mutation pool. They go
into `background_buses.rou.xml`.

**Q: What does the "sanitization" step do?**
LuST has anomalous edge IDs like `--30694#2` (double leading dash) that
break JanuX. The sanitizer rewrites them to `30694#2` in the source
`.net.xml` before cropping. This is applied consistently to the network,
the demand, and the reference exit-times so nothing breaks.

**Q: What if I want to keep the `--` prefix?**
You can't — JanuX cannot parse it. The sanitization is required.

**Q: What if I don't want the OD path filter?**
Add `--skip-path-filter` to `build_lux_crop.py`. The agents for all OD pairs
are kept, but the JanuX path generation may hang on the problematic ones.
Use `check_janux_od.py` + `apply_bad_od_filter.py` instead to only drop the
ones that actually fail.

---

## 13. License

The LuST scenario is MIT-licensed (see LuSTScenario/scenario/). The scripts
in this repository are released under the same license.
