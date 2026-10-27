#!/usr/bin/env python3
"""
apply_bad_od_filter.py — Remove agents whose (origin, destination) pair
failed in JanuX (i.e. present in bad_od.json).

Reads:
    bad_od.json               from check_janux_od.py
    agents.csv                id,start_time,origin,destination,kind
    demand_routes.csv         agent_id,source_id,route_id,probability,free_flow_time,edges
    od_<network>.txt          {'origins': [...], 'destinations': [...]}

Writes (only with --apply):
    agents.csv                renumbered 0..N-1
    demand_routes.csv         renumbered
    od_<network>.txt          filtered

Backups of the three overwritten files are saved as <file>.bak.
"""

import argparse
import ast
import json
import os
import shutil
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--network-dir", required=True,
                    help="Folder with agents.csv, demand_routes.csv, "
                         "od_<net>.txt, and bad_od.json")
    ap.add_argument("--net-name", default=None,
                    help="Base name (default: folder name)")
    ap.add_argument("--bad-od", default=None,
                    help="Path to bad_od.json")
    ap.add_argument("--apply", action="store_true",
                    help="Actually overwrite files (default: dry-run)")
    args = ap.parse_args()

    folder = os.path.abspath(args.network_dir)
    name = args.net_name or os.path.basename(folder)

    bad_path    = args.bad_od or os.path.join(folder, "bad_od.json")
    agents_path = os.path.join(folder, "agents.csv")
    demand_path = os.path.join(folder, "demand_routes.csv")
    od_path     = os.path.join(folder, f"od_{name}.txt")

    for p in (bad_path, agents_path, demand_path, od_path):
        if not os.path.exists(p):
            print(f"[error] not found: {p}")
            return 2

    # --- Load bad OD pairs -----------------------------------------------
    with open(bad_path, "r", encoding="utf-8") as f:
        bad_data = json.load(f)

    bad = set()
    for key in ("bad_od_pairs", "timeout_od_pairs", "error_od_pairs"):
        for entry in bad_data.get(key, []):
            if isinstance(entry, (list, tuple)) and len(entry) == 2:
                bad.add((str(entry[0]), str(entry[1])))
    print(f"  bad OD pairs loaded: {len(bad)}")

    # --- Load od edges ---------------------------------------------------
    with open(od_path, "r", encoding="utf-8") as f:
        od_data = ast.literal_eval(f.read())
    od_origins      = od_data["origins"]
    od_destinations = od_data["destinations"]

    # --- Read agents.csv -------------------------------------------------
    with open(agents_path, "r", encoding="utf-8") as f:
        agent_lines = f.read().splitlines()
    header_agents = agent_lines[0].split(",")

    try:
        col_id     = header_agents.index("id")
        col_origin = header_agents.index("origin")
        col_dest   = header_agents.index("destination")
    except ValueError as e:
        print(f"[error] agents.csv header missing column: {e}")
        return 2

    kept_rows   = []
    drop_reasons = {}

    for line in agent_lines[1:]:
        if not line.strip():
            continue
        cols = line.split(",")
        try:
            o_idx = int(cols[col_origin])
            d_idx = int(cols[col_dest])
        except (ValueError, IndexError):
            continue
        o_edge = str(od_origins[o_idx])
        d_edge = str(od_destinations[d_idx])
        if (o_edge, d_edge) in bad:
            drop_reasons[(o_edge, d_edge)] = drop_reasons.get((o_edge, d_edge), 0) + 1
        else:
            kept_rows.append((cols, o_edge, d_edge))

    n_before  = len(agent_lines) - 1
    n_kept    = len(kept_rows)
    n_dropped = n_before - n_kept

    print(f"\n=== Filter report ===")
    print(f"  Agents before    : {n_before}")
    print(f"  Agents dropped   : {n_dropped}")
    print(f"  Agents remaining : {n_kept}")
    if n_before > 0:
        print(f"  Drop ratio       : {100.0 * n_dropped / n_before:.1f}%")

    print(f"\n  Drop breakdown by bad pair (top 10):")
    for (o, d), cnt in sorted(drop_reasons.items(), key=lambda kv: -kv[1])[:10]:
        print(f"    {o} -> {d}: {cnt} agents")

    if not args.apply:
        print(f"\n[DRY RUN] Re-run with --apply to overwrite files.")
        return 0

    # --- Rebuild od with only used edges ---------------------------------
    used_o = set(); used_d = set()
    for _, o_edge, d_edge in kept_rows:
        used_o.add(o_edge); used_d.add(d_edge)
    new_origins      = sorted(used_o)
    new_destinations = sorted(used_d)
    new_o_idx = {e: i for i, e in enumerate(new_origins)}
    new_d_idx = {e: i for i, e in enumerate(new_destinations)}

    # --- Write new agents.csv --------------------------------------------
    shutil.copy2(agents_path, agents_path + ".bak")
    with open(agents_path, "w", encoding="utf-8") as f:
        f.write(",".join(header_agents) + "\n")
        for new_id, (cols, o_edge, d_edge) in enumerate(kept_rows):
            out = list(cols)
            out[col_id]     = str(new_id)
            out[col_origin] = str(new_o_idx[o_edge])
            out[col_dest]   = str(new_d_idx[d_edge])
            f.write(",".join(out) + "\n")

    # --- Renumber demand_routes.csv --------------------------------------
    with open(demand_path, "r", encoding="utf-8") as f:
        demand_lines = f.read().splitlines()
    header_demand = demand_lines[0].split(",")
    col_agent = header_demand.index("agent_id")

    kept_old_ids_order = [int(cols[col_id]) for cols, _, _ in kept_rows]
    old_to_new = {old_id: new_id
                  for new_id, old_id in enumerate(kept_old_ids_order)}

    shutil.copy2(demand_path, demand_path + ".bak")
    with open(demand_path, "w", encoding="utf-8") as f:
        f.write(",".join(header_demand) + "\n")
        for line in demand_lines[1:]:
            if not line.strip():
                continue
            cols = line.split(",", maxsplit=len(header_demand) - 1)
            old_id = int(cols[col_agent])
            if old_id not in old_to_new:
                continue
            cols[col_agent] = str(old_to_new[old_id])
            f.write(",".join(cols) + "\n")

    # --- Write new od file -----------------------------------------------
    shutil.copy2(od_path, od_path + ".bak")
    with open(od_path, "w", encoding="utf-8") as f:
        json.dump({"origins": new_origins,
                   "destinations": new_destinations}, f, indent=2)
        f.write("\n")

    print(f"\n[APPLIED] Files overwritten (backups saved as *.bak):")
    print(f"  agents.csv")
    print(f"  demand_routes.csv")
    print(f"  od_{name}.txt")
    print(f"\n  New origins      : {len(new_origins)}")
    print(f"  New destinations : {len(new_destinations)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())