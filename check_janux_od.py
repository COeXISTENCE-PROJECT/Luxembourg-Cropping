#!/usr/bin/env python3
"""
check_janux_od.py — Detect OD pairs on which JanuX hangs.

Reads agents.csv (or falls back to the full cartesian product if absent) to
probe only the (origin, destination) pairs that the demand actually uses.

Before calling JanuX, each pair is validated with networkx:
  - origin and destination must be in the graph
  - a path must exist (nx.has_path)
Only then is JanuX launched, in a subprocess with a wall-clock timeout.

Windows-compatible replacement for the signal.SIGALRM approach used in
COeXISTENCE-PROJECT/extract_resco_demand.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import sys
import time
from multiprocessing import Process, Queue


# =============================================================================
# JanuX worker (child process)
# =============================================================================

def _janux_worker(network, origin, destination, num_paths, num_samples,
                  beta, result_queue):
    try:
        import janux as jx
        kwargs = {
            "number_of_paths": num_paths,
            "random_seed": 42,
            "num_samples": num_samples,
            "beta": beta,
            "weight": "time",
            "verbose": False,
        }
        routes = jx.extended_generator(
            network, [origin], [destination],
            as_df=True, calc_free_flow=True, **kwargs,
        )
        n = 0 if routes is None else len(routes)
        result_queue.put({"status": "ok", "n_paths": n})
    except Exception as e:
        result_queue.put({"status": "error",
                          "error": f"{type(e).__name__}: {e}"})


def run_with_timeout(fn, timeout_s, *args, **kwargs):
    q = Queue()
    p = Process(target=fn, args=(*args, q), kwargs=kwargs)
    p.start()
    p.join(timeout_s)
    if p.is_alive():
        p.terminate()
        p.join(1.0)
        if p.is_alive():
            p.kill()
        return {"status": "timeout"}
    if not q.empty():
        return q.get()
    return {"status": "error", "error": "no result from child"}


# =============================================================================
# OD pairs extraction
# =============================================================================

def _is_int_like(x) -> bool:
    try:
        int(x)
        return True
    except (TypeError, ValueError):
        return False


def load_used_od_pairs(folder: str, od_path: str):
    """
    Return the list of (origin_edge, destination_edge) pairs actually used
    by the demand, based on agents.csv.

    Two formats are supported:
      - new: id,start_time,origin,destination,kind         (origin = int)
      - old: agent_id,id,start_time,origin,destination,...  (origin = edge)

    If agents.csv is missing or unrecognized, falls back to the cartesian
    product of origins x destinations from od_*.txt.
    """
    import pandas as pd

    agents_path = os.path.join(folder, "agents.csv")
    with open(od_path, "r", encoding="utf-8") as f:
        od_data = ast.literal_eval(f.read())
    od_origins = od_data["origins"]
    od_destinations = od_data["destinations"]

    if not os.path.exists(agents_path):
        print(f"  [warn] agents.csv not found, falling back to cartesian product")
        pairs = [(o, d) for o in od_origins for d in od_destinations]
        return pairs, "cartesian"

    df = pd.read_csv(agents_path)
    if "origin" not in df.columns or "destination" not in df.columns:
        print(f"  [warn] agents.csv has no origin/destination columns, "
              f"falling back to cartesian product")
        pairs = [(o, d) for o in od_origins for d in od_destinations]
        return pairs, "cartesian"

    # Detect format: int indices (new) vs edge IDs (old)
    sample_origins = df["origin"].dropna().head(20).tolist()
    is_new_format = all(_is_int_like(x) for x in sample_origins)

    if is_new_format:
        print(f"  agents.csv format: new (origin/destination are indices)")
        used = set()
        for _, row in df.iterrows():
            try:
                o = od_origins[int(row["origin"])]
                d = od_destinations[int(row["destination"])]
            except (KeyError, IndexError, ValueError):
                continue
            used.add((o, d))
        return sorted(used), "agents"

    # Old format: origin/destination already hold edge IDs
    print(f"  agents.csv format: old (origin/destination are edge IDs)")
    used = set()
    for _, row in df.iterrows():
        o = str(row["origin"])
        d = str(row["destination"])
        if o and d:
            used.add((o, d))
    return sorted(used), "agents"


# =============================================================================
# Main
# =============================================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--network-dir", required=True)
    ap.add_argument("--net-name", default=None)
    ap.add_argument("--od-file", default=None)
    ap.add_argument("--num-paths", type=int, default=4)
    ap.add_argument("--num-samples", type=int, default=10)
    ap.add_argument("--beta", type=float, default=-5.0)
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="Seconds before killing a single OD generation")
    ap.add_argument("--max-od", type=int, default=None)
    ap.add_argument("--out-json", default=None)
    ap.add_argument("--cartesian", action="store_true",
                    help="Ignore agents.csv, probe the full origins x "
                         "destinations product (slow)")
    args = ap.parse_args()

    folder = os.path.abspath(args.network_dir)
    name = args.net_name or os.path.basename(folder)

    conn = os.path.join(folder, f"{name}.con.xml")
    edg  = os.path.join(folder, f"{name}.edg.xml")
    rou  = os.path.join(folder, f"{name}.rou.xml")
    od   = args.od_file or os.path.join(folder, f"od_{name}.txt")

    for p in (conn, edg, rou, od):
        if not os.path.exists(p):
            print(f"[error] not found: {p}")
            return 2

    import janux as jx
    import networkx as nx

    print(f"Building JanuX network...")
    network = jx.build_digraph(conn, edg, rou)
    print(f"  graph: {network.number_of_nodes()} nodes, "
          f"{network.number_of_edges()} edges")

    # --- Load OD pairs (used-only) ----------------------------------------
    if args.cartesian:
        with open(od, "r", encoding="utf-8") as f:
            od_data = ast.literal_eval(f.read())
        od_pairs = [(o, d) for o in od_data["origins"]
                            for d in od_data["destinations"]]
        source = "cartesian"
    else:
        od_pairs, source = load_used_od_pairs(folder, od)

    print(f"  OD pairs to probe: {len(od_pairs)}  (source = {source})")

    if args.max_od:
        od_pairs = od_pairs[:args.max_od]
        print(f"  limited to first {len(od_pairs)} pairs (--max-od)")

    # --- Pre-filter with networkx (skip disconnected) ---------------------
    print(f"\nPre-filtering with networkx (connectivity)...")
    skipped_nopath = []
    skipped_missing = []
    valid_pairs = []
    for (o, d) in od_pairs:
        if o not in network or d not in network:
            skipped_missing.append((o, d))
            continue
        if not nx.has_path(network, o, d):
            skipped_nopath.append((o, d))
            continue
        valid_pairs.append((o, d))

    print(f"  origin/destination missing in graph : {len(skipped_missing)}")
    print(f"  origin cannot reach destination     : {len(skipped_nopath)}")
    print(f"  pairs to send to JanuX             : {len(valid_pairs)}")

    if skipped_missing:
        print(f"\n  First 5 'missing node' pairs:")
        for (o, d) in skipped_missing[:5]:
            print(f"    {o} -> {d}")

    if skipped_nopath:
        print(f"\n  First 5 'no path' pairs:")
        for (o, d) in skipped_nopath[:5]:
            print(f"    {o} -> {d}")

    # --- Probe valid pairs with JanuX (subprocess + timeout) --------------
    print(f"\nProbing {len(valid_pairs)} pairs with JanuX "
          f"(timeout = {args.timeout}s per pair)...\n")

    bad_timeout = []
    bad_error = []
    ok = 0
    t0 = time.time()

    for i, (o, d) in enumerate(valid_pairs, 1):
        res = run_with_timeout(
            _janux_worker, args.timeout,
            network, o, d, args.num_paths, args.num_samples, args.beta,
        )
        status = res["status"]
        if status == "timeout":
            bad_timeout.append((o, d))
            print(f"  [{i:4d}/{len(valid_pairs)}] TIMEOUT  {o} -> {d}")
        elif status == "ok":
            ok += 1
            if i % 20 == 0:
                print(f"  [{i:4d}/{len(valid_pairs)}] ok  "
                      f"(ok={ok}, timeout={len(bad_timeout)}, "
                      f"error={len(bad_error)})")
        else:
            bad_error.append((o, d))
            print(f"  [{i:4d}/{len(valid_pairs)}] ERROR    {o} -> {d}  "
                  f"({res.get('error')})")

        if i % 25 == 0:
            elapsed = time.time() - t0
            rate = i / elapsed if elapsed > 0 else 0
            eta = (len(valid_pairs) - i) / rate if rate > 0 else 0
            print(f"  --- progress: {i}/{len(valid_pairs)}  "
                  f"ok={ok}  timeout={len(bad_timeout)}  "
                  f"error={len(bad_error)}  eta≈{eta:.0f}s ---")

    # --- Save results -----------------------------------------------------
    out = args.out_json or os.path.join(folder, "bad_od.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump({
            "bad_od_pairs":         [list(p) for p in bad_timeout + bad_error],
            "timeout_od_pairs":     [list(p) for p in bad_timeout],
            "error_od_pairs":       [list(p) for p in bad_error],
            "nopath_od_pairs":      [list(p) for p in skipped_nopath],
            "missing_node_od_pairs":[list(p) for p in skipped_missing],
        }, f, indent=2)
    print(f"\nSaved results to {out}")

    # --- Summary ----------------------------------------------------------
    print(f"\n=== Summary ===")
    print(f"  Pairs from agents.csv  : {len(od_pairs)}")
    print(f"  Skipped (no path)      : {len(skipped_nopath)}")
    print(f"  Skipped (missing node) : {len(skipped_missing)}")
    print(f"  Sent to JanuX          : {len(valid_pairs)}")
    print(f"    ok                   : {ok}")
    print(f"    timeout              : {len(bad_timeout)}")
    print(f"    error                : {len(bad_error)}")
    print(f"  Total time             : {time.time() - t0:.1f}s")

    return 0


if __name__ == "__main__":
    sys.exit(main())