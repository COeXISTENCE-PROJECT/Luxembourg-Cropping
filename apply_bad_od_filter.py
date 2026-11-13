#!/usr/bin/env python3
"""
apply_bad_od_filter.py — Remove agents whose (origin, destination) pair
failed in JanuX (i.e. present in bad_od.json).

Two modes (--dropped-action):
  discard    : dropped agents are removed from agents.csv /
               demand_routes.csv / od_<net>.txt.  (default, README
               pipeline unchanged.)
  background : dropped agents are removed from the RL-visible files AND
               MOVED (not copied) from human_demand.rou.xml into
               background_buses.rou.xml, so they still contribute to
               congestion and interactions in SUMO, but RouteRL never
               sees them and cannot mistake them for AVs.

               IMPORTANT: this mode is idempotent.  A second --apply run
               is a no-op (dropped_rows is empty once agents.csv has been
               rewritten).  No <vehicle> is ever duplicated across the
               two .rou.xml files: the strip step runs first and only
               vehicles whose (source_id, edges) match a dropped row are
               removed from human_demand.rou.xml.

Reads:
    bad_od.json               from check_janux_od.py
    agents.csv                id,start_time,origin,destination,kind
    demand_routes.csv         agent_id,source_id,route_id,probability,free_flow_time,edges
    od_<network>.txt          {'origins': [...], 'destinations': [...]}
    human_demand.rou.xml      (only with --dropped-action background)
    background_buses.rou.xml  (only with --dropped-action background)

Writes (only with --apply):
    agents.csv                renumbered 0..N-1
    demand_routes.csv         renumbered
    od_<network>.txt          filtered
    human_demand.rou.xml      (only with --dropped-action background,
                               stripped, *.bak saved)
    background_buses.rou.xml  (only with --dropped-action background,
                               appended, *.bak saved)

Backups of all overwritten files are saved as <file>.bak.
"""

import argparse
import ast
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET


# =============================================================================
# XML HELPERS
# =============================================================================

def indented_xml_write(root: ET.Element, path: str) -> None:
    try:
        ET.indent(root, space="    ")
    except AttributeError:
        pass
    ET.ElementTree(root).write(path, encoding="UTF-8", xml_declaration=True)


def _pick_vtype_from_bg(bg_path: str) -> str:
    """
    Choose a passenger-like vType already defined in background_buses.rou.xml.
    Falls back to DEFAULT_VEHTYPE (always available in SUMO) if none found.
    """
    if not os.path.exists(bg_path):
        return "DEFAULT_VEHTYPE"
    try:
        root = ET.parse(bg_path).getroot()
    except ET.ParseError:
        return "DEFAULT_VEHTYPE"

    available = set()
    for c in root:
        if c.tag in ("vType", "vTypeDistribution"):
            tid = c.get("id")
            if tid:
                available.add(tid)

    for pref in ("passenger", "passenger1",
                 "passenger2a", "passenger2b", "passenger3",
                 "car", "Car"):
        if pref in available:
            return pref

    for tid in sorted(available):
        low = tid.lower()
        if "passenger" in low or "car" in low:
            return tid

    return "DEFAULT_VEHTYPE"


def strip_dropped_from_human_demand(human_path: str,
                                    dropped_pairs: list) -> int:
    """
    Remove <vehicle> elements from human_demand.rou.xml that correspond to
    dropped agents.

    Matching rule (both must hold):
      - id is `source_id` or starts with `source_id + "_seg"`
      - route@edges, whitespace-normalised, equals one of dropped_pairs' edges

    `dropped_pairs` is a list of (source_id, edges_str) tuples — one per
    dropped SEGMENT (not per source vehicle).  Since each segment has its
    own <vehicle> in human_demand.rou.xml, this removes exactly the segments
    that were dropped, never the kept siblings of the same source vehicle.
    """
    if not os.path.exists(human_path) or not dropped_pairs:
        return 0

    edges_to_remove = {" ".join(e.split()) for _, e in dropped_pairs}
    source_ids      = {s for s, _ in dropped_pairs}

    shutil.copy2(human_path, human_path + ".bak")
    tree = ET.parse(human_path)
    root = tree.getroot()

    removed = 0
    for veh in list(root.findall("vehicle")):
        vid = veh.get("id", "")
        if not any(vid == s or vid.startswith(s + "_seg") for s in source_ids):
            continue
        route = veh.find("route")
        if route is None or not route.get("edges"):
            continue
        norm = " ".join(route.get("edges").split())
        if norm in edges_to_remove:
            root.remove(veh)
            removed += 1

    if removed:
        indented_xml_write(root, human_path)
    return removed


def append_background_vehicles(bg_path: str,
                               vehicles: list,
                               vtype: str,
                               backup: bool = True) -> int:
    """
    Append <vehicle> elements to background_buses.rou.xml, then re-sort
    by depart.  Refuses to add an id that already exists (idempotency safety).
    """
    if not vehicles:
        return 0
    if not os.path.exists(bg_path):
        raise FileNotFoundError(f"background file not found: {bg_path}")

    if backup:
        shutil.copy2(bg_path, bg_path + ".bak")

    tree = ET.parse(bg_path)
    root = tree.getroot()

    existing_ids = {v.get("id") for v in root.findall("vehicle")}

    added = 0
    for v in vehicles:
        if v["id"] in existing_ids:
            continue
        el = ET.SubElement(root, "vehicle")
        el.set("id",     v["id"])
        el.set("type",   vtype)
        el.set("depart", f"{v['depart']:.2f}")
        el.set("departLane",  "best")
        el.set("departPos",   "random_free")
        el.set("departSpeed", "0.00")
        ET.SubElement(el, "route").set("edges", " ".join(v["edges"]))
        existing_ids.add(v["id"])
        added += 1

    all_vehicles = root.findall("vehicle")
    all_vehicles.sort(key=lambda v: float(v.get("depart", "0")))
    for v in all_vehicles:
        root.remove(v)
    for v in all_vehicles:
        root.append(v)

    indented_xml_write(root, bg_path)
    return added


# =============================================================================
# MAIN
# =============================================================================

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
    ap.add_argument("--dropped-action", default="discard",
                    choices=["discard", "background"],
                    help="discard (default): drop agents entirely. "
                         "background: move them from human_demand.rou.xml "
                         "into background_buses.rou.xml as pure SUMO traffic.")
    ap.add_argument("--background-file", default=None,
                    help="Path to background_buses.rou.xml "
                         "(default: <network-dir>/background_buses.rou.xml)")
    ap.add_argument("--human-demand-file", default=None,
                    help="Path to human_demand.rou.xml "
                         "(default: <network-dir>/human_demand.rou.xml)")
    args = ap.parse_args()

    folder = os.path.abspath(args.network_dir)
    name   = args.net_name or os.path.basename(folder)

    bad_path    = args.bad_od or os.path.join(folder, "bad_od.json")
    agents_path = os.path.join(folder, "agents.csv")
    demand_path = os.path.join(folder, "demand_routes.csv")
    od_path     = os.path.join(folder, f"od_{name}.txt")
    bg_path     = args.background_file or os.path.join(
        folder, "background_buses.rou.xml")
    human_path  = args.human_demand_file or os.path.join(
        folder, "human_demand.rou.xml")

    required = [bad_path, agents_path, demand_path, od_path]
    if args.dropped_action == "background":
        required.append(bg_path)
    for p in required:
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
        col_start  = header_agents.index("start_time")
    except ValueError as e:
        print(f"[error] agents.csv header missing column: {e}")
        return 2

    kept_rows    = []
    dropped_rows = []
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
            drop_reasons[(o_edge, d_edge)] = \
                drop_reasons.get((o_edge, d_edge), 0) + 1
            dropped_rows.append(cols)
        else:
            kept_rows.append((cols, o_edge, d_edge))

    n_before  = len(agent_lines) - 1
    n_kept    = len(kept_rows)
    n_dropped = len(dropped_rows)

    print(f"\n=== Filter report ===")
    print(f"  Agents before    : {n_before}")
    print(f"  Agents dropped   : {n_dropped}")
    print(f"  Agents remaining : {n_kept}")
    if n_before > 0:
        print(f"  Drop ratio       : {100.0 * n_dropped / n_before:.1f}%")

    print(f"\n  Drop breakdown by bad pair (top 10):")
    for (o, d), cnt in sorted(drop_reasons.items(), key=lambda kv: -kv[1])[:10]:
        print(f"    {o} -> {d}: {cnt} agents")

    print(f"\n  Dropped-action   : {args.dropped_action}")

    if not args.apply:
        print(f"\n[DRY RUN] Re-run with --apply to overwrite files.")
        if args.dropped_action == "background" and n_dropped > 0:
            print(f"  Would MOVE {n_dropped} agent segment(s):")
            print(f"    human_demand.rou.xml     (-{n_dropped})")
            print(f"    background_buses.rou.xml (+{n_dropped})")
        return 0

    # =====================================================================
    # STEP 0 (background mode only):
    #   MOVE dropped segments from human_demand.rou.xml → background_buses
    #   This happens BEFORE any RL-visible file is touched.
    #   If the append fails after the strip, human_demand.rou.xml is
    #   restored from its .bak so we never lose agents.
    # =====================================================================
    n_reinjected = 0
    n_stripped   = 0
    if args.dropped_action == "background" and dropped_rows:
        # Build old_agent_id -> (source_id, edges_str) from demand_routes.csv
        with open(demand_path, "r", encoding="utf-8") as f:
            demand_lines_all = f.read().splitlines()
        header_demand_all = demand_lines_all[0].split(",")
        c_agent  = header_demand_all.index("agent_id")
        c_source = header_demand_all.index("source_id")
        c_edges  = header_demand_all.index("edges")

        id_to_info = {}
        for line in demand_lines_all[1:]:
            if not line.strip():
                continue
            cols_d = line.split(",", maxsplit=len(header_demand_all) - 1)
            try:
                aid = int(cols_d[c_agent])
            except (ValueError, IndexError):
                continue
            id_to_info[aid] = (cols_d[c_source], cols_d[c_edges])

        vtype = _pick_vtype_from_bg(bg_path)
        print(f"\n[background] Using vType: {vtype}")

        bg_vehicles   = []
        dropped_pairs = []
        for cols in dropped_rows:
            try:
                old_id = int(cols[col_id])
                start  = float(cols[col_start])
            except (ValueError, IndexError):
                continue
            info = id_to_info.get(old_id)
            if not info:
                continue
            src_id, edges_str = info
            bg_vehicles.append({
                "id":     f"bg_{old_id}",
                "depart": start,
                "edges":  edges_str.split(),
            })
            dropped_pairs.append((src_id, edges_str))

        if not os.path.exists(human_path):
            print(f"[warn] {human_path} not found, skipping strip step")
        else:
            n_stripped = strip_dropped_from_human_demand(
                human_path, dropped_pairs)
            print(f"[background] {n_stripped} vehicle(s) removed from "
                  f"human_demand.rou.xml "
                  f"(backup: {os.path.basename(human_path)}.bak)")

        try:
            n_reinjected = append_background_vehicles(bg_path, bg_vehicles, vtype)
            print(f"[background] {n_reinjected} vehicle(s) appended to "
                  f"{os.path.basename(bg_path)} "
                  f"(backup: {os.path.basename(bg_path)}.bak)")
        except Exception as e:
            print(f"[error] could not write background file: {e}")
            # Roll back human_demand.rou.xml so we don't lose agents
            if n_stripped and os.path.exists(human_path + ".bak"):
                shutil.copy2(human_path + ".bak", human_path)
                print(f"[rollback] restored {os.path.basename(human_path)} "
                      f"from .bak")
            return 3

        if n_stripped != n_reinjected:
            print(f"[warn] strip/reinject mismatch: "
                  f"stripped={n_stripped}, reinjected={n_reinjected}. "
                  f"Check *.bak files before running SUMO.")

    # =====================================================================
    # STEP 1-3: rewrite the RL-visible files (unchanged behavior)
    # =====================================================================

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
    if n_stripped > 0:
        print(f"  human_demand.rou.xml      (-{n_stripped} vehicles)")
    if n_reinjected > 0:
        print(f"  background_buses.rou.xml  (+{n_reinjected} vehicles)")
    print(f"\n  New origins      : {len(new_origins)}")
    print(f"  New destinations : {len(new_destinations)}")

    # --- Final sanity check (only meaningful in background mode) ---------
    if args.dropped_action == "background" and n_reinjected > 0:
        n_human = 0
        if os.path.exists(human_path):
            try:
                n_human = len(ET.parse(human_path).getroot().findall("vehicle"))
            except ET.ParseError:
                pass
        n_bg = 0
        if os.path.exists(bg_path):
            try:
                n_bg = len(ET.parse(bg_path).getroot().findall("vehicle"))
            except ET.ParseError:
                pass
        print(f"\n  Sanity check:")
        print(f"    human_demand.rou.xml     : {n_human} vehicles")
        print(f"    background_buses.rou.xml : {n_bg} vehicles")
        print(f"    (custom.sumocfg is UNCHANGED — both files are already "
              f"in <route-files>)")

    return 0


if __name__ == "__main__":
    sys.exit(main())