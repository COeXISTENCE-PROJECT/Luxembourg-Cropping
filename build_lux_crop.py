#!/usr/bin/env python3
"""
build_lux_crop.py — Clean crop pipeline for LuST → RouteRL / URB.

Spatially crops the LuST Luxembourg scenario around a geographic window and
temporally crops it to a configurable window, producing a network folder that
matches the URB conventions (see networks/ingolstadt21, networks/gretz_armainvilliers).

LuST carries anomalous edge IDs like "--30694#2" (double leading dash), a
side effect of its multi-step OSM -> SUMO conversion. We strip the anomalous
leading "--" so the crop uses the same naming convention as the rest of URB
and JanuX accepts it.

Path availability filter
------------------------
JanuX generates a fixed number of paths per OD pair. When an OD pair has
fewer distinct paths than required, JanuX can hang or fail. This script
filters out any (origin, destination) pair that does not have at least
`--min-paths` distinct paths in the cropped network, using networkx.
The agents that use dropped OD pairs are removed from the demand.

URB convention notes
--------------------
- <network>.rou.xml is NOT a demand file. It is a copy of <network>.net.xml.
  JanuX builds its graph from it.
- The human vehicle demand goes into human_demand.rou.xml.
- vTypes from LuST are merged inline into the demand files.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from collections import Counter
from typing import Dict, List, Optional, Tuple

import xml.etree.ElementTree as ET

try:
    import networkx as nx
    _HAS_NX = True
except ImportError:
    _HAS_NX = False


# =============================================================================
# CONSTANTS
# =============================================================================

SEG_SUFFIX_RE = re.compile(r"_seg\d+$")

LUST_LOCAL_DEMAND_FILES = [
    os.path.join("DUERoutes", "local.static.0.rou.xml"),
    os.path.join("DUERoutes", "local.static.1.rou.xml"),
    os.path.join("DUERoutes", "local.static.2.rou.xml"),
]
LUST_TRANSIT_DEMAND_FILES = ["transit.rou.xml"]
LUST_BUS_FILES = [
    "buslines.rou.xml",
    "busses.rou.xml",
    "trams.rou.xml",
    "trains.rou.xml",
]
LUST_VTYPES_FILE   = "vtypes.add.xml"
LUST_BUSSTOPS_FILE = "busstops.add.xml"
LUST_TLL_STATIC    = "tll.static.xml"

LUST_VEHROUTE_CANDIDATES = [
    "due.static.vehroute.until9.unfinished.xml",
    "due.static.vehroute.until9.xml",
    "due.static.vehroute.xml",
]
LUST_TRIPINFO_CANDIDATES = [
    "due.static.until9.tripinfo.xml",
    "due.static.tripinfo.xml",
]

ATTRS_ORIGIN  = ("departLane", "departPos", "departSpeed")
ATTRS_ARRIVAL = ("arrivalLane", "arrivalPos", "arrivalSpeed")

OUTSIDE_PRESETS = {
    "last_desired":          {"departPos": "last", "departSpeed": "desired"},
    "last_desired_best":     {"departLane": "best", "departPos": "last", "departSpeed": "desired"},
    "free_desired_best":     {"departLane": "best", "departPos": "free", "departSpeed": "desired"},
    "randfree_desired_best": {"departLane": "best", "departPos": "random_free", "departSpeed": "desired"},
    "randfree_0_best":       {"departLane": "best", "departPos": "random_free", "departSpeed": "0.00"},
    "base_desired_best":     {"departLane": "best", "departPos": "base", "departSpeed": "desired"},
}

SIM_PRESETS = {
    "current":  {"time-to-teleport": 300},
    "lust_ref": {"time-to-teleport": 600,
                 "ignore-junction-blocker": 20,
                 "max-depart-delay": 600},
}

DEFAULT_GEO = (6500.0, 9500.0, 4200.0, 7100.0)
DEFAULT_T0_H = 7.0
DEFAULT_T1_H = 9.0


# =============================================================================
# ID SANITIZATION
# =============================================================================

def sanitize_id(s: str) -> str:
    if not s:
        return s
    if s.startswith("---"):
        raise ValueError(f"unexpected id with three or more leading dashes: {s}")
    if s.startswith("--"):
        return s[2:]
    return s


def sanitize_edge_list(edges: List[str]) -> List[str]:
    return [sanitize_id(e) for e in edges]


def write_sanitized_source_net(src_path: str, dst_path: str) -> int:
    with open(src_path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()

    pattern = re.compile(r'\b(id|from|to)="(--[^"]*)"')
    n_subs = 0

    def repl(m):
        nonlocal n_subs
        n_subs += 1
        return f'{m.group(1)}="{m.group(2)[2:]}"'

    text = pattern.sub(repl, text)

    if n_subs == 0:
        shutil.copy2(src_path, dst_path)
        return 0

    check_path = dst_path + ".check"
    with open(check_path, "w", encoding="utf-8") as f:
        f.write(text)

    root = ET.parse(check_path).getroot()
    seen = set()
    dups = []
    for e in root.findall("edge"):
        eid = e.get("id")
        if eid in seen:
            dups.append(eid)
        seen.add(eid)
        for lane in e.findall("lane"):
            lid = lane.get("id")
            if lid in seen:
                dups.append(lid)
            seen.add(lid)

    if dups:
        os.remove(check_path)
        raise RuntimeError(
            f"ID sanitization created {len(dups)} duplicate edge/lane ids "
            f"(first few: {dups[:5]}). Aborting."
        )

    os.replace(check_path, dst_path)
    return n_subs


# =============================================================================
# PATH AVAILABILITY FILTER
# =============================================================================

def _build_path_graph(con_xml_path: str):
    """
    Build a directed graph where nodes are edge IDs and links are SUMO
    connections (from -> to). This mirrors what JanuX builds from the
    same file, so 'path exists here' == 'path exists for JanuX'.
    """
    root = ET.parse(con_xml_path).getroot()
    G = nx.DiGraph()
    for c in root.findall("connection"):
        f = c.get("from")
        t = c.get("to")
        if f and t and f != t:
            G.add_edge(f, t)
    return G


def _count_distinct_paths(G, origin, destination,
                          max_paths: int = 20,
                          similarity_threshold: float = 0.85,
                          timeout_s: float = 2.0) -> List[List[str]]:
    """
    Return up to `max_paths` paths between origin and destination that are
    sufficiently distinct from each other. Two paths are considered the same
    when their edge-set Jaccard similarity >= similarity_threshold.
    A probe budget (5 * max_paths) and a wall-clock timeout keep the call
    bounded on dense graphs.
    """
    if origin not in G or destination not in G or origin == destination:
        return []

    kept: List[List[str]] = []
    probe_budget = max_paths * 5
    probes = 0
    start = time.time()

    try:
        for path in nx.shortest_simple_paths(G, origin, destination):
            probes += 1
            if probes > probe_budget:
                break
            if time.time() - start > timeout_s:
                break
            if len(kept) >= max_paths:
                break
            path_set = set(path)
            too_similar = False
            for existing in kept:
                existing_set = set(existing)
                union = len(path_set | existing_set)
                if union > 0:
                    jaccard = len(path_set & existing_set) / union
                    if jaccard >= similarity_threshold:
                        too_similar = True
                        break
            if not too_similar:
                kept.append(path)
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        pass
    return kept


def filter_od_by_path_availability(
        segments: List[dict],
        con_xml_path: str,
        min_paths: int = 6,
        max_paths_probe: int = 20,
        similarity_threshold: float = 0.85,
        timeout_s: float = 2.0,
) -> Tuple[List[dict], dict]:
    """
    Drop agents whose (origin, destination) pair has fewer than `min_paths`
    distinct paths in the cropped network.
    """
    if not _HAS_NX:
        print("  [warn] networkx not installed, skipping path filter")
        return segments, {
            "od_pairs_total": 0, "od_pairs_kept": 0, "od_pairs_dropped": 0,
            "agents_before": len(segments), "agents_after": len(segments),
            "path_count_distribution": {},
        }

    if not os.path.exists(con_xml_path):
        print(f"  [warn] con file not found: {con_xml_path}, skipping filter")
        return segments, {
            "od_pairs_total": 0, "od_pairs_kept": 0, "od_pairs_dropped": 0,
            "agents_before": len(segments), "agents_after": len(segments),
            "path_count_distribution": {},
        }

    G = _build_path_graph(con_xml_path)
    print(f"  graph           : {G.number_of_nodes()} nodes, "
          f"{G.number_of_edges()} edges")

    od_pairs = sorted({(r["edges"][0], r["edges"][-1]) for r in segments})
    print(f"  unique OD pairs : {len(od_pairs)}")

    kept_od = set()
    dropped_od = set()
    per_od_paths = {}

    for i, (o, d) in enumerate(od_pairs, 1):
        paths = _count_distinct_paths(
            G, o, d,
            max_paths=max_paths_probe,
            similarity_threshold=similarity_threshold,
            timeout_s=timeout_s,
        )
        per_od_paths[(o, d)] = len(paths)
        if len(paths) >= min_paths:
            kept_od.add((o, d))
        else:
            dropped_od.add((o, d))
        if i % 10 == 0 or i == len(od_pairs):
            print(f"    {i}/{len(od_pairs)} OD pairs  "
                  f"(kept={len(kept_od)}, dropped={len(dropped_od)})")

    filtered = [
        r for r in segments
        if (r["edges"][0], r["edges"][-1]) in kept_od
    ]

    dist = Counter(per_od_paths.values())

    if dropped_od:
        print(f"  first 10 dropped OD pairs (origin -> destination: n_paths):")
        for (o, d) in list(dropped_od)[:10]:
            print(f"    {o} -> {d}: {per_od_paths[(o, d)]} path(s)")

    return filtered, {
        "od_pairs_total": len(od_pairs),
        "od_pairs_kept": len(kept_od),
        "od_pairs_dropped": len(dropped_od),
        "agents_before": len(segments),
        "agents_after": len(filtered),
        "path_count_distribution": dict(sorted(dist.items())),
    }


# =============================================================================
# ARGUMENT PARSING
# =============================================================================

def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Crop the LuST scenario spatially + temporally, "
                    "produce a URB-compatible network folder.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--base-dir", default=os.path.abspath("."))
    p.add_argument("--scenario-dir", default=None)
    p.add_argument("--network-dir", default=None)
    p.add_argument("--output-dir", default=None)
    p.add_argument("--network-name", default="luxembourg")

    p.add_argument("--geo", nargs=4, type=float,
                   metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                   default=list(DEFAULT_GEO))
    p.add_argument("--t0", type=float, default=DEFAULT_T0_H)
    p.add_argument("--t1", type=float, default=DEFAULT_T1_H)
    p.add_argument("--preload-hours", type=float, default=1.0)

    p.add_argument("--entry-estimator", default="vehroute",
                   choices=["vehroute", "prop_ff", "ff_global", "ff"])
    p.add_argument("--outside-insertion", default="base_desired_best",
                   choices=list(OUTSIDE_PRESETS.keys()))
    p.add_argument("--min-edges", type=int, default=2)
    p.add_argument("--reentry-gap", type=float, default=0.0)

    p.add_argument("--tls-mode", default="static", choices=["static", "original"])
    p.add_argument("--sim-preset", default="lust_ref", choices=list(SIM_PRESETS.keys()))

    # --- Path availability filter ------------------------------------------
    p.add_argument("--min-paths", type=int, default=6,
                   help="Minimum number of distinct paths per OD pair. "
                        "OD pairs with fewer paths are dropped.")
    p.add_argument("--max-paths-probe", type=int, default=12,
                   help="Max paths to probe per OD pair before giving up.")
    p.add_argument("--path-similarity", type=float, default=0.85,
                   help="Jaccard similarity >= threshold means two paths are "
                        "considered the same.")
    p.add_argument("--path-timeout", type=float, default=2.0,
                   help="Max seconds probing paths for a single OD pair.")
    p.add_argument("--skip-path-filter", action="store_true",
                   help="Disable the OD path availability filter.")

    # --- URB post-processing ------------------------------------------------
    p.add_argument("--vtypes-mode", default="merge",
                   choices=["merge", "keep", "remove"])
    p.add_argument("--busstops-mode", default="strip",
                   choices=["strip", "keep"])
    p.add_argument("--extract-typ", action="store_true", default=True)
    p.add_argument("--write-metadata", action="store_true", default=True)

    p.add_argument("--cleanup-outputs", action="store_true", default=True)
    p.add_argument("--keep-sumocfg", action="store_true", default=False)
    p.add_argument("--keep-output", action="store_true")

    p.add_argument("--run-sumo", action="store_true")
    p.add_argument("--sumo-gui", action="store_true")
    p.add_argument("--skip-validation", action="store_true")

    p.add_argument("--sumo-home",
                   default=os.environ.get("SUMO_HOME",
                                          r"C:\Program Files (x86)\Eclipse\Sumo"))
    return p.parse_args(argv)


# =============================================================================
# SMALL UTILITIES
# =============================================================================

def indented_xml_write(root: ET.Element, path: str) -> None:
    try:
        ET.indent(root, space="    ")
    except AttributeError:
        pass
    ET.ElementTree(root).write(path, encoding="UTF-8", xml_declaration=True)


def first_existing(candidates: List[str], *dirs: str) -> Optional[str]:
    for d in dirs:
        for name in candidates:
            p = os.path.join(d, name)
            if os.path.exists(p):
                return p
    return None


def wipe_output_dir(path: str) -> None:
    if os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path, exist_ok=True)


def n_links(tl: ET.Element) -> int:
    ph = tl.find("phase")
    return len(ph.get("state", "")) if ph is not None else 0


# =============================================================================
# DEMAND LOADING
# =============================================================================

def load_rouxml_records(path: str, t_start: float, t_end: float,
                        source_tag: str) -> List[dict]:
    records: List[dict] = []
    if not os.path.exists(path):
        print(f"  [warn] missing demand file: {path}")
        return records

    root = ET.parse(path).getroot()
    route_defs = {
        r.get("id"): r.get("edges").split()
        for r in root.findall("route")
        if r.get("id") and r.get("edges")
    }

    for veh in root.findall("vehicle"):
        try:
            dep = float(veh.get("depart", "-1"))
        except (TypeError, ValueError):
            continue
        if not (t_start <= dep <= t_end):
            continue

        edges: Optional[List[str]] = None
        child = veh.find("route")
        if child is not None and child.get("edges"):
            edges = child.get("edges").split()
        elif veh.get("route") in route_defs:
            edges = route_defs[veh.get("route")]
        if not edges or len(edges) < 2:
            continue

        edges = sanitize_edge_list(edges)

        rec = {
            "id": veh.get("id"),
            "type": veh.get("type", "passenger"),
            "depart": dep,
            "edges": edges,
            "source": source_tag,
        }
        for a in ATTRS_ORIGIN + ATTRS_ARRIVAL:
            v = veh.get(a)
            if v is not None:
                rec[a] = v
        records.append(rec)

    return records


def load_all_human_demand(scenario_dir: str, t_pre: float, t_end: float) -> List[dict]:
    records: List[dict] = []

    print("Loading human demand...")
    for rel in LUST_LOCAL_DEMAND_FILES:
        path = os.path.join(scenario_dir, rel)
        r = load_rouxml_records(path, t_pre, t_end, source_tag="local")
        print(f"  {rel:40s} -> {len(r):6d} vehicles kept (local)")
        records.extend(r)

    for rel in LUST_TRANSIT_DEMAND_FILES:
        path = os.path.join(scenario_dir, rel)
        r = load_rouxml_records(path, t_pre, t_end, source_tag="transit")
        print(f"  {rel:40s} -> {len(r):6d} vehicles kept (transit -> merged with humans)")
        records.extend(r)

    print(f"  TOTAL human records kept: {len(records)}")
    return records


# =============================================================================
# REFERENCE OUTPUTS
# =============================================================================

def load_tripinfo(path: str, wanted_ids: set) -> Dict[str, dict]:
    out: Dict[str, dict] = {}
    if not path or not os.path.exists(path):
        return out
    for _, el in ET.iterparse(path, events=("end",)):
        if el.tag == "tripinfo":
            vid = el.get("id")
            if vid in wanted_ids:
                try:
                    out[vid] = {k: float(el.get(k))
                                for k in ("depart", "arrival", "duration",
                                          "timeLoss", "departDelay")}
                except (TypeError, ValueError):
                    pass
            el.clear()
    return out


def load_vehroute(path: str, wanted_ids: set) -> Dict[str, dict]:
    out: Dict[str, dict] = {}
    if not path or not os.path.exists(path):
        return out
    for _, el in ET.iterparse(path, events=("end",)):
        if el.tag != "vehicle":
            continue
        vid = el.get("id")
        if vid in wanted_ids:
            routes = el.findall(".//route")
            r = routes[-1] if routes else None
            if r is not None and r.get("exitTimes") and r.get("edges"):
                arr = el.get("arrival")
                try:
                    exit_times = [float(x) for x in r.get("exitTimes").split()]
                except ValueError:
                    exit_times = []
                out[vid] = {
                    "depart": float(el.get("depart", "0")),
                    "arrival": (float(arr) if arr not in (None, "", "-1") else None),
                    "edges": sanitize_edge_list(r.get("edges").split()),
                    "exit": exit_times,
                }
        el.clear()
    return out


# =============================================================================
# NETWORK CROPPING
# =============================================================================

def segment_intersects_rect(x1, y1, x2, y2, xmin, xmax, ymin, ymax) -> bool:
    dx, dy = x2 - x1, y2 - y1
    t0, t1 = 0.0, 1.0
    for p, q in [(-dx, x1 - xmin), (dx, xmax - x1),
                 (-dy, y1 - ymin), (dy, ymax - y1)]:
        if p == 0:
            if q < 0:
                return False
        else:
            r = q / p
            if p < 0:
                if r > t1: return False
                if r > t0: t0 = r
            else:
                if r < t0: return False
                if r < t1: t1 = r
    return True


def collect_edges_intersecting_window(net_root: ET.Element,
                                      geo: Tuple[float, float, float, float]) -> set:
    xmin, xmax, ymin, ymax = geo
    node_xy = {j.get("id"): (float(j.get("x")), float(j.get("y")))
               for j in net_root.findall("junction")}

    edges: set = set()
    for e in net_root.findall("edge"):
        eid = e.get("id")
        if not eid or eid.startswith(":"):
            continue
        a = node_xy.get(e.get("from"))
        b = node_xy.get(e.get("to"))
        if not a or not b:
            continue
        if segment_intersects_rect(a[0], a[1], b[0], b[1],
                                   xmin, xmax, ymin, ymax):
            edges.add(eid)
    return edges


def compute_edge_free_flow_times(net_root: ET.Element) -> Dict[str, float]:
    ff: Dict[str, float] = {}
    for e in net_root.findall("edge"):
        eid = e.get("id")
        if not eid or eid.startswith(":"):
            continue
        times = []
        for lane in e.findall("lane"):
            L = float(lane.get("length", 0.0))
            v = float(lane.get("speed", 0.0))
            if v > 0:
                times.append(L / v)
        if times:
            ff[eid] = sum(times) / len(times)
    return ff


def crop_network(source_net_path: str, network_name: str, output_dir: str,
                 candidate_edges: set, netconvert_bin: str,
                 temp_work_dir: str) -> str:
    temp_in  = os.path.join(temp_work_dir, "in")
    temp_out = os.path.join(temp_work_dir, "out")
    for d in (temp_in, temp_out):
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)

    if not os.path.exists(source_net_path):
        raise FileNotFoundError(f"source network not found: {source_net_path}")
    shutil.copy2(source_net_path, os.path.join(temp_in, f"{network_name}.net.xml"))

    with open(os.path.join(temp_in, "edges_to_keep.txt"), "w", encoding="utf-8") as f:
        for e in candidate_edges:
            f.write(f"{e}\n")

    cmd = [
        netconvert_bin,
        "-s", f"in/{network_name}.net.xml",
        "-o", f"out/{network_name}.net.xml",
        "--keep-edges.input-file", "in/edges_to_keep.txt",
        "--plain-output-prefix",   f"out/{network_name}",
        "--no-turnarounds", "true",
        "--junctions.corner-detail", "0",
        "--no-internal-links", "false",
        "--junctions.join", "false",
        "--geometry.remove", "false",
        "--keep-edges.components", "1",
    ]
    r = subprocess.run(cmd, cwd=temp_work_dir, capture_output=True,
                       text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print(r.stderr[-3000:])
        raise RuntimeError(f"netconvert failed (code {r.returncode})")

    copied = []
    for fname in os.listdir(temp_out):
        if fname.startswith(network_name):
            shutil.copy2(os.path.join(temp_out, fname),
                         os.path.join(output_dir, fname))
            copied.append(fname)
    shutil.rmtree(temp_in,  ignore_errors=True)
    shutil.rmtree(temp_out, ignore_errors=True)

    print(f"  netconvert output: {sorted(copied)}")
    return os.path.join(output_dir, f"{network_name}.net.xml")


def bake_static_tls(src_net: str, dst_net: str,
                    tll_static_root: ET.Element) -> int:
    static_by_id = {t.get("id"): t for t in tll_static_root.iter("tlLogic")}
    tree = ET.parse(src_net)
    root = tree.getroot()
    replaced = 0
    for t in list(root.findall("tlLogic")):
        tid = t.get("id")
        stl = static_by_id.get(tid)
        if stl is None:
            continue
        if n_links(t) != n_links(stl):
            continue
        pos = list(root).index(t)
        root.remove(t)
        root.insert(pos, copy.deepcopy(stl))
        replaced += 1
    tree.write(dst_net, encoding="UTF-8", xml_declaration=True)
    return replaced


# =============================================================================
# SEGMENTATION + ENTRY-TIME ESTIMATION
# =============================================================================

def split_with_idx(edges: List[str],
                   allowed: set) -> List[Tuple[int, List[str]]]:
    segs, cur, start = [], [], None
    for i, e in enumerate(edges):
        if e in allowed:
            if not cur:
                start = i
            cur.append(e)
        else:
            if len(cur) >= 2:
                segs.append((start, cur))
            cur = []
    if len(cur) >= 2:
        segs.append((start, cur))
    return segs


def estimate_entry_time(edges: List[str], idx: int, rec: dict, mode: str,
                        edge_ff: Dict[str, float],
                        ref_tripinfo: Optional[Dict[str, dict]] = None,
                        ref_vehroute: Optional[Dict[str, dict]] = None,
                        k_global: float = 1.0) -> float:
    if idx == 0:
        return rec["depart"]

    ff_before = sum(edge_ff.get(e, 0.0) for e in edges[:idx])

    if mode == "vehroute" and ref_vehroute is not None:
        v = ref_vehroute.get(rec["id"])
        if v is not None and v["edges"] == edges and idx - 1 < len(v["exit"]):
            t = v["exit"][idx - 1]
            if t >= 0:
                return t

    if mode in ("vehroute", "prop_ff") and ref_tripinfo is not None:
        t = ref_tripinfo.get(rec["id"])
        if t is not None:
            ff_tot = sum(edge_ff.get(e, 0.0) for e in edges)
            if ff_tot > 0:
                return t["depart"] + t["duration"] * ff_before / ff_tot

    scale = 1.0 if mode == "ff" else k_global
    return rec["depart"] + ff_before * scale


def segment_human_demand(records: List[dict],
                         allowed_edges: set,
                         edge_ff: Dict[str, float],
                         t0: float, t_end_local: float,
                         mode: str,
                         ref_tripinfo: Optional[Dict[str, dict]],
                         ref_vehroute: Optional[Dict[str, dict]],
                         k_global: float,
                         min_edges: int = 2,
                         reentry_gap: float = 0.0) -> Tuple[List[dict], dict]:
    out: List[dict] = []
    st = Counter()

    st_src_records   = Counter()
    st_src_with_segs = Counter()
    st_src_segments  = Counter()

    non_split_vehicles = 0
    split_vehicles     = 0
    seg_generated      = 0
    seg_from_non_split = 0
    seg_from_split     = 0

    for rec in records:
        st_src_records[rec["source"]] += 1
        edges = rec["edges"]
        segs = split_with_idx(edges, allowed_edges)
        segs = [(i, s) for (i, s) in segs if len(s) >= min_edges]
        if not segs:
            continue

        st_src_with_segs[rec["source"]] += 1
        st["trajectories_in_crop"] += 1
        seg_generated += len(segs)

        is_split = len(segs) > 1
        if is_split:
            st["split_trajectories"] += 1
            split_vehicles += 1
        else:
            non_split_vehicles += 1

        segs_kept_from_this = 0
        for k, (idx, seg) in enumerate(segs):
            t_abs = estimate_entry_time(
                edges, idx, rec, mode, edge_ff,
                ref_tripinfo=ref_tripinfo,
                ref_vehroute=ref_vehroute,
                k_global=k_global,
            )
            t_local = t_abs + reentry_gap * k - t0
            if not (0 <= t_local <= t_end_local):
                st["dropped_out_of_window"] += 1
                continue

            if not all(e in allowed_edges for e in seg):
                st["rejected_bad_edges"] += 1
                continue

            end_idx = idx + len(seg)
            new = {
                "id":             rec["id"] if k == 0 else f"{rec['id']}_seg{k}",
                "orig_id":        rec["id"],
                "source":         rec["source"],
                "type":           rec["type"],
                "depart":         t_local,
                "edges":          seg,
                "free_flow_time": float(sum(edge_ff.get(e, 0.0) for e in seg)),
                "start_idx":      idx,
                "starts_at_origin": idx == 0,
                "ends_at_dest":     end_idx == len(edges),
                "from_outside":     idx > 0,
                "to_outside":       end_idx < len(edges),
            }
            if idx == 0:
                for a in ATTRS_ORIGIN:
                    if a in rec:
                        new[a] = rec[a]
            if new["ends_at_dest"]:
                for a in ATTRS_ARRIVAL:
                    if a in rec:
                        new[a] = rec[a]
            out.append(new)
            segs_kept_from_this += 1
            st_src_segments[rec["source"]] += 1

        if is_split:
            seg_from_split += segs_kept_from_this
        else:
            seg_from_non_split += segs_kept_from_this

    unique_ids = len({r["id"] for r in out})

    st["segments_generated"]           = seg_generated
    st["segments_kept"]                = len(out)
    st["segments_from_non_split"]      = seg_from_non_split
    st["segments_from_split"]          = seg_from_split
    st["non_split_vehicles"]           = non_split_vehicles
    st["split_vehicles"]               = split_vehicles
    st["unique_agent_ids"]             = unique_ids
    st["from_outside"]                 = sum(1 for r in out if r["from_outside"])
    st["invariant_agents_eq_segments"] = (unique_ids == len(out))
    st["invariant_split_sum_ok"]       = (
        seg_from_non_split + seg_from_split == len(out)
    )

    dropped  = st.get("dropped_out_of_window", 0)
    rejected = st.get("rejected_bad_edges", 0)
    st["invariant_arithmetic_ok"] = (seg_generated - dropped - rejected == len(out))

    st["_records_by_source"]   = dict(st_src_records)
    st["_with_segs_by_source"] = dict(st_src_with_segs)
    st["_segments_by_source"]  = dict(st_src_segments)
    return out, dict(st)


# =============================================================================
# ROUTE EXPORT
# =============================================================================

def export_human_routes(records: List[dict], output_path: str,
                        outside_preset: str) -> str:
    preset = OUTSIDE_PRESETS[outside_preset]

    root = ET.Element("routes")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation",
             "http://sumo.dlr.de/xsd/routes_file.xsd")

    for r in sorted(records, key=lambda r: r["depart"]):
        v = ET.SubElement(root, "vehicle")
        v.set("id",     r["id"])
        v.set("type",   r.get("type", "passenger"))
        v.set("depart", f"{r['depart']:.2f}")

        if r["from_outside"]:
            for a, val in preset.items():
                v.set(a, val)
        elif "departPos" in r:
            for a in ATTRS_ORIGIN:
                if a in r:
                    v.set(a, r[a])
        else:
            v.set("departLane",  "best")
            v.set("departPos",   "random_free")
            v.set("departSpeed", "0.00")

        if r["ends_at_dest"]:
            for a in ATTRS_ARRIVAL:
                if a in r:
                    v.set(a, r[a])

        ET.SubElement(v, "route").set("edges", " ".join(r["edges"]))

    indented_xml_write(root, output_path)
    return output_path


# =============================================================================
# BACKGROUND BUSES
# =============================================================================

def load_lane_lengths(net_path: str) -> Dict[str, float]:
    root = ET.parse(net_path).getroot()
    lanes = {}
    for edge in root.findall("edge"):
        if (edge.get("id") or "").startswith(":"):
            continue
        for lane in edge.findall("lane"):
            lid = lane.get("id")
            if lid:
                lanes[lid] = float(lane.get("length", 0.0))
    return lanes


def filter_busstops(src_path: str, allowed_edges: set,
                    lane_lengths: Dict[str, float],
                    out_path: str) -> Tuple[set, Dict[str, str]]:
    if not os.path.exists(src_path):
        return set(), {}

    root = ET.parse(src_path).getroot()
    kept = dropped = clipped = 0
    for ch in list(root):
        lane_id = ch.get("lane")
        if not lane_id:
            root.remove(ch); dropped += 1; continue
        edge_id = lane_id.rsplit("_", 1)[0]
        if edge_id not in allowed_edges or lane_id not in lane_lengths:
            root.remove(ch); dropped += 1; continue
        L = lane_lengths[lane_id]
        try:
            sp = float(ch.get("startPos", "0"))
            ep = float(ch.get("endPos",   f"{L}"))
        except (TypeError, ValueError):
            root.remove(ch); dropped += 1; continue
        sp2 = max(0.0, min(sp, L))
        ep2 = max(0.0, min(ep, L))
        if ep2 - sp2 < 1.0:
            root.remove(ch); dropped += 1; continue
        ch.set("startPos", f"{sp2:.2f}")
        ch.set("endPos",   f"{ep2:.2f}")
        ch.set("friendlyPos", "true")
        if (sp2, ep2) != (sp, ep):
            clipped += 1
        kept += 1

    indented_xml_write(root, out_path)

    valid, mapping = set(), {}
    for s in root.findall("busStop"):
        sid = s.get("id"); lane = s.get("lane")
        if not sid or not lane:
            continue
        valid.add(sid)
        mapping[sid] = lane.rsplit("_", 1)[0]

    print(f"  busstops.add.xml: {kept} kept "
          f"({dropped} dropped, {clipped} clipped)")
    return valid, mapping


def _collect_route_defs(root: ET.Element) -> Dict[str, List[str]]:
    return {r.get("id"): r.get("edges").split()
            for r in root.findall("route")
            if r.get("id") and r.get("edges")}


def _vehicle_edges(veh: ET.Element,
                   route_defs: Dict[str, List[str]]) -> Optional[List[str]]:
    child = veh.find("route")
    if child is not None and child.get("edges"):
        return child.get("edges").split()
    if veh.get("route") in route_defs:
        return route_defs[veh.get("route")]
    return None


def filter_background_buses(src_path: str, allowed_edges: set,
                            t0: float, t_end: float,
                            out_path: str,
                            valid_busstops: set,
                            busstop_to_edge: Dict[str, str],
                            edge_ff: Dict[str, float]) -> int:
    if not os.path.exists(src_path):
        return 0

    root = ET.parse(src_path).getroot()
    route_defs = _collect_route_defs(root)

    out_root = ET.Element("routes")
    out_root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    out_root.set("xsi:noNamespaceSchemaLocation",
                 "http://sumo.dlr.de/xsd/routes_file.xsd")
    for t in root.findall("vType"):
        out_root.append(t)

    kept = 0
    for veh in root.findall("vehicle"):
        try:
            d_raw = float(veh.get("depart", "-1"))
        except (TypeError, ValueError):
            continue
        edges = _vehicle_edges(veh, route_defs)
        if not edges:
            continue
        edges = sanitize_edge_list(edges)

        segs = split_with_idx(edges, allowed_edges)
        if not segs:
            continue

        for i, (idx, seg) in enumerate(segs):
            ff_before = sum(edge_ff.get(e, 0.0) for e in edges[:idx])
            dep = d_raw + ff_before - t0
            if not (0 <= dep <= t_end):
                continue

            seg_set = set(seg)
            v = ET.SubElement(out_root, "vehicle")
            v.set("id",     veh.get("id") if i == 0 else f"{veh.get('id')}_seg{i}")
            v.set("type",   veh.get("type", "bus"))
            v.set("depart", f"{dep:.2f}")
            v.set("departSpeed", "0.00")

            for child in veh:
                if child.tag == "param":
                    v.append(child)

            ET.SubElement(v, "route").set("edges", " ".join(seg))

            for child in veh:
                if child.tag in ("param", "route"):
                    continue
                if child.tag == "stop":
                    bs_id = child.get("busStop")
                    if bs_id is not None and bs_id not in valid_busstops:
                        continue
                    lane = child.get("lane")
                    if lane:
                        lane_edge = lane.rsplit("_", 1)[0]
                    elif bs_id is not None:
                        lane_edge = busstop_to_edge.get(bs_id)
                    else:
                        lane_edge = None
                    if lane_edge is not None and lane_edge not in seg_set:
                        continue
                    v.append(child)
                else:
                    v.append(child)
            kept += 1

    vehicles = out_root.findall("vehicle")
    vehicles.sort(key=lambda v: float(v.get("depart", "0")))
    for v in vehicles:
        out_root.remove(v)
    for v in vehicles:
        out_root.append(v)

    indented_xml_write(out_root, out_path)
    return kept


# =============================================================================
# AGENTS / OD / SUMOCFG
# =============================================================================

def write_agents_csv(records: List[dict], out_path: str) -> Tuple[int, int, int]:
    """
    Write agents.csv in the URB/RouteRL format expected by iql and iql_arh:
        id,start_time,origin,destination,kind

    - `id` is the clean 0..N-1 agent index (matches agent_id in demand_routes.csv)
    - origin / destination are integer indices into the od_*.txt lists
    - the original LuST vehicle id is preserved in demand_routes.csv (source_id)
    """
    seen = set()
    dupes = []
    for r in records:
        if r["id"] in seen:
            dupes.append(r["id"])
        seen.add(r["id"])
    if dupes:
        raise RuntimeError(
            f"write_agents_csv: duplicate agent ids detected ({len(dupes)}). "
            f"First few: {dupes[:5]}"
        )

    unique_origins      = sorted({r["edges"][0]  for r in records})
    unique_destinations = sorted({r["edges"][-1] for r in records})
    o_idx = {e: i for i, e in enumerate(unique_origins)}
    d_idx = {e: i for i, e in enumerate(unique_destinations)}

    lines = ["id,start_time,origin,destination,kind"]
    for i, r in enumerate(records):
        lines.append(
            f"{i},{r['depart']:.6f},"
            f"{int(o_idx[r['edges'][0]])},{int(d_idx[r['edges'][-1]])},Human"
        )
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return len(records), len(unique_origins), len(unique_destinations)


def write_demand_routes_csv(records: List[dict], out_path: str) -> int:
    """
    Write demand_routes.csv with the same columns as the Ingolstadt model:
        agent_id,source_id,route_id,probability,free_flow_time,edges
    """
    lines = ["agent_id,source_id,route_id,probability,free_flow_time,edges"]
    for i, r in enumerate(records):
        fft = float(r.get("free_flow_time", 0.0))
        lines.append(
            f"{i},{r['orig_id']},0,1.0,{fft},{' '.join(r['edges'])}"
        )
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return len(records)


def write_od_txt(records: List[dict], network_name: str,
                 out_path: str) -> Tuple[int, int]:
    """Write the O/D file as pretty-printed JSON (one entry per line)."""
    unique_origins      = sorted({r["edges"][0]  for r in records})
    unique_destinations = sorted({r["edges"][-1] for r in records})
    data = {
        "origins":      unique_origins,
        "destinations": unique_destinations,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(json.dumps(data, indent=2))
        f.write("\n")
    return len(unique_origins), len(unique_destinations)


def write_sumocfg(network_name: str, output_dir: str,
                  route_files: List[str],
                  additional_files: List[str],
                  sim_params: Dict[str, int]) -> str:
    processing_lines = [f'        <{k} value="{v}"/>' for k, v in sim_params.items()]
    processing_xml = "\n".join(processing_lines)

    cfg = f"""<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
               xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
    <input>
        <net-file value="{network_name}.net.xml"/>
        <route-files value="{','.join(route_files)}"/>
        <additional-files value="{','.join(additional_files)}"/>
    </input>
    <time>
        <begin value="0"/>
    </time>
    <output>
        <statistic-output value="stats.xml"/>
        <summary-output value="summary.xml"/>
        <tripinfo-output value="tripinfo.xml"/>
    </output>
    <processing>
{processing_xml}
    </processing>
    <report>
        <no-step-log value="true"/>
        <duration-log.statistics value="true"/>
        <message-log value="sumo_messages.log"/>
        <error-log value="sumo_errors.log"/>
    </report>
</configuration>
"""
    path = os.path.join(output_dir, "custom.sumocfg")
    with open(path, "w", encoding="utf-8") as f:
        f.write(cfg)
    return path


# =============================================================================
# URB POST-PROCESSING
# =============================================================================

def extract_edge_types(cropped_net_path: str, source_net_path: str,
                       typ_path: str) -> int:
    """Reconstruct <type> definitions from the edges of the cropped network."""
    crop_root = ET.parse(cropped_net_path).getroot()

    ATTRS = ["priority", "numLanes", "speed", "disallow", "allow",
             "oneway", "width", "spreadType"]

    groups: Dict[str, Dict[str, Counter]] = {}

    for e in crop_root.findall("edge"):
        if (e.get("id") or "").startswith(":"):
            continue
        tid = e.get("type")
        if not tid:
            continue
        bucket = groups.setdefault(tid, {a: Counter() for a in ATTRS})
        for a in ATTRS:
            v = e.get(a)
            if v is not None:
                bucket[a][v] += 1
        if not bucket["numLanes"]:
            lanes = e.findall("lane")
            if lanes:
                bucket["numLanes"][str(len(lanes))] += 1

    if not groups:
        return 0

    typ_root = ET.Element("types")
    typ_root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    typ_root.set("xsi:noNamespaceSchemaLocation",
                 "http://sumo.dlr.de/xsd/types_file.xsd")

    for tid in sorted(groups):
        attrs = {}
        for a in ATTRS:
            c = groups[tid].get(a)
            if c and len(c) > 0:
                attrs[a] = c.most_common(1)[0][0]
        t = ET.SubElement(typ_root, "type")
        t.set("id", tid)
        for a in ATTRS:
            if a in attrs:
                t.set(a, attrs[a])

    indented_xml_write(typ_root, typ_path)
    return len(groups)


def update_netccfg_with_typ(netccfg_path: str, network_name: str) -> bool:
    if not os.path.exists(netccfg_path):
        return False
    tree = ET.parse(netccfg_path)
    root = tree.getroot()
    inp = root.find("input")
    if inp is None:
        inp = ET.SubElement(root, "input")
    for existing in inp.findall("type-files"):
        inp.remove(existing)
    tf = ET.SubElement(inp, "type-files")
    tf.set("value", f"{network_name}.typ.xml")
    indented_xml_write(root, netccfg_path)
    return True


def merge_vtypes_into_routes(routes_path: str, vtypes_path: str) -> int:
    if not os.path.exists(vtypes_path) or not os.path.exists(routes_path):
        return 0
    vt_root = ET.parse(vtypes_path).getroot()
    vtypes = vt_root.findall("vType")
    if not vtypes:
        return 0

    tree = ET.parse(routes_path)
    root = tree.getroot()
    existing = list(root)
    for child in existing:
        root.remove(child)
    for vt in vtypes:
        root.append(copy.deepcopy(vt))
    for child in existing:
        root.append(child)

    indented_xml_write(root, routes_path)
    return len(vtypes)


def strip_bus_stops_from_routes(routes_path: str) -> int:
    if not os.path.exists(routes_path):
        return 0
    tree = ET.parse(routes_path)
    root = tree.getroot()
    removed = 0
    for veh in root.findall("vehicle"):
        for stop in list(veh.findall("stop")):
            veh.remove(stop)
            removed += 1
    if removed:
        indented_xml_write(root, routes_path)
    return removed


def write_metadata_json(out_path: str, *,
                        name: str, description: str,
                        geo: Tuple[float, float, float, float],
                        t0: float, t1: float, preload_hours: float,
                        entry_estimator: str, outside_insertion: str,
                        tls_mode: str, sim_preset: str,
                        network_stats: dict, demand_stats: dict,
                        sumo_stats: Optional[dict] = None,
                        od_filter: Optional[dict] = None) -> None:
    meta = {
        "name": name,
        "description": description,
        "source": "LuST Scenario (Luxembourg SUMO Traffic)",
        "generation": {
            "tool": "build_lux_crop.py",
            "entry_estimator": entry_estimator,
            "outside_insertion": outside_insertion,
            "tls_mode": tls_mode,
            "sim_preset": sim_preset,
        },
        "bounding_box": {
            "x_min": geo[0], "x_max": geo[1],
            "y_min": geo[2], "y_max": geo[3],
            "width_m": geo[1] - geo[0],
            "height_m": geo[3] - geo[2],
        },
        "time_window": {
            "start_hour": t0,
            "end_hour": t1,
            "duration_minutes": int(round((t1 - t0) * 60)),
            "preload_hours": preload_hours,
        },
        "network_stats": network_stats,
        "demand_stats": demand_stats,
    }
    if od_filter is not None:
        meta["od_filter"] = od_filter
    if sumo_stats is not None:
        meta["sumo_validation"] = sumo_stats

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def postprocess_for_urb(output_dir: str, network_name: str,
                        vtypes_mode: str, busstops_mode: str,
                        extract_typ_flag: bool,
                        cleanup_outputs: bool,
                        keep_sumocfg: bool,
                        sumo_ran: bool,
                        source_net_path: Optional[str] = None) -> dict:
    summary = {}

    net_path = os.path.join(output_dir, f"{network_name}.net.xml")
    typ_path = os.path.join(output_dir, f"{network_name}.typ.xml")
    if extract_typ_flag and os.path.exists(net_path) and source_net_path:
        n_types = extract_edge_types(net_path, source_net_path, typ_path)
        summary["typ_xml_types"] = n_types
        if n_types > 0:
            print(f"  extracted {n_types} <type> definitions -> {network_name}.typ.xml")
        else:
            print(f"  no <type> definitions found in source network, skipping .typ.xml")

    netccfg = os.path.join(output_dir, f"{network_name}.netccfg")
    if os.path.exists(netccfg) and os.path.exists(typ_path):
        if update_netccfg_with_typ(netccfg, network_name):
            print(f"  updated {network_name}.netccfg to reference .typ.xml")

    net_src = os.path.join(output_dir, f"{network_name}.net.xml")
    rou_dst = os.path.join(output_dir, f"{network_name}.rou.xml")
    old_local = os.path.join(output_dir, "local.0.rou.xml")
    demand_dst = os.path.join(output_dir, "human_demand.rou.xml")

    if os.path.exists(old_local) and not os.path.exists(demand_dst):
        os.rename(old_local, demand_dst)
        print(f"  renamed local.0.rou.xml -> human_demand.rou.xml")

    if os.path.exists(net_src):
        shutil.copy2(net_src, rou_dst)
        print(f"  copied {network_name}.net.xml -> {network_name}.rou.xml "
              f"(URB convention)")

    old_bus = os.path.join(output_dir, "background_buslines.rou.xml")
    new_bus = os.path.join(output_dir, "background_buses.rou.xml")
    if os.path.exists(old_bus) and not os.path.exists(new_bus):
        os.rename(old_bus, new_bus)
        print(f"  renamed background_buslines.rou.xml -> background_buses.rou.xml")

    vtypes_path = os.path.join(output_dir, "vtypes.add.xml")
    if vtypes_mode == "merge" and os.path.exists(vtypes_path):
        n_h = merge_vtypes_into_routes(demand_dst, vtypes_path)
        n_b = merge_vtypes_into_routes(new_bus, vtypes_path)
        summary["vtypes_merged"] = n_h + n_b
        print(f"  merged {n_h} vTypes into human_demand.rou.xml, "
              f"{n_b} into background_buses.rou.xml")
        os.remove(vtypes_path)
        print(f"  removed vtypes.add.xml (inlined)")
    elif vtypes_mode == "remove" and os.path.exists(vtypes_path):
        os.remove(vtypes_path)
        print(f"  removed vtypes.add.xml (--vtypes-mode remove)")

    busstops_path = os.path.join(output_dir, "busstops.add.xml")
    if busstops_mode == "strip":
        n_removed = strip_bus_stops_from_routes(new_bus)
        summary["bus_stops_removed"] = n_removed
        if n_removed:
            print(f"  removed {n_removed} <stop> from background_buses.rou.xml")
        if os.path.exists(busstops_path):
            os.remove(busstops_path)
            print(f"  removed busstops.add.xml")

    if cleanup_outputs and sumo_ran:
        val_dir = os.path.join(output_dir, "_validation")
        os.makedirs(val_dir, exist_ok=True)
        moved = []
        for fname in ("stats.xml", "summary.xml", "tripinfo.xml",
                      "sumo_messages.log", "sumo_errors.log"):
            p = os.path.join(output_dir, fname)
            if os.path.exists(p):
                shutil.move(p, os.path.join(val_dir, fname))
                moved.append(fname)
        summary["moved_to_validation"] = moved
        if moved:
            print(f"  moved {len(moved)} output/log files into _validation/")

    cfg = os.path.join(output_dir, "custom.sumocfg")
    if os.path.exists(cfg) and not keep_sumocfg:
        if cleanup_outputs:
            val_dir = os.path.join(output_dir, "_validation")
            os.makedirs(val_dir, exist_ok=True)
            shutil.move(cfg, os.path.join(val_dir, "custom.sumocfg"))
            summary["moved_to_validation"] = summary.get("moved_to_validation", []) + ["custom.sumocfg"]
            print(f"  moved custom.sumocfg into _validation/")
        else:
            os.remove(cfg)
            print(f"  removed custom.sumocfg")

    return summary


# =============================================================================
# POST-GENERATION VALIDATION
# =============================================================================

def validate_outputs(output_dir: str, network_name: str,
                     cropped_edge_ids: set,
                     segments_kept: int) -> None:
    print("\n" + "=" * 75)
    print("POST-CROP VALIDATION (reads files back from disk)")
    print("=" * 75)

    required = [
        f"{network_name}.net.xml",
        f"{network_name}.edg.xml",
        f"{network_name}.nod.xml",
        f"{network_name}.con.xml",
        f"{network_name}.tll.xml",
        f"{network_name}.rou.xml",
        "agents.csv",
        "demand_routes.csv",
        f"od_{network_name}.txt",
        "human_demand.rou.xml",
    ]
    print("\nRequired files:")
    for f in required:
        p = os.path.join(output_dir, f)
        print(f"  {'OK ' if os.path.exists(p) else 'MISSING'} {f}")

    print("\nChecking .rou.xml (ordering + orphan edges):")
    for f in sorted(os.listdir(output_dir)):
        if not f.endswith(".rou.xml"):
            continue
        if f == f"{network_name}.rou.xml":
            continue
        path = os.path.join(output_dir, f)
        root = ET.parse(path).getroot()
        route_defs = _collect_route_defs(root)
        deps, missing = [], set()
        for v in root.findall("vehicle"):
            try: deps.append(float(v.get("depart")))
            except (TypeError, ValueError): pass
            edges = _vehicle_edges(v, route_defs)
            if edges:
                missing |= {e for e in edges if e not in cropped_edge_ids}
        is_sorted = all(deps[i] <= deps[i+1] for i in range(len(deps)-1)) if deps else True
        rng = f"[{min(deps):.0f}, {max(deps):.0f}]" if deps else "n/a"
        ok = (not missing) and is_sorted
        print(f"  {'OK ' if ok else 'ERR'} {f:35s} {len(deps):5d} veh "
              f"depart={rng} sorted={is_sorted} orphan={len(missing)}")

    agents_path = os.path.join(output_dir, "agents.csv")
    if os.path.exists(agents_path):
        with open(agents_path, encoding="utf-8") as f:
            lines = f.read().splitlines()
        rows = [r for r in lines[1:] if r]
        agent_ids = [r.split(",")[0] for r in rows if len(r.split(",")) >= 5]
        n_rows = len(rows)
        n_uniq = len(set(agent_ids))
        ok_veh = (n_rows == n_uniq)
        ok_count = (n_rows == segments_kept)
        print(f"\nagents.csv: {n_rows} rows, {n_uniq} unique ids "
              f"({'OK ' if ok_veh else 'ERR'}), "
              f"count match: {'OK ' if ok_count else 'ERR'}")

    print("=" * 75)


# =============================================================================
# SUMO RUN
# =============================================================================

def run_sumo_validation(cfg_path: str, output_dir: str,
                        sumo_bin: str) -> Optional[dict]:
    print("\n" + "=" * 75)
    print(f"RUNNING SUMO (headless): {sumo_bin}")
    print("=" * 75)
    r = subprocess.run([sumo_bin, "-c", os.path.basename(cfg_path)],
                       cwd=output_dir, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    print(r.stdout[-3000:] if r.stdout else "(no stdout)")
    if r.returncode != 0:
        print("SUMO FAILED:", r.returncode)
        print(r.stderr[-3000:])
        return None

    stats = {}
    stats_path = os.path.join(output_dir, "stats.xml")
    if os.path.exists(stats_path):
        root = ET.parse(stats_path).getroot()
        tp = root.find("teleports")
        if tp is not None:
            stats["teleports"] = int(tp.get("total", 0))
            stats["jam"] = int(tp.get("jam", 0))
            stats["yield"] = int(tp.get("yield", 0))
            stats["wrongLane"] = int(tp.get("wrongLane", 0))
        ts = root.find("vehicleTripStatistics")
        if ts is not None:
            stats["inserted"] = int(ts.get("count", 0))
            stats["duration"] = float(ts.get("duration", 0))
            stats["speed"] = float(ts.get("speed", 0))
            stats["routeLength"] = float(ts.get("routeLength", 0))

    warn_path = os.path.join(output_dir, "sumo_errors.log")
    if os.path.exists(warn_path):
        with open(warn_path, encoding="utf-8", errors="replace") as f:
            warns = [l.strip() for l in f if l.strip().startswith("Warning")]
        stats["warnings"] = len(warns)

    return stats if stats else None


def launch_sumo_gui(cfg_path: str, sumo_gui_bin: str) -> None:
    print(f"\nLaunching sumo-gui on {cfg_path}")
    subprocess.Popen([sumo_gui_bin, "-c", cfg_path])


# =============================================================================
# MAIN
# =============================================================================

def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    base = os.path.abspath(args.base_dir)
    scenario_dir = args.scenario_dir or os.path.join(base, "LuSTScenario", "scenario")
    network_dir  = args.network_dir  or os.path.join(base, "luxembourg_network")

    xmin, xmax, ymin, ymax = args.geo
    if args.output_dir:
        output_dir = os.path.abspath(args.output_dir)
    elif args.network_name != "luxembourg":
        output_dir = os.path.join(base, args.network_name)
    else:
        output_dir = os.path.join(
            base,
            f"luxembourg_custom_{int(xmin)}_{int(xmax)}_{int(ymin)}_{int(ymax)}"
        )

    print("=" * 75)
    print("build_lux_crop")
    print("=" * 75)
    print(f"  scenario  : {scenario_dir}")
    print(f"  network   : {network_dir}")
    print(f"  output    : {output_dir}")
    print(f"  net name  : {args.network_name}")
    print(f"  geo window: x in [{xmin}, {xmax}]  y in [{ymin}, {ymax}]")
    print(f"  time      : [{args.t0}h, {args.t1}h]  (preload {args.preload_hours}h)")
    print(f"  estimator : {args.entry_estimator}")
    print(f"  insertion : {args.outside_insertion}")
    print(f"  TLS mode  : {args.tls_mode}")
    print(f"  SIM preset: {args.sim_preset}")
    print(f"  vtypes    : {args.vtypes_mode}")
    print(f"  busstops  : {args.busstops_mode}")
    print(f"  path filter: "
          f"{'OFF' if args.skip_path_filter else f'ON (min_paths={args.min_paths})'}")

    if args.keep_output:
        os.makedirs(output_dir, exist_ok=True)
        print(f"  output dir: kept (--keep-output)")
    else:
        if os.path.isdir(output_dir):
            print(f"  output dir: wiping existing content")
        wipe_output_dir(output_dir)
        print(f"  output dir: fresh")

    sumo_home = args.sumo_home
    netconvert_bin = os.path.join(sumo_home, "bin", "netconvert.exe")
    if not os.path.exists(netconvert_bin):
        netconvert_bin = os.path.join(sumo_home, "bin", "netconvert")
    sumo_bin = os.path.join(sumo_home, "bin", "sumo.exe")
    if not os.path.exists(sumo_bin):
        sumo_bin = os.path.join(sumo_home, "bin", "sumo")
    sumo_gui_bin = os.path.join(sumo_home, "bin", "sumo-gui.exe")
    if not os.path.exists(sumo_gui_bin):
        sumo_gui_bin = os.path.join(sumo_home, "bin", "sumo-gui")

    if not os.path.exists(netconvert_bin):
        print(f"[error] netconvert not found under {sumo_home}")
        return 2

    T0 = args.t0 * 3600.0
    T1 = args.t1 * 3600.0
    T_END = T1 - T0
    T_PRE = T0 - args.preload_hours * 3600.0

    demand_records = load_all_human_demand(scenario_dir, T_PRE, T1)
    if not demand_records:
        print("[error] no demand loaded.")
        return 2

    wanted = {r["id"] for r in demand_records}
    vehroute_path = first_existing(LUST_VEHROUTE_CANDIDATES, scenario_dir)
    tripinfo_path = first_existing(LUST_TRIPINFO_CANDIDATES, scenario_dir)
    print(f"\nReference files:")
    print(f"  vehroute : {vehroute_path}")
    print(f"  tripinfo : {tripinfo_path}")

    ref_tripinfo = load_tripinfo(tripinfo_path, wanted) if tripinfo_path else {}
    ref_vehroute = load_vehroute(vehroute_path, wanted) if vehroute_path else {}
    if ref_vehroute:
        finished   = sum(1 for v in ref_vehroute.values()
                         if v["exit"] and all(t >= 0 for t in v["exit"]))
        unfinished = sum(1 for v in ref_vehroute.values()
                         if v["exit"] and any(t < 0 for t in v["exit"]))
        print(f"  vehroute coverage: {len(ref_vehroute)} / {len(wanted)}  "
              f"(finished={finished}, unfinished={unfinished})")
    print(f"  tripinfo coverage: {len(ref_tripinfo)} / {len(wanted)}")

    k_global = 1.0
    if ref_tripinfo:
        sel = [t for t in ref_tripinfo.values() if T_PRE <= t["depart"] <= T1]
        denom = sum(max(t["duration"] - t["timeLoss"], 1.0) for t in sel)
        if denom > 0:
            k_global = sum(t["duration"] for t in sel) / denom
    print(f"  K_GLOBAL  = {k_global:.3f}")

    raw_src_net = os.path.join(os.path.dirname(network_dir),
                               "LuSTScenario", "scenario", "lust.net.xml")
    if not os.path.exists(raw_src_net):
        print(f"[error] source network not found: {raw_src_net}")
        return 2

    sanitized_src_net = os.path.join(
        tempfile.gettempdir(),
        f"{args.network_name}_sanitized_source.net.xml",
    )
    n_subs = write_sanitized_source_net(raw_src_net, sanitized_src_net)
    print(f"  source net: {raw_src_net}")
    if n_subs:
        print(f"  sanitized {n_subs} occurrences of '--' prefix -> stripped")
    src_net = sanitized_src_net
    orig_root = ET.parse(src_net).getroot()

    edge_ff = compute_edge_free_flow_times(orig_root)
    edges_in_window = collect_edges_intersecting_window(orig_root, tuple(args.geo))
    print(f"\n  candidate edges (segment/rect): {len(edges_in_window)}")
    print(f"  edge free-flow times            : {len(edge_ff)}")

    print("\nCropping network with netconvert...")
    temp_work = os.path.join(tempfile.gettempdir(), "lux_crop_work")
    os.makedirs(temp_work, exist_ok=True)

    cropped_net_path = crop_network(src_net, args.network_name,
                                    output_dir, edges_in_window,
                                    netconvert_bin, temp_work)

    crop_root = ET.parse(cropped_net_path).getroot()
    cropped_edge_ids = {e.get("id") for e in crop_root.findall("edge")
                        if not (e.get("id") or "").startswith(":")}
    print(f"  final cropped edges: {len(cropped_edge_ids)} "
          f"(dropped vs candidates: {len(edges_in_window - cropped_edge_ids)})")

    if args.tls_mode == "static":
        tll_src = os.path.join(scenario_dir, LUST_TLL_STATIC)
        if os.path.exists(tll_src):
            print("\nBaking static TLS programs into the .net.xml...")
            tll_root = ET.parse(tll_src).getroot()
            n_replaced = bake_static_tls(cropped_net_path, cropped_net_path, tll_root)
            print(f"  replaced tlLogic: {n_replaced}")

    print("\nSegmenting human demand...")
    segments, st = segment_human_demand(
        demand_records, cropped_edge_ids, edge_ff,
        T0, T_END, args.entry_estimator,
        ref_tripinfo=ref_tripinfo or None,
        ref_vehroute=ref_vehroute or None,
        k_global=k_global,
        min_edges=args.min_edges,
        reentry_gap=args.reentry_gap,
    )

    print(f"\n  --- Segmentation summary ---")
    print(f"    input records                = {len(demand_records)}")
    print(f"    trajectories in crop         = {st['trajectories_in_crop']}")
    print(f"    non-split vehicles           = {st['non_split_vehicles']}")
    print(f"    split vehicles               = {st['split_vehicles']}")
    print(f"    segments generated           = {st['segments_generated']}")
    print(f"    dropped (out of time window) = {st.get('dropped_out_of_window', 0)}")
    print(f"    rejected (bad edges)         = {st.get('rejected_bad_edges', 0)}")
    print(f"    segments kept (= agents)     = {st['segments_kept']}")
    print(f"    unique agent IDs             = {st['unique_agent_ids']}")
    print(f"    from outside                 = {st['from_outside']}")

    if not st["invariant_agents_eq_segments"]:
        print("  [fatal] duplicate agent IDs detected. Aborting.")
        return 3

    if "_records_by_source" in st:
        print("\n  Breakdown by demand source:")
        for src in sorted(st["_records_by_source"]):
            n_rec  = st["_records_by_source"].get(src, 0)
            n_ws   = st["_with_segs_by_source"].get(src, 0)
            n_segs = st["_segments_by_source"].get(src, 0)
            print(f"    {src:10s} records={n_rec:6d}  "
                  f"with_crop_segments={n_ws:6d}  agents={n_segs:6d}")

    segments.sort(key=lambda r: r["depart"])

    # --- Path availability filter ------------------------------------------
    od_filter_stats = None
    if not args.skip_path_filter:
        print("\n" + "=" * 75)
        print("FILTERING OD PAIRS BY PATH AVAILABILITY")
        print("=" * 75)
        cropped_con_path = os.path.join(output_dir, f"{args.network_name}.con.xml")
        segments, od_filter_stats = filter_od_by_path_availability(
            segments, cropped_con_path,
            min_paths=args.min_paths,
            max_paths_probe=args.max_paths_probe,
            similarity_threshold=args.path_similarity,
            timeout_s=args.path_timeout,
        )
        print(f"\n  --- OD filter summary ---")
        print(f"    OD pairs total         : {od_filter_stats['od_pairs_total']}")
        print(f"    OD pairs kept          : {od_filter_stats['od_pairs_kept']}  "
              f"(>= {args.min_paths} distinct paths)")
        print(f"    OD pairs dropped       : {od_filter_stats['od_pairs_dropped']}")
        print(f"    Agents before filter   : {od_filter_stats['agents_before']}")
        print(f"    Agents after filter    : {od_filter_stats['agents_after']}")
        dropped_n = od_filter_stats['agents_before'] - od_filter_stats['agents_after']
        if od_filter_stats['agents_before'] > 0:
            pct = 100 * dropped_n / od_filter_stats['agents_before']
            print(f"    Agents dropped         : {dropped_n} ({pct:.1f}%)")
        print(f"    Path-count distribution: "
              f"{od_filter_stats['path_count_distribution']}")
        if len(segments) == 0:
            print("  [fatal] no agents left after path filtering. Aborting.")
            return 3
        # Update stats so validate_outputs uses the filtered count
        st["segments_kept"] = len(segments)

    final_agent_count = len(segments)

    # --- Export URB files ---------------------------------------------------
    print("\nExporting route files...")
    local_rou = os.path.join(output_dir, "local.0.rou.xml")
    export_human_routes(segments, local_rou, args.outside_insertion)
    n_agents, n_o, n_d = write_agents_csv(
        segments, os.path.join(output_dir, "agents.csv"))
    write_demand_routes_csv(
        segments, os.path.join(output_dir, "demand_routes.csv"))
    write_od_txt(segments, args.network_name,
                 os.path.join(output_dir, f"od_{args.network_name}.txt"))
    print(f"  agents.csv          : {n_agents} agents, "
          f"{n_o} origins / {n_d} destinations")

    # --- Background buses ---------------------------------------------------
    print("\nFiltering background buses and bus stops...")
    lane_lengths = load_lane_lengths(cropped_net_path)

    valid_busstops, busstop_to_edge = filter_busstops(
        os.path.join(scenario_dir, LUST_BUSSTOPS_FILE),
        cropped_edge_ids, lane_lengths,
        os.path.join(output_dir, "busstops.add.xml"),
    )

    bus_route_files = []
    for rel in LUST_BUS_FILES:
        src = os.path.join(scenario_dir, rel)
        if not os.path.exists(src):
            continue
        stem = rel.replace(".rou.xml", "")
        out_name = f"background_{stem}.rou.xml"
        out_path = os.path.join(output_dir, out_name)
        n = filter_background_buses(src, cropped_edge_ids, T0, T_END,
                                    out_path, valid_busstops,
                                    busstop_to_edge, edge_ff)
        if n > 0:
            bus_route_files.append(out_name)
            print(f"  {rel:30s} -> {n} segments")
        else:
            if os.path.exists(out_path):
                os.remove(out_path)

    src_vtypes = os.path.join(scenario_dir, LUST_VTYPES_FILE)
    if os.path.exists(src_vtypes):
        shutil.copy2(src_vtypes, os.path.join(output_dir, "vtypes.add.xml"))

    route_files = ["local.0.rou.xml"] + sorted(bus_route_files)
    additional_files = ["vtypes.add.xml"]
    if os.path.exists(os.path.join(output_dir, "busstops.add.xml")):
        additional_files.append("busstops.add.xml")

    sim_params = SIM_PRESETS[args.sim_preset]
    cfg = write_sumocfg(args.network_name, output_dir,
                        route_files, additional_files, sim_params)
    print(f"\n  custom.sumocfg -> routes={route_files} adds={additional_files}")

    sumo_stats = None
    sumo_ran = False
    if args.run_sumo and os.path.exists(sumo_bin):
        sumo_stats = run_sumo_validation(cfg, output_dir, sumo_bin)
        sumo_ran = True
    if args.sumo_gui and os.path.exists(sumo_gui_bin):
        launch_sumo_gui(cfg, sumo_gui_bin)
        sumo_ran = True

    print("\n" + "=" * 75)
    print("POST-PROCESSING FOR URB")
    print("=" * 75)
    post = postprocess_for_urb(
        output_dir, args.network_name,
        vtypes_mode=args.vtypes_mode,
        busstops_mode=args.busstops_mode,
        extract_typ_flag=args.extract_typ,
        cleanup_outputs=args.cleanup_outputs,
        keep_sumocfg=args.keep_sumocfg,
        sumo_ran=sumo_ran,
        source_net_path=src_net,
    )

    if args.write_metadata:
        final_net = ET.parse(
            os.path.join(output_dir, f"{args.network_name}.net.xml")
        ).getroot()
        n_tls = len(final_net.findall("tlLogic"))
        n_edges = len(cropped_edge_ids)

        n_bus_segments = 0
        bus_path = os.path.join(output_dir, "background_buses.rou.xml")
        if os.path.exists(bus_path):
            bus_root = ET.parse(bus_path).getroot()
            n_bus_segments = len(bus_root.findall("vehicle"))

        write_metadata_json(
            os.path.join(output_dir, "metadata.json"),
            name=args.network_name,
            description=f"LuST crop {args.network_name}",
            geo=(xmin, xmax, ymin, ymax),
            t0=args.t0, t1=args.t1,
            preload_hours=args.preload_hours,
            entry_estimator=args.entry_estimator,
            outside_insertion=args.outside_insertion,
            tls_mode=args.tls_mode,
            sim_preset=args.sim_preset,
            network_stats={
                "num_edges": n_edges,
                "num_tls": n_tls,
                "num_origins": n_o,
                "num_destinations": n_d,
                "num_edge_types": post.get("typ_xml_types", 0),
            },
            demand_stats={
                "num_agents": n_agents,
                "num_split_vehicles": st["split_vehicles"],
                "num_from_outside": st["from_outside"],
                "num_bus_segments": n_bus_segments,
            },
            od_filter=od_filter_stats,
            sumo_stats=sumo_stats,
        )
        print(f"  wrote metadata.json")

    if not args.skip_validation:
        validate_outputs(output_dir, args.network_name,
                         cropped_edge_ids, segments_kept=final_agent_count)

    if os.path.exists(sanitized_src_net):
        os.remove(sanitized_src_net)

    print(f"\nDone. Output: {output_dir}")
    print(f"Final agent count after all filters: {final_agent_count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())