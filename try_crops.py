#!/usr/bin/env python3
"""
try_crops.py — Test several LuST crop windows.

Each window is drawn on the full LuST map + a zoomed view, with:
  - traffic-light junctions  (orange triangles)
  - roundabouts               (blue circles, if any in the network)
  - highway / motorway edges  (thick blue lines)

Edges are drawn using the real lane geometry (the `shape` attribute in the
.net.xml), not the straight line between their two junctions.  This matches
what you see in sumo-gui and makes curved ramps and interchanges readable.

The SELECTION of what belongs to a crop still uses the junction-to-junction
segment (same as b_crop.py), so the visualization is consistent with
what the pipeline will actually keep.

For each window the script counts:
  agents       — real count, after a --run-pipeline pass
  TLS          — traffic-light junctions inside the window
  RB           — roundabouts inside the window
  HWY          — highway edges intersecting the window
  HWV_EX       — highway ACCESS points: junctions inside the window where
                 at least one highway edge and at least one urban edge meet.
                 Aim for >= 2 to give AVs a real highway on/off choice.

Per-crop time windows
---------------------
Each CropCandidate can carry its own `t0`, `t1` (in hours, e.g. 7.75 = 7h45)
and `preload_hours`.  If left as None, the defaults from build_lux_crop.py
(t0=7.0, t1=9.0, preload_hours=1.0) are used.

Examples:
    CropCandidate("hwy_20min", ..., t0=7.83, t1=8.17)      # 7h50 -> 8h10
    CropCandidate("hwy_40min", ..., t0=8.0,  t1=8.67)      # 8h00 -> 8h40
    CropCandidate("hwy_45min_no_preload", ..., t0=7.75, t1=8.5, preload_hours=0.0)

Usage
-----
python try_crops.py --visualize-only
python try_crops.py --run-pipeline
python try_crops.py --run-pipeline --run-sumo
python try_crops.py --run-pipeline --sumo-gui
python try_crops.py --run-pipeline --clean-builds
python try_crops.py --run-pipeline --windows-file my_windows.jsonl
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Circle


# =============================================================================
# CANDIDATE WINDOWS
# =============================================================================

@dataclass
class CropCandidate:
    name: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    # --- optional per-crop time window ---------------------------------
    t0: Optional[float] = None             # start hour (e.g. 7.83 = 7h50)
    t1: Optional[float] = None             # end hour   (e.g. 8.17 = 8h10)
    preload_hours: Optional[float] = None  # hours before t0 (default 1.0 in pipeline)
    # -------------------------------------------------------------------
    description: str = ""
    n_agents: Optional[int] = None
    n_tls: Optional[int] = None
    n_roundabouts: Optional[int] = None
    n_highway_edges: Optional[int] = None
    n_highway_access: Optional[int] = None
    output_dir: Optional[str] = None

    @property
    def width(self) -> float:
        return self.xmax - self.xmin

    @property
    def height(self) -> float:
        return self.ymax - self.ymin

    def geo(self) -> List[float]:
        return [self.xmin, self.xmax, self.ymin, self.ymax]

    def time_label(self) -> str:
        """Human-readable duration label for the summary table."""
        if self.t0 is None and self.t1 is None:
            return "2h"      # pipeline default
        t0 = self.t0 if self.t0 is not None else 7.0
        t1 = self.t1 if self.t1 is not None else 9.0
        minutes = int(round((t1 - t0) * 60))
        return f"{minutes}min"

    def folder_name(self) -> str:
        # The build pipeline is invoked with --network-name cand.name,
        # so the output folder is simply <base>/<name>.
        return self.name


@dataclass
class EdgeInfo:
    """One SUMO edge with both its visual geometry and its selection info."""
    polyline: List[Tuple[float, float]]   # real shape (from lane shape)
    a: Tuple[float, float]                # from-junction coordinates
    b: Tuple[float, float]                # to-junction coordinates
    is_hwy: bool


def default_candidates() -> List[CropCandidate]:
    """
    Kept windows.

    00-04 : validated urban references (do not touch).
    05-06 : highway-choice windows, reduced to ~2k agents.

    Each highway window carries an explicit 45-min time slice centred on the
    morning peak, which keeps the whole geometry (highway + urban + ramps)
    while cutting the agent count by ~60 %.
    """

    """CropCandidate(
                name="city_west_900",
                xmin=7200, xmax=8100, ymin=5300, ymax=6200,
                description="city west (Belair/Merl) — kept reference",
            ),
            CropCandidate(
                name="city_center_800",
                xmin=7750, xmax=8550, ymin=5450, ymax=6250,
                description="city center (Ville Haute/Gare) — kept reference",
            ),'''

             '''CropCandidate(
                        name="far_west_900",
                        xmin=1500, xmax=2400, ymin=7200, ymax=8100,
                        description="western residential cluster — kept reference",
                    ),
                    CropCandidate(
                        name="north_plateau_1000",
                        xmin=6400, xmax=7400, ymin=8000, ymax=9000,
                        description="northern plateau approach — kept reference",
                    ),"""

    return [
        # ------------------------------------------------------------------
        # 00-04 : URBAN REFERENCES (validated, do not touch)
        # ------------------------------------------------------------------

        CropCandidate(
            name="city_south_900",
            xmin=7450, xmax=8950, ymin=4610, ymax=5700,
            t0=7.75, t1=8.50,
            description="south city (Bonnevoie/Gasperich) — kept reference",
        ),


        # ------------------------------------------------------------------
        # 05-06 : HIGHWAY-CHOICE WINDOWS (EDIT ME)
        # ------------------------------------------------------------------
        # Target: ~2000 agents + at least 2 highway access points.
        # Time slice: 45 min centred on the morning peak (7h45 -> 8h30).
        # If you want to test without time slicing, remove the t0/t1 lines.
        CropCandidate(
            name="hwy_neast_1400",
            xmin=9000, xmax=12600, ymin=7000, ymax=10000,
            t0=7.75, t1=8.50,
            description="east ring + Kirchberg feeders — 45-min window",
        ),
        CropCandidate(
            name="hwy_center_1300",
            xmin=5350, xmax=10500, ymin=1500, ymax=4200,
            t0=7.75, t1=8.50,
            description="city core + west ring access — 45-min window",
        )
    ]


def load_candidates_from_file(path: str) -> List[CropCandidate]:
    """
    Read windows from a .jsonl file (one JSON per line).

    Recognized keys:
      name, xmin, xmax, ymin, ymax    (required)
      t0, t1, preload_hours           (optional)
      description                     (optional)
    """
    out: List[CropCandidate] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            obj = json.loads(line)
            out.append(CropCandidate(
                name=obj["name"],
                xmin=float(obj["xmin"]), xmax=float(obj["xmax"]),
                ymin=float(obj["ymin"]), ymax=float(obj["ymax"]),
                t0=(float(obj["t0"]) if "t0" in obj and obj["t0"] is not None else None),
                t1=(float(obj["t1"]) if "t1" in obj and obj["t1"] is not None else None),
                preload_hours=(float(obj["preload_hours"])
                               if "preload_hours" in obj
                               and obj["preload_hours"] is not None else None),
                description=obj.get("description", ""),
            ))
    return out


# =============================================================================
# NETWORK PARSING
# =============================================================================

HIGHWAY_SPEED_THRESHOLD = 25.0  # m/s ≈ 90 km/h


def _parse_shape(shape_str: str) -> Optional[List[Tuple[float, float]]]:
    if not shape_str:
        return None
    pts = []
    try:
        for pair in shape_str.split():
            x, y = pair.split(",")
            pts.append((float(x), float(y)))
    except (ValueError, IndexError):
        return None
    return pts if len(pts) >= 2 else None


def load_network(net_path: str):
    root = ET.parse(net_path).getroot()

    # --- nodes ------------------------------------------------------------
    node_xy: Dict[str, Tuple[float, float]] = {}
    node_type: Dict[str, str] = {}
    for j in root.findall("junction"):
        jid = j.get("id")
        if jid is None:
            continue
        try:
            x = float(j.get("x")); y = float(j.get("y"))
        except (TypeError, ValueError):
            continue
        node_xy[jid] = (x, y)
        node_type[jid] = j.get("type", "")

    # --- edges ------------------------------------------------------------
    edges: List[EdgeInfo] = []
    edge_rb_flags: Counter = Counter()
    explicit_rb_edges: List[Tuple[str, str]] = []
    node_edge_flags: Dict[str, List[bool]] = defaultdict(list)

    for e in root.findall("edge"):
        eid = e.get("id")
        if not eid or eid.startswith(":"):
            continue
        a_id, b_id = e.get("from"), e.get("to")
        a = node_xy.get(a_id); b = node_xy.get(b_id)
        if not a or not b:
            continue

        speeds = []
        for lane in e.findall("lane"):
            try:
                speeds.append(float(lane.get("speed", "0")))
            except ValueError:
                pass
        is_hwy = bool(speeds) and max(speeds) >= HIGHWAY_SPEED_THRESHOLD

        first_lane = e.find("lane")
        polyline = None
        if first_lane is not None:
            polyline = _parse_shape(first_lane.get("shape", ""))
        if not polyline:
            polyline = [a, b]

        edges.append(EdgeInfo(polyline=polyline, a=a, b=b, is_hwy=is_hwy))

        if a_id: node_edge_flags[a_id].append(is_hwy)
        if b_id: node_edge_flags[b_id].append(is_hwy)

        rb = e.get("roundabout", "")
        edge_rb_flags[rb if rb else "<absent>"] += 1
        if rb == "true" and a_id and b_id:
            explicit_rb_edges.append((a_id, b_id))

    # --- traffic lights ---------------------------------------------------
    tls_nodes = [node_xy[nid] for nid, t in node_type.items()
                 if t == "traffic_light"]

    # --- roundabouts ------------------------------------------------------
    roundabouts: List[Tuple[float, float, float]] = []
    seen_centers: List[Tuple[float, float]] = []

    def _add_roundabout(center, radius):
        for (cx, cy) in seen_centers:
            if (cx - center[0]) ** 2 + (cy - center[1]) ** 2 < 30 ** 2:
                return
        seen_centers.append(center)
        roundabouts.append((center[0], center[1], radius))

    for nid, t in node_type.items():
        if t == "roundabout" and nid in node_xy:
            _add_roundabout(node_xy[nid], 25.0)

    if explicit_rb_edges:
        parent: Dict[str, str] = {}
        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]; x = parent[x]
            return x
        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb: parent[ra] = rb
        for a, b in explicit_rb_edges:
            for n in (a, b):
                if n not in parent: parent[n] = n
            union(a, b)
        clusters: Dict[str, List[str]] = {}
        for n in parent:
            clusters.setdefault(find(n), []).append(n)
        for nodes in clusters.values():
            pts = [node_xy[n] for n in nodes if n in node_xy]
            if not pts: continue
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            r = max(max(((p[0]-cx)**2 + (p[1]-cy)**2) ** 0.5 for p in pts), 25.0)
            _add_roundabout((cx, cy), r)

    xs = [p[0] for p in node_xy.values()]
    ys = [p[1] for p in node_xy.values()]
    diag = {"junction_types": Counter(node_type.values()),
            "edge_rb_flags": edge_rb_flags}
    return (edges, node_xy, dict(node_edge_flags),
            tls_nodes, roundabouts, xs, ys, diag)


def count_features_in_rect(node_xy, node_edge_flags,
                           tls_nodes, roundabouts,
                           edges: List[EdgeInfo],
                           cand: CropCandidate) -> Tuple[int, int, int, int]:
    xmin, xmax, ymin, ymax = cand.xmin, cand.xmax, cand.ymin, cand.ymax

    n_tls = sum(1 for (x, y) in tls_nodes
                if xmin <= x <= xmax and ymin <= y <= ymax)
    n_rb = sum(1 for (cx, cy, _r) in roundabouts
               if xmin <= cx <= xmax and ymin <= cy <= ymax)

    n_hwy = 0
    for e in edges:
        if e.is_hwy and _seg_intersects_rect(e.a, e.b, xmin, xmax, ymin, ymax):
            n_hwy += 1

    n_access = 0
    for nid, flags in node_edge_flags.items():
        p = node_xy.get(nid)
        if p is None: continue
        x, y = p
        if not (xmin <= x <= xmax and ymin <= y <= ymax): continue
        if any(flags) and any(not f for f in flags) and len(flags) >= 3:
            n_access += 1

    return n_tls, n_rb, n_hwy, n_access


def _seg_intersects_rect(p1, p2, xmin, xmax, ymin, ymax) -> bool:
    x1, y1 = p1; x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    t0, t1 = 0.0, 1.0
    for p, q in [(-dx, x1 - xmin), (dx, xmax - x1),
                 (-dy, y1 - ymin), (dy, ymax - y1)]:
        if p == 0:
            if q < 0: return False
        else:
            r = q / p
            if p < 0:
                if r > t1: return False
                if r > t0: t0 = r
            else:
                if r < t0: return False
                if r < t1: t1 = r
    return True


# =============================================================================
# RENDERING HELPERS
# =============================================================================

def _draw_edges(ax, edges: List[EdgeInfo],
                only_selected: Optional[bool] = None,
                cand: Optional[CropCandidate] = None,
                color_urban="#5a5a5a", color_hwy="steelblue",
                color_selected="crimson",
                lw_urban=0.4, lw_hwy=1.4,
                lw_selected_urban=1.2, lw_selected_hwy=2.2,
                zorder_base=1, zorder_selected=4):
    base_segments, base_colors, base_widths = [], [], []
    sel_segments, sel_colors, sel_widths = [], [], []

    for e in edges:
        inside = (cand is not None and
                  _seg_intersects_rect(e.a, e.b,
                                       cand.xmin, cand.xmax,
                                       cand.ymin, cand.ymax))
        if only_selected is True and not inside: continue
        if only_selected is False and inside: continue

        if inside:
            sel_segments.append(e.polyline)
            sel_colors.append(color_hwy if e.is_hwy else color_selected)
            sel_widths.append(lw_selected_hwy if e.is_hwy else lw_selected_urban)
        else:
            base_segments.append(e.polyline)
            base_colors.append(color_hwy if e.is_hwy else color_urban)
            base_widths.append(lw_hwy if e.is_hwy else lw_urban)

    if base_segments:
        ax.add_collection(LineCollection(base_segments, colors=base_colors,
                                         linewidths=base_widths,
                                         zorder=zorder_base))
    if sel_segments:
        ax.add_collection(LineCollection(sel_segments, colors=sel_colors,
                                         linewidths=sel_widths,
                                         zorder=zorder_selected))


def _plot_features(ax, tls_nodes, roundabouts,
                   tls_size=40.0, rb_alpha=0.35, zorder=6):
    if tls_nodes:
        xs = [p[0] for p in tls_nodes]; ys = [p[1] for p in tls_nodes]
        ax.scatter(xs, ys, marker="^", s=tls_size,
                   facecolor="darkorange", edgecolor="black",
                   linewidths=0.4, zorder=zorder)
    for (cx, cy, r) in roundabouts:
        ax.add_patch(Circle((cx, cy), r, facecolor="royalblue",
                            edgecolor="navy", alpha=rb_alpha,
                            linewidth=1.2, zorder=zorder))


def _plot_crop_rectangle(ax, cand, color="crimson", fill_alpha=0.12,
                         linestyle="-", zorder=5):
    ax.add_patch(Rectangle((cand.xmin, cand.ymin), cand.width, cand.height,
                           linewidth=2.5, edgecolor=color, facecolor=color,
                           alpha=fill_alpha, linestyle=linestyle, zorder=zorder))


def _legend_handles(with_roundabout: bool = True):
    handles = [
        Line2D([0], [0], color="steelblue", lw=2.0, label="highway"),
        Line2D([0], [0], color="crimson", lw=1.4, label="urban (in crop)"),
        Line2D([0], [0], marker="^", color="w",
               markerfacecolor="darkorange", markeredgecolor="black",
               markersize=9, label="traffic light"),
    ]
    if with_roundabout:
        handles.append(Line2D([0], [0], marker="o", color="w",
                              markerfacecolor="royalblue",
                              markeredgecolor="navy", markersize=11,
                              label="roundabout"))
    return handles


# =============================================================================
# RENDERING — SINGLE (2-PANEL)
# =============================================================================

def render_single(cand: CropCandidate, edges: List[EdgeInfo],
                  tls_nodes, roundabouts, xs, ys, out_png: str):
    has_rb = bool(roundabouts)
    fig, (ax_full, ax_zoom) = plt.subplots(
        1, 2, figsize=(20, 10), gridspec_kw={"width_ratios": [1, 1]})

    # --- LEFT: full map ---------------------------------------------------
    ax_full.set_aspect("equal")
    ax_full.set_xlim(min(xs), max(xs)); ax_full.set_ylim(min(ys), max(ys))
    ax_full.set_xlabel("x (m)"); ax_full.set_ylabel("y (m)")

    _draw_edges(ax_full, edges, only_selected=None, cand=cand,
                color_urban="#5a5a5a", color_hwy="steelblue",
                lw_urban=0.4, lw_hwy=1.4,
                lw_selected_urban=1.2, lw_selected_hwy=2.2)

    _plot_features(ax_full, tls_nodes, roundabouts,
                   tls_size=20.0, rb_alpha=0.30, zorder=7)
    _plot_crop_rectangle(ax_full, cand, color="crimson")

    parts = [f"Full LuST — '{cand.name}'"]
    parts.append(f"T={cand.time_label()}")
    if cand.n_agents is not None: parts.append(f"{cand.n_agents} agents")
    if cand.n_tls is not None: parts.append(f"{cand.n_tls} TLS")
    if cand.n_roundabouts is not None and has_rb:
        parts.append(f"{cand.n_roundabouts} rb")
    if cand.n_highway_edges is not None:
        parts.append(f"{cand.n_highway_edges} hwy")
    if cand.n_highway_access is not None:
        parts.append(f"{cand.n_highway_access} exits")
    ax_full.set_title("  |  ".join(parts), fontsize=12)
    ax_full.legend(handles=_legend_handles(has_rb), loc="upper right",
                   fontsize=9, framealpha=0.9)

    # --- RIGHT: zoom ------------------------------------------------------
    pad = 0.25 * max(cand.width, cand.height)
    ax_zoom.set_aspect("equal")
    ax_zoom.set_xlim(cand.xmin - pad, cand.xmax + pad)
    ax_zoom.set_ylim(cand.ymin - pad, cand.ymax + pad)
    ax_zoom.set_xlabel("x (m)"); ax_zoom.set_ylabel("y (m)")

    _draw_edges(ax_zoom, edges, only_selected=False, cand=cand,
                color_urban="#cccccc", color_hwy="steelblue",
                lw_urban=0.6, lw_hwy=1.0)
    _draw_edges(ax_zoom, edges, only_selected=True, cand=cand,
                color_selected="crimson", color_hwy="steelblue",
                lw_selected_urban=1.4, lw_selected_hwy=2.6)

    tls_zoom = [p for p in tls_nodes
                if cand.xmin - pad <= p[0] <= cand.xmax + pad
                and cand.ymin - pad <= p[1] <= cand.ymax + pad]
    rb_zoom = [(cx, cy, r) for (cx, cy, r) in roundabouts
               if cand.xmin - pad <= cx <= cand.xmax + pad
               and cand.ymin - pad <= cy <= cand.ymax + pad]

    _plot_features(ax_zoom, tls_zoom, rb_zoom,
                   tls_size=110.0, rb_alpha=0.30, zorder=8)
    _plot_crop_rectangle(ax_zoom, cand, color="black", fill_alpha=0.0,
                         linestyle="--", zorder=10)

    parts = [f"Zoom '{cand.name}' ({int(cand.width)}x{int(cand.height)} m)",
             f"T={cand.time_label()}"]
    if cand.n_agents is not None: parts.append(f"{cand.n_agents} agents")
    if cand.n_tls is not None: parts.append(f"{cand.n_tls} TLS")
    if cand.n_roundabouts is not None and has_rb:
        parts.append(f"{cand.n_roundabouts} rb")
    if cand.n_highway_edges is not None:
        parts.append(f"{cand.n_highway_edges} hwy")
    if cand.n_highway_access is not None:
        parts.append(f"{cand.n_highway_access} exits")
    title = "  |  ".join(parts)
    if cand.description: title += f"\n{cand.description}"
    ax_zoom.set_title(title, fontsize=12)
    ax_zoom.legend(handles=_legend_handles(has_rb), loc="upper right",
                   fontsize=9, framealpha=0.9)

    plt.tight_layout(); plt.savefig(out_png, dpi=120); plt.close(fig)


# =============================================================================
# RENDERING — OVERVIEW
# =============================================================================

def render_overview(candidates: List[CropCandidate], edges: List[EdgeInfo],
                    tls_nodes, roundabouts, xs, ys, out_png: str):
    fig, ax = plt.subplots(figsize=(16, 13))
    ax.set_aspect("equal")
    ax.set_xlim(min(xs), max(xs)); ax.set_ylim(min(ys), max(ys))
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("Candidate crop windows on the full LuST map", fontsize=13)

    _draw_edges(ax, edges, only_selected=None, cand=None,
                color_urban="#6a6a6a", color_hwy="steelblue",
                lw_urban=0.35, lw_hwy=1.4)

    _plot_features(ax, tls_nodes, roundabouts,
                   tls_size=14.0, rb_alpha=0.28, zorder=4)

    palette = ["crimson", "darkgreen", "purple", "darkorange",
               "teal", "brown", "deeppink", "navy",
               "olive", "maroon", "slategray", "goldenrod"]

    for i, c in enumerate(candidates):
        color = palette[i % len(palette)]
        ax.add_patch(Rectangle((c.xmin, c.ymin), c.width, c.height,
                               linewidth=2.2, edgecolor=color,
                               facecolor=color, alpha=0.15, zorder=6))
        parts = [c.name, c.time_label()]
        if c.n_agents is not None: parts.append(f"{c.n_agents} ag")
        if c.n_tls is not None: parts.append(f"{c.n_tls} TLS")
        if c.n_highway_access: parts.append(f"{c.n_highway_access} ex")
        label = " / ".join(parts)

        cx = c.xmin + c.width / 2; cy = c.ymin + c.height / 2
        ax.annotate(label, (cx, cy), ha="center", va="bottom",
                    fontsize=8, color=color, weight="bold", zorder=10,
                    xytext=(0, 5), textcoords="offset points",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                              edgecolor=color, alpha=0.9))

    ax.legend(handles=_legend_handles(bool(roundabouts)),
              loc="upper right", fontsize=10, framealpha=0.9)
    plt.tight_layout(); plt.savefig(out_png, dpi=130); plt.close(fig)


# =============================================================================
# PIPELINE RUNNER
# =============================================================================

AGENT_RE = re.compile(r"segments kept \(= agents\)\s*=\s*(\d+)")


def run_pipeline(build_script: str, cand: CropCandidate,
                 extra_args: List[str], out_log: str,
                 base_dir: str) -> int:
    cmd = [
        sys.executable, build_script,
        "--base-dir",     base_dir,
        "--network-name", cand.name,
        "--geo", *[str(v) for v in cand.geo()],
        "--skip-validation",
    ]
    # Optional per-crop time window
    if cand.t0 is not None:
        cmd += ["--t0", str(cand.t0)]
    if cand.t1 is not None:
        cmd += ["--t1", str(cand.t1)]
    if cand.preload_hours is not None:
        cmd += ["--preload-hours", str(cand.preload_hours)]
    cmd += extra_args

    print(f"    $ {' '.join(cmd)}")
    with open(out_log, "w", encoding="utf-8") as logf:
        r = subprocess.run(cmd, capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        logf.write(r.stdout)
        if r.stderr:
            logf.write("\n--- stderr ---\n"); logf.write(r.stderr)

    tail = "\n".join(r.stdout.strip().splitlines()[-6:])
    print(tail)

    if r.returncode != 0:
        print(f"    [error] pipeline failed (code {r.returncode}); see {out_log}")
        return -1

    m = AGENT_RE.search(r.stdout)
    if not m:
        print(f"    [error] could not parse agent count; see {out_log}")
        return -1

    n = int(m.group(1))
    # build_lux_crop with --network-name != "luxembourg" creates <base>/<name>
    cand.output_dir = os.path.join(base_dir, cand.name)
    return n


# =============================================================================
# CLEANUP
# =============================================================================

def clean_crop_tests(path: str) -> None:
    if os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path, exist_ok=True)


def clean_build_dirs(base: str, extra_names: Optional[List[str]] = None) -> int:
    n = 0
    prefixes = ["luxembourg_custom_"]
    if extra_names:
        prefixes.extend(extra_names)
    for name in os.listdir(base):
        p = os.path.join(base, name)
        if not os.path.isdir(p):
            continue
        if any(name == pat or name.startswith(pat) for pat in prefixes):
            shutil.rmtree(p, ignore_errors=True)
            n += 1
    return n


# =============================================================================
# MAIN
# =============================================================================

def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Try several LuST crop windows and visualize them.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--base-dir", default=os.path.abspath("."))
    p.add_argument("--build-script", default=None)
    p.add_argument("--out-dir", default="crop_tests")
    p.add_argument("--windows-file", default=None)
    p.add_argument("--visualize-only", action="store_true")
    p.add_argument("--run-pipeline", action="store_true")
    p.add_argument("--run-sumo", action="store_true")
    p.add_argument("--sumo-gui", action="store_true")
    p.add_argument("--max-agents", type=int, default=2000)
    p.add_argument("--clean-builds", action="store_true")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    base = os.path.abspath(args.base_dir)
    net_path = os.path.join(base, "LuSTScenario", "scenario", "lust.net.xml")

    if not os.path.exists(net_path):
        print(f"[error] network not found: {net_path}")
        return 2

    build_script = args.build_script or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "build_lux_crop.py")
    if not args.visualize_only and not os.path.exists(build_script):
        print(f"[error] build_lux_crop.py not found: {build_script}")
        return 2

    # Load candidates BEFORE cleaning so --clean-builds can target their names.
    candidates = (load_candidates_from_file(args.windows_file)
                  if args.windows_file else default_candidates())

    out_dir = os.path.join(base, args.out_dir)
    clean_crop_tests(out_dir)
    print(f"[clean] wiped {out_dir}")

    if args.clean_builds:
        n = clean_build_dirs(base, extra_names=[c.name for c in candidates])
        print(f"[clean] removed {n} crop folder(s)")

    print("Loading network geometry and features...")
    (edges, node_xy, node_edge_flags,
     tls_nodes, roundabouts, xs, ys, diag) = load_network(net_path)

    n_hwy_edges = sum(1 for e in edges if e.is_hwy)
    print(f"  edges          : {len(edges)}")
    print(f"  highway edges  : {n_hwy_edges}  "
          f"(speed >= {int(HIGHWAY_SPEED_THRESHOLD)} m/s)")
    print(f"  TLS junctions  : {len(tls_nodes)}")
    print(f"  roundabouts    : {len(roundabouts)}")

    if not roundabouts:
        print("  [diag] no roundabouts detected in this .net.xml.")
        print("         junction types found:")
        for t, n in diag["junction_types"].most_common(6):
            print(f"           {t:20s} : {n}")
        print("         edge roundabout flag values:")
        for v, n in diag["edge_rb_flags"].most_common(4):
            print(f"           {v:20s} : {n}")

    for c in candidates:
        tls_in, rb_in, hwy_in, acc_in = count_features_in_rect(
            node_xy, node_edge_flags, tls_nodes, roundabouts, edges, c)
        c.n_tls = tls_in
        c.n_roundabouts = rb_in
        c.n_highway_edges = hwy_in
        c.n_highway_access = acc_in

    print("\n" + "=" * 88)
    print("try_crops — candidate windows")
    print("=" * 88)
    for c in candidates:
        tlab = c.time_label()
        print(f"  {c.name:22s}  geo=({int(c.xmin)}, {int(c.xmax)}, "
              f"{int(c.ymin)}, {int(c.ymax)})  "
              f"{int(c.width)}x{int(c.height)} m  T={tlab:>6s}  "
              f"TLS={c.n_tls:2d}  HWY={c.n_highway_edges:3d}  "
              f"EXITS={c.n_highway_access:2d}"
              + (f"  — {c.description}" if c.description else ""))

    if args.run_pipeline and not args.visualize_only:
        print("\n" + "=" * 88)
        print("Running pipeline for each window...")
        print("=" * 88)

        extra = []
        if args.run_sumo: extra.append("--run-sumo")
        if args.sumo_gui: extra.append("--sumo-gui")

        for c in candidates:
            print(f"\n  [{c.name}]  T={c.time_label()}")
            log = os.path.join(out_dir, f"{c.name}.log")
            n = run_pipeline(build_script, c, extra, log, base)
            c.n_agents = n if n >= 0 else None
            if c.n_agents is not None:
                mark = "OK " if c.n_agents <= args.max_agents else "HIGH"
                print(f"    {mark} {c.n_agents} agents "
                      f"(target <= {args.max_agents})")

    print("\n" + "=" * 88)
    print("Rendering visualizations...")
    print("=" * 88)

    for i, c in enumerate(candidates):
        png = os.path.join(out_dir, f"{i:02d}_{c.name}.png")
        render_single(c, edges, tls_nodes, roundabouts, xs, ys, png)
        print(f"  saved: {png}")

    overview = os.path.join(out_dir, "_overview.png")
    render_overview(candidates, edges, tls_nodes, roundabouts, xs, ys, overview)
    print(f"  saved: {overview}")

    print("\n" + "=" * 88)
    print("Summary")
    print("=" * 88)
    print(f"  {'name':22s}  {'size':>10s}  {'T':>6s}  {'agents':>7s}  "
          f"{'TLS':>4s}  {'HWY':>4s}  {'EXITS':>5s}  output")
    for c in candidates:
        size = f"{int(c.width)}x{int(c.height)}"
        n = str(c.n_agents) if c.n_agents is not None else "—"
        t = str(c.n_tls) if c.n_tls is not None else "—"
        h = str(c.n_highway_edges) if c.n_highway_edges is not None else "—"
        e = str(c.n_highway_access) if c.n_highway_access is not None else "—"
        d = c.output_dir or ""
        print(f"  {c.name:22s}  {size:>10s}  {c.time_label():>6s}  "
              f"{n:>7s}  {t:>4s}  {h:>4s}  {e:>5s}  {d}")

    return 0


if __name__ == "__main__":
    sys.exit(main())