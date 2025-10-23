# nodes_io.py
# Persistencia extendida: nodos/edges JSONL + CSV de segmentos + nav_attempts + versión + agregados

import os, json, time, csv, math
from typing import List, Dict, Tuple, Optional

NODES_DIR = "nodes"
NODES_FILE = os.path.join(NODES_DIR, "nodes.jsonl")
EDGES_FILE = os.path.join(NODES_DIR, "edges.jsonl")
CSV_DIR = os.path.join(NODES_DIR, "logs")
VERSION_FILE = os.path.join(NODES_DIR, "VERSION")

MAP_VERSION = 2  # incrementa si cambias estructura

def _ensure_dir():
    os.makedirs(NODES_DIR, exist_ok=True)

def _ensure_csv_dir():
    os.makedirs(CSV_DIR, exist_ok=True)

def write_version():
    _ensure_dir()
    with open(VERSION_FILE, "w", encoding="utf-8") as f:
        f.write(f"map_format: {MAP_VERSION}\n")

def read_version() -> int:
    try:
        with open(VERSION_FILE, "r", encoding="utf-8") as f:
            for line in f:
                if "map_format" in line:
                    return int(line.split(":")[1].strip())
    except Exception:
        pass
    return 1

def load_jsonl(path: str) -> List[Dict]:
    if not os.path.exists(path):
        return []
    out = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out

def save_jsonl_line(path: str, data: Dict):
    _ensure_dir()
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(data, ensure_ascii=False) + "\n")

def load_nodes() -> List[Dict]:
    nodes = load_jsonl(NODES_FILE)
    nodes.sort(key=lambda n: n.get("id", 0))
    return nodes

def load_edges() -> List[Dict]:
    return load_jsonl(EDGES_FILE)

def next_node_id() -> int:
    nodes = load_nodes()
    return (nodes[-1]["id"] + 1) if nodes else 1

def append_node(x: float, y: float, theta: float, name: str = None, tags=None, quality: float = None, source: str = "teleop") -> Dict:
    node = {
        "id": next_node_id(),
        "name": name or "Nodo",
        "x": float(x),
        "y": float(y),
        "theta": float(theta),
        "tags": tags or [],
        "quality": float(quality) if (quality is not None) else None,
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": source
    }
    save_jsonl_line(NODES_FILE, node)
    write_version()
    return node

def append_edge(node_from: int, node_to: int, segments: List[Dict], agg: Dict = None, quality: float = None):
    edge = {
        "from": int(node_from),
        "to": int(node_to),
        "segments": segments,
        "agg": agg or {},
        "quality": float(quality) if (quality is not None) else None,
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    save_jsonl_line(EDGES_FILE, edge)
    write_version()
    return edge

def nodes_index_by_id() -> Dict[int, Dict]:
    return {n["id"]: n for n in load_nodes()}

def nodes_index_by_name() -> Dict[str, Dict]:
    idx = {}
    for n in load_nodes():
        idx[n["name"].strip().lower()] = n
    return idx

def resolve_node(token: str) -> Optional[Dict]:
    nodes_by_id = nodes_index_by_id()
    nodes_by_name = nodes_index_by_name()
    try:
        nid = int(token)
        return nodes_by_id.get(nid)
    except:
        pass
    return nodes_by_name.get(token.strip().lower())

def neighbors_of(node_id: int) -> List[Dict]:
    edges = load_edges()
    return [e for e in edges if e.get("from") == node_id]

def compute_missing_routes(all_nodes: List[Dict], edges: List[Dict]) -> List[Tuple[int,int]]:
    ids = [n["id"] for n in all_nodes]
    have = {(e["from"], e["to"]) for e in edges}
    missing = []
    for a in ids:
        for b in ids:
            if a == b:
                continue
            if (a, b) not in have:
                missing.append((a, b))
    return missing

def log_edge_segments_csv(node_from: int, node_to: int, segments: List[Dict]) -> str:
    """CSV con métricas planificadas vs odometría de cada segmento."""
    _ensure_csv_dir()
    fname = time.strftime(f"edge_{node_from}_to_{node_to}_%Y%m%d_%H%M%S.csv")
    path = os.path.join(CSV_DIR, fname)
    fieldnames = [
        "idx","state","t","dist_cm","deg","odom_dist_cm","odom_deg","err_dist_cm","err_deg"
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for i, seg in enumerate(segments):
            row = {"idx": i}
            for k in fieldnames:
                if k in seg:
                    row[k] = seg[k]
            row.setdefault("state", "?")
            row.setdefault("t", 0.0)
            row.setdefault("dist_cm", 0.0)
            row.setdefault("deg", 0.0)
            row.setdefault("odom_dist_cm", 0.0)
            row.setdefault("odom_deg", 0.0)
            row.setdefault("err_dist_cm", 0.0)
            row.setdefault("err_deg", 0.0)
            w.writerow(row)
    return path

def log_nav_attempt(target: str,
                    plan_x: float,
                    plan_y: float,
                    plan_theta: Optional[float],
                    start_pose: Tuple[float,float,float],
                    end_pose: Tuple[float,float,float],
                    origin_info: Dict):
    _ensure_csv_dir()
    path = os.path.join(CSV_DIR, "nav_attempts.csv")
    ts = time.strftime("%Y-%m-%dT%H:%M:%S")
    sx, sy, sth = start_pose
    ex, ey, eth = end_pose
    try:
        err_dist = math.hypot(ex - plan_x, ey - plan_y)
        err_deg = eth - (plan_theta if plan_theta is not None else eth)
        while err_deg >= 180.0: err_deg -= 360.0
        while err_deg < -180.0: err_deg += 360.0
    except Exception:
        err_dist = 0.0
        err_deg = 0.0
    header = [
        "ts","target","plan_x","plan_y","plan_theta","start_x","start_y","start_theta",
        "end_x","end_y","end_theta","err_dist_cm","err_deg","origin_type","origin_id"
    ]
    row = {
        "ts": ts,
        "target": target,
        "plan_x": round(float(plan_x), 2),
        "plan_y": round(float(plan_y), 2),
        "plan_theta": round(float(plan_theta), 2) if plan_theta is not None else "",
        "start_x": round(float(sx), 2),
        "start_y": round(float(sy), 2),
        "start_theta": round(float(sth), 2),
        "end_x": round(float(ex), 2),
        "end_y": round(float(ey), 2),
        "end_theta": round(float(eth), 2),
        "err_dist_cm": round(float(err_dist), 2),
        "err_deg": round(float(err_deg), 2),
        "origin_type": origin_info.get("type"),
        "origin_id": (origin_info.get("node") or {}).get("id") if origin_info.get("type") == "node" else "dock"
    }
    write_header = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=header)
        if write_header:
            w.writeheader()
        w.writerow(row)
    return path

def aggregate_edge(segments: List[Dict]) -> Dict:
    """Agregados y estimación de 'calidad' simple basada en error."""
    total_len = sum(abs(seg.get("odom_dist_cm", seg.get("dist_cm", 0.0))) for seg in segments)
    total_rot = sum(abs(seg.get("odom_deg", seg.get("deg", 0.0))) for seg in segments)
    # error medio normalizado
    dist_err = sum(abs(seg.get("err_dist_cm", 0.0)) for seg in segments)
    ang_err = sum(abs(seg.get("err_deg", 0.0)) for seg in segments)
    # calidad simple [0..1]
    denom = (total_len + 1e-6) + 0.5*(total_rot + 1e-6)
    quality = max(0.0, 1.0 - (dist_err + 0.5*ang_err) / (denom + 1e-6))
    return {"len_cm": round(total_len,2), "rot_deg": round(total_rot,2), "quality": round(quality,3)}
