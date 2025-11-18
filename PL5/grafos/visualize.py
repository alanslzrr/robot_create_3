import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Dict, Tuple

import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
import tkinter.font as tkfont

GRAPH_FILE = Path(__file__).resolve().parent / "grafo.json"


# ---------------------- Modelo de datos ---------------------- #

@dataclass
class NodeCoord:
    x: float
    y: float
    theta: float = 0.0
    label: Optional[str] = None


@dataclass
class Edge:
    src: int
    dst: int
    weight: float = 0.0


@dataclass
class Graph:
    names: List[str]
    coords: List[Optional[NodeCoord]]  # mismo índice que names
    edges: List[Edge]

    @classmethod
    def load(cls, path: Path = GRAPH_FILE) -> "Graph":
        if not path.exists():
            raise FileNotFoundError(f"No se encontró el archivo: {path}")

        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        names = data.get("nombres", [])
        coords_raw = data.get("coordenadas", [])
        edges_raw = data.get("aristas", [])

        coords: List[Optional[NodeCoord]] = []
        for i, c in enumerate(coords_raw):
            if c is None:
                coords.append(None)
                continue
            x = float(c.get("x", 0.0))
            y = float(c.get("y", 0.0))
            theta = float(c.get("theta", 0.0))
            label = c.get("label") or (names[i] if i < len(names) else f"Nodo {i}")
            coords.append(NodeCoord(x=x, y=y, theta=theta, label=label))

        if len(coords) < len(names):
            coords.extend([None] * (len(names) - len(coords)))

        edges: List[Edge] = []
        for e in edges_raw:
            try:
                src = int(e.get("from"))
                dst = int(e.get("to"))
            except (TypeError, ValueError):
                continue
            weight = float(e.get("weight", 0.0))
            edges.append(Edge(src=src, dst=dst, weight=weight))

        return cls(names=names, coords=coords, edges=edges)

    def save(self, path: Path = GRAPH_FILE) -> None:
        nombres = self.names
        coords_out: List[Optional[dict]] = []

        for i, coord in enumerate(self.coords):
            if coord is None:
                coords_out.append(None)
                continue
            label = coord.label or (self.names[i] if i < len(self.names) else f"Nodo {i}")
            coords_out.append(
                {
                    "x": float(coord.x),
                    "y": float(coord.y),
                    "theta": float(coord.theta),
                    "label": label,
                }
            )

        aristas = [
            {"from": e.src, "to": e.dst, "weight": e.weight}
            for e in self.edges
        ]

        data = {
            "nombres": nombres,
            "coordenadas": coords_out,
            "aristas": aristas,
        }

        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)


@dataclass
class Transform:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    width: int
    height: int
    padding: int = 60

    def world_to_screen(self, x: float, y: float) -> Tuple[float, float]:
        if self.max_x == self.min_x:
            sx = self.width / 2
        else:
            sx = self.padding + (x - self.min_x) / (self.max_x - self.min_x) * (
                self.width - 2 * self.padding
            )

        if self.max_y == self.min_y:
            sy = self.height / 2
        else:
            sy = self.padding + (y - self.min_y) / (self.max_y - self.min_y) * (
                self.height - 2 * self.padding
            )

        return sx, self.height - sy

    def screen_to_world(self, sx: float, sy_canvas: float) -> Tuple[float, float]:
        sy = self.height - sy_canvas

        if self.width - 2 * self.padding <= 0 or self.height - 2 * self.padding <= 0:
            return 0.0, 0.0

        if self.max_x == self.min_x:
            x = self.min_x
        else:
            x = self.min_x + (sx - self.padding) / (self.width - 2 * self.padding) * (
                self.max_x - self.min_x
            )

        if self.max_y == self.min_y:
            y = self.min_y
        else:
            y = self.min_y + (sy - self.padding) / (self.height - 2 * self.padding) * (
                self.max_y - self.min_y
            )

        return x, y


# ---------------------- Editor gráfico ---------------------- #

class GraphEditorApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()

        self.title("Editor interactivo de grafos")
        self.geometry("1100x650")
        self.minsize(900, 550)

        # Fuentes personalizadas para un look más moderno (Windows suele tener Segoe UI)
        self.font_base = tkfont.Font(family="Segoe UI", size=10)
        self.font_small = tkfont.Font(family="Segoe UI", size=9)
        self.font_small_bold = tkfont.Font(family="Segoe UI", size=8, weight="bold")
        self.font_title = tkfont.Font(family="Segoe UI", size=13, weight="bold")

        try:
            self.graph = Graph.load()
        except FileNotFoundError as e:
            messagebox.showerror("Error", str(e))
            self.graph = Graph(names=[], coords=[], edges=[])

        self.node_items: Dict[int, Dict[str, int]] = {}
        self.edge_items: List[Dict[str, int]] = []
        self._item_to_node: Dict[int, int] = {}
        self._line_to_edge_index: Dict[int, int] = {}

        self._transform: Optional[Transform] = None
        self._drag_node_index: Optional[int] = None
        self._drag_offset: Tuple[float, float] = (0.0, 0.0)
        self._selected_node_index: Optional[int] = None

        self.status_var = tk.StringVar(value="Listo")

        self._configure_style()
        self._build_ui()
        self._bind_shortcuts()
        self._refresh_lists()
        self._redraw_canvas()

    # ---------- UI ---------- #

    def _configure_style(self) -> None:
        """Configura un tema oscuro moderno para la aplicación."""
        self.configure(bg="#020617")
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            # Si no está disponible, usamos el tema por defecto.
            pass

        base_bg = "#020617"
        surface_bg = "#020617"
        accent = "#1d4ed8"
        accent_hover = "#2563eb"
        text_primary = "#e5e7eb"
        text_muted = "#9ca3af"

        style.configure("TFrame", background=base_bg)
        style.configure("Main.TFrame", background=base_bg)
        style.configure("Surface.TFrame", background=surface_bg)
        style.configure("TopBar.TFrame", background="#020617")

        style.configure(
            "TLabel",
            background=base_bg,
            foreground=text_primary,
        )
        style.configure(
            "Muted.TLabel",
            background=base_bg,
            foreground=text_muted,
        )
        style.configure(
            "TLabelframe",
            background=surface_bg,
            foreground=text_primary,
        )
        style.configure(
            "TLabelframe.Label",
            background=surface_bg,
            foreground=text_primary,
        )

        style.configure(
            "TButton",
            padding=6,
            relief=tk.FLAT,
        )
        style.map(
            "TButton",
            foreground=[("!disabled", text_primary)],
            background=[("!disabled", accent), ("active", accent_hover)],
        )

    def _build_ui(self) -> None:
        main = ttk.Frame(self, style="Main.TFrame")
        main.pack(fill=tk.BOTH, expand=True)

        # Barra superior
        top_bar = ttk.Frame(main, style="TopBar.TFrame")
        top_bar.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(10, 5))

        ttk.Label(
            top_bar,
            text="Editor interactivo de grafos",
            font=self.font_title,
        ).pack(side=tk.LEFT)

        ttk.Label(
            top_bar,
            text="Click derecho en el lienzo para añadir nodos, arrastra para mover.",
            style="Muted.TLabel",
        ).pack(side=tk.LEFT, padx=12)

        ttk.Button(top_bar, text="Guardar grafo", command=self._save).pack(
            side=tk.RIGHT
        )

        # Contenedor principal (tres columnas)
        content = ttk.Frame(main, style="Surface.TFrame")
        content.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=(0, 5))

        left_frame = ttk.Frame(content, style="Surface.TFrame")
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8), pady=5)

        center_frame = ttk.Frame(content, style="Surface.TFrame")
        center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=5)

        right_frame = ttk.Frame(content, style="Surface.TFrame")
        right_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 0), pady=5)

        ttk.Label(left_frame, text="Nodos").pack(anchor="w")
        nodes_list_frame = ttk.Frame(left_frame, style="Surface.TFrame")
        nodes_list_frame.pack(fill=tk.Y)
        self.list_nodes = tk.Listbox(
            nodes_list_frame,
            width=32,
            height=20,
            bg="#020617",
            fg="#e5e7eb",
            selectbackground="#1d4ed8",
            selectforeground="#f9fafb",
            highlightthickness=1,
            highlightbackground="#1e293b",
            relief=tk.FLAT,
        )
        self.list_nodes.pack(side=tk.LEFT, fill=tk.Y)
        nodes_scroll = ttk.Scrollbar(
            nodes_list_frame, orient=tk.VERTICAL, command=self.list_nodes.yview
        )
        nodes_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_nodes.configure(yscrollcommand=nodes_scroll.set)
        self.list_nodes.bind("<<ListboxSelect>>", self._on_node_select)

        node_edit = ttk.LabelFrame(left_frame, text="Editar nodo")
        node_edit.pack(fill=tk.X, pady=5)

        ttk.Label(node_edit, text="Nombre:").grid(row=0, column=0, sticky="e")
        self.entry_name = ttk.Entry(node_edit, width=20)
        self.entry_name.grid(row=0, column=1, sticky="w")

        ttk.Label(node_edit, text="x:").grid(row=1, column=0, sticky="e")
        self.entry_x = ttk.Entry(node_edit, width=10)
        self.entry_x.grid(row=1, column=1, sticky="w")

        ttk.Label(node_edit, text="y:").grid(row=2, column=0, sticky="e")
        self.entry_y = ttk.Entry(node_edit, width=10)
        self.entry_y.grid(row=2, column=1, sticky="w")

        ttk.Label(node_edit, text="θ:").grid(row=3, column=0, sticky="e")
        self.entry_theta = ttk.Entry(node_edit, width=10)
        self.entry_theta.grid(row=3, column=1, sticky="w")

        btn_frame_node = ttk.Frame(node_edit)
        btn_frame_node.grid(row=4, column=0, columnspan=2, pady=5)

        ttk.Button(btn_frame_node, text="Añadir nodo", command=self._add_node).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame_node, text="Actualizar nodo", command=self._update_node).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame_node, text="Eliminar nodo", command=self._delete_node).pack(
            side=tk.LEFT, padx=2
        )

        self.canvas = tk.Canvas(
            center_frame,
            bg="#020617",
            highlightthickness=0,
            bd=0,
            relief=tk.FLAT,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        ttk.Label(right_frame, text="Aristas").pack(anchor="w")
        edges_list_frame = ttk.Frame(right_frame, style="Surface.TFrame")
        edges_list_frame.pack(fill=tk.Y)
        self.list_edges = tk.Listbox(
            edges_list_frame,
            width=42,
            height=20,
            bg="#020617",
            fg="#e5e7eb",
            selectbackground="#1d4ed8",
            selectforeground="#f9fafb",
            highlightthickness=1,
            highlightbackground="#1e293b",
            relief=tk.FLAT,
        )
        self.list_edges.pack(side=tk.LEFT, fill=tk.Y)
        edges_scroll = ttk.Scrollbar(
            edges_list_frame, orient=tk.VERTICAL, command=self.list_edges.yview
        )
        edges_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_edges.configure(yscrollcommand=edges_scroll.set)
        self.list_edges.bind("<<ListboxSelect>>", self._on_edge_select)

        edge_edit = ttk.LabelFrame(right_frame, text="Editar arista")
        edge_edit.pack(fill=tk.X, pady=5)

        ttk.Label(edge_edit, text="Desde (índice):").grid(row=0, column=0, sticky="e")
        self.entry_from = ttk.Entry(edge_edit, width=10)
        self.entry_from.grid(row=0, column=1, sticky="w")

        ttk.Label(edge_edit, text="Hasta (índice):").grid(row=1, column=0, sticky="e")
        self.entry_to = ttk.Entry(edge_edit, width=10)
        self.entry_to.grid(row=1, column=1, sticky="w")

        ttk.Label(edge_edit, text="Peso:").grid(row=2, column=0, sticky="e")
        self.entry_weight = ttk.Entry(edge_edit, width=10)
        self.entry_weight.grid(row=2, column=1, sticky="w")

        btn_frame_edge = ttk.Frame(edge_edit)
        btn_frame_edge.grid(row=3, column=0, columnspan=2, pady=5)

        ttk.Button(btn_frame_edge, text="Añadir arista", command=self._add_edge).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame_edge, text="Actualizar arista", command=self._update_edge).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame_edge, text="Eliminar arista", command=self._delete_edge).pack(
            side=tk.LEFT, padx=2
        )

        # Barra de estado
        status_bar = ttk.Frame(self, style="Surface.TFrame")
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(
            status_bar,
            textvariable=self.status_var,
            style="Muted.TLabel",
            anchor="w",
        ).pack(side=tk.LEFT, padx=10, pady=4, fill=tk.X, expand=True)

        self.canvas.bind("<Configure>", lambda e: self._redraw_canvas())
        self.canvas.bind("<ButtonPress-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.canvas.bind("<ButtonPress-3>", self._on_canvas_right_click)

        self.canvas.tag_bind("edge_line", "<Button-1>", self._on_edge_click)

    def _bind_shortcuts(self) -> None:
        self.bind("<Control-s>", self._on_save_shortcut)
        self.list_nodes.bind("<Delete>", self._delete_node)
        self.list_edges.bind("<Delete>", self._delete_edge)

    # ---------- Utilidades ---------- #

    def _get_selected_node_index(self) -> Optional[int]:
        sel = self.list_nodes.curselection()
        if not sel:
            return None
        idx = sel[0]
        if 0 <= idx < len(self.graph.names):
            return idx
        return None

    def _get_selected_edge_index(self) -> Optional[int]:
        sel = self.list_edges.curselection()
        if not sel:
            return None
        idx = sel[0]
        if 0 <= idx < len(self.graph.edges):
            return idx
        return None

    # ---------- Listas ---------- #

    def _refresh_lists(self) -> None:
        self.list_nodes.delete(0, tk.END)
        for i, name in enumerate(self.graph.names):
            coord = self.graph.coords[i] if i < len(self.graph.coords) else None
            if coord is not None:
                desc = f"{i} - {name} (x={coord.x:.2f}, y={coord.y:.2f})"
            else:
                desc = f"{i} - {name}"
            self.list_nodes.insert(tk.END, desc)

        self.list_edges.delete(0, tk.END)
        for idx, e in enumerate(self.graph.edges):
            f = e.src
            t = e.dst
            w = e.weight
            nf = self.graph.names[f] if 0 <= f < len(self.graph.names) else f"#{f}"
            nt = self.graph.names[t] if 0 <= t < len(self.graph.names) else f"#{t}"
            self.list_edges.insert(
                tk.END, f"{idx}: {f} ({nf}) -> {t} ({nt}) [w={w}]"
            )

    # ---------- Canvas ---------- #

    def _redraw_canvas(self) -> None:
        self.canvas.delete("all")
        self._draw_background_grid()
        self.node_items.clear()
        self.edge_items.clear()
        self._item_to_node.clear()
        self._line_to_edge_index.clear()
        self._transform = None

        if not self.graph.names:
            return

        xs = [c.x for c in self.graph.coords if c is not None]
        ys = [c.y for c in self.graph.coords if c is not None]
        if not xs or not ys:
            return

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 500

        self._transform = Transform(min_x, max_x, min_y, max_y, width, height)
        tf = self._transform.world_to_screen

        for idx, e in enumerate(self.graph.edges):
            f = e.src
            t = e.dst
            if not (0 <= f < len(self.graph.coords) and 0 <= t < len(self.graph.coords)):
                continue
            cf = self.graph.coords[f]
            ct = self.graph.coords[t]
            if cf is None or ct is None:
                continue

            x1, y1 = tf(cf.x, cf.y)
            x2, y2 = tf(ct.x, ct.y)
            line_id = self.canvas.create_line(
                x1, y1, x2, y2, fill="#64748b", width=2, arrow=tk.LAST, tags=("edge_line",)
            )
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            weight_text = str(e.weight)
            weight_id = self.canvas.create_text(
                mx,
                my - 8,
                text=weight_text,
                fill="#e5e7eb",
                font=self.font_small_bold,
            )
            self.edge_items.append(
                {"line": line_id, "weight": weight_id, "from": f, "to": t}
            )
            self._line_to_edge_index[line_id] = idx

        r = 18
        for i, name in enumerate(self.graph.names):
            coord = self.graph.coords[i] if i < len(self.graph.coords) else None
            if coord is None:
                continue
            x, y = tf(coord.x, coord.y)
            oval_id = self.canvas.create_oval(
                x - r,
                y - r,
                x + r,
                y + r,
                fill="#38bdf8",
                outline="#0f172a",
                width=2,
            )
            index_id = self.canvas.create_text(
                x,
                y,
                text=f"{i}",
                fill="#0f172a",
                font=self.font_small_bold,
            )
            label_text = coord.label or name
            label_id = self.canvas.create_text(
                x,
                y + r + 12,
                text=label_text,
                fill="#e5e7eb",
                font=self.font_small,
            )

            self.node_items[i] = {"oval": oval_id, "index": index_id, "label": label_id}
            self._item_to_node[oval_id] = i
            self._item_to_node[index_id] = i
            self._item_to_node[label_id] = i

        self._apply_highlight()

    def _draw_background_grid(self) -> None:
        """Dibuja una rejilla sutil en el lienzo para dar contexto visual moderno."""
        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 500

        spacing = 40
        minor_color = "#111827"
        major_color = "#1f2937"

        # Líneas verticales
        for x in range(0, width, spacing):
            color = major_color if x % (spacing * 5) == 0 else minor_color
            self.canvas.create_line(x, 0, x, height, fill=color, width=1)

        # Líneas horizontales
        for y in range(0, height, spacing):
            color = major_color if y % (spacing * 5) == 0 else minor_color
            self.canvas.create_line(0, y, width, y, fill=color, width=1)

    def _get_node_center(self, idx: int) -> Tuple[Optional[float], Optional[float]]:
        node = self.node_items.get(idx)
        if not node:
            return None, None
        x1, y1, x2, y2 = self.canvas.coords(node["oval"])
        return (x1 + x2) / 2, (y1 + y2) / 2

    def _update_edges_for_node(self, idx: int) -> None:
        cx, cy = self._get_node_center(idx)
        if cx is None:
            return
        for edge in self.edge_items:
            f = edge["from"]
            t = edge["to"]
            if not (0 <= f < len(self.graph.coords) and 0 <= t < len(self.graph.coords)):
                continue
            cfx, cfy = self._get_node_center(f)
            ctx, cty = self._get_node_center(t)
            if cfx is None or ctx is None:
                continue
            line_id = edge["line"]
            weight_id = edge["weight"]
            self.canvas.coords(line_id, cfx, cfy, ctx, cty)
            mx, my = (cfx + ctx) / 2, (cfy + cty) / 2
            self.canvas.coords(weight_id, mx, my - 8)

    # ---------- Eventos de lista ---------- #

    def _on_node_select(self, event: Optional[tk.Event]) -> None:
        idx = self._get_selected_node_index()
        if idx is None:
            return

        self._selected_node_index = idx
        self.entry_name.delete(0, tk.END)
        self.entry_name.insert(0, self.graph.names[idx])

        coord = self.graph.coords[idx] if idx < len(self.graph.coords) else None
        self.entry_x.delete(0, tk.END)
        self.entry_y.delete(0, tk.END)
        self.entry_theta.delete(0, tk.END)

        if coord is not None:
            self.entry_x.insert(0, str(coord.x))
            self.entry_y.insert(0, str(coord.y))
            self.entry_theta.insert(0, str(coord.theta))

        self.status_var.set(
            f"Nodo {idx} seleccionado - x={coord.x:.2f}, y={coord.y:.2f}"
            if coord is not None
            else f"Nodo {idx} seleccionado"
        )

        self._apply_highlight()

    def _on_edge_select(self, event: Optional[tk.Event]) -> None:
        idx = self._get_selected_edge_index()
        if idx is None:
            return
        e = self.graph.edges[idx]
        self.entry_from.delete(0, tk.END)
        self.entry_from.insert(0, str(e.src))
        self.entry_to.delete(0, tk.END)
        self.entry_to.insert(0, str(e.dst))
        self.entry_weight.delete(0, tk.END)
        self.entry_weight.insert(0, str(e.weight))

        self.status_var.set(f"Arista {idx} seleccionada: {e.src} -> {e.dst} (w={e.weight})")

    # ---------- Nodos ---------- #

    def _add_node(self) -> None:
        name = self.entry_name.get().strip() or f"Nodo {len(self.graph.names)}"
        try:
            x = float(self.entry_x.get()) if self.entry_x.get() else 0.0
            y = float(self.entry_y.get()) if self.entry_y.get() else 0.0
            theta = float(self.entry_theta.get()) if self.entry_theta.get() else 0.0
        except ValueError:
            messagebox.showerror("Error", "x, y y θ deben ser números.")
            return

        self.graph.names.append(name)
        self.graph.coords.append(NodeCoord(x=x, y=y, theta=theta, label=name))
        self._refresh_lists()
        self._redraw_canvas()

    def _update_node(self) -> None:
        idx = self._get_selected_node_index()
        if idx is None:
            messagebox.showwarning("Aviso", "Selecciona un nodo para actualizar.")
            return

        name = self.entry_name.get().strip() or self.graph.names[idx]
        try:
            x = float(self.entry_x.get()) if self.entry_x.get() else 0.0
            y = float(self.entry_y.get()) if self.entry_y.get() else 0.0
            theta = float(self.entry_theta.get()) if self.entry_theta.get() else 0.0
        except ValueError:
            messagebox.showerror("Error", "x, y y θ deben ser números.")
            return

        self.graph.names[idx] = name

        if idx >= len(self.graph.coords):
            self.graph.coords.extend([None] * (idx - len(self.graph.coords) + 1))

        coord = self.graph.coords[idx]
        if coord is None:
            coord = NodeCoord(x=x, y=y, theta=theta, label=name)
            self.graph.coords[idx] = coord
        else:
            coord.x = x
            coord.y = y
            coord.theta = theta
            coord.label = name

        self._refresh_lists()
        self._redraw_canvas()

    def _delete_node(self, event: Optional[tk.Event] = None) -> None:
        idx = self._get_selected_node_index()
        if idx is None:
            messagebox.showwarning("Aviso", "Selecciona un nodo para eliminar.")
            return

        if not messagebox.askyesno(
            "Confirmar",
            f"¿Seguro que quieres eliminar el nodo {idx} - {self.graph.names[idx]} y sus aristas asociadas?",
        ):
            return

        self.graph.names.pop(idx)
        if idx < len(self.graph.coords):
            self.graph.coords.pop(idx)

        nuevas_aristas: List[Edge] = []
        for e in self.graph.edges:
            f = e.src
            t = e.dst
            if f == idx or t == idx:
                continue
            if f > idx:
                f -= 1
            if t > idx:
                t -= 1
            nuevas_aristas.append(Edge(src=f, dst=t, weight=e.weight))

        self.graph.edges = nuevas_aristas

        self._selected_node_index = None
        self._refresh_lists()
        self._redraw_canvas()

    # ---------- Aristas ---------- #

    def _add_edge(self) -> None:
        try:
            f = int(self.entry_from.get())
            t = int(self.entry_to.get())
            w = float(self.entry_weight.get() or 0.0)
        except ValueError:
            messagebox.showerror("Error", "from, to deben ser enteros y weight un número.")
            return

        if not (0 <= f < len(self.graph.names) and 0 <= t < len(self.graph.names)):
            messagebox.showerror("Error", "Índices de nodo fuera de rango.")
            return

        self.graph.edges.append(Edge(src=f, dst=t, weight=w))
        self._refresh_lists()
        self._redraw_canvas()

    def _update_edge(self) -> None:
        idx = self._get_selected_edge_index()
        if idx is None:
            messagebox.showwarning("Aviso", "Selecciona una arista para actualizar.")
            return

        try:
            f = int(self.entry_from.get())
            t = int(self.entry_to.get())
            w = float(self.entry_weight.get() or 0.0)
        except ValueError:
            messagebox.showerror("Error", "from, to deben ser enteros y weight un número.")
            return

        if not (0 <= f < len(self.graph.names) and 0 <= t < len(self.graph.names)):
            messagebox.showerror("Error", "Índices de nodo fuera de rango.")
            return

        self.graph.edges[idx] = Edge(src=f, dst=t, weight=w)
        self._refresh_lists()
        self._redraw_canvas()

    def _delete_edge(self, event: Optional[tk.Event] = None) -> None:
        idx = self._get_selected_edge_index()
        if idx is None:
            messagebox.showwarning("Aviso", "Selecciona una arista para eliminar.")
            return
        self.graph.edges.pop(idx)
        self._refresh_lists()
        self._redraw_canvas()

    # ---------- Guardado ---------- #

    def _save(self) -> None:
        try:
            self.graph.save()
            self.status_var.set(f"Grafo guardado en {GRAPH_FILE}")
            messagebox.showinfo("Guardado", f"Grafo guardado en {GRAPH_FILE}")
        except Exception as e:
            self.status_var.set("Error al guardar el grafo")
            messagebox.showerror("Error", f"No se pudo guardar el grafo:\n{e}")

    def _on_save_shortcut(self, event: tk.Event) -> str:
        self._save()
        return "break"

    # ---------- Canvas: interacción ---------- #

    def _on_canvas_press(self, event: tk.Event) -> None:
        item = self.canvas.find_closest(event.x, event.y)
        if not item:
            self._drag_node_index = None
            return
        item_id = item[0]
        idx = self._item_to_node.get(item_id)
        if idx is None:
            self._drag_node_index = None
            return

        self._drag_node_index = idx
        cx, cy = self._get_node_center(idx)
        if cx is None:
            self._drag_node_index = None
            return

        self._drag_offset = (event.x - cx, event.y - cy)

        self.list_nodes.selection_clear(0, tk.END)
        self.list_nodes.selection_set(idx)
        self.list_nodes.see(idx)
        self._selected_node_index = idx
        self._on_node_select(None)

    def _on_canvas_drag(self, event: tk.Event) -> None:
        if self._drag_node_index is None:
            return
        idx = self._drag_node_index
        node = self.node_items.get(idx)
        if not node:
            return

        cx, cy = self._get_node_center(idx)
        if cx is None:
            return

        target_x = event.x - self._drag_offset[0]
        target_y = event.y - self._drag_offset[1]
        dx = target_x - cx
        dy = target_y - cy

        self.canvas.move(node["oval"], dx, dy)
        self.canvas.move(node["index"], dx, dy)
        self.canvas.move(node["label"], dx, dy)

        self._update_edges_for_node(idx)

    def _on_canvas_release(self, event: tk.Event) -> None:
        if self._drag_node_index is None or self._transform is None:
            self._drag_node_index = None
            return
        idx = self._drag_node_index
        self._drag_node_index = None

        cx, cy = self._get_node_center(idx)
        if cx is None:
            return

        x, y = self._transform.screen_to_world(cx, cy)

        if 0 <= idx < len(self.graph.coords):
            coord = self.graph.coords[idx]
            if coord is not None:
                coord.x = x
                coord.y = y

        self._refresh_lists()
        self._redraw_canvas()

    def _on_canvas_right_click(self, event: tk.Event) -> None:
        if self._transform is not None:
            x, y = self._transform.screen_to_world(event.x, event.y)
        else:
            x, y = float(event.x), float(event.y)

        name = f"Nodo {len(self.graph.names)}"
        self.graph.names.append(name)
        self.graph.coords.append(NodeCoord(x=x, y=y, theta=0.0, label=name))
        self._refresh_lists()
        self._redraw_canvas()

    def _on_edge_click(self, event: tk.Event) -> None:
        item = self.canvas.find_withtag("current")
        if not item:
            return
        line_id = item[0]
        idx = self._line_to_edge_index.get(line_id)
        if idx is None:
            return
        self.list_edges.selection_clear(0, tk.END)
        self.list_edges.selection_set(idx)
        self.list_edges.see(idx)
        self._on_edge_select(None)

    # ---------- Resaltado ---------- #

    def _apply_highlight(self) -> None:
        for idx, node in self.node_items.items():
            self.canvas.itemconfig(
                node["oval"], fill="#38bdf8", outline="#0f172a", width=2
            )
        for edge in self.edge_items:
            self.canvas.itemconfig(edge["line"], fill="#64748b", width=2)

        if self._selected_node_index is None:
            return

        sel = self._selected_node_index
        node = self.node_items.get(sel)
        if node:
            self.canvas.itemconfig(
                node["oval"], fill="#f97316", outline="#f97316", width=2
            )

        for edge in self.edge_items:
            if edge["from"] == sel or edge["to"] == sel:
                self.canvas.itemconfig(edge["line"], fill="#f97316", width=3)


if __name__ == "__main__":
    app = GraphEditorApp()
    app.mainloop()
