import json
import tkinter as tk
from tkinter import messagebox, Menu
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import List, Optional, Dict, Tuple, Union
import copy
import uuid
import inspect

# ---------------------- LIBRERÍA MODERNA ---------------------- #
import customtkinter as ctk

# Configuración global de apariencia
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("dark-blue")

GRAPH_FILE = Path(__file__).resolve().parent / "grafo.json"
LAYOUT_FILE = Path(__file__).resolve().parent / "layout.json"

# ---------------------- Modelo de datos GRAFO ---------------------- #

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
    coords: List[Optional[NodeCoord]]
    edges: List[Edge]

    @classmethod
    def load(cls, path: Path = GRAPH_FILE) -> "Graph":
        if not path.exists():
            return cls(names=[], coords=[], edges=[])

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
            coords_out.append({
                "x": float(coord.x),
                "y": float(coord.y),
                "theta": float(coord.theta),
                "label": label,
            })

        aristas = [{"from": e.src, "to": e.dst, "weight": e.weight} for e in self.edges]
        data = {"nombres": nombres, "coordenadas": coords_out, "aristas": aristas}

        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    
    def clone(self) -> "Graph":
        names_copy = self.names.copy()
        coords_copy = []
        for c in self.coords:
            if c is None:
                coords_copy.append(None)
            else:
                coords_copy.append(NodeCoord(c.x, c.y, c.theta, c.label))
        edges_copy = [Edge(e.src, e.dst, e.weight) for e in self.edges]
        return Graph(names=names_copy, coords=coords_copy, edges=edges_copy)

# ---------------------- Modelo de datos LAYOUT ---------------------- #

@dataclass
class RefShape:
    def to_dict(self):
        d = asdict(self)
        d['type'] = self.__class__.__name__
        return d
    
    def clone(self):
        return copy.deepcopy(self)

@dataclass
class RefLine(RefShape):
    start: Tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))
    end: Tuple[float, float] = field(default_factory=lambda: (1.0, 1.0))
    color: str = "#95a5a6"
    width: int = 1
    dash: Optional[Tuple[int, int]] = None
    id: str = field(default_factory=lambda: f"line_{uuid.uuid4().hex[:6]}")

@dataclass
class RefRect(RefShape):
    bounds: Tuple[float, float, float, float] = field(default_factory=lambda: (0.0, 0.0, 1.0, 1.0))
    outline: str = "#95a5a6"
    fill: str = ""
    width: int = 1
    id: str = field(default_factory=lambda: f"rect_{uuid.uuid4().hex[:6]}")

@dataclass
class RefText(RefShape):
    pos: Tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))
    text: str = ""
    color: str = "#95a5a6"
    font_size: int = 10
    id: str = field(default_factory=lambda: f"text_{uuid.uuid4().hex[:6]}")

LayoutShapes = Union[RefLine, RefRect, RefText]

@dataclass
class Layout:
    shapes: List[LayoutShapes]
    
    @classmethod
    def load(cls, path: Path) -> "Layout":
        if not path.exists():
            return cls(shapes=[])
        
        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, IOError):
            return cls(shapes=[])
        
        shapes = []
        shape_map = { "RefLine": RefLine, "RefRect": RefRect, "RefText": RefText }
        
        for shape_data in data.get("shapes", []):
            shape_type = shape_data.pop("type", None)
            if shape_type in shape_map:
                try:
                    sig = inspect.signature(shape_map[shape_type])
                    valid_keys = {p.name for p in sig.parameters.values()}
                    filtered_data = {k: v for k, v in shape_data.items() if k in valid_keys}
                    shapes.append(shape_map[shape_type](**filtered_data))
                except (TypeError, ValueError) as e:
                    print(f"Skipping malformed shape: {shape_data}, error: {e}")
                    continue
        return cls(shapes=shapes)
    
    def save(self, path: Path):
        data = {"shapes": [s.to_dict() for s in self.shapes]}
        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    
    def clone(self) -> "Layout":
        return Layout(shapes=[s.clone() for s in self.shapes])
    
    def get_shape_by_id(self, shape_id: str) -> Optional[LayoutShapes]:
        for shape in self.shapes:
            if shape.id == shape_id:
                return shape
        return None

# ---------------------- GESTIÓN DE ESTADO (History) ---------------------- #

@dataclass
class AppState:
    graph: Graph
    layout: Layout

class HistoryManager:
    """Gestiona el historial de cambios (Grafo + Layout) para undo/redo"""
    def __init__(self, max_history: int = 50):
        self.undo_stack: List[AppState] = []
        self.redo_stack: List[AppState] = []
        self.max_history = max_history
    
    def push_state(self, graph: Graph, layout: Layout):
        """Guarda un estado completo"""
        self.redo_stack.clear()
        self.undo_stack.append(AppState(graph.clone(), layout.clone()))
        if len(self.undo_stack) > self.max_history:
            self.undo_stack.pop(0)
    
    def can_undo(self) -> bool:
        return len(self.undo_stack) > 0
    
    def can_redo(self) -> bool:
        return len(self.redo_stack) > 0
    
    def undo(self, current_graph: Graph, current_layout: Layout) -> Optional[AppState]:
        if not self.can_undo():
            return None
        
        self.redo_stack.append(AppState(current_graph.clone(), current_layout.clone()))
        return self.undo_stack.pop()
    
    def redo(self, current_graph: Graph, current_layout: Layout) -> Optional[AppState]:
        if not self.can_redo():
            return None
        
        self.undo_stack.append(AppState(current_graph.clone(), current_layout.clone()))
        return self.redo_stack.pop()

# ---------------------- CAMARA ---------------------- #

@dataclass
class Camera:
    zoom: float = 1.0
    pan_x: float = 0.0
    pan_y: float = 0.0
    width: int = 800
    height: int = 600
    
    MIN_ZOOM = 0.1
    MAX_ZOOM = 20.0
    
    def world_to_screen(self, x: float, y: float) -> Tuple[float, float]:
        sx = (x - self.pan_x) * self.zoom + self.width / 2
        sy = (y - self.pan_y) * self.zoom + self.height / 2
        return sx, self.height - sy
    
    def screen_to_world(self, sx: float, sy_canvas: float) -> Tuple[float, float]:
        sy = self.height - sy_canvas
        x = (sx - self.width / 2) / self.zoom + self.pan_x
        y = (sy - self.height / 2) / self.zoom + self.pan_y
        return x, y
    
    def zoom_at_point(self, screen_x: float, screen_y: float, delta: float):
        world_x, world_y = self.screen_to_world(screen_x, screen_y)
        self.zoom = max(self.MIN_ZOOM, min(self.MAX_ZOOM, self.zoom * delta))
        new_world_x, new_world_y = self.screen_to_world(screen_x, screen_y)
        self.pan_x += world_x - new_world_x
        self.pan_y += world_y - new_world_y
    
    def pan_by(self, dx: float, dy: float):
        world_dx = dx / self.zoom
        world_dy = -dy / self.zoom
        self.pan_x -= world_dx
        self.pan_y -= world_dy
    
    def fit_to_bounds(self, min_x: float, max_x: float, min_y: float, max_y: float, padding: float = 50.0):
        if max_x == min_x and max_y == min_y:
            self.pan_x = min_x
            self.pan_y = min_y
            self.zoom = min(self.width, self.height) / 200.0
            return
        
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        width_world = max_x - min_x
        height_world = max_y - min_y
        
        zoom_x = (self.width - 2 * padding) / width_world if width_world > 0 else 1.0
        zoom_y = (self.height - 2 * padding) / height_world if height_world > 0 else 1.0
        self.zoom = min(zoom_x, zoom_y)
        self.pan_x = center_x
        self.pan_y = center_y


# ---------------------- UI PRINCIPAL ---------------------- #

class ModernGraphEditor(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Graph Editor Pro")
        self.geometry("1200x800")
        self.minsize(1000, 600)
        
        # Cargar Datos
        try:
            self.graph = Graph.load()
            self.layout = Layout.load(LAYOUT_FILE)
        except Exception as e:
            print(f"Error loading files: {e}")
            self.graph = Graph(names=[], coords=[], edges=[])
            self.layout = Layout(shapes=[])
        
        # Estado guardado
        self._saved_state_graph = self.graph.clone()
        self._saved_state_layout = self.layout.clone()
        
        # Historial
        self._history = HistoryManager(max_history=50)
        
        # Estado UI
        self.node_items = {}
        self.edge_items = []
        self._item_to_node = {}
        self._item_to_shape_id = {}
        self._line_to_edge_index = {}
        
        self._camera = Camera(width=1200, height=800)
        
        # Interacción
        self._drag_node_index = None
        self._drag_shape_id = None
        self._drag_offset = (0.0, 0.0)
        self._drag_handle_index = None  # None, 0 (start/top-left), 1 (end/bottom-right)
        
        self._selected_node_index = None
        self._selected_edge_index = None
        self._selected_shape_id = None
        
        self._pan_start = None
        self._is_dragging = False
        self._drag_start_pos = None
        
        self._context_menu_pos = (0, 0) # Para saber donde añadir items

        self._setup_ui()
        self._setup_menus()
        self._bind_shortcuts()
        self._initialize_camera()
        
        self.protocol("WM_DELETE_WINDOW", self._on_closing)
        
        self.after(100, self._redraw_canvas)
        self.after(100, self._refresh_ui_lists)
        self.after(100, self._update_undo_redo_buttons)
        self.after(100, self._update_unsaved_indicator)
    
    def _setup_ui(self):
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # --- SIDEBAR ---
        self.sidebar = ctk.CTkFrame(self, width=300, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_rowconfigure(5, weight=1)

        # Título
        self.title_label = ctk.CTkLabel(self.sidebar, text="GRAPH EDITOR", font=ctk.CTkFont(size=20, weight="bold"))
        self.title_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        
        # Botones Undo/Redo
        self.action_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        self.action_frame.grid(row=1, column=0, padx=20, pady=10, sticky="ew")
        self.action_frame.grid_columnconfigure(0, weight=1)
        self.action_frame.grid_columnconfigure(1, weight=1)
        
        self.btn_undo = ctk.CTkButton(self.action_frame, text="↶ Deshacer", command=self._undo, width=100, fg_color="#34495e")
        self.btn_undo.grid(row=0, column=0, padx=2)
        self.btn_redo = ctk.CTkButton(self.action_frame, text="↷ Rehacer", command=self._redo, width=100, fg_color="#34495e")
        self.btn_redo.grid(row=0, column=1, padx=2)
        
        # Botones Globales
        self.btn_fit_view = ctk.CTkButton(self.sidebar, text="🔍 Ajustar Vista", command=self._fit_view_to_nodes, fg_color="#3498db")
        self.btn_fit_view.grid(row=2, column=0, padx=20, pady=5, sticky="ew")
        
        self.btn_save = ctk.CTkButton(self.sidebar, text="💾 Guardar (Ctrl+S)", command=self._save, fg_color="#2ecc71")
        self.btn_save.grid(row=3, column=0, padx=20, pady=5, sticky="ew")
        
        self.unsaved_label = ctk.CTkLabel(self.sidebar, text="", text_color="#e74c3c", font=ctk.CTkFont(size=11, weight="bold"))
        self.unsaved_label.grid(row=4, column=0, padx=20, pady=5)

        # TABVIEW
        self.tabview = ctk.CTkTabview(self.sidebar, width=280)
        self.tabview.grid(row=5, column=0, padx=10, pady=10, sticky="nsew")
        self.tabview.add("Nodos")
        self.tabview.add("Aristas")
        self.tabview.add("Layout")
        
        # Evento de cambio de tab (hack para CTK: bind al click no funciona bien, usaremos chequeo en canvas)
        # Pero sí configuramos el contenido
        
        # -- NODOS --
        self.frame_nodes = self.tabview.tab("Nodos")
        self.frame_nodes.grid_columnconfigure(0, weight=1)
        self.frame_nodes.grid_rowconfigure(0, weight=1)
        self.scroll_nodes = ctk.CTkScrollableFrame(self.frame_nodes, label_text="Lista de Nodos")
        self.scroll_nodes.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        self.frm_node_edit = ctk.CTkFrame(self.frame_nodes)
        self.frm_node_edit.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        self.ent_node_name = self._create_labeled_entry(self.frm_node_edit, "Nombre:", 0)
        self.ent_node_x = self._create_labeled_entry(self.frm_node_edit, "Pos X:", 1)
        self.ent_node_y = self._create_labeled_entry(self.frm_node_edit, "Pos Y:", 2)
        
        btn_box_n = ctk.CTkFrame(self.frm_node_edit, fg_color="transparent")
        btn_box_n.grid(row=4, column=0, columnspan=2, pady=10)
        ctk.CTkButton(btn_box_n, text="Actualizar", width=80, command=self._update_node).pack(side="left", padx=2)
        ctk.CTkButton(btn_box_n, text="Borrar", width=80, fg_color="#e74c3c", command=self._delete_node).pack(side="left", padx=2)

        # -- ARISTAS --
        self.frame_edges = self.tabview.tab("Aristas")
        self.frame_edges.grid_columnconfigure(0, weight=1)
        self.frame_edges.grid_rowconfigure(0, weight=1)
        self.scroll_edges = ctk.CTkScrollableFrame(self.frame_edges, label_text="Lista de Aristas")
        self.scroll_edges.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        self.frm_edge_edit = ctk.CTkFrame(self.frame_edges)
        self.frm_edge_edit.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        self.ent_edge_from = self._create_labeled_entry(self.frm_edge_edit, "Desde ID:", 0)
        self.ent_edge_to = self._create_labeled_entry(self.frm_edge_edit, "Hasta ID:", 1)
        self.ent_edge_w = self._create_labeled_entry(self.frm_edge_edit, "Peso:", 2)
        
        btn_box_e = ctk.CTkFrame(self.frm_edge_edit, fg_color="transparent")
        btn_box_e.grid(row=3, column=0, columnspan=2, pady=10)
        ctk.CTkButton(btn_box_e, text="Añadir/Act", width=80, command=self._update_edge).pack(side="left", padx=2)
        ctk.CTkButton(btn_box_e, text="Borrar", width=80, fg_color="#e74c3c", command=self._delete_edge).pack(side="left", padx=2)

        # -- LAYOUT --
        self.frame_layout = self.tabview.tab("Layout")
        self.frame_layout.grid_columnconfigure(0, weight=1)
        self.frame_layout.grid_rowconfigure(1, weight=1) # Scroll list
        
        self.lbl_layout_instr = ctk.CTkLabel(self.frame_layout, text="Clic derecho en canvas para añadir", text_color="gray")
        self.lbl_layout_instr.grid(row=0, column=0, pady=5)
        
        self.scroll_layout = ctk.CTkScrollableFrame(self.frame_layout, label_text="Objetos Layout")
        self.scroll_layout.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        
        self.frm_shape_edit = ctk.CTkFrame(self.frame_layout)
        self.frm_shape_edit.grid(row=2, column=0, sticky="ew", padx=5, pady=5)

        # --- CANVAS ---
        self.right_panel = ctk.CTkFrame(self, fg_color="transparent")
        self.right_panel.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        self.right_panel.grid_rowconfigure(1, weight=1)
        self.right_panel.grid_columnconfigure(0, weight=1)

        self.info_label = ctk.CTkLabel(self.right_panel, text="Izquierda: Seleccionar/Mover | Rueda: Zoom | Centro: Pan | Derecha: Menú Contextual", text_color="gray")
        self.info_label.grid(row=0, column=0, sticky="w", pady=(0, 5))

        self.canvas = tk.Canvas(self.right_panel, bg="#1a1a1a", highlightthickness=0, bd=0)
        self.canvas.grid(row=1, column=0, sticky="nsew")
        
        # Bindings
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.canvas.bind("<ButtonPress-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.canvas.bind("<ButtonPress-2>", self._on_canvas_pan_start)
        self.canvas.bind("<B2-Motion>", self._on_canvas_pan_drag)
        self.canvas.bind("<ButtonRelease-2>", self._on_canvas_pan_end)
        self.canvas.bind("<ButtonPress-3>", self._on_canvas_right_click)
        self.canvas.bind("<MouseWheel>", self._on_canvas_wheel)
        
        # Linux Scroll
        self.canvas.bind("<Button-4>", self._on_canvas_wheel)
        self.canvas.bind("<Button-5>", self._on_canvas_wheel)

    def _create_labeled_entry(self, parent, text, row):
        ctk.CTkLabel(parent, text=text).grid(row=row, column=0, padx=5, pady=2, sticky="e")
        entry = ctk.CTkEntry(parent, height=28)
        entry.grid(row=row, column=1, padx=5, pady=2, sticky="ew")
        return entry

    def _setup_menus(self):
        """Crea los menús contextuales para clic derecho"""
        self.menu_graph = Menu(self, tearoff=0, bg="#2c3e50", fg="white", activebackground="#3498db")
        self.menu_graph.add_command(label="Añadir Nodo Aquí", command=lambda: self._add_node_at_mouse())
        
        self.menu_layout = Menu(self, tearoff=0, bg="#2c3e50", fg="white", activebackground="#3498db")
        self.menu_layout.add_command(label="Añadir Texto", command=lambda: self._add_shape_at_mouse("text"))
        self.menu_layout.add_command(label="Añadir Línea", command=lambda: self._add_shape_at_mouse("line"))
        self.menu_layout.add_command(label="Añadir Rectángulo", command=lambda: self._add_shape_at_mouse("rect"))
        
        self.menu_colors = Menu(self.menu_layout, tearoff=0)
        self._color_menu_label = "Cambiar Color"
        colors = [("#ecf0f1", "Blanco"), ("#95a5a6", "Gris"), ("#e74c3c", "Rojo"), ("#f1c40f", "Amarillo"), ("#2ecc71", "Verde"), ("#3498db", "Azul")]
        for hex_c, name in colors:
            self.menu_colors.add_command(label=name, command=lambda c=hex_c: self._change_shape_color(c))

    def _bind_shortcuts(self):
        self.bind("<Control-s>", lambda e: self._save())
        self.bind("<Control-z>", lambda e: self._undo())
        self.bind("<Control-y>", lambda e: self._redo())
    
    def _initialize_camera(self):
        if not self.graph.names and not self.layout.shapes: return
        # Lógica simple de fit view
        self._fit_view_to_nodes()

    # ---------------------- LOGIC: MODES & STATE ---------------------- #

    def _get_active_mode(self):
        """Retorna 'GRAPH' o 'LAYOUT' dependiendo del tab seleccionado"""
        tab = self.tabview.get()
        if tab == "Layout":
            return "LAYOUT"
        return "GRAPH"

    def _mark_change(self):
        """Guarda estado en historial"""
        self._history.push_state(self.graph, self.layout)
        self._update_unsaved_indicator()
        self._update_undo_redo_buttons()

    def _update_unsaved_indicator(self):
        # Lógica simplificada: si el historial tiene items para deshacer, asumimos cambio
        # O comparamos con saved state (más robusto)
        has_changes = (len(self.graph.names) != len(self._saved_state_graph.names) or 
                       len(self.layout.shapes) != len(self._saved_state_layout.shapes))
        # (Comparación profunda omitida por brevedad, pero funcional)
        
        if has_changes or self._history.can_undo():
            self.unsaved_label.configure(text="● Cambios sin guardar")
            self.title("Graph Editor Pro *")
        else:
            self.unsaved_label.configure(text="")
            self.title("Graph Editor Pro")

    def _update_undo_redo_buttons(self):
        self.btn_undo.configure(state="normal" if self._history.can_undo() else "disabled")
        self.btn_redo.configure(state="normal" if self._history.can_redo() else "disabled")

    def _undo(self):
        state = self._history.undo(self.graph, self.layout)
        if state:
            self.graph = state.graph
            self.layout = state.layout
            self._selected_node_index = None
            self._selected_shape_id = None
            self._refresh_ui_lists()
            self._redraw_canvas()
            self._update_undo_redo_buttons()

    def _redo(self):
        state = self._history.redo(self.graph, self.layout)
        if state:
            self.graph = state.graph
            self.layout = state.layout
            self._selected_node_index = None
            self._selected_shape_id = None
            self._refresh_ui_lists()
            self._redraw_canvas()
            self._update_undo_redo_buttons()

    def _save(self):
        try:
            self.graph.save(GRAPH_FILE)
            self.layout.save(LAYOUT_FILE)
            self._saved_state_graph = self.graph.clone()
            self._saved_state_layout = self.layout.clone()
            self.unsaved_label.configure(text="")
            self.title("Graph Editor Pro")
            
            orig_text = self.info_label.cget("text")
            self.info_label.configure(text="¡Guardado Correctamente!", text_color="#2ecc71")
            self.after(2000, lambda: self.info_label.configure(text=orig_text, text_color="gray"))
        except Exception as e:
            messagebox.showerror("Error Guardando", str(e))

    # ---------------------- UI LIST UPDATES ---------------------- #

    def _refresh_ui_lists(self):
        # --- NODOS ---
        for w in self.scroll_nodes.winfo_children(): w.destroy()
        for i, name in enumerate(self.graph.names):
            fg = "#1f6aa5" if i == self._selected_node_index else "transparent"
            btn = ctk.CTkButton(self.scroll_nodes, text=f"{i}: {name}", fg_color=fg, anchor="w", height=24,
                                command=lambda x=i: self._select_node(x))
            btn.pack(fill="x", pady=1)
        
        # --- ARISTAS ---
        for w in self.scroll_edges.winfo_children(): w.destroy()
        for i, e in enumerate(self.graph.edges):
            fg = "#1f6aa5" if i == self._selected_edge_index else "transparent"
            btn = ctk.CTkButton(self.scroll_edges, text=f"{e.src}->{e.dst} ({e.weight})", fg_color=fg, anchor="w", height=24,
                                command=lambda x=i: self._select_edge(x))
            btn.pack(fill="x", pady=1)
        
        # --- LAYOUT ---
        for w in self.scroll_layout.winfo_children(): w.destroy()
        for shape in self.layout.shapes:
            fg = "#1f6aa5" if shape.id == self._selected_shape_id else "transparent"
            txt = "Shape"
            if isinstance(shape, RefText): txt = f"T: {shape.text}"
            elif isinstance(shape, RefLine): txt = "Línea"
            elif isinstance(shape, RefRect): txt = "Rectángulo"
            
            btn = ctk.CTkButton(self.scroll_layout, text=txt, fg_color=fg, anchor="w", height=24,
                                command=lambda s=shape.id: self._select_shape(s))
            btn.pack(fill="x", pady=1)

    # ---------------------- SELECTION & EDITING ---------------------- #

    def _select_node(self, idx):
        self.tabview.set("Nodos")
        self._selected_node_index = idx
        self._selected_edge_index = None
        self._selected_shape_id = None
        
        name = self.graph.names[idx]
        c = self.graph.coords[idx]
        self.ent_node_name.delete(0,"end"); self.ent_node_name.insert(0, name)
        self.ent_node_x.delete(0,"end"); self.ent_node_x.insert(0, str(c.x))
        self.ent_node_y.delete(0,"end"); self.ent_node_y.insert(0, str(c.y))
        
        self._refresh_ui_lists()
        self._redraw_canvas()

    def _select_edge(self, idx):
        self.tabview.set("Aristas")
        self._selected_edge_index = idx
        self._selected_node_index = None
        
        e = self.graph.edges[idx]
        self.ent_edge_from.delete(0,"end"); self.ent_edge_from.insert(0, str(e.src))
        self.ent_edge_to.delete(0,"end"); self.ent_edge_to.insert(0, str(e.dst))
        self.ent_edge_w.delete(0,"end"); self.ent_edge_w.insert(0, str(e.weight))
        
        self._refresh_ui_lists()
        self._redraw_canvas()

    def _select_shape(self, shape_id):
        self.tabview.set("Layout")
        self._selected_shape_id = shape_id
        self._selected_node_index = None
        self._build_shape_editor()
        self._refresh_ui_lists()
        self._redraw_canvas()

    def _update_node(self):
        if self._selected_node_index is None: return
        try:
            self._mark_change()
            idx = self._selected_node_index
            self.graph.names[idx] = self.ent_node_name.get()
            self.graph.coords[idx].x = float(self.ent_node_x.get())
            self.graph.coords[idx].y = float(self.ent_node_y.get())
            self._refresh_ui_lists()
            self._redraw_canvas()
        except ValueError: pass

    def _delete_node(self):
        if self._selected_node_index is None: return
        self._mark_change()
        idx = self._selected_node_index
        self.graph.names.pop(idx)
        self.graph.coords.pop(idx)
        # Reconstruir edges
        new_edges = []
        for e in self.graph.edges:
            if e.src == idx or e.dst == idx: continue
            s = e.src - 1 if e.src > idx else e.src
            d = e.dst - 1 if e.dst > idx else e.dst
            new_edges.append(Edge(s, d, e.weight))
        self.graph.edges = new_edges
        self._selected_node_index = None
        self._refresh_ui_lists()
        self._redraw_canvas()

    def _update_edge(self):
        try:
            src, dst = int(self.ent_edge_from.get()), int(self.ent_edge_to.get())
            w = float(self.ent_edge_w.get())
            self._mark_change()
            if self._selected_edge_index is not None:
                self.graph.edges[self._selected_edge_index] = Edge(src, dst, w)
            else:
                self.graph.edges.append(Edge(src, dst, w))
            self._refresh_ui_lists()
            self._redraw_canvas()
        except ValueError: pass

    def _delete_edge(self):
        if self._selected_edge_index is None: return
        self._mark_change()
        self.graph.edges.pop(self._selected_edge_index)
        self._selected_edge_index = None
        self._refresh_ui_lists()
        self._redraw_canvas()

    # --- SHAPE EDITING ---
    def _build_shape_editor(self):
        for w in self.frm_shape_edit.winfo_children(): w.destroy()
        shape = self.layout.get_shape_by_id(self._selected_shape_id)
        if not shape: return

        ents = {}
        
        def add_row(label, key, val, r):
            ctk.CTkLabel(self.frm_shape_edit, text=label).grid(row=r, column=0, sticky="e", padx=5)
            e = ctk.CTkEntry(self.frm_shape_edit)
            e.insert(0, str(val))
            e.grid(row=r, column=1, sticky="ew", padx=5)
            ents[key] = e

        r = 0
        if isinstance(shape, RefText):
            add_row("Texto:", "text", shape.text, r); r+=1
            add_row("Color:", "color", shape.color, r); r+=1
            add_row("Tamaño:", "size", shape.font_size, r); r+=1
            add_row("X:", "x", shape.pos[0], r); r+=1
            add_row("Y:", "y", shape.pos[1], r); r+=1
        
        elif isinstance(shape, RefLine):
            add_row("X1:", "x1", shape.start[0], r); r+=1
            add_row("Y1:", "y1", shape.start[1], r); r+=1
            add_row("X2:", "x2", shape.end[0], r); r+=1
            add_row("Y2:", "y2", shape.end[1], r); r+=1
            add_row("Grosor:", "width", shape.width, r); r+=1
        
        elif isinstance(shape, RefRect):
            add_row("X1:", "x1", shape.bounds[0], r); r+=1
            add_row("Y1:", "y1", shape.bounds[1], r); r+=1
            add_row("X2:", "x2", shape.bounds[2], r); r+=1
            add_row("Y2:", "y2", shape.bounds[3], r); r+=1
            add_row("Relleno:", "fill", shape.fill, r); r+=1

        btn_row = ctk.CTkFrame(self.frm_shape_edit, fg_color="transparent")
        btn_row.grid(row=r+1, column=0, columnspan=2, pady=10)
        ctk.CTkButton(btn_row, text="Guardar", width=80, command=lambda: self._save_shape_changes(shape, ents)).pack(side="left", padx=2)
        ctk.CTkButton(btn_row, text="Eliminar", width=80, fg_color="#e74c3c", command=self._delete_shape).pack(side="left", padx=2)

    def _save_shape_changes(self, shape, ents):
        try:
            self._mark_change()
            if isinstance(shape, RefText):
                shape.text = ents["text"].get()
                shape.font_size = int(ents["size"].get())
                shape.color = ents["color"].get()
                shape.pos = (float(ents["x"].get()), float(ents["y"].get()))
            elif isinstance(shape, RefLine):
                shape.start = (float(ents["x1"].get()), float(ents["y1"].get()))
                shape.end = (float(ents["x2"].get()), float(ents["y2"].get()))
                shape.width = int(ents["width"].get())
            elif isinstance(shape, RefRect):
                shape.bounds = (float(ents["x1"].get()), float(ents["y1"].get()),
                                float(ents["x2"].get()), float(ents["y2"].get()))
                shape.fill = ents["fill"].get()
            
            self._refresh_ui_lists()
            self._redraw_canvas()
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _delete_shape(self):
        if not self._selected_shape_id: return
        self._mark_change()
        self.layout.shapes = [s for s in self.layout.shapes if s.id != self._selected_shape_id]
        self._selected_shape_id = None
        self._build_shape_editor()
        self._refresh_ui_lists()
        self._redraw_canvas()

    # ---------------------- DRAWING ---------------------- #

    def _on_canvas_configure(self, event):
        self._camera.width = event.width
        self._camera.height = event.height
        self._redraw_canvas()

    def _redraw_canvas(self):
        self.canvas.delete("all")
        self.node_items.clear()
        self.edge_items.clear()
        self._item_to_node.clear()
        self._item_to_shape_id.clear()
        
        # Grid
        self._draw_grid()
        
        # Layout Shapes (Always drawn, but selectable only in layout mode)
        self._draw_reference_geometry()

        # Graph
        tf = self._camera.world_to_screen
        
        # Edges
        for i, e in enumerate(self.graph.edges):
            if e.src >= len(self.graph.coords) or e.dst >= len(self.graph.coords): continue
            c1, c2 = self.graph.coords[e.src], self.graph.coords[e.dst]
            x1, y1 = tf(c1.x, c1.y)
            x2, y2 = tf(c2.x, c2.y)
            
            col = "#f39c12" if i == self._selected_edge_index else "#555"
            w = 3 if i == self._selected_edge_index else 1
            self.canvas.create_line(x1, y1, x2, y2, fill=col, width=w, tags="edge")

        # Nodes
        r = max(5, int(10 * self._camera.zoom))
        for i, c in enumerate(self.graph.coords):
            if not c: continue
            x, y = tf(c.x, c.y)
            
            fill = "#3498db"
            if i == self._selected_node_index: fill = "#e74c3c"
            
            item = self.canvas.create_oval(x-r, y-r, x+r, y+r, fill=fill, outline="white")
            self.canvas.create_text(x, y, text=str(i), fill="white", font=("Arial", int(r)))
            self.canvas.create_text(x, y+r+10, text=c.label, fill="gray")
            
            self.node_items[i] = item
            self._item_to_node[item] = i

    def _draw_grid(self):
        w, h = self._camera.width, self._camera.height
        step = 50 * self._camera.zoom
        if step < 20: step = 20 # Limit grid density
        
        # Simplificado para rendimiento
        self.canvas.create_line(w/2, 0, w/2, h, fill="#333", dash=(4,4))
        self.canvas.create_line(0, h/2, w, h/2, fill="#333", dash=(4,4))

    def _draw_reference_geometry(self):
        tf = self._camera.world_to_screen
        
        for shape in self.layout.shapes:
            is_sel = (shape.id == self._selected_shape_id)
            
            # FIXED: CRASH WAS HERE. Now we check type first.
            
            if isinstance(shape, RefLine):
                width = max(1, int(shape.width * self._camera.zoom))
                if is_sel: width += 2
                x1, y1 = tf(*shape.start)
                x2, y2 = tf(*shape.end)
                color = "#e74c3c" if is_sel else shape.color
                item = self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width, dash=shape.dash)
                self._item_to_shape_id[item] = shape.id
            
            elif isinstance(shape, RefRect):
                width = max(1, int(shape.width * self._camera.zoom))
                if is_sel: width += 2
                x1, y1 = tf(shape.bounds[0], shape.bounds[1])
                x2, y2 = tf(shape.bounds[2], shape.bounds[3])
                outline = "#e74c3c" if is_sel else shape.outline
                item = self.canvas.create_rectangle(x1, y1, x2, y2, outline=outline, fill=shape.fill, width=width)
                self._item_to_shape_id[item] = shape.id
            
            elif isinstance(shape, RefText):
                # Text doesn't have width, so we don't access it
                x, y = tf(*shape.pos)
                color = "#e74c3c" if is_sel else shape.color
                size = max(6, int(shape.font_size * self._camera.zoom))
                item = self.canvas.create_text(x, y, text=shape.text, fill=color, font=("Arial", size))
                self._item_to_shape_id[item] = shape.id

        # Draw Selection Handles (if layout mode and item selected)
        if self._selected_shape_id and self._get_active_mode() == "LAYOUT":
            self._draw_selection_handles()

    def _draw_selection_handles(self):
        shape = self.layout.get_shape_by_id(self._selected_shape_id)
        if not shape: return
        
        tf = self._camera.world_to_screen
        handles = []

        if isinstance(shape, RefLine):
            handles = [shape.start, shape.end]
        elif isinstance(shape, RefRect):
            # Top-Left and Bottom-Right handles
            handles = [(shape.bounds[0], shape.bounds[1]), (shape.bounds[2], shape.bounds[3])]
        elif isinstance(shape, RefText):
            handles = [shape.pos]

        r = 4 # Handle radius
        for i, (wx, wy) in enumerate(handles):
            sx, sy = tf(wx, wy)
            # Create a small square handle
            # Tag 'handle' allows us to detect clicks specifically on these boxes
            col = "#f1c40f"
            self.canvas.create_rectangle(sx-r, sy-r, sx+r, sy+r, fill=col, outline="black", tags=f"handle_{i}")


    # ---------------------- INTERACTION ---------------------- #

    def _on_canvas_right_click(self, event):
        # Guardar posición donde se hizo clic
        self._context_menu_pos = (event.x, event.y)
        
        mode = self._get_active_mode()
        if mode == "LAYOUT":
            # Check if we clicked on a shape to show specific options
            closest = self.canvas.find_closest(event.x, event.y)
            item_id = closest[0] if closest else None
            if item_id and item_id in self._item_to_shape_id:
                shape_id = self._item_to_shape_id[item_id]
                self._select_shape(shape_id)
                # Add Change Color option dynamically
                self._toggle_color_menu(show=True)
            else:
                self._toggle_color_menu(show=False)
            
            self.menu_layout.post(event.x_root, event.y_root)
        else:
            self.menu_graph.post(event.x_root, event.y_root)

    def _add_node_at_mouse(self):
        self._mark_change()
        wx, wy = self._camera.screen_to_world(*self._context_menu_pos)
        name = f"Nodo {len(self.graph.names)}"
        self.graph.names.append(name)
        self.graph.coords.append(NodeCoord(wx, wy, 0.0, name))
        self._redraw_canvas()
        self._refresh_ui_lists()

    def _add_shape_at_mouse(self, shape_type):
        self._mark_change()
        wx, wy = self._camera.screen_to_world(*self._context_menu_pos)
        
        new_shape = None
        if shape_type == "text":
            new_shape = RefText(pos=(wx, wy), text="Texto Nuevo")
        elif shape_type == "line":
            # Create a diagonal line so it's visible (not a dot)
            new_shape = RefLine(start=(wx, wy), end=(wx + 5.0, wy - 5.0))
        elif shape_type == "rect":
            new_shape = RefRect(bounds=(wx, wy, wx + 5.0, wy - 5.0))
            
        if new_shape:
            self.layout.shapes.append(new_shape)
            self._select_shape(new_shape.id)
            self._refresh_ui_lists()
            
    def _change_shape_color(self, color_hex):
        if not self._selected_shape_id: return
        shape = self.layout.get_shape_by_id(self._selected_shape_id)
        if hasattr(shape, 'color'): shape.color = color_hex
        if hasattr(shape, 'outline'): shape.outline = color_hex
        self._mark_change()
        self._redraw_canvas()
        self._build_shape_editor()

    def _on_canvas_press(self, event):
        mode = self._get_active_mode()
        
        item_id = self.canvas.find_closest(event.x, event.y)[0] if self.canvas.find_closest(event.x, event.y) else None
        
        # Reset selections
        self._selected_node_index = None
        # Don't deselect shape immediately if clicking handles
        self._selected_edge_index = None
        self._drag_handle_index = None
        
        # Check for Handle Click (Resizing/Rotating)
        if mode == "LAYOUT" and self._selected_shape_id:
            # Check if we clicked near a handle of the CURRENT selection
            # We scan tags "handle_0", "handle_1" nearby
            overlapping = self.canvas.find_overlapping(event.x-5, event.y-5, event.x+5, event.y+5)
            for item in overlapping:
                tags = self.canvas.gettags(item)
                for t in tags:
                    if t.startswith("handle_"):
                        self._drag_handle_index = int(t.split("_")[1])
                        self._is_dragging = True
                        return # Stop processing, we are resizing
        
        if item_id:
            # Lógica de selección dependiente del modo
            if mode == "GRAPH":
                if item_id in self._item_to_node:
                    node_idx = self._item_to_node[item_id]
                    self._select_node(node_idx)
                    self._drag_node_index = node_idx
                    c = self.graph.coords[node_idx]
                    wx, wy = self._camera.world_to_screen(c.x, c.y)
                    self._drag_offset = (event.x - wx, event.y - wy)
                    self._is_dragging = True
                    self._drag_start_pos = (c.x, c.y)
            
            elif mode == "LAYOUT":
                if item_id in self._item_to_shape_id:
                    s_id = self._item_to_shape_id[item_id]
                    self._select_shape(s_id)
                    self._drag_shape_id = s_id
                    self._is_dragging = True
                    wx, wy = self._camera.screen_to_world(event.x, event.y)
                    self._drag_last_world = (wx, wy)
                else:
                    self._selected_shape_id = None
        else:
            # Clicked empty space
            self._selected_shape_id = None

        self._refresh_ui_lists()
        self._redraw_canvas()

    def _on_canvas_drag(self, event):
        if not self._is_dragging: return
        
        wx, wy = self._camera.screen_to_world(event.x, event.y)
        
        if self._drag_node_index is not None:
            # Actualizar visualmente sin guardar historial aun
            c = self.graph.coords[self._drag_node_index]
            # Ajuste offset pantalla -> mundo es complejo con zoom, simplificado:
            c.x, c.y = wx, wy 
            self._redraw_canvas()
            self.ent_node_x.delete(0,"end"); self.ent_node_x.insert(0, f"{c.x:.2f}")
            self.ent_node_y.delete(0,"end"); self.ent_node_y.insert(0, f"{c.y:.2f}")
            
        elif self._drag_handle_index is not None and self._selected_shape_id:
            # --- RESIZING / ROTATING VIA HANDLES ---
            shape = self.layout.get_shape_by_id(self._selected_shape_id)
            if isinstance(shape, RefLine):
                if self._drag_handle_index == 0:
                    shape.start = (wx, wy) # Move Start
                else:
                    shape.end = (wx, wy)   # Move End
            
            elif isinstance(shape, RefRect):
                b = list(shape.bounds)
                if self._drag_handle_index == 0: # Top-Left
                    b[0], b[1] = wx, wy
                else: # Bottom-Right
                    b[2], b[3] = wx, wy
                shape.bounds = tuple(b)
                
            elif isinstance(shape, RefText):
                shape.pos = (wx, wy)
            
            self._redraw_canvas()
            # Update sidebar entry values live
            self._build_shape_editor()
        
        elif self._drag_shape_id is not None:
            # --- MOVING WHOLE SHAPE ---
            shape = self.layout.get_shape_by_id(self._drag_shape_id)
            dx = wx - self._drag_last_world[0]
            dy = wy - self._drag_last_world[1]
            
            if isinstance(shape, RefText):
                shape.pos = (shape.pos[0] + dx, shape.pos[1] + dy)
            elif isinstance(shape, RefLine):
                shape.start = (shape.start[0] + dx, shape.start[1] + dy)
                shape.end = (shape.end[0] + dx, shape.end[1] + dy)
            elif isinstance(shape, RefRect):
                shape.bounds = (shape.bounds[0] + dx, shape.bounds[1] + dy,
                                shape.bounds[2] + dx, shape.bounds[3] + dy)
            
            self._drag_last_world = (wx, wy)
            self._redraw_canvas()

    def _on_canvas_release(self, event):
        if self._is_dragging:
            self._mark_change() # Guardar estado FINAL del arrastre
        self._is_dragging = False
        self._drag_node_index = None
        self._drag_handle_index = None
        self._drag_shape_id = None

    def _toggle_color_menu(self, show: bool):
        """Attach or remove the color submenu without causing Tk index errors."""
        idx = self._find_menu_index(self.menu_layout, self._color_menu_label)
        if show and idx is None:
            self.menu_layout.add_cascade(label=self._color_menu_label, menu=self.menu_colors)
        elif not show and idx is not None:
            self.menu_layout.delete(idx)

    @staticmethod
    def _find_menu_index(menu: Menu, label: str) -> Optional[int]:
        try:
            last_index = menu.index("end")
        except tk.TclError:
            return None
        if last_index is None:
            return None
        for i in range(last_index + 1):
            try:
                if menu.entrycget(i, "label") == label:
                    return i
            except tk.TclError:
                continue
        return None

    # Pan
    def _on_canvas_pan_start(self, event):
        self._pan_start = (event.x, event.y)
    
    def _on_canvas_pan_drag(self, event):
        if not self._pan_start: return
        dx = event.x - self._pan_start[0]
        dy = event.y - self._pan_start[1]
        self._camera.pan_by(dx, dy)
        self._pan_start = (event.x, event.y)
        self._redraw_canvas()

    def _on_canvas_pan_end(self, event):
        self._pan_start = None

    # Zoom
    def _on_canvas_wheel(self, event):
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            delta = 1.1
        else:
            delta = 0.9
        self._camera.zoom_at_point(event.x, event.y, delta)
        self._redraw_canvas()

    def _fit_view_to_nodes(self):
        all_x = [c.x for c in self.graph.coords if c]
        all_y = [c.y for c in self.graph.coords if c]
        if not all_x: return
        self._camera.fit_to_bounds(min(all_x), max(all_x), min(all_y), max(all_y))
        self._redraw_canvas()
        
    def _on_closing(self):
        self.destroy()

if __name__ == "__main__":
    app = ModernGraphEditor()
    app.mainloop()