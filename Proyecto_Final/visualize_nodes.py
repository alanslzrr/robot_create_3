# visualize_nodes.py
# Visualización del grafo de navegación con matplotlib
# - Nodos con flechas de orientación
# - Aristas coloreadas por calidad
# - Tooltips con información detallada

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Circle
import numpy as np
import math
from nodes_io import load_nodes, load_edges, nodes_index_by_id

def plot_graph(show_grid=True, show_labels=True, show_quality=True, figsize=(12, 10)):
    """
    Visualiza el grafo de navegación completo
    
    Args:
        show_grid: Mostrar cuadrícula de fondo
        show_labels: Mostrar etiquetas de nodos
        show_quality: Colorear aristas por calidad
        figsize: Tamaño de la figura
    """
    nodes = load_nodes()
    edges = load_edges()
    idx = nodes_index_by_id()
    
    if not nodes:
        print("❌ No hay nodos para visualizar.")
        return
    
    fig, ax = plt.subplots(figsize=figsize)
    
    # Configuración de ejes
    ax.set_aspect('equal')
    if show_grid:
        ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_xlabel('X (cm)', fontsize=12)
    ax.set_ylabel('Y (cm)', fontsize=12)
    ax.set_title('Grafo de Navegación - Create3', fontsize=14, fontweight='bold')
    
    # Calcular límites
    xs = [n['x'] for n in nodes]
    ys = [n['y'] for n in nodes]
    margin = 50.0
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    
    # 1) Dibujar aristas
    if edges:
        for edge in edges:
            from_node = idx.get(edge['from'])
            to_node = idx.get(edge['to'])
            
            if not from_node or not to_node:
                continue
            
            x1, y1 = from_node['x'], from_node['y']
            x2, y2 = to_node['x'], to_node['y']
            
            # Color por calidad
            quality = edge.get('quality')
            if show_quality and quality is not None:
                # Escala de color: rojo (mala) -> amarillo -> verde (buena)
                if quality >= 0.7:
                    color = (0, 0.8, 0, 0.6)  # Verde
                elif quality >= 0.4:
                    color = (1, 0.8, 0, 0.6)  # Amarillo
                else:
                    color = (1, 0, 0, 0.6)    # Rojo
                linewidth = 1.5 + quality * 1.5  # Más grueso = mejor calidad
            else:
                color = (0.5, 0.5, 0.5, 0.5)
                linewidth = 1.0
            
            # Flecha direccional
            arrow = FancyArrowPatch(
                (x1, y1), (x2, y2),
                arrowstyle='-|>',
                color=color,
                linewidth=linewidth,
                mutation_scale=15,
                zorder=1
            )
            ax.add_patch(arrow)
            
            # Etiqueta de calidad en el punto medio
            if show_quality and quality is not None:
                mx, my = (x1 + x2) / 2, (y1 + y2) / 2
                ax.text(mx, my, f'{quality:.2f}', 
                       fontsize=8, ha='center', va='center',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', 
                                edgecolor='none', alpha=0.7),
                       zorder=2)
    
    # 2) Dibujar nodos
    for node in nodes:
        x, y, theta = node['x'], node['y'], node['theta']
        nid, name = node['id'], node['name']
        
        # Color del nodo según tags o calidad
        if 'dock' in name.lower():
            node_color = 'blue'
            node_size = 150
        elif 'start' in name.lower():
            node_color = 'green'
            node_size = 120
        else:
            node_color = 'orange'
            node_size = 100
        
        # Círculo del nodo
        circle = Circle((x, y), radius=8, color=node_color, alpha=0.8, zorder=3)
        ax.add_patch(circle)
        
        # Flecha de orientación
        arrow_len = 20.0
        theta_rad = math.radians(theta)
        dx = arrow_len * math.cos(theta_rad)
        dy = arrow_len * math.sin(theta_rad)
        
        orientation_arrow = FancyArrowPatch(
            (x, y), (x + dx, y + dy),
            arrowstyle='-|>',
            color='black',
            linewidth=2,
            mutation_scale=12,
            zorder=4
        )
        ax.add_patch(orientation_arrow)
        
        # Etiqueta del nodo
        if show_labels:
            label = f"{nid}: {name}"
            ax.text(x, y - 15, label, 
                   fontsize=9, ha='center', va='top',
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='white', 
                            edgecolor='gray', alpha=0.9),
                   zorder=5)
        
        # ID en el centro del nodo
        ax.text(x, y, str(nid), 
               fontsize=10, ha='center', va='center', 
               color='white', fontweight='bold', zorder=6)
    
    # 3) Leyenda
    legend_elements = [
        mpatches.Patch(color='blue', label='Dock'),
        mpatches.Patch(color='green', label='StartFromDock'),
        mpatches.Patch(color='orange', label='Nodo normal')
    ]
    
    if show_quality and edges:
        legend_elements.extend([
            mpatches.Patch(color=(0, 0.8, 0), label='Calidad alta (≥0.7)'),
            mpatches.Patch(color=(1, 0.8, 0), label='Calidad media (0.4-0.7)'),
            mpatches.Patch(color=(1, 0, 0), label='Calidad baja (<0.4)')
        ])
    
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
    
    plt.tight_layout()
    return fig, ax

def plot_node_stats():
    """Genera estadísticas visuales de los nodos y aristas"""
    nodes = load_nodes()
    edges = load_edges()
    
    if not nodes:
        print("❌ No hay datos para estadísticas.")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Estadísticas del Grafo de Navegación', fontsize=16, fontweight='bold')
    
    # 1) Distribución de calidad de aristas
    ax1 = axes[0, 0]
    qualities = [e.get('quality', 0) for e in edges if e.get('quality') is not None]
    if qualities:
        ax1.hist(qualities, bins=20, color='steelblue', edgecolor='black', alpha=0.7)
        ax1.axvline(np.mean(qualities), color='red', linestyle='--', 
                   label=f'Media: {np.mean(qualities):.2f}')
        ax1.set_xlabel('Calidad')
        ax1.set_ylabel('Frecuencia')
        ax1.set_title('Distribución de Calidad de Aristas')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
    else:
        ax1.text(0.5, 0.5, 'Sin datos de calidad', ha='center', va='center')
        ax1.set_title('Distribución de Calidad de Aristas')
    
    # 2) Longitud de aristas
    ax2 = axes[0, 1]
    lengths = [e.get('agg', {}).get('len_cm', 0) for e in edges]
    lengths = [l for l in lengths if l > 0]
    if lengths:
        ax2.hist(lengths, bins=20, color='green', edgecolor='black', alpha=0.7)
        ax2.axvline(np.mean(lengths), color='red', linestyle='--', 
                   label=f'Media: {np.mean(lengths):.1f} cm')
        ax2.set_xlabel('Longitud (cm)')
        ax2.set_ylabel('Frecuencia')
        ax2.set_title('Distribución de Longitud de Aristas')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
    else:
        ax2.text(0.5, 0.5, 'Sin datos de longitud', ha='center', va='center')
        ax2.set_title('Distribución de Longitud de Aristas')
    
    # 3) Grado de salida de nodos (out-degree)
    ax3 = axes[1, 0]
    out_degree = {}
    for n in nodes:
        out_degree[n['id']] = sum(1 for e in edges if e['from'] == n['id'])
    
    if out_degree:
        ids = list(out_degree.keys())
        degrees = list(out_degree.values())
        ax3.bar(ids, degrees, color='purple', alpha=0.7, edgecolor='black')
        ax3.set_xlabel('ID de Nodo')
        ax3.set_ylabel('Aristas salientes')
        ax3.set_title('Grado de Salida por Nodo')
        ax3.grid(True, alpha=0.3, axis='y')
    else:
        ax3.text(0.5, 0.5, 'Sin aristas', ha='center', va='center')
        ax3.set_title('Grado de Salida por Nodo')
    
    # 4) Tabla de resumen
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    total_nodes = len(nodes)
    total_edges = len(edges)
    avg_quality = np.mean(qualities) if qualities else 0
    avg_length = np.mean(lengths) if lengths else 0
    total_distance = sum(lengths) if lengths else 0
    
    completeness = (total_edges / (total_nodes * (total_nodes - 1))) * 100 if total_nodes > 1 else 0
    
    summary_text = f"""
    RESUMEN DEL GRAFO
    ━━━━━━━━━━━━━━━━━━━━━━━━
    
    Nodos totales:       {total_nodes}
    Aristas totales:     {total_edges}
    
    Calidad promedio:    {avg_quality:.2f}
    Longitud promedio:   {avg_length:.1f} cm
    Distancia total:     {total_distance:.1f} cm
    
    Completitud:         {completeness:.1f}%
    
    Nodos sin salida:    {sum(1 for d in out_degree.values() if d == 0)}
    Grado max salida:    {max(out_degree.values()) if out_degree else 0}
    """
    
    ax4.text(0.1, 0.9, summary_text, transform=ax4.transAxes,
            fontsize=11, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    return fig, axes

def plot_edge_details(from_id, to_id):
    """Visualiza detalles de una arista específica"""
    edges = load_edges()
    edge = None
    for e in edges:
        if e['from'] == from_id and e['to'] == to_id:
            edge = e
            break
    
    if not edge:
        print(f"❌ No se encontró arista {from_id} -> {to_id}")
        return
    
    segments = edge.get('segments', [])
    if not segments:
        print("❌ La arista no tiene segmentos.")
        return
    
    idx = nodes_index_by_id()
    from_node = idx.get(from_id, {"name": "?"})
    to_node = idx.get(to_id, {"name": "?"})
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Análisis de Arista: {from_id}:{from_node["name"]} → {to_id}:{to_node["name"]}', 
                fontsize=14, fontweight='bold')
    
    # Extraer datos
    indices = list(range(len(segments)))
    states = [s.get('state', 'unknown') for s in segments]
    times = [s.get('t', 0) for s in segments]
    dist_plan = [s.get('dist_cm', 0) for s in segments]
    dist_odom = [s.get('odom_dist_cm', 0) for s in segments]
    err_dist = [s.get('err_dist_cm', 0) for s in segments]
    deg_plan = [s.get('deg', 0) for s in segments]
    deg_odom = [s.get('odom_deg', 0) for s in segments]
    err_deg = [s.get('err_deg', 0) for s in segments]
    
    # 1) Error de distancia
    ax1 = axes[0, 0]
    ax1.plot(indices, err_dist, marker='o', color='red', label='Error distancia')
    ax1.axhline(0, color='black', linestyle='--', linewidth=0.8)
    ax1.set_xlabel('Segmento')
    ax1.set_ylabel('Error (cm)')
    ax1.set_title('Error de Distancia por Segmento')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2) Error angular
    ax2 = axes[0, 1]
    ax2.plot(indices, err_deg, marker='s', color='blue', label='Error angular')
    ax2.axhline(0, color='black', linestyle='--', linewidth=0.8)
    ax2.set_xlabel('Segmento')
    ax2.set_ylabel('Error (deg)')
    ax2.set_title('Error Angular por Segmento')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3) Comparación planificado vs odometría (distancia)
    ax3 = axes[1, 0]
    x_seg = np.arange(len(segments))
    width = 0.35
    ax3.bar(x_seg - width/2, dist_plan, width, label='Planificado', alpha=0.7, color='green')
    ax3.bar(x_seg + width/2, dist_odom, width, label='Odometría', alpha=0.7, color='orange')
    ax3.set_xlabel('Segmento')
    ax3.set_ylabel('Distancia (cm)')
    ax3.set_title('Distancia: Planificado vs Odometría')
    ax3.legend()
    ax3.grid(True, alpha=0.3, axis='y')
    
    # 4) Duración de segmentos por estado
    ax4 = axes[1, 1]
    state_times = {}
    for s in segments:
        st = s.get('state', 'unknown')
        t = s.get('t', 0)
        state_times[st] = state_times.get(st, 0) + t
    
    if state_times:
        states_list = list(state_times.keys())
        times_list = list(state_times.values())
        colors_map = {
            'forward': 'green', 'backward': 'red', 'turn_left': 'blue', 
            'turn_right': 'cyan', 'stop': 'gray'
        }
        colors = [colors_map.get(s, 'orange') for s in states_list]
        
        ax4.pie(times_list, labels=states_list, autopct='%1.1f%%', 
               colors=colors, startangle=90)
        ax4.set_title('Distribución de Tiempo por Estado')
    else:
        ax4.text(0.5, 0.5, 'Sin datos de estados', ha='center', va='center')
        ax4.set_title('Distribución de Tiempo por Estado')
    
    plt.tight_layout()
    return fig, axes

def save_graph_image(filename="graph.png", **kwargs):
    """Guarda la visualización del grafo en un archivo"""
    fig, ax = plot_graph(**kwargs)
    if fig:
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✔ Grafo guardado en: {filename}")
        plt.close(fig)

def show_interactive():
    """Muestra visualización interactiva del grafo"""
    plot_graph()
    plt.show()

def show_stats():
    """Muestra estadísticas del grafo"""
    plot_node_stats()
    plt.show()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        cmd = sys.argv[1].lower()
        
        if cmd == "graph":
            show_interactive()
        elif cmd == "stats":
            show_stats()
        elif cmd == "edge" and len(sys.argv) >= 4:
            try:
                from_id = int(sys.argv[2])
                to_id = int(sys.argv[3])
                plot_edge_details(from_id, to_id)
                plt.show()
            except ValueError:
                print("❌ IDs deben ser números.")
        elif cmd == "save":
            filename = sys.argv[2] if len(sys.argv) > 2 else "graph.png"
            save_graph_image(filename)
        else:
            print("Uso:")
            print("  python visualize_nodes.py graph           - mostrar grafo interactivo")
            print("  python visualize_nodes.py stats           - mostrar estadísticas")
            print("  python visualize_nodes.py edge <A> <B>    - analizar arista A->B")
            print("  python visualize_nodes.py save [archivo]  - guardar grafo como imagen")
    else:
        # Por defecto, mostrar grafo
        show_interactive()
