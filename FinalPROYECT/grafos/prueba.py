
class Edge:
    """
    Clase que representa una arista en el grafo.
    Attributes:
        to (int): Nodo destino de la arista.
        weight (float): Peso de la arista.
    """
    def __init__(self, to, weight):
        self.to = to
        self.weight = float(weight)


class GrafoDP:
    """
    Clase que representa un grafo ponderado y dirigido para ciudades gallegas.
    Attributes:
        V (int): Número de vértices en el grafo.
        adjList (list): Lista de adyacencia que almacena las aristas.
        nombres (list): Lista de nombres de las ciudades.
    """
    def __init__(self, V, nombres, coords=None):
        """
        Inicializa el grafo con V vértices y una lista de nombres.
        Args:
            V (int): Número de vértices.
            nombres (list): Nombres de las ciudades.
        """
        self.V = V
        self.adjList = [[] for _ in range(V)]
        self.nombres = nombres  # Lista para almacenar los nombres de las ciudades
        # Lista opcional con coordenadas espaciales de cada nodo:
        # coords[i] = {"x": float, "y": float, "theta": float} o None si no hay datos
        if coords is not None:
            if len(coords) != V:
                raise ValueError(
                    f"El número de coordenadas ({len(coords)}) no coincide con el número de vértices ({V})."
                )
            self.coords = coords
        else:
            self.coords = [None] * V

    def Agregar_Arista(self, vs, ve, p):
        """
        Agrega una arista bidireccional entre dos vértices con su respectivo peso.
        Args:
            vs (int): Vértice de origen.
            ve (int): Vértice de destino.
            p (float): Peso de la arista.
        """
        self.adjList[vs].append(Edge(ve, p))
        self.adjList[ve].append(Edge(vs, p))  # Para hacerlo bidireccional

    def Muestra_GrafoDP(self):
        """
        Muestra un resumen legible del grafo en formato tabular.
        """
        print("\n" + "=" * 80)
        print("RESUMEN DEL GRAFO (NODOS Y ADYACENCIAS)")
        print("=" * 80)
        header = (
            f"{'ID':<3} {'Ciudad':<22} {'Coord (x,y,θ)':<32} "
            f"{'Out':>3} {'In':>3}  Adyacentes [peso]"
        )
        print(header)
        print("-" * 80)

        for i in range(self.V):
            out_deg = self.Grado_Out(i)
            in_deg = self.Grado_In(i)

            if self.coords and self.coords[i] is not None:
                c = self.coords[i]
                coord_str = f"x={c['x']:.2f}, y={c['y']:.2f}, θ={c['theta']:.2f}"
            else:
                coord_str = "-"

            adj_str = ", ".join(
                f"{self.nombres[edge.to]}({edge.weight:g})" for edge in self.adjList[i]
            ) or "-"

            print(
                f"{i:<3} {self.nombres[i]:<22} {coord_str:<32} "
                f"{out_deg:>3} {in_deg:>3}  {adj_str}"
            )

        print("=" * 80 + "\n")

    def Grado_Out(self, n):
        """
        Obtiene el grado de salida de un vértice.
        Args:
            n (int): Índice del vértice.
        Returns:
            int: Grado de salida.
        """
        return len(self.adjList[n])

    def Grado_In(self, n):
        """
        Obtiene el grado de entrada de un vértice.
        Args:
            n (int): Índice del vértice.
        Returns:
            int: Grado de entrada.
        """
        grado_in = 0
        for lista in self.adjList:
            for edge in lista:
                if edge.to == n:
                    grado_in += 1
        return grado_in

    def Caminos(self, VO, VD):
        """
        Encuentra y muestra todos los caminos posibles entre dos vértices.
        Args:
            VO (int): Vértice de origen.
            VD (int): Vértice de destino.
        """
        Visto = [False] * self.V
        Ruta = []
        Peso_T = [0]  # Se comporta como int& en C++

        print(
            f"Caminos entre {self.nombres[VO]} [{VO}] "
            f"y {self.nombres[VD]} [{VD}]:"
        )
        self.Buscar_CaminosAux(VO, VD, Visto, Ruta, Peso_T)

    def Buscar_CaminosAux(self, V_Actual, VD, Visto, Ruta, Peso_T):
        """
        Función auxiliar para buscar caminos entre dos vértices usando backtracking.
        Args:
            V_Actual (int): Vértice actual.
            VD (int): Vértice de destino.
            Visto (list): Lista de vértices visitados.
            Ruta (list): Ruta actual.
            Peso_T (list): Peso total acumulado.
        """
        Visto[V_Actual] = True
        Ruta.append(V_Actual)

        if V_Actual == VD:
            indices_line = " -> ".join(str(v) for v in Ruta)
            nombres_line = " -> ".join(self.nombres[v] for v in Ruta)
            print(f"  {nombres_line}")
            print(f"    Índices: {indices_line} | Costo total: {Peso_T[0]:g}")
        else:
            for e in self.adjList[V_Actual]:
                if not Visto[e.to]:
                    peso_original = Peso_T[0]
                    Peso_T[0] += int(e.weight)
                    self.Buscar_CaminosAux(e.to, VD, Visto, Ruta, Peso_T)
                    Peso_T[0] = peso_original  # Restaurar el peso original al retroceder

        Ruta.pop()
        Visto[V_Actual] = False


    def Camino_Minimo_Dijkstra(self, VO, VD):
        """
        Encuentra el camino mínimo entre dos vértices usando el algoritmo de Dijkstra.
        Args:
            VO (int): Vértice de origen.
            VD (int): Vértice de destino.

        Returns:
            tuple[list[int], float]: Una tupla con la lista de índices del camino
            en orden desde VO hasta VD y el coste total. Si no existe camino,
            devuelve ([], inf).
        """
        import heapq

        dist = [float("inf")] * self.V
        prev = [-1] * self.V
        visitado = [False] * self.V
        pq = []

        dist[VO] = 0.0
        heapq.heappush(pq, (0.0, VO))

        while pq:
            _, u = heapq.heappop(pq)

            if visitado[u]:
                continue  # Si ya se visitó este nodo, continuar con el siguiente.
            visitado[u] = True  # Marcar como visitado

            if u == VD:
                break  # Si se llegó al destino, terminar.

            for e in self.adjList[u]:
                v = e.to
                weight = e.weight

                if dist[u] + weight < dist[v]:
                    dist[v] = dist[u] + weight
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))

        # Si no se alcanzó el destino, no hay camino
        if dist[VD] == float("inf"):
            print(f"No existe camino desde {VO} a {VD}")
            return [], float("inf")

        # Reconstruir el camino
        path = []
        u = VD
        total_cost = dist[VD]
        while u != -1:
            path.append(u)
            u = prev[u]

        ordered = list(reversed(path))
        nombres_line = " -> ".join(self.nombres[v] for v in ordered)
        indices_line = " -> ".join(str(v) for v in ordered)

        # Mostrar el camino de forma legible
        print("Camino más corto usando Dijkstra:")
        print(f"  {nombres_line}")
        print(f"  Índices: {indices_line}")
        print(f"  Costo total: {total_cost:g}")
        return ordered, total_cost


    def Camino_Minimo_BFS(self, VO, VD):
        """
        Encuentra el camino mínimo entre dos vértices usando una versión modificada de BFS.
        Args:
            VO (int): Vértice de origen.
            VD (int): Vértice de destino.
        """
        from collections import deque

        dist = [float("inf")] * self.V
        prev = [-1] * self.V
        q = deque()

        dist[VO] = 0.0
        q.append(VO)

        while q:
            u = q.popleft()

            for e in self.adjList[u]:
                v = e.to
                weight = e.weight

                if dist[u] + weight < dist[v]:
                    dist[v] = dist[u] + weight
                    prev[v] = u
                    q.append(v)

        camino = []
        nodo_actual = VD
        if prev[nodo_actual] == -1:
            print(f"No existe camino desde {VO} a {VD}")
            return

        while nodo_actual != -1:
            camino.append(nodo_actual)
            nodo_actual = prev[nodo_actual]

        ordered = list(reversed(camino))
        nombres_line = " -> ".join(self.nombres[n] for n in ordered)
        indices_line = " -> ".join(str(n) for n in ordered)

        print("Camino más corto usando BFS modificado:")
        print(f"  {nombres_line}")
        print(f"  Índices: {indices_line}")
        print(f"  Costo total: {dist[VD]:g}")



def cargar_grafo_desde_json(json_path=None):
    """
    Carga un grafo desde un archivo JSON.

    Formato esperado:
    {
      "nombres": ["A Coruña", "Santiago", ...],
      "aristas": [
         {"from": 0, "to": 1, "weight": 22},
         ...
      ]
    }
    """
    import json
    from pathlib import Path

    if json_path is None:
        json_path = Path(__file__).resolve().parent / "grafo.json"
    else:
        json_path = Path(json_path)

    if not json_path.exists():
        raise FileNotFoundError(f"No se encontró el archivo de grafo: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    nombres = data.get("nombres")
    if nombres is None:
        raise KeyError("El JSON debe contener la clave 'nombres' con la lista de nombres de nodos.")

    # Cargar coordenadas opcionales de cada nodo (si existen)
    # Formato esperado en el JSON:
    # "coordenadas": [
    #   {"x": 0.0, "y": 0.0, "theta": 0.0},
    #   ...
    # ]
    coords_raw = data.get("coordenadas")
    coords = None
    if coords_raw is not None:
        if not isinstance(coords_raw, list):
            raise TypeError("La clave 'coordenadas' debe contener una lista.")
        if len(coords_raw) != len(nombres):
            raise ValueError(
                "La lista 'coordenadas' debe tener la misma longitud que 'nombres'."
            )

        coords = []
        for idx, c in enumerate(coords_raw):
            if c is None:
                coords.append(None)
                continue
            if not isinstance(c, dict):
                raise TypeError(
                    f"Cada entrada de 'coordenadas' debe ser un objeto con x, y, theta (índice {idx})."
                )
            x = float(c.get("x", 0.0))
            y = float(c.get("y", 0.0))
            theta = float(c.get("theta", 0.0))
            coords.append({"x": x, "y": y, "theta": theta})

    G = GrafoDP(len(nombres), nombres, coords)

    aristas = data.get("aristas", [])
    for a in aristas:
        if isinstance(a, dict):
            vs = int(a.get("from"))
            ve = int(a.get("to"))
            p = a.get("weight", 0)
        elif isinstance(a, (list, tuple)):
            if len(a) >= 3:
                vs, ve, p = a[0], a[1], a[2]
            else:
                raise ValueError("Arista en formato lista necesita tres elementos [from, to, weight].")
        else:
            raise ValueError("Formato de arista no reconocido. Use dicts o listas.")

        G.Agregar_Arista(int(vs), int(ve), p)

    return G


def main():
    """
    Función principal que inicializa el grafo, solicita datos al usuario y muestra los resultados de los algoritmos.
    Ahora carga los nodos y aristas desde `grafo.json` ubicado en el mismo directorio.
    """
    # Cargar el grafo desde el archivo JSON en el mismo directorio que este script.
    try:
        GND = cargar_grafo_desde_json()
    except Exception as e:
        print(f"Error al cargar grafo desde JSON: {e}")
        return

    # Mostrar el grafo
    print("\n" + "=" * 80)
    print("GRAFO DIRIGIDO PONDERADO - RED DE CIUDADES")
    print("=" * 80)
    GND.Muestra_GrafoDP()

    # Solicitar al usuario los vértices de origen y destino
    print("Seleccione vértices por índice según la tabla anterior.")
    vi = int(input("  Origen [ID]: "))
    vf = int(input("  Destino [ID]: "))
    print()

    # Encontrar y mostrar todos los caminos posibles entre origen y destino
    print("-" * 80)
    print(
        f"CAMINOS ENTRE {GND.nombres[vi]} [{vi}] "
        f"Y {GND.nombres[vf]} [{vf}]"
    )
    print("-" * 80)
    GND.Caminos(vi, vf)
    print()

    # Encontrar y mostrar el camino mínimo usando Dijkstra
    print("-" * 80)
    print("CAMINO MÍNIMO - DIJKSTRA")
    print("-" * 80)
    GND.Camino_Minimo_Dijkstra(vi, vf)
    print()

    # Encontrar y mostrar el camino mínimo usando BFS modificado
    print("-" * 80)
    print("CAMINO MÍNIMO - BFS MODIFICADO")
    print("-" * 80)
    GND.Camino_Minimo_BFS(vi, vf)


if __name__ == "__main__":
    main()
