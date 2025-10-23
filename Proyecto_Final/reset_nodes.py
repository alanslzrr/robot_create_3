"""
reset_nodes.py

Elimina datos de mapeo y logs generados por el sistema para comenzar de cero:
 - nodes/nodes.jsonl
 - nodes/edges.jsonl
 - nodes/VERSION
 - nodes/logs/* (telemetría, intentos de navegación, métricas de aristas)

Uso:
  python reset_nodes.py

Seguro si archivos no existen. No elimina código ni configuración.
"""

from pathlib import Path


def delete_file(path: Path) -> bool:
    try:
        if path.exists():
            path.unlink()
            return True
    except Exception:
        pass
    return False


def delete_dir_files(dir_path: Path) -> int:
    count = 0
    try:
        if dir_path.exists() and dir_path.is_dir():
            for p in dir_path.iterdir():
                try:
                    if p.is_file() or p.is_symlink():
                        p.unlink()
                        count += 1
                    elif p.is_dir():
                        # Eliminar contenido de subdirectorios (si los hubiera)
                        count += delete_dir_files(p)
                        # No borramos el directorio raíz, solo su contenido
                except Exception:
                    # Ignorar errores individuales
                    pass
    except Exception:
        pass
    return count


def main() -> None:
    root = Path(__file__).parent
    nodes_dir = root / "nodes"
    logs_dir = nodes_dir / "logs"

    removed = []
    for fname in ("nodes.jsonl", "edges.jsonl", "VERSION"):
        fpath = nodes_dir / fname
        if delete_file(fpath):
            removed.append(str(fpath))

    logs_removed = delete_dir_files(logs_dir)

    print("=== Reset de datos de mapeo y logs ===")
    for r in removed:
        print(f"- Eliminado: {r}")
    print(f"- Eliminados en logs: {logs_removed} archivo(s) en {logs_dir}")
    print("Listo.")


if __name__ == "__main__":
    main()


