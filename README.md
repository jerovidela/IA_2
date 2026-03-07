# Documentación Técnica: API del Simulador de Almacén

Este documento detalla la estructura interna de las clases, sus responsabilidades y cómo instanciarlas para desarrollar sobre el simulador.

---

## 1. Módulo de Búsqueda (Prioridad Principal)

### Clase `AStar` (`astar.py`)

Implementa el algoritmo de búsqueda de caminos óptimos. Está diseñado para integrarse con la grilla y soportar pesos dinámicos y penalizaciones para evitar colisiones.

**Atributos:**

* `grid`: Referencia a la instancia actual de `Grid`.
* `heuristic`: Función heurística inyectada (ej. `manhattan_distance`).
* `penalized_cells` (Dict): Diccionario que mapea objetos `Cell` a un peso extra temporal (`float`), usado para evitar celdas con posibles colisiones temporales.
* `known_obstacles` (Set): Conjunto de objetos `Cell` que el agente descubrió como bloqueos físicos insalvables.

**Métodos:**

* `__init__(grid, heuristic, penalized_cells=None, known_obstacles=None)`: Constructor que inicializa el buscador con el entorno y las memorias del agente.
* `search(start: Cell, goal: Cell) -> List[Cell]`: Ejecuta el algoritmo. Evalúa los vecinos transitables utilizando el costo $g$ (costo acumulado incluyendo el `movement_weight` de la celda y las penalizaciones) y el costo $f$ ($g$ + heurística). Retorna la lista de celdas que conforman la ruta óptima, o una lista vacía si no hay camino.

**Cómo implementarlo:**
Normalmente, el `Forklift` instancia esta clase internamente en su método `plan_path`, pasándole sus propias penalizaciones. Para usarlo manualmente:

```python
from pathfinding.astar import AStar
from pathfinding.heuristics import manhattan_distance

# Requiere tener instanciados un 'Grid' y dos 'Cell'
buscador = AStar(grid=mi_almacen, heuristic=manhattan_distance)
ruta = buscador.search(celda_inicio, celda_objetivo)

```

---

## 2. Entorno y Mapa (`grid.py` y `cell.py`)

### Clase `Grid` (`grid.py`)

Maneja la matriz bidimensional del almacén y actúa como registro central de todas las entidades.

**Atributos:**

* `width`, `height`: Dimensiones enteras del mapa.
* `cells`: Matriz 2D (lista de listas) que contiene todas las instancias de `Cell`.
* `forklifts`, `shelves`, `obstacles`, `charging_stations`: Diccionarios que almacenan las entidades del mapa indexadas por su nombre (`name`).

**Métodos Principales:**

* `get_cell(x: int, y: int) -> Cell`: Retorna la celda exacta en esas coordenadas.
* `get_neighbors(cell: Cell) -> List[Cell]`: Devuelve las celdas ortogonales adyacentes (arriba, abajo, izquierda, derecha) respetando los límites del mapa.
* `create_forklift(name: str, x: int=None, y: int=None, priority: float=None) -> Forklift`: Instancia un montacargas. Si `x` e `y` son nulos, busca una `ChargingStation` libre.
* `resolve_goal(target) -> Cell`: Convierte un nombre de estantería (ej. `"SH01"`) en la celda transitable inmediatamente adyacente para que el montacargas pueda llegar a ella.

### Clase abstracta `Cell` y Subclases (`cell.py`)

Representan cada cuadrante físico. Sus subclases son: `AisleCell`, `ShelfCell`, `ChargingStationCell` y `ObstacleCell`.

**Atributos:**

* `pos_x`, `pos_y`: Coordenadas enteras.
* `cell_type`: Enumerador que define su tipo.
* `movement_weight`: Costo base de atravesarla (`1.0` por defecto, `float('inf')` para celdas bloqueadas permanentemente como estanterías).
* `occupant`: Referencia a la entidad física apoyada sobre ella (o `None`).
* `is_walkable` (Property): Retorna `True` si no es estantería ni obstáculo fijo.

**Cómo implementarlo:**
Se interactúa con la grilla mediante sus creadores:

```python
almacen = Grid(width=13, height=11)
almacen.create_shelf("SH01", x=2, y=1) # Reemplaza el AisleCell por un ShelfCell
celda = almacen.get_cell(x=2, y=2)
vecinos = almacen.get_neighbors(celda)

```

---

## 3. Entidades (`entities.py`)

### Clase `Forklift`

Es el agente inteligente. Controla su propio movimiento, prioridades de tráfico y su interacción con el entorno.

**Atributos:**

* `name`: Identificador (ej. `"Alpha"`).
* `current_cell`, `goal_cell`: Celdas de ubicación actual y destino final.
* `path`: Lista de `Cell` pendientes por recorrer.
* `base_priority`: Prioridad inicial de paso al haber colisión (0.0 a 1.0).
* `wait_steps`, `blocked_steps`: Contadores de turnos que el vehículo lleva estancado.
* `effective_priority` (Property): Calcula `base_priority + (wait_steps * PRIORITY_INCREMENT)` para desempatar colisiones.

**Métodos Principales:**

* `plan_path(start, goal, heuristic)`: Instancia `AStar` y actualiza la lista `self.path`.
* `detect_obstacle(next_cell: Cell) -> bool`: Revisa si la próxima celda no es `walkable` o si está ocupada por otro montacargas. Si es un bloqueo físico, la añade a `known_obstacles`.
* `move() -> bool`: Mueve físicamente el ocupante de `current_cell` a `next_cell` en la grilla y recorta el `path`.
* `step(heuristic) -> str`: Función principal que se llama en cada turno. Maneja si el vehículo avanza, espera, recalcula (`plan_path`) o ha llegado a destino.

**Cómo implementarlo:**
No se debe instanciar manualmente. Se crea y controla a través de `Grid` y `Simulation`:

```python
agente = almacen.create_forklift("Beta", priority=0.8)
# Para moverlo un paso, el motor llamará a:
estado = agente.step(manhattan_distance)

```

---

## 4. Motor de Simulación (`simulation.py`)

### Clase `Simulation`

Coordina los turnos (timesteps), los agentes y la resolución de colisiones previas al movimiento.

**Atributos:**

* `grid`: Instancia principal de `Grid`.
* `timestep`: Turno actual de la simulación.
* `forklifts` (Property): Lista de todos los agentes activos extraídos de la grilla.
* `_resolver`: Instancia interna de `CollisionResolver`.

**Métodos Principales:**

* `assign_goal(forklift_name: str, target)`: Asigna un objetivo (ej. `"SH12"`) al agente y dispara su cálculo inicial de ruta.
* `step() -> Dict`: Avanza el reloj un turno. Ordena a los agentes por prioridad, invoca al `CollisionResolver` y luego ejecuta el método `step()` de cada agente, retornando sus estados.
* `run()`: Ejecuta automáticamente `step()` en un bucle hasta cumplir los objetivos o agotar el tiempo máximo.

### Clase `CollisionResolver`

Analiza los próximos movimientos de todos los agentes *antes* de que ocurran para evitar cruces no válidos.

**Métodos de Resolución:**

* `_resolve_same_target()`: Detecta si dos agentes buscan ocupar la misma celda en el turno actual. Frena al de menor prioridad, forzándolo a recalcular su ruta.
* `_resolve_position_swap()`: Detecta si dos agentes intentan intercambiar lugares (choque frontal). Detiene al de mayor prioridad obligando al de menor prioridad a retroceder y esquivar.