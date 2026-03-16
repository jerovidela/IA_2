# Autonomous Forklift Warehouse Simulation

A modular, object-oriented Python simulation of autonomous forklifts navigating a warehouse using the A* search algorithm.

---

## Project Structure

```
warehouse_sim/
├── main.py                        # Entry point & demonstration script
│
├── warehouse/                     # Domain model
│   ├── cell.py                    # Cell ABC + AisleCell, ShelfCell, etc.
│   ├── entities.py                # Shelf, ChargingStation, Obstacle, Forklift
│   └── grid.py                    # Grid — spatial model of the warehouse
│
├── pathfinding/                   # Generic pathfinding (reusable)
│   ├── astar.py                   # A* search algorithm
│   └── heuristics.py              # Manhattan, Euclidean, Zero heuristics
│
├── simulation/
│   └── simulation.py              # Simulation loop + CollisionResolver
│
└── rendering/
    └── renderer.py                # Pygame graphical renderer (optional)
```

---

## Running

### Graphical mode (requires pygame)
```bash
pip install pygame
python main.py
```

### Headless / terminal mode
```bash
python main.py --headless
```

---

## Architecture Overview

### Cell (ABC)
Abstract base. Concrete subclasses: `AisleCell`, `ShelfCell`, `ChargingStationCell`, `ObstacleCell`. Each enforces its own occupation rules in `set_occupant()`.

### Grid
Owns the 2-D cell matrix and all entity dictionaries. Factory methods (`create_shelf`, `create_forklift`, etc.) mutate the grid in a controlled way. `resolve_goal()` converts a shelf name or (x,y) tuple into the target cell.

### AStar
Stateless, generic A* over any `Grid`. Accepts an injectable heuristic and a per-forklift `penalized_cells` dict for obstacle avoidance. Uses a min-heap via `heapq` for O(log n) open-set operations.

### Forklift
Autonomous agent. Holds its path, goal, priority counters, and penalized cell map. `step()` implements the full decision loop: plan → detect → block/replan → move. Priority grows with `wait_steps` to prevent deadlocks.

### Simulation + CollisionResolver
`Simulation.step()` sorts forklifts by `effective_priority`, calls `CollisionResolver.resolve()` to handle head-on swaps and same-target conflicts *before* movement, then executes each forklift's `step()`.

### WarehouseRenderer
Optional Pygame layer. Draws cell colours, translucent path overlays, forklift circles, and a live stats panel. Completely decoupled from the simulation logic.

---

## Key Design Decisions

| Decision | Rationale |
|---|---|
| Cell ABC with concrete subclasses | Enforces different occupation rules per cell type at the type level |
| Heuristic injection | Keeps AStar reusable for non-warehouse problems |
| `penalized_cells` per forklift | Local obstacle memory; each agent has independent avoidance state |
| `blocked_steps >= threshold` before replan | Prevents path oscillation in congested corridors |
| Dynamic priority (`base + wait * increment`) | Guarantees livelock freedom under contention |
| `CollisionResolver` as separate class | Single-responsibility; easy to swap or extend resolution strategies |

---

## Extending the System

- **New cell type**: Add a variant to `CellType`, create a subclass of `Cell`, override `set_occupant`.
- **New heuristic**: Add a function with signature `(Cell, Cell) -> float` in `heuristics.py`.
- **Diagonal movement**: Extend `Grid.get_neighbors()` to include diagonal directions.
- **Multi-goal missions**: Give `Forklift` a queue of `goal_cell` values; pop on arrival.
- **Battery / charging logic**: Add `battery` attribute to `Forklift`; return to `ChargingStation` when low.
