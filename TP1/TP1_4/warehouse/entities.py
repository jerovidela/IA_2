from __future__ import annotations
import random
from typing import List, Optional, TYPE_CHECKING 



if TYPE_CHECKING:
    from warehouse.cell import Cell
    from warehouse.grid import Grid
    from pathfinding.astar import AStar
    from pathfinding.heuristics import HeuristicFn

PRIORITY_INCREMENT: float = 0.1
REPLAN_THRESHOLD: int = 3

class Shelf:
    def __init__(self, name: str, cell: "Cell") -> None:
        self.name: str = name
        self.cell: "Cell" = cell

    def __repr__(self) -> str:
        return f"Shelf({self.name} @ ({self.cell.pos_x},{self.cell.pos_y}))"


class ChargingStation:
    def __init__(self, name: str, cell: "Cell") -> None:
        self.name: str = name
        self.cell: "Cell" = cell

    def __repr__(self) -> str:
        return f"ChargingStation({self.name} @ ({self.cell.pos_x},{self.cell.pos_y}))"


class Obstacle:
    def __init__(self, name: str, cell: "Cell") -> None:
        self.name: str = name
        self.cell: "Cell" = cell

    def __repr__(self) -> str:
        return f"Obstacle({self.name} @ ({self.cell.pos_x},{self.cell.pos_y}))"


class Forklift:
    def __init__(self, name: str, current_cell: "Cell", priority: Optional[float] = None) -> None:
        self.name: str = name
        self.current_cell: "Cell" = current_cell
        self.base_priority: float = random.random() if priority is None else priority
        self.wait_steps: int = 0
        self.blocked_steps: int = 0
        self.path: List["Cell"] = []
        self.goal_cell: Optional["Cell"] = None
        self.penalized_cells: dict["Cell", float] = {}
        self._grid: Optional["Grid"] = None
        self._skipping_turn: bool = False
        self.known_obstacles: set["Cell"] = set()

    @property
    def effective_priority(self) -> float:
        return self.base_priority + self.wait_steps * PRIORITY_INCREMENT

    def plan_path(
        self,
        start_cell: "Cell",
        goal_cell: "Cell",
        heuristic: "HeuristicFn",
    ) -> List["Cell"]:
        from pathfinding.astar import AStar

        assert self._grid is not None, "Forklift must be attached to a grid before planning."

        planner = AStar(
            grid=self._grid,
            heuristic=heuristic,
            penalized_cells=self.penalized_cells,
            known_obstacles=self.known_obstacles,
        )
        path = planner.search(start_cell, goal_cell)
        self.path = path
        return path

    def detect_obstacle(self, next_cell: "Cell") -> bool:
        if not next_cell.is_walkable:
            self.known_obstacles.add(next_cell)
            self._penalize_cell(next_cell, extra_weight=float('inf'))
            return True
        from warehouse.entities import Forklift as _Forklift
        if isinstance(next_cell.occupant, _Forklift) and next_cell.occupant is not self:
            return True
        return False

    def _penalize_cell(self, cell: "Cell", extra_weight: float) -> None:
        current = self.penalized_cells.get(cell, 0.0)
        self.penalized_cells[cell] = current + extra_weight

    def move(self) -> bool:
        if not self.path:
            return False

        #next_cell = self.path[0]
        # 1. Obtener coordenadas de la ruta y consultar la celda real en el Grid
        next_cell_memoria = self.path[0]
        next_cell = self._grid.get_cell(next_cell_memoria.pos_x, next_cell_memoria.pos_y)
        if self.detect_obstacle(next_cell):
            self.blocked_steps += 1
            self.wait_steps += 1
            self.path = []
            return False

        self.current_cell.set_occupant(None)
        next_cell.set_occupant(self)
        self.current_cell = next_cell
        self.path.pop(0)
        self.penalized_cells = {}
        self.wait_steps = 0
        self.blocked_steps = 0
        return True

    def step(self, heuristic: "HeuristicFn") -> str:
        if self.goal_cell is None:
            return "idle"

        if self.current_cell == self.goal_cell:
            self.path = []
            return "at_goal"
        
        if getattr(self, '_skipping_turn', False):
            self._skipping_turn = False
            return "waiting"

        if not self.path:
            planned = self.plan_path(self.current_cell, self.goal_cell, heuristic)
            if not planned:
                self.wait_steps += 1
                return "no_path"

        if self.path:
            next_cell_memoria = self.path[0]
            next_cell = self._grid.get_cell(next_cell_memoria.pos_x, next_cell_memoria.pos_y)
            #next_cell = self.path[0]
            if self.detect_obstacle(next_cell):
               # self.blocked_steps += 1
                self.wait_steps += 1
                if not next_cell.is_walkable:
                    self.blocked_steps = 0
                    self.plan_path(self.current_cell, self.goal_cell, heuristic)
                
                # B) Si es transitable (es otro montacargas): usar paciencia (REPLAN_THRESHOLD)
                else:
                    self.blocked_steps += 1
                    if self.blocked_steps >= REPLAN_THRESHOLD:
                        self.blocked_steps = 0
                        self._penalize_cell(next_cell, extra_weight=50.0)
                        self.plan_path(self.current_cell, self.goal_cell, heuristic)
                        
                return "blocked"
                #if self.blocked_steps >= REPLAN_THRESHOLD:
                    #self.blocked_steps = 0
                    #self._penalize_cell(next_cell, extra_weight=50.0)
                    #self.plan_path(self.current_cell, self.goal_cell, heuristic)
                #return "blocked"

        moved = self.move()
        return "moved" if moved else "waiting"

    def __repr__(self) -> str:
        pos = f"({self.current_cell.pos_x},{self.current_cell.pos_y})"
        return f"Forklift({self.name} @ {pos}, priority={self.effective_priority:.3f})"
