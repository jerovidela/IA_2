from __future__ import annotations
import heapq
from typing import Dict, List, Optional, Set, TYPE_CHECKING

if TYPE_CHECKING:
    from warehouse.cell import Cell
    from warehouse.grid import Grid
    from pathfinding.heuristics import HeuristicFn


class _PrioritizedCell:
    __slots__ = ("f_score", "cell")

    def __init__(self, f_score: float, cell: "Cell") -> None:
        self.f_score = f_score
        self.cell = cell

    def __lt__(self, other: "_PrioritizedCell") -> bool:
        return self.f_score < other.f_score

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, _PrioritizedCell):
            return NotImplemented
        return self.f_score == other.f_score


class AStar:
    def __init__(
        self,
        grid: "Grid",
        heuristic: "HeuristicFn",
        penalized_cells: Optional[Dict["Cell", float]] = None,
        known_obstacles: Optional[set] = None,
    ) -> None:
        self.grid = grid
        self.heuristic = heuristic
        self.penalized_cells: Dict["Cell", float] = penalized_cells or {}
        self.known_obstacles: set = known_obstacles or set()

    def search(self, start: "Cell", goal: "Cell") -> List["Cell"]:
        if start == goal:
            return []

        open_set: List[_PrioritizedCell] = []
        heapq.heappush(open_set, _PrioritizedCell(0.0, start))
        came_from: Dict["Cell", Optional["Cell"]] = {start: None}
        g_score: Dict["Cell", float] = {start: 0.0}
        closed_set: Set["Cell"] = set()

        while open_set:
            current_wrapper = heapq.heappop(open_set)
            current: "Cell" = current_wrapper.cell

            if current in closed_set:
                continue

            if current == goal:
                return self._reconstruct_path(came_from, goal)

            closed_set.add(current)

            for neighbour in self.grid.get_neighbors(current):
                if neighbour in closed_set:
                    continue
                if neighbour in self.known_obstacles:
                    continue

                step_cost = (
                    neighbour.movement_weight
                    + self.penalized_cells.get(neighbour, 0.0)
                )
                tentative_g = g_score[current] + step_cost

                if tentative_g < g_score.get(neighbour, float("inf")):
                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbour, goal)
                    heapq.heappush(open_set, _PrioritizedCell(f_score, neighbour))

        return []
    
    @staticmethod
    def _reconstruct_path(
        came_from: Dict["Cell", Optional["Cell"]],
        goal: "Cell",
    ) -> List["Cell"]:
        path: List["Cell"] = []
        current: Optional["Cell"] = goal
        while current is not None and came_from.get(current) is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path
