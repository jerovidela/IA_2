from __future__ import annotations
from typing import Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from warehouse.cell import Cell

HeuristicFn = Callable[["Cell", "Cell"], float]


def manhattan_distance(a: "Cell", b: "Cell") -> float:
    return float(abs(a.pos_x - b.pos_x) + abs(a.pos_y - b.pos_y))


def euclidean_distance(a: "Cell", b: "Cell") -> float:
    return float(((a.pos_x - b.pos_x) ** 2 + (a.pos_y - b.pos_y) ** 2) ** 0.5)


def zero_heuristic(a: "Cell", b: "Cell") -> float:
    return 0.0
