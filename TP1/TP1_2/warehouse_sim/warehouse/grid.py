from __future__ import annotations
from typing import Dict, List, Optional, Tuple

from warehouse.cell import (
    Cell, AisleCell, ShelfCell, ChargingStationCell, ObstacleCell, CellType
)
from warehouse.entities import Shelf, ChargingStation, Obstacle, Forklift


class Grid:
    def __init__(self, width: int, height: int) -> None:
        self.width: int = width
        self.height: int = height
        self.cells: List[List[Cell]] = [
            [AisleCell(x, y) for x in range(width)]
            for y in range(height)
        ]
        self.forklifts: Dict[str, Forklift] = {}
        self.shelves: Dict[str, Shelf] = {}
        self.obstacles: Dict[str, Obstacle] = {}
        self.charging_stations: Dict[str, ChargingStation] = {}

    def get_cell(self, x: int, y: int) -> Cell:
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise IndexError(
                f"Position ({x},{y}) is outside grid bounds ({self.width}x{self.height})."
            )
        return self.cells[y][x]

    def _replace_cell(self, new_cell: Cell) -> None:
        self.cells[new_cell.pos_y][new_cell.pos_x] = new_cell

    def get_neighbors(self, cell: Cell) -> List[Cell]:
        directions: List[Tuple[int, int]] = [(0, -1), (0, 1), (-1, 0), (1, 0)]
        neighbours: List[Cell] = []
        for dx, dy in directions:
            nx, ny = cell.pos_x + dx, cell.pos_y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                neighbours.append(self.cells[ny][nx])
        return neighbours

    def create_shelf(self, name: str, x: int, y: int) -> Shelf:
        if name in self.shelves:
            raise ValueError(f"A shelf named '{name}' already exists.")
        shelf_cell = ShelfCell(x, y)
        self._replace_cell(shelf_cell)
        shelf = Shelf(name=name, cell=shelf_cell)
        shelf_cell.set_occupant(shelf)
        self.shelves[name] = shelf
        return shelf

    def create_charging_station(self, name: str, x: int, y: int) -> ChargingStation:
        if name in self.charging_stations:
            raise ValueError(f"A charging station named '{name}' already exists.")
        station_cell = ChargingStationCell(x, y)
        self._replace_cell(station_cell)
        station = ChargingStation(name=name, cell=station_cell)
        station_cell.set_occupant(station)
        self.charging_stations[name] = station
        return station

    def create_obstacle(self, name: str, x: int, y: int) -> Obstacle:
        if name in self.obstacles:
            raise ValueError(f"An obstacle named '{name}' already exists.")
        obs_cell = ObstacleCell(x, y)
        self._replace_cell(obs_cell)
        obstacle = Obstacle(name=name, cell=obs_cell)
        obs_cell.set_occupant(obstacle)
        self.obstacles[name] = obstacle
        return obstacle

    def remove_obstacle(self, name: str) -> None:
        if name not in self.obstacles:
            raise KeyError(f"No obstacle named '{name}' found.")
        obstacle = self.obstacles.pop(name)
        x, y = obstacle.cell.pos_x, obstacle.cell.pos_y
        aisle_cell = AisleCell(x, y)
        self._replace_cell(aisle_cell)

    def create_forklift(
        self,
        name: str,
        x: Optional[int] = None,
        y: Optional[int] = None,
        priority: Optional[float] = None,
    ) -> Forklift:
        if name in self.forklifts:
            raise ValueError(f"A forklift named '{name}' already exists.")

        if x is None or y is None:
            start_cell = self._find_free_charging_station()
            if start_cell is None:
                start_cell = self.get_cell(0, 0) #Si no existe una estación de carga libre, colocar en (0,0) si es posible
                if not start_cell.is_walkable:
                    raise ValueError(
                        f"No free charging station available and cell (0,0) is not walkable — cannot place forklift."
                    )   
        else:
            start_cell = self.get_cell(x, y)
            if not start_cell.is_walkable:
                raise ValueError(
                    f"Cell ({x},{y}) is not walkable — cannot place forklift."
                )

        forklift = Forklift(name=name, current_cell=start_cell, priority=priority if priority is not None else None)
        forklift._grid = self
        for shelf in self.shelves.values():
            forklift.known_obstacles.add(shelf.cell)
        start_cell.set_occupant(forklift)
        self.forklifts[name] = forklift
        return forklift

    def _find_free_charging_station(self) -> Optional[Cell]:
        for station in self.charging_stations.values():
            cell = station.cell
            if not isinstance(cell.occupant, Forklift):
                return cell
        return None

    def resolve_goal(self, target: str | Tuple[int, int]) -> Cell:
        if isinstance(target, tuple):
            x, y = target
            if isinstance(self.get_cell(x, y), Shelf):
                return self._shelf_goal_cell(self.shelves[self.get_cell(x, y).name])
            return self.get_cell(x, y)

        if target in self.shelves:
            return self._shelf_goal_cell(self.shelves[target])

        raise ValueError(f"Unknown goal target: {target!r}")

    def _shelf_goal_cell(self, shelf: Shelf) -> Cell:
        sx, sy = shelf.cell.pos_x, shelf.cell.pos_y
        for dx in (-1, 1):
            nx = sx + dx
            if 0 <= nx < self.width:
                candidate = self.get_cell(nx, sy)
                if candidate.is_walkable:
                    return candidate
        raise RuntimeError(
            f"Shelf '{shelf.name}' has no accessible adjacent cell for delivery."
        )

    def ascii_render(self) -> str:
        """
        Return a simple ASCII representation of the grid for debugging.
        Legend:
            .  — aisle (empty)
            F  — forklift
            S  — shelf
            C  — charging station
            X  — obstacle
        """
        lines: List[str] = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                cell = self.cells[y][x]
                if cell.cell_type == CellType.SHELF:
                    row.append("S")
                elif cell.cell_type == CellType.OBSTACLE:
                    row.append("X")
                elif cell.cell_type == CellType.CHARGING_STATION:
                    row.append("C" if not isinstance(cell.occupant, Forklift) else "F")
                elif isinstance(cell.occupant, Forklift):
                    row.append("F")
                else:
                    row.append(".")
            lines.append(" ".join(row))
        return "\n".join(lines)

    def __repr__(self) -> str:
        return f"Grid({self.width}x{self.height})"
