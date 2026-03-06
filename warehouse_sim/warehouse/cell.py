from __future__ import annotations
from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from warehouse.entities import Forklift, Shelf, ChargingStation, Obstacle


class CellType(Enum):
    AISLE = auto()
    SHELF = auto()
    CHARGING_STATION = auto()
    OBSTACLE = auto()


class Cell(ABC):
    def __init__(
        self,
        pos_x: int,
        pos_y: int,
        cell_type: CellType,
        movement_weight: float = 1.0,
    ) -> None:
        self.pos_x: int = pos_x
        self.pos_y: int = pos_y
        self.cell_type: CellType = cell_type
        self.movement_weight: float = movement_weight
        self.occupant: Optional[object] = None

    @property
    def is_walkable(self) -> bool:
        return self.cell_type not in (CellType.SHELF, CellType.OBSTACLE)

    @abstractmethod
    def set_occupant(self, obj: Optional[object]) -> None:
        """
        Assign or clear the occupant of this cell.
        Subclasses must enforce their own occupation rules.
        """

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}(x={self.pos_x}, y={self.pos_y}, "
            f"type={self.cell_type.name}, occupant={self.occupant})"
        )

    def __hash__(self) -> int:
        return hash((self.pos_x, self.pos_y))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Cell):
            return NotImplemented
        return self.pos_x == other.pos_x and self.pos_y == other.pos_y


class AisleCell(Cell):
    def __init__(self, pos_x: int, pos_y: int, movement_weight: float = 1.0) -> None:
        super().__init__(pos_x, pos_y, CellType.AISLE, movement_weight)

    def set_occupant(self, obj: Optional[object]) -> None:
        from warehouse.entities import Forklift, Shelf, Obstacle

        if obj is None:
            self.occupant = None
            return
        if isinstance(obj, Shelf):
            raise ValueError(f"Cannot place a shelf on an aisle cell {self}.")
        if isinstance(obj, Forklift) and self.occupant is not None:
            raise ValueError(
                f"Cell ({self.pos_x},{self.pos_y}) is already occupied by {self.occupant}."
            )
        if isinstance(obj, Obstacle):
            raise ValueError(f"Cannot place an obstacle on an aisle cell {self}.")
        
        self.occupant = obj


class ShelfCell(Cell):
    def __init__(self, pos_x: int, pos_y: int) -> None:
        super().__init__(pos_x, pos_y, CellType.SHELF, movement_weight=float('inf'))

    def set_occupant(self, obj: Optional[object]) -> None:
        from warehouse.entities import Shelf

        if obj is None:
            self.occupant = None
            return
        if not isinstance(obj, Shelf):
            raise ValueError(f"Only a Shelf entity may occupy a ShelfCell, got {type(obj)}.")
        self.occupant = obj


class ChargingStationCell(Cell):
    def __init__(self, pos_x: int, pos_y: int, movement_weight: float = 1.0) -> None:
        super().__init__(pos_x, pos_y, CellType.CHARGING_STATION, movement_weight)

    def set_occupant(self, obj: Optional[object]) -> None:
        from warehouse.entities import Forklift, ChargingStation

        if obj is None:
            self.occupant = None
            return
        if isinstance(obj, Forklift):
            self.occupant = obj
            return
        if isinstance(obj, ChargingStation):
            self.occupant = obj
            return
        raise ValueError(f"Unexpected occupant type {type(obj)} for ChargingStationCell.")


class ObstacleCell(Cell):
    def __init__(self, pos_x: int, pos_y: int) -> None:
        super().__init__(pos_x, pos_y, CellType.OBSTACLE, movement_weight=float('inf'))

    def set_occupant(self, obj: Optional[object]) -> None:
        from warehouse.entities import Obstacle

        if obj is None:
            self.occupant = None
            return
        if not isinstance(obj, Obstacle):
            raise ValueError(f"Only an Obstacle entity may occupy an ObstacleCell.")
        self.occupant = obj
