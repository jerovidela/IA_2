from __future__ import annotations
import logging
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

from warehouse.entities import Forklift
from pathfinding.heuristics import HeuristicFn, manhattan_distance

if TYPE_CHECKING:
    from warehouse.grid import Grid
    from warehouse.cell import Cell

logger = logging.getLogger(__name__)


class CollisionResolver:
    def resolve(
        self,
        forklifts: List[Forklift],
        heuristic: HeuristicFn,
    ) -> None:
        intent: Dict["Cell", List[Forklift]] = {}
        for fl in forklifts:
            if fl.path:
                next_cell = fl.path[0]
                intent.setdefault(next_cell, []).append(fl)

        self._resolve_same_target(intent, heuristic)
        self._resolve_position_swap(forklifts, heuristic)

    def _resolve_same_target(
        self,
        intent: Dict["Cell", List[Forklift]],
        heuristic: HeuristicFn,
    ) -> None:
        for cell, candidates in intent.items():
            if len(candidates) <= 1:
                continue
            #candidates.sort(key=lambda f: f.effective_priority, reverse=True)
            candidates.sort(key=lambda f: (heuristic(f.current_cell, f.goal_cell) if f.goal_cell else float('inf')) - f.wait_steps)
            winner = candidates[0]
            for loser in candidates[1:]:
                logger.debug(
                    "Collision (same target): %s yields to %s at (%d,%d)",
                    loser.name, winner.name, cell.pos_x, cell.pos_y,
                )
                loser.blocked_steps += 1
                loser.wait_steps += 1
                loser.path = []
                if loser.goal_cell:
                    loser.plan_path(loser.current_cell, loser.goal_cell, heuristic)

    def _resolve_position_swap(
        self,
        forklifts: List[Forklift],
        heuristic: HeuristicFn,
    ) -> None:
        cell_to_forklift: Dict["Cell", Forklift] = {
            fl.current_cell: fl for fl in forklifts if fl.path
        }

        checked: set[Tuple[str, str]] = set()
        for fl_a in forklifts:
            if not fl_a.path:
                continue
            next_a = fl_a.path[0]
            fl_b = cell_to_forklift.get(next_a)
            if fl_b is None or not fl_b.path:
                continue
            next_b = fl_b.path[0]

            if next_b == fl_a.current_cell:
                pair = tuple(sorted([fl_a.name, fl_b.name]))
                if pair in checked:
                    continue
                checked.add(pair)

                #high, low = (
                    #(fl_a, fl_b)
                    #if fl_a.effective_priority >= fl_b.effective_priority
                    #else (fl_b, fl_a)
                #)
                # Calculamos la heurística de ambos (distancia al objetivo - impaciencia)
                h_a = (heuristic(fl_a.current_cell, fl_a.goal_cell) if fl_a.goal_cell else float('inf')) - fl_a.wait_steps
                h_b = (heuristic(fl_b.current_cell, fl_b.goal_cell) if fl_b.goal_cell else float('inf')) - fl_b.wait_steps
                
                # El de MENOR heurística (más cerca) es el 'high' (el que tiene prioridad y no se mueve)
                high, low = (fl_a, fl_b) if h_a <= h_b else (fl_b, fl_a)
                logger.debug(
                    "Collision (swap): %s waits, %s replans",
                    high.name, low.name,
                )
                high.wait_steps += 1
                if high.path:
                    high._skipping_turn = True

                low.blocked_steps += 1
                low.wait_steps += 1
                low._penalize_cell(high.current_cell, extra_weight=50.0)
                low.path = []
                if low.goal_cell:
                    low.plan_path(low.current_cell, low.goal_cell, heuristic)


class Simulation:
    def __init__(
        self,
        grid: "Grid",
        heuristic: HeuristicFn = manhattan_distance,
        max_steps: int = 1000,
    ) -> None:
        self.grid = grid
        self.heuristic: HeuristicFn = heuristic
        self.timestep: int = 0
        self.max_steps: int = max_steps
        self._resolver = CollisionResolver()

    @property
    def forklifts(self) -> List[Forklift]:
        return list(self.grid.forklifts.values())

    def step(self) -> Dict[str, str]:
        self.timestep += 1
        sorted_forklifts = sorted(
            self.forklifts,
            key=lambda f: f.effective_priority,
            reverse=True,
        )

        self._resolver.resolve(sorted_forklifts, self.heuristic)

        statuses: Dict[str, str] = {}
        for forklift in sorted_forklifts:
            status = forklift.step(self.heuristic)
            statuses[forklift.name] = status
            logger.debug("[t=%d] %s → %s", self.timestep, forklift.name, status)

        return statuses

    def run(
        self,
        callback: Optional[callable] = None,
        stop_when_all_done: bool = True,
    ) -> int:
        while self.timestep < self.max_steps:
            statuses = self.step()

            if callback:
                callback(self, statuses)

            if stop_when_all_done and all(
                s in ("at_goal", "idle") for s in statuses.values()
            ):
                logger.info("All forklifts reached their goals at t=%d.", self.timestep)
                break
        else:
            logger.warning("Simulation reached max_steps=%d without completion.", self.max_steps)

        return self.timestep

    def assign_goal(self, forklift_name: str, target) -> None:
        forklift = self.grid.forklifts[forklift_name]
        goal_cell = self.grid.resolve_goal(target)
        forklift.goal_cell = goal_cell
        forklift.plan_path(forklift.current_cell, goal_cell, self.heuristic)
        logger.info(
            "%s assigned goal → (%d,%d)",
            forklift_name, goal_cell.pos_x, goal_cell.pos_y,
        )

    def all_done(self) -> bool:
        return all(
            fl.current_cell == fl.goal_cell or fl.goal_cell is None
            for fl in self.forklifts
        )
