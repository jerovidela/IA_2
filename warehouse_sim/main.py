from __future__ import annotations
import argparse
import logging
import sys
import os
import time

sys.path.insert(0, os.path.dirname(__file__))

from warehouse.grid import Grid
from simulation.simulation import Simulation
from pathfinding.heuristics import manhattan_distance

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


def build_warehouse() -> Grid:

    grid = Grid(width=13, height=11)

    grid.create_charging_station("CS_West", x=0,  y=5)
    grid.create_charging_station("CS_East", x=12, y=5)
    block_positions = [
        (2, 1), (6, 1), (10, 1),
        (2, 6), (6, 6), (10, 6)
    ]
    shelf_id = 1
    for block_x, block_y in block_positions:
        for dy in range(4):
            for dx in range(2):
                grid.create_shelf(
                    f"SH{shelf_id:02d}",
                    x=block_x + dx,
                    y=block_y + dy
                )
                shelf_id += 1
    return grid


def setup_simulation(grid: Grid) -> Simulation:
    sim = Simulation(grid=grid, heuristic=manhattan_distance, max_steps=50)

    fl_alpha = grid.create_forklift("Alpha")
    fl_beta  = grid.create_forklift("Beta")
    fl_teta = grid.create_forklift("Teta")
    fl_gamma = grid.create_forklift("Gamma", x=5, y=10)

    log.info("Alpha spawned at (%d,%d)", fl_alpha.current_cell.pos_x, fl_alpha.current_cell.pos_y)
    log.info("Beta  spawned at (%d,%d)", fl_beta.current_cell.pos_x,  fl_beta.current_cell.pos_y)
    log.info("Teta  spawned at (%d,%d)", fl_teta.current_cell.pos_x,  fl_teta.current_cell.pos_y)
    log.info("Gamma spawned at (%d,%d)", fl_gamma.current_cell.pos_x, fl_gamma.current_cell.pos_y)

    sim.assign_goal("Alpha", "SH12")
    sim.assign_goal("Beta",  "SH03")
    sim.assign_goal("Teta",  "SH26")
    sim.assign_goal("Gamma", "SH09")

    log.info("Alpha goal → shelf SH12 → cell (%d,%d)",
             grid.resolve_goal("SH12").pos_x, grid.resolve_goal("SH12").pos_y)
    log.info("Beta  goal → shelf SH03 → cell (%d,%d)",
             grid.resolve_goal("SH03").pos_x, grid.resolve_goal("SH03").pos_y)
    log.info("Teta  goal → shelf SH26 → cell (%d,%d)",
             grid.resolve_goal("SH26").pos_x, grid.resolve_goal("SH26").pos_y)
    log.info("Gamma goal → shelf SH09 → cell (%d,%d)",
             grid.resolve_goal("SH09").pos_x, grid.resolve_goal("SH09").pos_y)

    return sim


def run_graphical(sim: Simulation) -> None:
    try:
        from rendering.renderer import WarehouseRenderer
    except ImportError:
        log.error("Could not import renderer — falling back to headless mode.")
        run_headless(sim)
        return

    renderer = WarehouseRenderer(sim.grid, cell_size=62, fps=3)
    renderer.init()

    obstacle_inserted = False
    obstacle_removed  = False

    log.info("Graphical simulation started. Press ESC or Q to quit.")

    while sim.timestep < sim.max_steps:
        if renderer.should_quit():
            log.info("User requested quit.")
            break

        if sim.timestep == 1 and not obstacle_inserted:
            log.info("[t=%d] *** Inserting dynamic obstacle OBS1 at (5,3) ***", sim.timestep)
            sim.grid.create_obstacle("OBS1", x=5, y=5)
            obstacle_inserted = True

        if sim.timestep == 301 and not obstacle_removed and obstacle_inserted:
            log.info("[t=%d] *** Removing obstacle OBS1 ***", sim.timestep)
            sim.grid.remove_obstacle("OBS1")
            obstacle_removed = True
            for fl in sim.forklifts:
                if fl.goal_cell and fl.current_cell != fl.goal_cell:
                    fl.path = []
                    fl.penalized_cells.clear()

        statuses = sim.step()
        renderer.render(sim, statuses)

        _print_tick(sim, statuses)

        if sim.all_done():
            log.info("All forklifts reached their goals at t=%d!", sim.timestep)
            time.sleep(2)
            break

    renderer.quit()
    log.info("Simulation ended at timestep %d.", sim.timestep)


def run_headless(sim: Simulation) -> None:
    obstacle_inserted = False
    obstacle_removed  = False

    log.info("Headless simulation started.")

    while sim.timestep < sim.max_steps:
        if sim.timestep == 1 and not obstacle_inserted:
            log.info("[t=%d] Inserting dynamic obstacle OBS1 at (5,3)", sim.timestep)
            sim.grid.create_obstacle("OBS1", x=5, y=3)
            obstacle_inserted = True

        if sim.timestep == 301 and not obstacle_removed and obstacle_inserted:
            log.info("[t=%d] Removing obstacle OBS1", sim.timestep)
            sim.grid.remove_obstacle("OBS1")
            obstacle_removed = True
            for fl in sim.forklifts:
                if fl.goal_cell and fl.current_cell != fl.goal_cell:
                    fl.path = []
                    fl.penalized_cells.clear()

        statuses = sim.step()
        _print_tick(sim, statuses)

        if sim.all_done():
            log.info("All forklifts reached their goals at t=%d!", sim.timestep)
            break

    print("\nFinal warehouse state:")
    print(sim.grid.ascii_render())
    log.info("Simulation ended at timestep %d.", sim.timestep)


def _print_tick(sim: Simulation, statuses: dict) -> None:
    parts = []
    for fl in sim.forklifts:
        pos = f"({fl.current_cell.pos_x},{fl.current_cell.pos_y})"
        parts.append(f"{fl.name}@{pos}={statuses.get(fl.name,'?')}")
    log.info("[t=%03d] %s", sim.timestep, "  |  ".join(parts))



def main() -> None:
    parser = argparse.ArgumentParser(description="Autonomous Forklift Warehouse Simulation")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without graphical display (ASCII terminal output only).",
    )
    args = parser.parse_args()

    grid = build_warehouse()
    sim  = setup_simulation(grid)

    log.info("Grid: %s", grid)
    log.info("Forklifts: %s", list(grid.forklifts.keys()))
    log.info("Shelves:   %s", list(grid.shelves.keys()))

    if args.headless:
        run_headless(sim)
    else:
        try:
            import pygame
            run_graphical(sim)
        except ImportError:
            log.warning("Pygame not found — switching to headless mode.")
            run_headless(sim)


if __name__ == "__main__":
    main()
