from __future__ import annotations
import sys
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    from warehouse.grid import Grid
    from warehouse.cell import Cell
    from simulation.simulation import Simulation

COLOR_BG          = (30,  30,  35)   # dark background
COLOR_GRID_LINE   = (50,  50,  60)
COLOR_AISLE       = (60,  65,  75)
COLOR_SHELF       = (90,  55,  30)   # brown
COLOR_OBSTACLE    = (200, 50,  50)   # red
COLOR_CHARGING    = (40, 120,  80)   # green
COLOR_FORKLIFT    = [                 # distinct colours per forklift
    (255, 200,  50),  # yellow
    ( 80, 160, 255),  # blue
    (255, 120,  80),  # orange
    (160, 255, 160),  # mint
    (220,  80, 220),  # purple
]
COLOR_PATH        = (180, 220, 255, 120)  # translucent light blue
COLOR_GOAL_MARKER = (255, 255, 100)
COLOR_TEXT        = (220, 220, 230)
COLOR_PANEL_BG    = (20,  20,  25)
COLOR_AT_GOAL     = (50, 200, 100)


class WarehouseRenderer:
    def __init__(
        self,
        grid: "Grid",
        cell_size: int = 64,
        fps: int = 8,
        panel_width: int = 280,
    ) -> None:
        self.grid = grid
        self.cell_size = cell_size
        self.fps = fps
        self.panel_width = panel_width

        self._pygame_ready = False
        self._screen = None
        self._clock = None
        self._font_large = None
        self._font_small = None

        self._forklift_colors: Dict[str, Tuple[int, int, int]] = {}

    def init(self) -> None:
        try:
            import pygame
            self._pygame = pygame
        except ImportError:
            raise ImportError(
                "Pygame is required for graphical rendering. "
                "Install it with: pip install pygame"
            )

        self._pygame.init()
        self._pygame.display.set_caption("Warehouse Forklift Simulation")

        screen_w = self.grid.width * self.cell_size + self.panel_width
        screen_h = self.grid.height * self.cell_size
        self._screen = self._pygame.display.set_mode((screen_w, screen_h))
        self._clock = self._pygame.time.Clock()
        self._font_large = self._pygame.font.SysFont("monospace", 15, bold=True)
        self._font_small  = self._pygame.font.SysFont("monospace", 12)
        self._path_surface = self._pygame.Surface(
            (self.grid.width * self.cell_size, self.grid.height * self.cell_size),
            self._pygame.SRCALPHA,
        )
        self._pygame_ready = True

        for i, name in enumerate(self.grid.forklifts):
            self._forklift_colors[name] = COLOR_FORKLIFT[i % len(COLOR_FORKLIFT)]

    def should_quit(self) -> bool:
        pygame = self._pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    return True
        return False

    def quit(self) -> None:
        if self._pygame_ready:
            self._pygame.quit()
            self._pygame_ready = False

    def render(self, sim: "Simulation", statuses: Optional[Dict[str, str]] = None) -> None:
        assert self._pygame_ready, "Call init() before render()."
        pygame = self._pygame

        self._screen.fill(COLOR_BG)
        self._draw_grid()
        self._draw_paths(sim)
        self._draw_entities(sim)
        self._draw_panel(sim, statuses or {})

        pygame.display.flip()
        self._clock.tick(self.fps)

    def _cell_rect(self, x: int, y: int):
        pygame = self._pygame
        cs = self.cell_size
        return pygame.Rect(x * cs, y * cs, cs, cs)

    def _draw_grid(self) -> None:
        pygame = self._pygame
        cs = self.cell_size
        from warehouse.cell import CellType

        for y in range(self.grid.height):
            for x in range(self.grid.width):
                cell = self.grid.cells[y][x]
                rect = self._cell_rect(x, y)

                if cell.cell_type == CellType.SHELF:
                    color = COLOR_SHELF
                elif cell.cell_type == CellType.OBSTACLE:
                    color = COLOR_OBSTACLE
                elif cell.cell_type == CellType.CHARGING_STATION:
                    color = COLOR_CHARGING
                else:
                    color = COLOR_AISLE

                pygame.draw.rect(self._screen, color, rect)
                pygame.draw.rect(self._screen, COLOR_GRID_LINE, rect, 1)

        for y in range(self.grid.height):
            for x in range(self.grid.width):
                cell = self.grid.cells[y][x]
                from warehouse.cell import CellType
                if cell.cell_type == CellType.SHELF:
                    self._blit_center("S", x, y, COLOR_TEXT, self._font_small)
                elif cell.cell_type == CellType.OBSTACLE:
                    self._blit_center("X", x, y, (255, 255, 255), self._font_large)
                elif cell.cell_type == CellType.CHARGING_STATION:
                    self._blit_center("⚡", x, y, (255, 255, 200), self._font_small)

    def _draw_paths(self, sim: "Simulation") -> None:
        pygame = self._pygame
        cs = self.cell_size
        self._path_surface.fill((0, 0, 0, 0))

        for fl in sim.forklifts:
            if not fl.path:
                continue
            color = self._forklift_colors.get(fl.name, COLOR_FORKLIFT[0])
            path_color = (*color, 60)

            for cell in fl.path:
                r = pygame.Rect(cell.pos_x * cs + 2, cell.pos_y * cs + 2, cs - 4, cs - 4)
                pygame.draw.rect(self._path_surface, path_color, r, border_radius=4)

            if fl.goal_cell:
                gc = fl.goal_cell
                pygame.draw.rect(
                    self._path_surface,
                    (*color, 180),
                    pygame.Rect(gc.pos_x * cs + cs // 4, gc.pos_y * cs + cs // 4, cs // 2, cs // 2),
                    border_radius=4,
                )

        self._screen.blit(self._path_surface, (0, 0))

    def _draw_entities(self, sim: "Simulation") -> None:
        pygame = self._pygame
        cs = self.cell_size

        for fl in sim.forklifts:
            color = self._forklift_colors.get(fl.name, COLOR_FORKLIFT[0])
            cx = fl.current_cell.pos_x * cs + cs // 2
            cy = fl.current_cell.pos_y * cs + cs // 2
            radius = cs // 2 - 4

            pygame.draw.circle(self._screen, (0, 0, 0), (cx + 2, cy + 2), radius)
            pygame.draw.circle(self._screen, color, (cx, cy), radius)
            pygame.draw.circle(self._screen, (255, 255, 255), (cx, cy), radius, 2)
            label = self._font_small.render(fl.name[:3], True, (20, 20, 20))
            self._screen.blit(label, label.get_rect(center=(cx, cy)))

    def _draw_panel(
        self,
        sim: "Simulation",
        statuses: Dict[str, str],
    ) -> None:
        pygame = self._pygame
        panel_x = self.grid.width * self.cell_size
        panel_rect = pygame.Rect(
            panel_x, 0, self.panel_width, self.grid.height * self.cell_size
        )
        pygame.draw.rect(self._screen, COLOR_PANEL_BG, panel_rect)

        lines = [
            f"Timestep: {sim.timestep}",
            "─" * 28,
            "Forklifts:",
        ]
        for fl in sim.forklifts:
            status = statuses.get(fl.name, "?")
            at_goal = fl.current_cell == fl.goal_cell
            goal_str = (
                f"({fl.goal_cell.pos_x},{fl.goal_cell.pos_y})" if fl.goal_cell else "none"
            )
            lines.append(
                f" {fl.name[:8]:<8} {status[:8]:<8} → {goal_str}"
            )
            lines.append(
                f"   pri={fl.effective_priority:.2f} "
                f"wait={fl.wait_steps}"
            )
        lines += [
            "─" * 28,
            "Legend:",
            " S  Shelf   ⚡ Charge",
            " X  Obstacle",
            " Circles = Forklifts",
            "─" * 28,
            "ESC / Q to quit",
        ]

        y_offset = 10
        for line in lines:
            surf = self._font_small.render(line, True, COLOR_TEXT)
            self._screen.blit(surf, (panel_x + 8, y_offset))
            y_offset += 18

    def _blit_center(
        self,
        text: str,
        cx: int,
        cy: int,
        color: Tuple,
        font,
    ) -> None:
        cs = self.cell_size
        surf = font.render(text, True, color)
        rect = surf.get_rect(
            center=(cx * cs + cs // 2, cy * cs + cs // 2)
        )
        self._screen.blit(surf, rect)
