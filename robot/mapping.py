"""Grid map loading, NF1 wavefront planning, and visualisation.

The map format comes from the course framework: a header ``sizeX sizeY
sizeCell`` followed by ``2 * sizeY + 1`` rows of ``2 * sizeX + 1`` connection
values. Odd indices are cell centres; even indices encode walls between cells.
A value of 1 means the connection is open, 0 means it is blocked.

The planner is a classic NF1 wavefront:

1. Run BFS from the goal over the 4-connected free cells.
2. Store the number of steps to the goal in a cost matrix.
3. Follow the cost gradient from the start cell to the goal.

Because the cost field is built from the goal, the gradient descent cannot get
trapped in a local minimum on a static grid. Replanning is implemented by
closing a connection and rebuilding the field.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Optional

import numpy as np

Cell = tuple[int, int]

# (dx, dy, neighbour index) matching the course map convention:
# 0 north, 2 east, 4 south, 6 west.
NEIGHBOURS: tuple[tuple[int, int, int], ...] = (
    (0, 1, 0),
    (1, 0, 2),
    (0, -1, 4),
    (-1, 0, 6),
)

_DIRECTION_TO_NEIGHBOUR = {
    (0, 1): 0,
    (1, 0): 2,
    (0, -1): 4,
    (-1, 0): 6,
}

UNREACHABLE = -1


class MapFormatError(ValueError):
    """Raised when a map file does not follow the expected format."""


class GridMap:
    """Occupancy/connection grid with NF1 planning."""

    def __init__(self, map_path: str | Path, verbose: bool = False) -> None:
        self.path = Path(map_path)
        self.verbose = verbose

        self.size_x = 0
        self.size_y = 0
        self.cell_size_mm = 0.0
        self.connections = np.zeros((0, 0), dtype=np.uint8)
        self.cost_matrix = np.full((0, 0), UNREACHABLE, dtype=int)
        self.current_path: Optional[list[Cell]] = None
        self.goal: Optional[Cell] = None

        self._load(self.path)

    # ------------------------------------------------------------------
    # Loading and conversion
    # ------------------------------------------------------------------
    @property
    def cell_size_cm(self) -> float:
        """Cell side in cm (map files store millimetres)."""

        return self.cell_size_mm / 10.0

    @property
    def shape(self) -> tuple[int, int]:
        return self.size_x, self.size_y

    def _load(self, path: Path) -> None:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            raise MapFormatError(f"cannot read map file {path}: {exc}") from exc

        rows = [line for line in lines if line.strip()]
        if len(rows) < 2:
            raise MapFormatError(f"map file {path} is empty")

        try:
            header = [int(value) for value in rows[0].split()]
        except ValueError as exc:
            raise MapFormatError(f"invalid map header in {path}: {rows[0]!r}") from exc

        if len(header) != 3:
            raise MapFormatError(
                f"map header must contain sizeX sizeY sizeCell, got {rows[0]!r}"
            )

        self.size_x, self.size_y, cell_size = header
        if min(self.size_x, self.size_y, cell_size) <= 0:
            raise MapFormatError("map dimensions and cell size must be positive")
        self.cell_size_mm = float(cell_size)

        expected_cols = 2 * self.size_x + 1
        expected_rows = 2 * self.size_y + 1
        data_rows = rows[1:]
        if len(data_rows) != expected_rows:
            raise MapFormatError(
                f"map {path} has {len(data_rows)} data rows, expected {expected_rows}"
            )

        connections = np.zeros((expected_cols, expected_rows), dtype=np.uint8)
        for row_index, line in enumerate(data_rows):
            try:
                values = [int(value) for value in line.split()]
            except ValueError as exc:
                raise MapFormatError(
                    f"invalid connection row {row_index + 1} in {path}: {line!r}"
                ) from exc
            if len(values) != expected_cols:
                raise MapFormatError(
                    f"row {row_index + 1} in {path} has {len(values)} values, "
                    f"expected {expected_cols}"
                )
            # The file is written top-to-bottom; matrix y increases upwards.
            column = expected_rows - 1 - row_index
            connections[:, column] = values

        self.connections = connections
        self.cost_matrix = np.full((self.size_x, self.size_y), UNREACHABLE, dtype=int)

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------
    def _connection_coordinates(self, cell: Cell, neighbour: int) -> tuple[int, int]:
        x, y = cell
        if not (0 <= x < self.size_x and 0 <= y < self.size_y):
            raise IndexError(f"cell {cell} is outside the map")
        if neighbour not in range(8):
            raise ValueError(f"neighbour index must be in 0..7, got {neighbour}")

        offsets = {
            0: (0, 1),
            1: (1, 1),
            2: (1, 0),
            3: (1, -1),
            4: (0, -1),
            5: (-1, -1),
            6: (-1, 0),
            7: (-1, 1),
        }
        dx, dy = offsets[neighbour]
        return 2 * x + 1 + dx, 2 * y + 1 + dy

    def is_connected(self, cell: Cell, neighbour: int) -> bool:
        conn_x, conn_y = self._connection_coordinates(cell, neighbour)
        return bool(self.connections[conn_x, conn_y])

    def set_connection(self, cell: Cell, neighbour: int, connected: bool) -> None:
        """Open or close one connection (edge) of the map."""

        conn_x, conn_y = self._connection_coordinates(cell, neighbour)
        self.connections[conn_x, conn_y] = 1 if connected else 0

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.size_x and 0 <= cell[1] < self.size_y

    def cell_center_cm(self, cell: Cell) -> tuple[float, float]:
        """Centre of a cell in arena coordinates (cm)."""

        x, y = cell
        size = self.cell_size_cm
        return (x + 0.5) * size, (y + 0.5) * size

    def position_to_cell(self, x_cm: float, y_cm: float) -> Cell:
        size = self.cell_size_cm
        return int(math.floor(x_cm / size)), int(math.floor(y_cm / size))

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------
    def fill_cost_matrix(self, goal: Cell) -> np.ndarray:
        """BFS wavefront from ``goal``; stores and returns the cost matrix."""

        if not self.in_bounds(goal):
            raise ValueError(f"goal {goal} is outside the map")

        costs = np.full((self.size_x, self.size_y), UNREACHABLE, dtype=int)
        costs[goal] = 0
        queue: deque[tuple[int, int, int]] = deque([(goal[0], goal[1], 0)])

        while queue:
            x, y, cost = queue.popleft()
            for dx, dy, neighbour in NEIGHBOURS:
                next_cell = (x + dx, y + dy)
                if not self.in_bounds(next_cell):
                    continue
                if not self.is_connected((x, y), neighbour):
                    continue
                if costs[next_cell] != UNREACHABLE:
                    continue
                costs[next_cell] = cost + 1
                queue.append((next_cell[0], next_cell[1], cost + 1))

        self.cost_matrix = costs
        self.goal = goal
        return costs

    def plan(self, start: Cell, goal: Cell) -> Optional[list[Cell]]:
        """Plan a shortest path using the NF1 cost field.

        Returns a list of cells including ``start`` and ``goal``, or ``None``
        when no path exists.
        """

        if not self.in_bounds(start):
            raise ValueError(f"start {start} is outside the map")
        if not self.in_bounds(goal):
            raise ValueError(f"goal {goal} is outside the map")

        costs = self.fill_cost_matrix(goal)
        if costs[start] == UNREACHABLE:
            self.current_path = None
            return None

        path: list[Cell] = [start]
        current = start
        max_steps = self.size_x * self.size_y

        for _ in range(max_steps):
            if current == goal:
                self.current_path = path
                return path

            current_cost = costs[current]
            best: Optional[Cell] = None
            best_cost = current_cost

            for dx, dy, neighbour in NEIGHBOURS:
                candidate = (current[0] + dx, current[1] + dy)
                if not self.in_bounds(candidate):
                    continue
                if not self.is_connected(current, neighbour):
                    continue
                candidate_cost = costs[candidate]
                if candidate_cost == UNREACHABLE:
                    continue
                if candidate_cost < best_cost:
                    best_cost = candidate_cost
                    best = candidate

            if best is None:
                # This should only happen with a malformed connection matrix.
                self.current_path = None
                return None
            current = best
            path.append(current)

        self.current_path = None
        return None

    # Alias kept for parity with the original course framework.
    find_path = plan

    def replan(
        self,
        current_cell: Cell,
        goal_cell: Cell,
        blocked_direction: Sequence[int],
    ) -> Optional[list[Cell]]:
        """Close the edge in ``blocked_direction`` and plan again.

        ``blocked_direction`` is the cell-space vector from the current cell to
        the blocked neighbour, e.g. ``(0, 1)`` for north.
        """

        direction = (int(blocked_direction[0]), int(blocked_direction[1]))
        neighbour = _DIRECTION_TO_NEIGHBOUR.get(direction)
        if neighbour is None:
            raise ValueError(
                f"blocked direction {direction} is not a 4-connected step; "
                "expected one of (0,1), (1,0), (0,-1), (-1,0)"
            )

        self.set_connection(current_cell, neighbour, connected=False)
        return self.plan(current_cell, goal_cell)

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------
    def draw(
        self,
        path: Optional[Iterable[Cell]] = None,
        robot_pose: Optional[Iterable[float]] = None,
        save_path: str | Path | None = None,
        show: bool = False,
        title: str | None = None,
    ):
        """Render walls, NF1 costs, path, and optional robot pose.

        Matplotlib is imported lazily so the planner can run headless without
        pulling a GUI backend into import time.
        """

        import matplotlib.pyplot as plt
        from matplotlib.patches import FancyArrowPatch

        size = self.cell_size_cm
        fig, ax = plt.subplots(figsize=(7.5, 7.5))

        # Cost heatmap (only reachable cells).
        masked = np.ma.masked_where(self.cost_matrix < 0, self.cost_matrix)
        if masked.count():
            image = ax.imshow(
                masked.T,
                origin="lower",
                extent=(0, self.size_x * size, 0, self.size_y * size),
                cmap="YlGnBu_r",
                alpha=0.55,
            )
            fig.colorbar(image, ax=ax, shrink=0.8, label="NF1 cost to goal")

        # Grid and walls.
        for x in range(self.size_x + 1):
            ax.plot([x * size, x * size], [0, self.size_y * size], color="0.85", lw=0.6)
        for y in range(self.size_y + 1):
            ax.plot([0, self.size_x * size], [y * size, y * size], color="0.85", lw=0.6)

        # Every internal edge is either the north edge of a cell or the east
        # edge of a cell, so checking those two directions covers the map.
        for cell_x in range(self.size_x):
            for cell_y in range(self.size_y):
                x0, y0 = cell_x * size, cell_y * size
                if cell_y < self.size_y - 1 and not self.is_connected((cell_x, cell_y), 0):
                    ax.plot([x0, x0 + size], [y0 + size, y0 + size], color="black", lw=2)
                if cell_x < self.size_x - 1 and not self.is_connected((cell_x, cell_y), 2):
                    ax.plot([x0 + size, x0 + size], [y0, y0 + size], color="black", lw=2)

        # Arena boundary.
        ax.plot(
            [0, self.size_x * size, self.size_x * size, 0, 0],
            [0, 0, self.size_y * size, self.size_y * size, 0],
            color="black",
            lw=2.5,
        )

        # Cost labels.
        if masked.count():
            for cell_x in range(self.size_x):
                for cell_y in range(self.size_y):
                    cost = self.cost_matrix[cell_x, cell_y]
                    if cost >= 0:
                        ax.text(
                            (cell_x + 0.5) * size,
                            (cell_y + 0.5) * size,
                            str(cost),
                            ha="center",
                            va="center",
                            fontsize=7,
                            color="0.25",
                        )

        if path is not None:
            pts = [self.cell_center_cm(cell) for cell in path]
            if pts:
                xs, ys = zip(*pts)
                ax.plot(xs, ys, "-o", color="#d62728", lw=2.2, ms=5, label="Planned path")
                ax.scatter(*pts[0], s=120, marker="s", color="#2ca02c", zorder=5, label="Start")
                ax.scatter(*pts[-1], s=140, marker="*", color="#ff7f0e", zorder=5, label="Goal")

        if robot_pose is not None:
            rx, ry, rtheta = robot_pose
            arrow = FancyArrowPatch(
                (rx, ry),
                (rx + 18 * math.cos(rtheta), ry + 18 * math.sin(rtheta)),
                arrowstyle="-|>",
                mutation_scale=18,
                color="#1f77b4",
                lw=2,
                label="Robot",
            )
            ax.add_patch(arrow)

        ax.set_xlim(0, self.size_x * size)
        ax.set_ylim(0, self.size_y * size)
        ax.set_aspect("equal")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_title(title or f"NF1 plan — {self.path.name}")
        ax.legend(
            loc="upper center",
            bbox_to_anchor=(0.5, -0.10),
            ncol=4,
            fontsize=8,
            frameon=False,
        )

        if save_path is not None:
            save_path = Path(save_path)
            save_path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(save_path, dpi=160, bbox_inches="tight")
        if show:
            plt.show()
        return fig
