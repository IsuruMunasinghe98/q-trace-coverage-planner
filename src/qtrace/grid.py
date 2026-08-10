"""Grid representation, motion constraints, and dataset loading."""

from __future__ import annotations

import ast
from dataclasses import dataclass
import heapq
from pathlib import Path
from typing import Iterator

import numpy as np

from .geometry import Cell, move_length

FREE = 1
OBSTACLE = 0

ORTHOGONAL_DIRECTIONS: tuple[Cell, ...] = (
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1),
)
DIAGONAL_DIRECTIONS: tuple[Cell, ...] = (
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1),
)

DATASET_CATEGORY_ORDER: tuple[str, ...] = (
    "low_obstacle",
    "cluttered",
    "narrow_passage",
    "clustered_obstacle",
)


@dataclass(frozen=True)
class GridMap:
    """One binary occupancy grid with start and destination cells."""

    cells: np.ndarray
    start: Cell
    destination: Cell

    def __post_init__(self) -> None:
        array = np.asarray(self.cells, dtype=np.int8)
        if array.ndim != 2 or array.size == 0:
            raise ValueError("The occupancy grid must be a non-empty 2D array.")
        if not np.isin(array, (OBSTACLE, FREE)).all():
            raise ValueError("The occupancy grid must contain only 0 and 1 values.")
        object.__setattr__(self, "cells", array)
        for name, cell in (("start", self.start), ("destination", self.destination)):
            if not self.in_bounds(cell):
                raise ValueError(f"{name} cell {cell} is outside the grid.")
            if not self.is_free(cell):
                raise ValueError(f"{name} cell {cell} is not traversable.")

    @property
    def shape(self) -> tuple[int, int]:
        return int(self.cells.shape[0]), int(self.cells.shape[1])

    def in_bounds(self, cell: Cell) -> bool:
        rows, columns = self.shape
        return 0 <= cell[0] < rows and 0 <= cell[1] < columns

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and int(self.cells[cell]) == FREE

    def valid_move(self, current: Cell, candidate: Cell, allow_diagonal: bool) -> bool:
        if not self.is_free(candidate):
            return False

        row_delta = candidate[0] - current[0]
        column_delta = candidate[1] - current[1]
        if max(abs(row_delta), abs(column_delta)) != 1:
            return False

        diagonal = abs(row_delta) == 1 and abs(column_delta) == 1
        if diagonal and not allow_diagonal:
            return False

        if diagonal:
            side_a = current[0] + row_delta, current[1]
            side_b = current[0], current[1] + column_delta
            if not self.is_free(side_a) and not self.is_free(side_b):
                return False

        return True

    def neighbors(self, cell: Cell, allow_diagonal: bool) -> Iterator[Cell]:
        directions = ORTHOGONAL_DIRECTIONS
        if allow_diagonal:
            directions += DIAGONAL_DIRECTIONS
        for row_delta, column_delta in directions:
            candidate = cell[0] + row_delta, cell[1] + column_delta
            if self.valid_move(cell, candidate, allow_diagonal):
                yield candidate

    def distance_map(self, origin: Cell, allow_diagonal: bool) -> dict[Cell, float]:
        """Compute weighted wavefront distances from ``origin``."""

        queue: list[tuple[float, Cell]] = [(0.0, origin)]
        distances: dict[Cell, float] = {origin: 0.0}
        while queue:
            distance, current = heapq.heappop(queue)
            if distance > distances.get(current, float("inf")):
                continue
            for candidate in self.neighbors(current, allow_diagonal):
                updated = distance + move_length(current, candidate)
                if updated < distances.get(candidate, float("inf")):
                    distances[candidate] = updated
                    heapq.heappush(queue, (updated, candidate))
        return distances

    def reachable(self, origin: Cell, allow_diagonal: bool) -> set[Cell]:
        return set(self.distance_map(origin, allow_diagonal))


@dataclass(frozen=True)
class MapRecord:
    map_id: int
    category: str
    grid_size: int
    grid_map: GridMap


def octile_distance(start: Cell, goal: Cell, allow_diagonal: bool) -> float:
    row_delta = abs(start[0] - goal[0])
    column_delta = abs(start[1] - goal[1])
    if not allow_diagonal:
        return float(row_delta + column_delta)
    diagonal_steps = min(row_delta, column_delta)
    return float(max(row_delta, column_delta) + (2.0**0.5 - 1.0) * diagonal_steps)


def _normalise_category(value: str) -> str:
    category = value.strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "irregular": "clustered_obstacle",
        "irregular_obstacle": "clustered_obstacle",
        "clustered": "clustered_obstacle",
        "low_obstacles": "low_obstacle",
        "narrow_passages": "narrow_passage",
    }
    return aliases.get(category, category)


def load_dataset(path: str | Path) -> list[MapRecord]:
    """Load line-oriented map records from the supplied research datasets."""

    raw_records: list[list[object]] = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                raw_records.append(ast.literal_eval(line))
            except (SyntaxError, ValueError) as exc:
                raise ValueError(f"Invalid map record at line {line_number}.") from exc

    if not raw_records:
        raise ValueError(f"No map records were found in {path}.")

    sizes = sorted(
        {
            len(record[1] if len(record) == 4 and isinstance(record[0], str) else record[0])
            for record in raw_records
        }
    )
    denominator = len(DATASET_CATEGORY_ORDER) * len(sizes)
    maps_per_category_size = max(1, len(raw_records) // denominator)
    category_block_size = len(sizes) * maps_per_category_size

    records: list[MapRecord] = []
    for index, record in enumerate(raw_records):
        if len(record) == 4 and isinstance(record[0], str):
            category, grid_data, start, destination = record
            category = _normalise_category(category)
        elif len(record) == 3:
            grid_data, start, destination = record
            category_index = min(
                index // category_block_size,
                len(DATASET_CATEGORY_ORDER) - 1,
            )
            category = DATASET_CATEGORY_ORDER[category_index]
        else:
            raise ValueError(f"Unsupported map record at line {index + 1}.")

        grid_map = GridMap(
            cells=np.asarray(grid_data, dtype=np.int8),
            start=tuple(start),
            destination=tuple(destination),
        )
        records.append(
            MapRecord(
                map_id=index + 1,
                category=str(category),
                grid_size=grid_map.shape[0],
                grid_map=grid_map,
            )
        )
    return records
