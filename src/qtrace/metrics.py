"""Coverage-path metrics used by Q-TRACE and its optimization objective."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass

from .geometry import Cell, direction, path_length, turn_class


@dataclass(frozen=True)
class PlanMetrics:
    reachable_free_cells: int
    covered_reachable_cells: int
    coverage_ratio: float
    path_length: float
    movement_steps: int
    total_turns: int
    moderate_turns: int
    reverse_turns: int
    distinct_revisit_cells: int
    revisit_visits: int

    def as_dict(self) -> dict[str, int | float]:
        return asdict(self)


def compute_metrics(path: list[Cell], reachable: set[Cell]) -> PlanMetrics:
    directions = [direction(path[index - 1], path[index]) for index in range(1, len(path))]
    categories = [
        turn_class(directions[index - 1], directions[index])
        for index in range(1, len(directions))
    ]
    counts = Counter(path)
    covered_reachable = len(set(path) & reachable)
    reachable_count = len(reachable)

    return PlanMetrics(
        reachable_free_cells=reachable_count,
        covered_reachable_cells=covered_reachable,
        coverage_ratio=covered_reachable / reachable_count if reachable_count else 0.0,
        path_length=path_length(path),
        movement_steps=max(0, len(path) - 1),
        total_turns=sum(category not in {"start", "straight"} for category in categories),
        moderate_turns=categories.count("moderate"),
        reverse_turns=categories.count("reverse"),
        distinct_revisit_cells=sum(count > 1 for count in counts.values()),
        revisit_visits=sum(max(0, count - 1) for count in counts.values()),
    )
