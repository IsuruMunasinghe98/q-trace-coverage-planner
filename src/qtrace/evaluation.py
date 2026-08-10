"""Dataset-level evaluation helpers."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean
from time import perf_counter

from .grid import MapRecord
from .planner import QTracePlanner


@dataclass(frozen=True)
class EvaluationRow:
    map_id: int
    category: str
    grid_size: int
    motion_model: str
    runtime_seconds: float
    coverage_ratio: float
    path_length: float
    movement_steps: int
    moderate_turns: int
    reverse_turns: int
    revisit_visits: int

    def as_dict(self) -> dict[str, int | float | str]:
        return self.__dict__.copy()


def evaluate_records(
    records: list[MapRecord],
    planner: QTracePlanner,
) -> list[EvaluationRow]:
    rows: list[EvaluationRow] = []
    for record in records:
        started = perf_counter()
        result = planner.plan(record.grid_map)
        runtime = perf_counter() - started
        metrics = result.metrics
        rows.append(
            EvaluationRow(
                map_id=record.map_id,
                category=record.category,
                grid_size=record.grid_size,
                motion_model=result.motion_model,
                runtime_seconds=runtime,
                coverage_ratio=metrics.coverage_ratio,
                path_length=metrics.path_length,
                movement_steps=metrics.movement_steps,
                moderate_turns=metrics.moderate_turns,
                reverse_turns=metrics.reverse_turns,
                revisit_visits=metrics.revisit_visits,
            )
        )
    return rows


def summarise(rows: list[EvaluationRow]) -> list[dict[str, int | float | str]]:
    summaries: list[dict[str, int | float | str]] = []
    categories = sorted({row.category for row in rows})
    for category in categories + ["all"]:
        selected = rows if category == "all" else [row for row in rows if row.category == category]
        summaries.append(
            {
                "category": category,
                "maps": len(selected),
                "mean_coverage_ratio": fmean(row.coverage_ratio for row in selected),
                "mean_path_length": fmean(row.path_length for row in selected),
                "mean_revisit_visits": fmean(row.revisit_visits for row in selected),
                "mean_moderate_turns": fmean(row.moderate_turns for row in selected),
                "mean_reverse_turns": fmean(row.reverse_turns for row in selected),
                "mean_runtime_seconds": fmean(row.runtime_seconds for row in selected),
            }
        )
    return summaries


def write_csv(path: str | Path, rows: list[object]) -> None:
    payload = [row.as_dict() if hasattr(row, "as_dict") else dict(row) for row in rows]
    if not payload:
        raise ValueError("Cannot write an empty evaluation table.")
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(payload[0]))
        writer.writeheader()
        writer.writerows(payload)
