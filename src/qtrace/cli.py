"""Command-line interface for planning, evaluation, and optimization."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Sequence

from .config import PlannerConfig
from .evaluation import evaluate_records, summarise, write_csv
from .grid import DATASET_CATEGORY_ORDER, load_dataset
from .optimization import optimize
from .planner import QTracePlanner
from .visualization import plot_plan


def _load_config(path: str) -> PlannerConfig:
    return PlannerConfig.from_toml(path)


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _plan(args: argparse.Namespace) -> int:
    records = load_dataset(args.dataset)
    matches = [record for record in records if record.map_id == args.map_id]
    if not matches:
        raise ValueError(f"Map ID {args.map_id} is not present in {args.dataset}.")

    record = matches[0]
    result = QTracePlanner(_load_config(args.config)).plan(record.grid_map)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    with (output / "path.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["step", "row", "column"])
        writer.writerows((index, cell[0], cell[1]) for index, cell in enumerate(result.path))
    _write_json(
        output / "metrics.json",
        {
            "map_id": record.map_id,
            "category": record.category,
            "grid_size": record.grid_size,
            "motion_model": result.motion_model,
            **result.metrics.as_dict(),
        },
    )
    plot_plan(record.grid_map, result, output / "coverage_path.png")
    print(json.dumps(result.metrics.as_dict(), indent=2))
    print(f"Results written to {output.resolve()}")
    return 0


def _evaluate(args: argparse.Namespace) -> int:
    records = load_dataset(args.dataset)
    if args.category:
        records = [record for record in records if record.category == args.category]
    if args.limit:
        records = records[: args.limit]
    if not records:
        raise ValueError("No maps match the requested evaluation selection.")

    rows = evaluate_records(records, QTracePlanner(_load_config(args.config)))
    summaries = summarise(rows)
    output = Path(args.output)
    write_csv(output / "per_map.csv", rows)
    write_csv(output / "summary.csv", summaries)
    _write_json(output / "summary.json", summaries)
    print(json.dumps(summaries, indent=2))
    print(f"Results written to {output.resolve()}")
    return 0


def _optimize(args: argparse.Namespace) -> int:
    records = load_dataset(args.dataset)
    if args.scope != "global":
        records = [record for record in records if record.category == args.scope]
    if args.limit:
        records = records[: args.limit]
    result = optimize(
        records=records,
        base_config=_load_config(args.config),
        trials=args.trials,
        seed=args.seed,
    )
    destination = Path(args.output)
    _write_json(destination, result)
    print(json.dumps(result, indent=2))
    print(f"Optimization summary written to {destination.resolve()}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="qtrace",
        description="Q-TRACE turn- and revisit-aware coverage path planning",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    plan = subparsers.add_parser("plan", help="Plan one map and create a visualization.")
    plan.add_argument("--dataset", default="data/evaluation_set.txt")
    plan.add_argument("--map-id", type=int, default=1)
    plan.add_argument("--config", default="configs/global.toml")
    plan.add_argument("--output", default="results/example")
    plan.set_defaults(handler=_plan)

    evaluation = subparsers.add_parser("evaluate", help="Evaluate a dataset selection.")
    evaluation.add_argument("--dataset", default="data/evaluation_set.txt")
    evaluation.add_argument("--config", default="configs/global.toml")
    evaluation.add_argument("--category", choices=DATASET_CATEGORY_ORDER)
    evaluation.add_argument("--limit", type=int)
    evaluation.add_argument("--output", default="results/evaluation")
    evaluation.set_defaults(handler=_evaluate)

    optimization = subparsers.add_parser("optimize", help="Run Bayesian parameter optimization.")
    optimization.add_argument("--dataset", default="data/optimization_set.txt")
    optimization.add_argument("--config", default="configs/global.toml")
    optimization.add_argument(
        "--scope",
        choices=("global",) + DATASET_CATEGORY_ORDER,
        default="global",
    )
    optimization.add_argument("--trials", type=int, default=100)
    optimization.add_argument("--seed", type=int, default=42)
    optimization.add_argument("--limit", type=int)
    optimization.add_argument("--output", default="results/optimization.json")
    optimization.set_defaults(handler=_optimize)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.handler(args))
    except (RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    return 2
