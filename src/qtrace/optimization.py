"""Bayesian optimization of the seven Q-TRACE weighting parameters."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

from .config import LocalWeights, PlannerConfig, ReconnectionWeights
from .grid import MapRecord
from .planner import QTracePlanner

PARAMETER_BOUNDS: dict[str, tuple[float, float]] = {
    "wavefront": (0.1, 10.0),
    "local_turn": (0.1, 10.0),
    "uncovered_degree": (0.1, 10.0),
    "path_length": (0.1, 10.0),
    "revisit": (0.0, 10.0),
    "moderate_turn": (0.0, 10.0),
    "reverse_turn": (0.0, 10.0),
}


def objective_score(metrics: Any) -> float:
    uncovered = metrics.reachable_free_cells - metrics.covered_reachable_cells
    coverage_penalty = uncovered * max(metrics.path_length, metrics.reachable_free_cells)
    return float(
        coverage_penalty
        + metrics.path_length
        + 2.0 * metrics.revisit_visits
        + 0.7 * metrics.moderate_turns
        + 2.0 * metrics.reverse_turns
    )


def _config_from_trial(trial: Any, base: PlannerConfig) -> PlannerConfig:
    values = {
        name: trial.suggest_float(name, lower, upper)
        for name, (lower, upper) in PARAMETER_BOUNDS.items()
    }
    return PlannerConfig(
        local=LocalWeights(
            wavefront=values["wavefront"],
            turn=values["local_turn"],
            uncovered_degree=values["uncovered_degree"],
        ),
        reconnection=ReconnectionWeights(
            path_length=values["path_length"],
            revisit=values["revisit"],
            moderate_turn=values["moderate_turn"],
            reverse_turn=values["reverse_turn"],
        ),
        motion_mode=base.motion_mode,
        max_reconnect_targets=base.max_reconnect_targets,
        moderate_local_turn_cost=base.moderate_local_turn_cost,
        reverse_local_turn_cost=base.reverse_local_turn_cost,
    )


def optimize(
    records: list[MapRecord],
    base_config: PlannerConfig,
    trials: int = 100,
    seed: int = 42,
) -> dict[str, Any]:
    """Run a deterministic-seed TPE study and return a serializable summary."""

    try:
        import optuna
    except ImportError as exc:
        raise RuntimeError(
            "Bayesian optimization requires the optional dependency: "
            "pip install -e '.[optimize]'"
        ) from exc

    if not records:
        raise ValueError("At least one optimization map is required.")
    if trials < 1:
        raise ValueError("trials must be positive.")

    def objective(trial: Any) -> float:
        planner = QTracePlanner(_config_from_trial(trial, base_config))
        scores: list[float] = []
        for step, record in enumerate(records):
            result = planner.plan(record.grid_map)
            scores.append(objective_score(result.metrics))
            trial.report(sum(scores) / len(scores), step=step)
            if trial.should_prune():
                raise optuna.TrialPruned()
        return sum(scores) / len(scores)

    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.TPESampler(seed=seed),
        pruner=optuna.pruners.MedianPruner(n_startup_trials=10),
        study_name="qtrace_parameter_optimization",
    )
    study.optimize(objective, n_trials=trials, show_progress_bar=True)

    return {
        "study": study.study_name,
        "sampler": "Optuna TPESampler",
        "seed": seed,
        "trials_requested": trials,
        "maps": len(records),
        "best_trial": study.best_trial.number,
        "best_score": study.best_value,
        "best_parameters": study.best_params,
        "parameter_bounds": PARAMETER_BOUNDS,
        "base_configuration": asdict(base_config),
    }
