"""Configuration models for the Q-TRACE planner."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import tomllib


@dataclass(frozen=True)
class LocalWeights:
    """Weights used for wavefront-guided local neighbor selection."""

    wavefront: float = 9.92
    turn: float = 6.26
    uncovered_degree: float = 8.36


@dataclass(frozen=True)
class ReconnectionWeights:
    """Weights used by the turn-aware A* reconnection stage."""

    path_length: float = 4.55
    revisit: float = 3.06
    moderate_turn: float = 2.01
    reverse_turn: float = 9.84


@dataclass(frozen=True)
class PlannerConfig:
    """Complete configuration for one Q-TRACE execution."""

    local: LocalWeights = LocalWeights()
    reconnection: ReconnectionWeights = ReconnectionWeights()
    motion_mode: str = "adaptive"
    max_reconnect_targets: int | None = 4
    moderate_local_turn_cost: float = 1.0
    reverse_local_turn_cost: float = 2.0

    def __post_init__(self) -> None:
        if self.motion_mode not in {"4", "8", "adaptive"}:
            raise ValueError("motion_mode must be '4', '8', or 'adaptive'.")
        if self.max_reconnect_targets is not None and self.max_reconnect_targets < 1:
            raise ValueError("max_reconnect_targets must be positive or None.")

    @classmethod
    def from_toml(cls, path: str | Path) -> "PlannerConfig":
        """Load a planner configuration from a TOML file."""

        with Path(path).open("rb") as stream:
            payload = tomllib.load(stream)

        local_data = payload.get("local", {})
        reconnect_data = payload.get("reconnection", {})
        planner_data = payload.get("planner", {})

        target_limit = planner_data.get("max_reconnect_targets", 4)
        if target_limit == 0:
            target_limit = None

        return cls(
            local=LocalWeights(
                wavefront=float(local_data.get("wavefront", 9.92)),
                turn=float(local_data.get("turn", 6.26)),
                uncovered_degree=float(local_data.get("uncovered_degree", 8.36)),
            ),
            reconnection=ReconnectionWeights(
                path_length=float(reconnect_data.get("path_length", 4.55)),
                revisit=float(reconnect_data.get("revisit", 3.06)),
                moderate_turn=float(reconnect_data.get("moderate_turn", 2.01)),
                reverse_turn=float(reconnect_data.get("reverse_turn", 9.84)),
            ),
            motion_mode=str(planner_data.get("motion_mode", "adaptive")),
            max_reconnect_targets=target_limit,
            moderate_local_turn_cost=float(
                planner_data.get("moderate_local_turn_cost", 1.0)
            ),
            reverse_local_turn_cost=float(
                planner_data.get("reverse_local_turn_cost", 2.0)
            ),
        )


GLOBAL_CONFIG = PlannerConfig()
