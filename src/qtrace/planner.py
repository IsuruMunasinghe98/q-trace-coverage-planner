"""The Q-TRACE coverage path planner."""

from __future__ import annotations

from dataclasses import dataclass

from .astar import ReconnectionPath, turn_aware_astar
from .config import GLOBAL_CONFIG, PlannerConfig
from .geometry import Cell, Direction, direction, local_turn_cost
from .grid import GridMap, octile_distance
from .metrics import PlanMetrics, compute_metrics


@dataclass(frozen=True)
class PlanResult:
    path: list[Cell]
    reachable: set[Cell]
    motion_model: str
    metrics: PlanMetrics


class QTracePlanner:
    """Wavefront-guided local coverage with turn-aware A* reconnection."""

    def __init__(self, config: PlannerConfig = GLOBAL_CONFIG) -> None:
        self.config = config

    def plan(self, grid_map: GridMap) -> PlanResult:
        mode = self.config.motion_mode
        if mode == "4":
            return self._plan_motion(grid_map, allow_diagonal=False)
        if mode == "8":
            return self._plan_motion(grid_map, allow_diagonal=True)

        reference_reachable = grid_map.reachable(grid_map.start, allow_diagonal=True)
        candidates: list[PlanResult] = []
        for allow_diagonal in (False, True):
            try:
                candidate = self._plan_motion(grid_map, allow_diagonal)
            except ValueError:
                continue
            metrics = compute_metrics(candidate.path, reference_reachable)
            candidates.append(
                PlanResult(
                    path=candidate.path,
                    reachable=reference_reachable,
                    motion_model=candidate.motion_model,
                    metrics=metrics,
                )
            )

        if not candidates:
            raise ValueError("Both adaptive motion variants failed.")

        return min(
            candidates,
            key=lambda item: (
                item.metrics.reachable_free_cells - item.metrics.covered_reachable_cells,
                item.metrics.path_length,
                item.metrics.total_turns,
                item.metrics.revisit_visits,
                item.metrics.reverse_turns,
                item.metrics.moderate_turns,
                item.motion_model,
            ),
        )

    def _local_score(
        self,
        grid_map: GridMap,
        candidate: Cell,
        current: Cell,
        previous_direction: Direction,
        uncovered: set[Cell],
        wavefront: dict[Cell, float],
        allow_diagonal: bool,
    ) -> tuple[float, float, int, float, int, int]:
        candidate_direction = direction(current, candidate)
        turn_cost = local_turn_cost(
            previous_direction,
            candidate_direction,
            self.config.moderate_local_turn_cost,
            self.config.reverse_local_turn_cost,
        )
        degree = sum(
            neighbor in uncovered
            for neighbor in grid_map.neighbors(candidate, allow_diagonal)
        )
        wave_value = wavefront.get(candidate, float("-inf"))
        score = (
            self.config.local.wavefront * wave_value
            - self.config.local.turn * turn_cost
            + self.config.local.uncovered_degree * degree
        )
        return score, wave_value, degree, -turn_cost, -candidate[0], -candidate[1]

    def _candidate_targets(
        self,
        current: Cell,
        uncovered: set[Cell],
        wavefront: dict[Cell, float],
        allow_diagonal: bool,
    ) -> list[Cell]:
        targets = sorted(
            uncovered,
            key=lambda cell: (
                octile_distance(current, cell, allow_diagonal),
                -wavefront.get(cell, 0.0),
                cell[0],
                cell[1],
            ),
        )
        limit = self.config.max_reconnect_targets
        return targets if limit is None else targets[:limit]

    def _select_reconnection(
        self,
        grid_map: GridMap,
        current: Cell,
        previous_direction: Direction,
        covered: set[Cell],
        uncovered: set[Cell],
        wavefront: dict[Cell, float],
        allow_diagonal: bool,
    ) -> ReconnectionPath:
        routes: list[tuple[tuple[float, float, int, int, int, float, int, int], ReconnectionPath]] = []
        for target in self._candidate_targets(
            current,
            uncovered,
            wavefront,
            allow_diagonal,
        ):
            route = turn_aware_astar(
                grid_map=grid_map,
                start=current,
                goal=target,
                start_direction=previous_direction,
                covered=covered,
                allow_diagonal=allow_diagonal,
                weights=self.config.reconnection,
            )
            if route is None:
                continue
            key = (
                route.cost,
                route.path_length,
                route.revisit_visits,
                route.moderate_turns,
                route.reverse_turns,
                -wavefront.get(target, 0.0),
                target[0],
                target[1],
            )
            routes.append((key, route))

        if not routes:
            raise ValueError("No feasible reconnection route was found.")
        return min(routes, key=lambda item: item[0])[1]

    def _plan_motion(self, grid_map: GridMap, allow_diagonal: bool) -> PlanResult:
        reachable = grid_map.reachable(grid_map.start, allow_diagonal)
        if grid_map.destination not in reachable:
            raise ValueError("The destination is not reachable from the start.")

        wavefront = grid_map.distance_map(grid_map.destination, allow_diagonal)
        uncovered = set(reachable)
        uncovered.discard(grid_map.start)
        covered = {grid_map.start}
        path = [grid_map.start]
        current = grid_map.start
        previous_direction: Direction = (0, 0)

        while uncovered:
            local_candidates = [
                candidate
                for candidate in grid_map.neighbors(current, allow_diagonal)
                if candidate in uncovered
            ]
            if local_candidates:
                candidate = max(
                    local_candidates,
                    key=lambda cell: self._local_score(
                        grid_map,
                        cell,
                        current,
                        previous_direction,
                        uncovered,
                        wavefront,
                        allow_diagonal,
                    ),
                )
                previous_direction = direction(current, candidate)
                current = candidate
                path.append(current)
                covered.add(current)
                uncovered.discard(current)
                continue

            route = self._select_reconnection(
                grid_map,
                current,
                previous_direction,
                covered,
                uncovered,
                wavefront,
                allow_diagonal,
            )
            for node in route.path[1:]:
                previous_direction = direction(current, node)
                current = node
                path.append(current)
                covered.add(current)
                uncovered.discard(current)

        if current != grid_map.destination:
            route = turn_aware_astar(
                grid_map=grid_map,
                start=current,
                goal=grid_map.destination,
                start_direction=previous_direction,
                covered=covered,
                allow_diagonal=allow_diagonal,
                weights=self.config.reconnection,
            )
            if route is None:
                raise ValueError("No final route to the destination was found.")
            path.extend(route.path[1:])

        motion_model = "8-connected" if allow_diagonal else "4-connected"
        return PlanResult(
            path=path,
            reachable=reachable,
            motion_model=motion_model,
            metrics=compute_metrics(path, reachable),
        )
