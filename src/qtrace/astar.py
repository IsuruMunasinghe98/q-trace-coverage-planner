"""Heading-augmented A* used for Q-TRACE reconnection."""

from __future__ import annotations

from dataclasses import dataclass
import heapq

from .config import ReconnectionWeights
from .geometry import Cell, Direction, direction, move_length, turn_class
from .grid import GridMap, octile_distance

State = tuple[Cell, Direction]


@dataclass(frozen=True)
class ReconnectionPath:
    path: list[Cell]
    final_direction: Direction
    cost: float
    path_length: float
    revisit_visits: int
    moderate_turns: int
    reverse_turns: int


def _summarise_route(
    path: list[Cell],
    start_direction: Direction,
    covered: set[Cell],
    goal: Cell,
    weights: ReconnectionWeights,
) -> ReconnectionPath:
    previous = start_direction
    length = 0.0
    revisits = 0
    moderate_turns = 0
    reverse_turns = 0

    for index in range(1, len(path)):
        current_direction = direction(path[index - 1], path[index])
        length += move_length(path[index - 1], path[index])
        if path[index] in covered and path[index] != goal:
            revisits += 1
        category = turn_class(previous, current_direction)
        if category == "moderate":
            moderate_turns += 1
        elif category == "reverse":
            reverse_turns += 1
        previous = current_direction

    cost = (
        weights.path_length * length
        + weights.revisit * revisits
        + weights.moderate_turn * moderate_turns
        + weights.reverse_turn * reverse_turns
    )
    return ReconnectionPath(
        path=path,
        final_direction=previous,
        cost=cost,
        path_length=length,
        revisit_visits=revisits,
        moderate_turns=moderate_turns,
        reverse_turns=reverse_turns,
    )


def turn_aware_astar(
    grid_map: GridMap,
    start: Cell,
    goal: Cell,
    start_direction: Direction,
    covered: set[Cell],
    allow_diagonal: bool,
    weights: ReconnectionWeights,
) -> ReconnectionPath | None:
    """Find a minimum weighted-cost route over ``(cell, incoming direction)`` states."""

    start_state: State = start, start_direction
    queue: list[tuple[float, float, int, int, int, int]] = [
        (
            weights.path_length * octile_distance(start, goal, allow_diagonal),
            0.0,
            start[0],
            start[1],
            start_direction[0],
            start_direction[1],
        )
    ]
    costs: dict[State, float] = {start_state: 0.0}
    parents: dict[State, State | None] = {start_state: None}
    goal_state: State | None = None

    while queue:
        _, current_cost, row, column, direction_row, direction_column = heapq.heappop(queue)
        current = row, column
        previous_direction = direction_row, direction_column
        state: State = current, previous_direction
        if current_cost > costs.get(state, float("inf")) + 1e-12:
            continue
        if current == goal:
            goal_state = state
            break

        for candidate in grid_map.neighbors(current, allow_diagonal):
            candidate_direction = direction(current, candidate)
            category = turn_class(previous_direction, candidate_direction)
            transition_cost = weights.path_length * move_length(current, candidate)
            if candidate in covered and candidate != goal:
                transition_cost += weights.revisit
            if category == "moderate":
                transition_cost += weights.moderate_turn
            elif category == "reverse":
                transition_cost += weights.reverse_turn

            updated_cost = current_cost + transition_cost
            candidate_state: State = candidate, candidate_direction
            if updated_cost + 1e-12 >= costs.get(candidate_state, float("inf")):
                continue

            costs[candidate_state] = updated_cost
            parents[candidate_state] = state
            heuristic = weights.path_length * octile_distance(
                candidate,
                goal,
                allow_diagonal,
            )
            heapq.heappush(
                queue,
                (
                    updated_cost + heuristic,
                    updated_cost,
                    candidate[0],
                    candidate[1],
                    candidate_direction[0],
                    candidate_direction[1],
                ),
            )

    if goal_state is None:
        return None

    path: list[Cell] = []
    state: State | None = goal_state
    while state is not None:
        path.append(state[0])
        state = parents[state]
    path.reverse()
    return _summarise_route(path, start_direction, covered, goal, weights)
