"""Movement and heading-change utilities."""

from __future__ import annotations

from math import acos, degrees, hypot, isclose

Cell = tuple[int, int]
Direction = tuple[int, int]


def direction(start: Cell, end: Cell) -> Direction:
    return end[0] - start[0], end[1] - start[1]


def move_length(start: Cell, end: Cell) -> float:
    return hypot(end[0] - start[0], end[1] - start[1])


def path_length(path: list[Cell] | tuple[Cell, ...]) -> float:
    return sum(move_length(path[index - 1], path[index]) for index in range(1, len(path)))


def turn_angle(previous: Direction, current: Direction) -> float:
    if previous == (0, 0) or current == (0, 0):
        return 0.0

    previous_norm = hypot(*previous)
    current_norm = hypot(*current)
    cosine = (
        previous[0] * current[0] + previous[1] * current[1]
    ) / (previous_norm * current_norm)
    cosine = max(-1.0, min(1.0, cosine))
    return degrees(acos(cosine))


def turn_class(previous: Direction, current: Direction) -> str:
    angle = turn_angle(previous, current)
    if isclose(angle, 0.0, abs_tol=1e-9):
        return "start" if previous == (0, 0) or current == (0, 0) else "straight"
    if isclose(angle, 180.0, abs_tol=1e-9):
        return "reverse"
    if isclose(angle, 90.0, abs_tol=1e-9) or isclose(angle, 135.0, abs_tol=1e-9):
        return "moderate"
    if isclose(angle, 45.0, abs_tol=1e-9):
        return "minor"
    return "other"


def local_turn_cost(
    previous: Direction,
    current: Direction,
    moderate_cost: float,
    reverse_cost: float,
) -> float:
    """Return the heading cost used by local neighbor selection."""

    category = turn_class(previous, current)
    if category == "reverse":
        return reverse_cost
    if category == "moderate":
        return moderate_cost
    if category == "minor":
        return 0.5 * moderate_cost
    if category == "other":
        return moderate_cost
    return 0.0
