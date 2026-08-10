"""Publication-style path visualization."""

from __future__ import annotations

from collections import Counter
from pathlib import Path

import numpy as np

from .grid import GridMap
from .planner import PlanResult


def plot_plan(grid_map: GridMap, result: PlanResult, output: str | Path) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rows, columns = grid_map.shape
    figure_width = min(12.0, max(6.0, columns / 4.0))
    figure_height = min(12.0, max(5.0, rows / 4.0))
    figure, axis = plt.subplots(figsize=(figure_width, figure_height), constrained_layout=True)
    axis.imshow(
        grid_map.cells,
        cmap=ListedColormap(["#172033", "#f5f7fb"]),
        origin="upper",
        interpolation="nearest",
    )

    path_array = np.asarray(result.path)
    axis.plot(
        path_array[:, 1],
        path_array[:, 0],
        color="#0f6b8d",
        linewidth=1.8,
        alpha=0.9,
        label="Q-TRACE path",
    )
    axis.scatter(
        grid_map.start[1],
        grid_map.start[0],
        s=70,
        color="#00a6a6",
        edgecolors="white",
        linewidths=1.0,
        zorder=4,
        label="Start",
    )
    axis.scatter(
        grid_map.destination[1],
        grid_map.destination[0],
        s=80,
        marker="*",
        color="#6dbd45",
        edgecolors="white",
        linewidths=0.8,
        zorder=4,
        label="Destination",
    )

    revisit_cells = [cell for cell, count in Counter(result.path).items() if count > 1]
    if revisit_cells:
        revisit_array = np.asarray(revisit_cells)
        axis.scatter(
            revisit_array[:, 1],
            revisit_array[:, 0],
            s=22,
            marker="x",
            color="#c03a8c",
            linewidths=1.1,
            zorder=3,
            label="Revisited cell",
        )

    axis.set_title(
        f"Q-TRACE coverage path | {result.motion_model} | "
        f"length={result.metrics.path_length:.2f}",
        fontsize=11,
        fontweight="bold",
    )
    axis.set_xticks(np.arange(-0.5, columns, 1), minor=True)
    axis.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    axis.grid(which="minor", color="#c8d0dc", linewidth=0.35, alpha=0.6)
    axis.tick_params(which="minor", bottom=False, left=False)
    axis.set_xlabel("Column")
    axis.set_ylabel("Row")
    axis.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=4, frameon=False)
    figure.savefig(output_path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(figure)
