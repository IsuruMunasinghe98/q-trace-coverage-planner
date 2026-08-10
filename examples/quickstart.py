"""Minimal programmatic Q-TRACE example."""

from qtrace import PlannerConfig, QTracePlanner, load_dataset

records = load_dataset("data/evaluation_set.txt")
planner = QTracePlanner(PlannerConfig.from_toml("configs/global.toml"))
result = planner.plan(records[0].grid_map)

print(f"Motion model: {result.motion_model}")
print(f"Coverage ratio: {result.metrics.coverage_ratio:.3f}")
print(f"Path length: {result.metrics.path_length:.3f}")
print(f"Revisit visits: {result.metrics.revisit_visits}")
print(f"Reverse turns: {result.metrics.reverse_turns}")
