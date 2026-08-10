import unittest

import numpy as np

from qtrace import GridMap, PlannerConfig, QTracePlanner
from qtrace.config import LocalWeights, ReconnectionWeights


class PlannerTests(unittest.TestCase):
    def test_complete_coverage_and_destination_constraint(self) -> None:
        grid = GridMap(
            np.array(
                [
                    [1, 1, 1],
                    [1, 0, 1],
                    [1, 1, 1],
                ],
                dtype=int,
            ),
            start=(0, 0),
            destination=(2, 2),
        )
        config = PlannerConfig(
            local=LocalWeights(2.0, 1.0, 1.0),
            reconnection=ReconnectionWeights(1.0, 2.0, 1.0, 4.0),
            motion_mode="4",
        )
        result = QTracePlanner(config).plan(grid)
        self.assertEqual(1.0, result.metrics.coverage_ratio)
        self.assertEqual(grid.destination, result.path[-1])
        self.assertEqual("4-connected", result.motion_model)

    def test_local_configuration_contains_three_weights(self) -> None:
        self.assertEqual(
            {"wavefront", "turn", "uncovered_degree"},
            set(LocalWeights.__dataclass_fields__),
        )


if __name__ == "__main__":
    unittest.main()
