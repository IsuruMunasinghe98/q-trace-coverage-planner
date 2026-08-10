import unittest

import numpy as np

from qtrace.grid import GridMap, load_dataset


class GridTests(unittest.TestCase):
    def test_diagonal_gap_between_two_obstacles_is_rejected(self) -> None:
        grid = GridMap(
            np.array([[1, 0], [0, 1]], dtype=int),
            start=(0, 0),
            destination=(0, 0),
        )
        self.assertFalse(grid.valid_move((0, 0), (1, 1), allow_diagonal=True))

    def test_one_clear_side_allows_diagonal_transition(self) -> None:
        grid = GridMap(
            np.array([[1, 1], [0, 1]], dtype=int),
            start=(0, 0),
            destination=(1, 1),
        )
        self.assertTrue(grid.valid_move((0, 0), (1, 1), allow_diagonal=True))

    def test_evaluation_dataset_metadata(self) -> None:
        records = load_dataset("data/evaluation_set.txt")
        self.assertEqual(100, len(records))
        self.assertEqual("low_obstacle", records[0].category)
        self.assertEqual("clustered_obstacle", records[-1].category)


if __name__ == "__main__":
    unittest.main()
