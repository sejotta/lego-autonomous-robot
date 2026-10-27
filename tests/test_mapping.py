import tempfile
import unittest
from pathlib import Path

from robot.mapping import NEIGHBOURS, GridMap, MapFormatError

ROOT = Path(__file__).resolve().parents[1]
ARENA_A = ROOT / "maps" / "arena_a.txt"
ARENA_B = ROOT / "maps" / "arena_b.txt"
DIRECTION_TO_NEIGHBOUR = {(dx, dy): neighbour for dx, dy, neighbour in NEIGHBOURS}


class MappingTests(unittest.TestCase):
    def assert_valid_path(self, grid, path, start, goal):
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        for current, nxt in zip(path, path[1:]):
            step = (nxt[0] - current[0], nxt[1] - current[1])
            self.assertIn(step, DIRECTION_TO_NEIGHBOUR, f"non-adjacent step {current} -> {nxt}")
            self.assertTrue(grid.is_connected(current, DIRECTION_TO_NEIGHBOUR[step]))
            self.assertEqual(grid.cost_matrix[current], grid.cost_matrix[nxt] + 1)

    def test_load_map_metadata(self):
        grid = GridMap(ARENA_A)
        self.assertEqual((grid.size_x, grid.size_y), (7, 7))
        self.assertAlmostEqual(grid.cell_size_cm, 40.0)
        self.assertEqual(grid.connections.shape, (15, 15))

    def test_nf1_path_is_valid_on_both_arenas(self):
        for map_path, start in [(ARENA_A, (1, 2)), (ARENA_B, (5, 2))]:
            with self.subTest(map=map_path.name):
                grid = GridMap(map_path)
                path = grid.plan(start, (3, 3))
                self.assertIsNotNone(path)
                self.assert_valid_path(grid, path, start, (3, 3))
                self.assertEqual(grid.cost_matrix[(3, 3)], 0)

    def test_cell_centres_match_robot_start_pose(self):
        grid = GridMap(ARENA_A)
        self.assertEqual(grid.cell_center_cm((1, 2)), (60.0, 100.0))
        self.assertEqual(grid.cell_center_cm((5, 2)), (220.0, 100.0))

    def test_unreachable_goal_returns_none(self):
        grid = GridMap(ARENA_A)
        # Isolate the top-right cell by closing its west and south edges.
        grid.set_connection((5, 6), 2, False)
        grid.set_connection((6, 5), 0, False)
        self.assertIsNone(grid.plan((1, 2), (6, 6)))

    def test_replan_closes_edge_and_returns_alternative(self):
        grid = GridMap(ARENA_A)
        path = grid.plan((1, 2), (3, 3))
        self.assertIsNotNone(path)

        current, nxt = path[0], path[1]
        direction = (nxt[0] - current[0], nxt[1] - current[1])
        neighbour = DIRECTION_TO_NEIGHBOUR[direction]

        new_path = grid.replan(current, (3, 3), direction)
        self.assertFalse(grid.is_connected(current, neighbour))
        if new_path is not None:
            self.assertEqual(new_path[0], current)
            self.assertNotEqual(new_path[1], nxt)
            self.assert_valid_path(grid, new_path, current, (3, 3))

    def test_invalid_goal_raises(self):
        grid = GridMap(ARENA_A)
        with self.assertRaises(ValueError):
            grid.plan((1, 2), (99, 99))

    def test_bad_header_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "bad.txt"
            bad.write_text("7 7\n", encoding="utf-8")
            with self.assertRaises(MapFormatError):
                GridMap(bad)


if __name__ == "__main__":
    unittest.main()
