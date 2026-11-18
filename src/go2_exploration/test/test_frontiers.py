import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from frontiers import select_goal


def test_rotated_map_goal_is_on_known_side_of_frontier():
    grid = np.full((50, 60), -1, dtype=np.int8)
    grid[5:45, 5:40] = 0
    grid[5:45, 5] = grid[5, 5:40] = grid[44, 5:40] = 100
    # yaw=90 degrees: world (x,y) = (10-local_y, 20+local_x)
    goal = select_goal(grid, .1, (10., 20., math.pi/2), (7.5, 21.5))
    assert goal is not None
    x, y, yaw = goal
    assert 3.1 < y-20 < 3.7 and 1. < 10-x < 4.
    assert math.sin(yaw) > .5


def test_frontier_across_solid_wall_is_unreachable():
    grid = np.full((40, 50), 100, dtype=np.int8)
    grid[2:38, 2:24] = grid[2:38, 25:45] = 0
    grid[:, 45:] = -1
    assert select_goal(grid, .1, (0., 0., 0.), (1., 2.)) is None


def test_narrow_opening_excludes_robot_footprint():
    grid = np.full((40, 60), 100, dtype=np.int8)
    grid[2:38, 2:24] = grid[2:38, 28:48] = grid[18:22, 24:28] = 0
    grid[:, 48:] = -1
    assert select_goal(grid, .1, (0., 0., 0.), (1., 2.), clearance=.36) is None
