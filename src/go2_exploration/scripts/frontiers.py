"""Select a reachable observation pose on the known side of a map frontier."""

from collections import deque
import math

import numpy as np
from scipy import ndimage


def select_goal(grid, resolution, origin, robot, visited=(), clearance=.36):
    """Return (world x, y, yaw), or None. Grid row 0 is OccupancyGrid's y=0."""
    if grid.ndim != 2 or not grid.size or not (resolution > 0 and clearance > 0):
        return None
    ox, oy, yaw = origin
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = robot[0] - ox, robot[1] - oy
    x0 = math.floor((c*dx + s*dy) / resolution)
    y0 = math.floor((-s*dx + c*dy) / resolution)
    h, w = grid.shape
    if not (0 <= x0 < w and 0 <= y0 < h):
        return None

    free = (grid >= 0) & (grid <= 20)
    if not free[y0, x0]:
        return None
    frontier = free & ndimage.binary_dilation(grid < 0, border_value=1)
    labels, _ = ndimage.label(frontier, structure=np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    frontier &= sizes[labels] * resolution >= .30
    if not frontier.any():
        return None
    # Leave room for the footprint even at diagonal cell corners and map edges.
    threshold = clearance + resolution / math.sqrt(2.)
    yy, xx = np.ogrid[:h, :w]
    # The current footprint can overlap unobserved cells near the sensor.
    # Exempt only those cells from clearance inflation at departure. Centres
    # and destinations must still be measured free, and occupied cells stand.
    footprint = ((xx-x0)**2 + (yy-y0)**2 <= (threshold/resolution)**2)
    clearance_mask = free | ((grid < 0) & footprint)
    margin = ndimage.distance_transform_edt(np.pad(clearance_mask, 1))[1:-1, 1:-1] * resolution
    safe = free & (margin >= threshold)
    if not safe[y0, x0]:
        return None
    # Four-connected shortest travel distance; no diagonal squeeze through a wall.
    hops = np.full((h+2, w+2), -1, dtype=np.int32)
    allowed = np.pad(safe, 1)
    todo = deque([(y0+1, x0+1)])
    hops[y0+1, x0+1] = 0
    while todo:
        y, x = todo.popleft()
        for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
            if allowed[ny, nx] and hops[ny, nx] == -1:
                hops[ny, nx] = hops[y, x] + 1
                todo.append((ny, nx))
    distance = hops[1:-1, 1:-1] * resolution
    offset, nearest = ndimage.distance_transform_edt(~frontier, return_indices=True)
    offset *= resolution
    ys, xs = np.nonzero((distance >= .60) & (offset <= clearance + .40))
    if not len(xs):
        return None
    lx, ly = (xs+.5)*resolution, (ys+.5)*resolution
    wx, wy = ox+c*lx-s*ly, oy+s*lx+c*ly
    fy, fx = nearest[:, ys, xs]
    size = max(3, round(1.0 / resolution))
    unknown_fraction = ndimage.uniform_filter((grid < 0).astype(float), size, mode='constant', cval=1)
    score = 2.0 * unknown_fraction[fy, fx] - .25 * distance[ys, xs] - offset[ys, xs]
    obstacles = (grid > 20)
    if obstacles.any():
        wall_distance = ndimage.distance_transform_edt(~obstacles)[ys, xs] * resolution
        score += 1.2 * np.minimum(wall_distance, 1.2)
    for gx, gy in visited:
        score[np.hypot(wx-gx, wy-gy) < .55] = -np.inf
    for i in np.argsort(-score):
        if not math.isfinite(score[i]):
            break
        # The geometrically nearest frontier may be on the far side of a wall.
        n = 2 * max(abs(fy[i]-ys[i]), abs(fx[i]-xs[i])) + 1
        iy = np.rint(np.linspace(ys[i], fy[i], n)).astype(int)
        ix = np.rint(np.linspace(xs[i], fx[i], n)).astype(int)
        if free[iy, ix].all():
            return float(wx[i]), float(wy[i]), yaw + math.atan2(fy[i]-ys[i], fx[i]-xs[i])
    return None
