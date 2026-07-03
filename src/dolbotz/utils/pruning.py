import numpy as np

_NEIGHBOR_OFFSETS = [(-1, -1), (-1, 0), (-1, 1),
                     (0, -1),           (0, 1),
                     (1, -1),  (1, 0),  (1, 1)]


def _neighbors(point, points):
    y, x = point
    return [(y + dy, x + dx) for dy, dx in _NEIGHBOR_OFFSETS
            if (y + dy, x + dx) in points]


def prune_branches(skeleton: np.ndarray, length_threshold: int) -> np.ndarray:
    """Remove skeleton spurs shorter than length_threshold pixels.

    Thinning produces short spurious branches at junctions; this repeatedly
    trims spurs (paths ending in a degree-1 pixel) until every remaining
    branch is at least length_threshold pixels long.
    """
    points = set(map(tuple, np.argwhere(skeleton > 0)))

    while True:
        endpoints = [p for p in points if len(_neighbors(p, points)) == 1]
        if not endpoints:
            break

        to_remove = set()
        for endpoint in endpoints:
            if endpoint in to_remove:
                continue
            branch = [endpoint]
            visited = {endpoint}
            current, prev = endpoint, None
            while True:
                candidates = [n for n in _neighbors(current, points) if n != prev]
                if len(candidates) != 1 or candidates[0] in visited:
                    break
                prev, current = current, candidates[0]
                visited.add(current)
                branch.append(current)
            if len(branch) < length_threshold:
                to_remove.update(branch)

        if not to_remove:
            break
        points -= to_remove

    pruned = np.zeros_like(skeleton, dtype=np.uint8)
    if points:
        ys, xs = zip(*points)
        pruned[list(ys), list(xs)] = 255
    return pruned
