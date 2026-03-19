import heapq
import random
import time
import math

# grid is 70x70, each cell = 1 sq km
# so the whole thing is a 70km x 70km battlefield
GRID_SIZE = 70
FREE = 0
OBSTACLE = 1

# three different density levels as asked in the question
densities = {
    "low": 0.10,
    "medium": 0.25,
    "high": 0.40
}


def create_grid(size, obstacle_prob, start, goal, seed=None):
    # using a seed so results are reproducible when running multiple times
    if seed is not None:
        random.seed(seed)

    grid = [[FREE] * size for _ in range(size)]

    for r in range(size):
        for c in range(size):
            # never block the start or goal cell
            if (r, c) == start or (r, c) == goal:
                continue
            if random.random() < obstacle_prob:
                grid[r][c] = OBSTACLE

    return grid


def octile_dist(a, b):
    # octile heuristic works for 8-directional movement
    # diagonal step costs sqrt(2), straight step costs 1
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


def astar(grid, start, goal):
    n = len(grid)

    # all 8 directions the UGV can move in
    directions = [
        (-1, 0, 1.0),   # up
        (1, 0, 1.0),    # down
        (0, -1, 1.0),   # left
        (0, 1, 1.0),    # right
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2))
    ]

    g_cost = {start: 0.0}
    came_from = {start: None}
    open_set = [(octile_dist(start, goal), 0.0, start)]
    visited = set()
    expanded = 0

    while open_set:
        f, g, node = heapq.heappop(open_set)

        if node in visited:
            continue
        visited.add(node)
        expanded += 1

        if node == goal:
            # trace back the path
            path = []
            curr = goal
            while curr is not None:
                path.append(curr)
                curr = came_from[curr]
            path.reverse()
            return path, g_cost[goal], expanded

        r, c = node
        for dr, dc, move_cost in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < n and 0 <= nc < n and grid[nr][nc] == FREE:
                nbr = (nr, nc)
                new_g = g + move_cost
                if nbr not in g_cost or new_g < g_cost[nbr]:
                    g_cost[nbr] = new_g
                    came_from[nbr] = node
                    f_val = new_g + octile_dist(nbr, goal)
                    heapq.heappush(open_set, (f_val, new_g, nbr))

    return [], float('inf'), expanded  # no path found


# only printing a 30x30 slice since 70x70 is too big for terminal
SHOW = 30

def print_map(grid, path, start, goal, title=""):
    size = min(SHOW, len(grid))
    path_cells = set(path)

    print(f"\n{title}")
    print("  " + "".join(str(c % 10) for c in range(size)))
    print("  " + "-" * size)

    for r in range(size):
        row = f"{r % 10}|"
        for c in range(size):
            if (r, c) == start:
                row += "S"
            elif (r, c) == goal:
                row += "G"
            elif (r, c) in path_cells:
                row += "*"
            elif grid[r][c] == OBSTACLE:
                row += "#"
            else:
                row += "."
        print(row)

    print("\n  S=start  G=goal  *=path  #=obstacle  .=free cell")
    if len(grid) > SHOW:
        print(f"  (only showing top-left {SHOW}x{SHOW} portion of the full {len(grid)}x{len(grid)} grid)")


def print_stats(grid, path, num_expanded, time_taken, level):
    size = len(grid)
    total = size * size
    obstacles = sum(grid[r][c] for r in range(size) for c in range(size))

    print("\nMeasures of Effectiveness:")
    print("-" * 40)
    print(f"  density setting    : {level}")
    print(f"  map size           : {size}x{size} km")
    print(f"  obstacle cells     : {obstacles} / {total}  ({100*obstacles/total:.1f}%)")
    print(f"  free cells         : {total - obstacles}")
    print(f"  path found         : {'yes' if path else 'no'}")

    if path:
        straight = octile_dist(path[0], path[-1])
        actual = len(path) - 1
        ratio = actual / straight if straight > 0 else 0
        print(f"  path waypoints     : {len(path)}")
        print(f"  path distance (km) : {g_val:.2f}" if False else f"  straight line dist : {straight:.2f} km")
        print(f"  path efficiency    : {ratio:.3f}  (lower = more detours needed)")

    print(f"  nodes expanded     : {num_expanded}")
    print(f"  time taken         : {time_taken*1000:.2f} ms")
    print("-" * 40)


def run(level="medium", seed=42):
    prob = densities[level]
    start = (0, 0)
    goal = (GRID_SIZE - 1, GRID_SIZE - 1)

    print(f"\n{'='*55}")
    print(f"  Running A* — obstacle density: {level} ({prob*100:.0f}%)")
    print(f"{'='*55}")
    print(f"  grid={GRID_SIZE}x{GRID_SIZE}  start={start}  goal={goal}")

    grid = create_grid(GRID_SIZE, prob, start, goal, seed=seed)

    t_start = time.perf_counter()
    path, cost, expanded = astar(grid, start, goal)
    t_end = time.perf_counter()

    if path:
        print(f"\n  Path found. Cost = {cost:.2f} km, waypoints = {len(path)}")
    else:
        print("\n  No path found. Too many obstacles blocking the way.")

    print_map(grid, path, start, goal,
              title=f"Map ({level} density) — showing top-left {SHOW}x{SHOW}")

    # recalculate stats properly
    size = len(grid)
    total = size * size
    obstacles = sum(grid[r][c] for r in range(size) for c in range(size))
    print(f"\nMeasures of Effectiveness:")
    print(f"-" * 40)
    print(f"  density setting    : {level}")
    print(f"  map size           : {size}x{size} km")
    print(f"  obstacle cells     : {obstacles} / {total}  ({100*obstacles/total:.1f}%)")
    print(f"  free cells         : {total - obstacles}")
    print(f"  path found         : {'yes' if path else 'no'}")
    if path:
        straight = octile_dist(path[0], path[-1])
        actual = len(path) - 1
        ratio = actual / straight if straight > 0 else 0
        print(f"  path waypoints     : {len(path)}")
        print(f"  actual dist (km)   : {cost:.2f}")
        print(f"  straight line dist : {straight:.2f} km")
        print(f"  efficiency ratio   : {ratio:.3f}")
    print(f"  nodes expanded     : {expanded}")
    print(f"  search time        : {(t_end-t_start)*1000:.2f} ms")
    print(f"-" * 40)

    return path, grid


if __name__ == "__main__":
    print("UGV Pathfinding — Static Obstacles (A* Search)")
    print("=" * 55)

    for lvl in ["low", "medium", "high"]:
        run(level=lvl, seed=7)
        print()
