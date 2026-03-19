import heapq
import random
import time
import math

# ─────────────────────────────────────────────────────────────────────────────
# Grid configuration
# Each cell represents 1 km x 1 km, so a 70x70 grid = 70 km x 70 km map.
# 0 = free, 1 = obstacle
# ─────────────────────────────────────────────────────────────────────────────
GRID_SIZE = 70
FREE      = 0
OBSTACLE  = 1

OBSTACLE_DENSITIES = {
    "low":    0.10,   # 10% of cells blocked
    "medium": 0.25,   # 25% of cells blocked
    "high":   0.40,   # 40% of cells blocked
}


# ─────────────────────────────────────────────────────────────────────────────
# Grid generation
# ─────────────────────────────────────────────────────────────────────────────
def generate_grid(size, density, start, goal, seed=None):
    """
    Generate a grid with random obstacles.
    Start and goal cells are always kept obstacle-free.
    """
    if seed is not None:
        random.seed(seed)

    grid = [[FREE] * size for _ in range(size)]

    for r in range(size):
        for c in range(size):
            if (r, c) != start and (r, c) != goal:
                grid[r][c] = OBSTACLE if random.random() < density else FREE

    return grid


# ─────────────────────────────────────────────────────────────────────────────
# Heuristic — octile distance (allows 8-directional movement)
# ─────────────────────────────────────────────────────────────────────────────
def heuristic(a, b):
    """
    Octile distance heuristic for 8-connected grids.
    Straight move = 1 km, diagonal = sqrt(2) km ≈ 1.414 km.
    """
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


# ─────────────────────────────────────────────────────────────────────────────
# A* Search
# ─────────────────────────────────────────────────────────────────────────────
def astar(grid, start, goal):
    """
    A* search on an 8-connected grid.
    Straight moves cost 1 km; diagonal moves cost sqrt(2) km.

    Returns:
        path      : list of (row, col) tuples from start to goal, or []
        g_cost    : shortest distance in km
        nodes_exp : number of nodes expanded (measure of effectiveness)
    """
    size = len(grid)

    # 8-directional moves: (dr, dc, cost)
    moves = [
        (-1,  0, 1.0),          # N
        ( 1,  0, 1.0),          # S
        ( 0, -1, 1.0),          # W
        ( 0,  1, 1.0),          # E
        (-1, -1, math.sqrt(2)), # NW
        (-1,  1, math.sqrt(2)), # NE
        ( 1, -1, math.sqrt(2)), # SW
        ( 1,  1, math.sqrt(2)), # SE
    ]

    g = {start: 0.0}
    prev = {start: None}
    # heap: (f, g, node)
    heap = [(heuristic(start, goal), 0.0, start)]
    closed = set()
    nodes_expanded = 0

    while heap:
        f, g_curr, node = heapq.heappop(heap)

        if node in closed:
            continue
        closed.add(node)
        nodes_expanded += 1

        if node == goal:
            # Reconstruct path
            path = []
            cur = goal
            while cur is not None:
                path.append(cur)
                cur = prev[cur]
            path.reverse()
            return path, g[goal], nodes_expanded

        r, c = node
        for dr, dc, cost in moves:
            nr, nc = r + dr, c + dc
            if 0 <= nr < size and 0 <= nc < size and grid[nr][nc] == FREE:
                neighbor = (nr, nc)
                new_g = g_curr + cost
                if neighbor not in g or new_g < g[neighbor]:
                    g[neighbor] = new_g
                    prev[neighbor] = node
                    f_new = new_g + heuristic(neighbor, goal)
                    heapq.heappush(heap, (f_new, new_g, neighbor))

    return [], float('inf'), nodes_expanded  # No path found


# ─────────────────────────────────────────────────────────────────────────────
# Visualisation helpers
# ─────────────────────────────────────────────────────────────────────────────
DISPLAY_SIZE = 30   # Print only a 30x30 sub-grid for readability

def display_grid(grid, path, start, goal, label=""):
    """Print an ASCII representation of the grid (clipped to DISPLAY_SIZE)."""
    size = min(DISPLAY_SIZE, len(grid))
    path_set = set(path)

    print(f"\n{label}")
    print("  " + "".join(str(c % 10) for c in range(size)))
    print("  " + "-" * size)

    for r in range(size):
        row_str = f"{r % 10 :1d}|"
        for c in range(size):
            if (r, c) == start:
                row_str += "S"
            elif (r, c) == goal:
                row_str += "G"
            elif (r, c) in path_set:
                row_str += "*"
            elif grid[r][c] == OBSTACLE:
                row_str += "█"
            else:
                row_str += "."
        print(row_str)

    print(f"\n  Legend: S=Start  G=Goal  *=Path  █=Obstacle  .=Free")
    if len(grid) > DISPLAY_SIZE:
        print(f"  (Showing top-left {DISPLAY_SIZE}x{DISPLAY_SIZE} of {len(grid)}x{len(grid)} grid)")


# ─────────────────────────────────────────────────────────────────────────────
# Measures of Effectiveness (MoE)
# ─────────────────────────────────────────────────────────────────────────────
def compute_moe(grid, path, nodes_expanded, elapsed_time, density_level):
    size        = len(grid)
    total_cells = size * size
    obstacle_cells = sum(grid[r][c] for r in range(size) for c in range(size))

    straight_dist = heuristic(path[0], path[-1]) if path else 0
    path_length   = len(path)

    print("\n📊 Measures of Effectiveness (MoE)")
    print("=" * 45)
    print(f"  Obstacle density level   : {density_level}")
    print(f"  Grid size                : {size} x {size} km")
    print(f"  Total cells              : {total_cells}")
    print(f"  Obstacle cells           : {obstacle_cells} ({100*obstacle_cells/total_cells:.1f}%)")
    print(f"  Free cells               : {total_cells - obstacle_cells}")
    print(f"  Path found               : {'Yes' if path else 'No'}")
    if path:
        print(f"  Path length (cells)      : {path_length} waypoints")
        print(f"  Shortest distance (km)   : {path[-1][0]:.2f} km  [A* g-cost]")
        print(f"    (Straight-line dist)   : {straight_dist:.2f} km")
        ratio = (path_length - 1) / straight_dist if straight_dist > 0 else float('inf')
        print(f"  Path efficiency ratio    : {ratio:.3f}  (1.0 = straight line)")
    print(f"  Nodes expanded           : {nodes_expanded}")
    print(f"  Search time              : {elapsed_time*1000:.3f} ms")
    print("=" * 45)


# ─────────────────────────────────────────────────────────────────────────────
# Main demo
# ─────────────────────────────────────────────────────────────────────────────
def run_ugv_static(density_level="medium", seed=42):
    density = OBSTACLE_DENSITIES[density_level]
    start   = (0, 0)
    goal    = (GRID_SIZE - 1, GRID_SIZE - 1)

    print(f"\n{'='*60}")
    print(f"  UGV STATIC OBSTACLE NAVIGATION — density: {density_level.upper()}")
    print(f"{'='*60}")
    print(f"  Grid  : {GRID_SIZE} x {GRID_SIZE} km  |  Start: {start}  |  Goal: {goal}")
    print(f"  Obstacle density: {density*100:.0f}%  |  Seed: {seed}")

    grid = generate_grid(GRID_SIZE, density, start, goal, seed=seed)

    t0 = time.perf_counter()
    path, dist_km, nodes_exp = astar(grid, start, goal)
    elapsed = time.perf_counter() - t0

    if path:
        print(f"\n  ✅ Path found! Distance = {dist_km:.2f} km, waypoints = {len(path)}")
    else:
        print("\n  ❌ No path found — goal is unreachable with current obstacle layout.")

    display_grid(
        grid, path, start, goal,
        label=f"Grid map [{density_level} obstacles] — top-left {DISPLAY_SIZE}x{DISPLAY_SIZE}"
    )
    compute_moe(grid, path, nodes_exp, elapsed, density_level)
    return path, grid


if __name__ == "__main__":
    print("=" * 60)
    print("  UGV PATHFINDING — STATIC OBSTACLE ENVIRONMENT (A*)")
    print("=" * 60)

    for level in ["low", "medium", "high"]:
        run_ugv_static(density_level=level, seed=7)
        print()
