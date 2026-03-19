"""
Assignment 3 — UGV Navigation in a Dynamic Obstacle Environment
---------------------------------------------------------------
Algorithm : D* Lite (Koenig & Likhachev, 2002)
            The algorithm plans backward from the goal to the start.
            When new obstacles are discovered, only the affected
            portions of the search graph are repaired incrementally —
            making this far more efficient than full replanning (A*)
            for online navigation in unknown terrain.

Simulation:
  • 70×70 km battlefield grid (1 cell = 1 km²)
  • UGV starts at (0,0), goal is (69,69)
  • 20% of the true map is blocked; UGV only sees 8% initially
  • At each step the UGV senses a 5-km radius around itself (LiDAR)
  • Newly discovered obstacles trigger an incremental D* Lite replan
  • The path and Measures of Effectiveness are printed step-by-step
"""

import heapq, random, time, math

# ── Constants ──────────────────────────────────────────────────────────────
GRID      = 70
FREE      = 0
WALL      = 1
S_RANGE   = 5          # sensor radius (km)
T_DENS    = 0.18       # true obstacle density
K_DENS    = 0.06       # initially-known obstacle density
MAX_STEPS = 15_000
DISP_INT  = 60         # display every N steps
DISPLAY   = 28         # ASCII grid side length

SQRT2 = math.sqrt(2)
MOVES = [(-1,0,1),( 1,0,1),(0,-1,1),(0,1,1),
         (-1,-1,SQRT2),(-1,1,SQRT2),(1,-1,SQRT2),(1,1,SQRT2)]
INF = float('inf')


# ── Heuristic (octile distance, admissible for 8-connected grid) ───────────
def h(a, b):
    dr, dc = abs(a[0]-b[0]), abs(a[1]-b[1])
    return max(dr, dc) + (SQRT2-1)*min(dr, dc)


# ── D* Lite ────────────────────────────────────────────────────────────────
class DStarLite:
    """
    Plans from goal → start so that when the robot moves forward,
    it can always reuse previous search results.
    """
    def __init__(self, grid, start, goal):
        self.N     = len(grid)
        self.grid  = [r[:] for r in grid]
        self.s     = start   # current robot position (updated externally)
        self.goal  = goal
        self.km    = 0.0     # heuristic correction for start shift

        self.g   = {node: INF for node in self._all()}
        self.rhs = {node: INF for node in self._all()}
        self.rhs[goal] = 0.0

        # Lazy-deletion heap: entries may be stale; check on pop
        self._open = {}   # node -> key (for O(1) membership test)
        self._heap = []
        self._push(goal)

    def _all(self):
        for r in range(self.N):
            for c in range(self.N):
                yield (r, c)

    def _key(self, u):
        val = min(self.g[u], self.rhs[u])
        return (val + h(self.s, u) + self.km, val)

    def _push(self, u):
        k = self._key(u)
        self._open[u] = k
        heapq.heappush(self._heap, (k, u))

    def _top_key(self):
        # Pop stale entries
        while self._heap:
            k, u = self._heap[0]
            if self._open.get(u) == k:
                return k
            heapq.heappop(self._heap)
        return (INF, INF)

    def _pop_min(self):
        while self._heap:
            k, u = heapq.heappop(self._heap)
            if self._open.get(u) == k:
                del self._open[u]
                return u
        return None

    def _remove(self, u):
        self._open.pop(u, None)   # mark stale; actual heap entry ignored on pop

    def _cost(self, u, v, mc):
        # Infinite cost if v is an obstacle (can't enter it)
        return INF if self.grid[v[0]][v[1]] == WALL else mc

    def _preds(self, u):
        """Predecessors/successors are symmetric for undirected grid."""
        r, c = u
        for dr, dc, mc in MOVES:
            nr, nc = r+dr, c+dc
            if 0 <= nr < self.N and 0 <= nc < self.N:
                yield (nr, nc), mc

    def _update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min(self._cost(u, v, mc) + self.g[v]
                              for v, mc in self._preds(u))
        if u in self._open:
            self._remove(u)
        if self.g[u] != self.rhs[u]:
            self._push(u)

    def compute_shortest_path(self):
        """Run D* Lite main loop until start is consistent."""
        iters = 0
        while True:
            top_k = self._top_key()
            s_key = self._key(self.s)
            if top_k >= s_key and self.rhs[self.s] == self.g[self.s]:
                break
            u = self._pop_min()
            if u is None:
                break
            iters += 1
            if iters > 500_000:
                break   # safety guard
            old_k = top_k
            new_k = self._key(u)
            if old_k < new_k:
                self._push(u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for v, _ in self._preds(u):
                    self._update_vertex(v)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for v, _ in self._preds(u):
                    self._update_vertex(v)

    def notify_change(self, cell, new_state):
        """Call when the UGV discovers a cell has changed."""
        if self.grid[cell[0]][cell[1]] == new_state:
            return False
        self.grid[cell[0]][cell[1]] = new_state
        # Update cell and all its neighbours
        self._update_vertex(cell)
        for v, _ in self._preds(cell):
            self._update_vertex(v)
        return True

    def best_move(self, pos):
        """Return the neighbouring cell with lowest g-cost (greedy step)."""
        best, nbr = INF, None
        for v, mc in self._preds(pos):
            cost = self._cost(pos, v, mc) + self.g[v]
            if cost < best:
                best, nbr = cost, v
        return nbr


# ── Grid helpers ───────────────────────────────────────────────────────────
def make_grid(size, density, start, goal, seed):
    random.seed(seed)
    g = [[FREE]*size for _ in range(size)]
    for r in range(size):
        for c in range(size):
            if (r,c) not in (start, goal) and random.random() < density:
                g[r][c] = WALL
    return g


# ── Sensor model ───────────────────────────────────────────────────────────
def scan(true_g, known_g, pos, rng, planner):
    """Reveal true_g cells within sensor range; notify planner of changes."""
    found = 0
    r, c  = pos
    for dr in range(-rng, rng+1):
        for dc in range(-rng, rng+1):
            nr, nc = r+dr, c+dc
            if 0 <= nr < len(true_g) and 0 <= nc < len(true_g[0]):
                if true_g[nr][nc] != known_g[nr][nc]:
                    known_g[nr][nc] = true_g[nr][nc]
                    if planner.notify_change((nr,nc), true_g[nr][nc]):
                        found += 1
    return found


# ── ASCII visualisation ────────────────────────────────────────────────────
def show(known, visited, cur, start, goal, step):
    n   = min(DISPLAY, len(known))
    vis = set(visited[-500:])   # only recent trail to keep display clean
    print(f"\n--- Step {step}  |  UGV at {cur} ---")
    print("   " + "".join(str(c%10) for c in range(n)))
    for r in range(n):
        row = f"{r%10:2d}|"
        for c in range(n):
            if   (r,c) == cur:   row += "U"
            elif (r,c) == start: row += "S"
            elif (r,c) == goal:  row += "G"
            elif (r,c) in vis:   row += "·"
            elif known[r][c]:    row += "█"
            else:                row += " "
        print(row)
    print(f"    U=UGV  S=Start  G=Goal  ·=Trail  █=Obstacle")
    if len(known) > DISPLAY:
        print(f"    (showing top-left {DISPLAY}×{DISPLAY} of {len(known)}×{len(known[0])} map)")


# ── MoE report ────────────────────────────────────────────────────────────
def report_moe(path, replans, discoveries, elapsed, reached):
    print("\n" + "="*55)
    print("  MEASURES OF EFFECTIVENESS — Dynamic Navigation")
    print("="*55)
    print(f"  Goal reached               : {'Yes' if reached else 'No'}")
    print(f"  Steps travelled            : {len(path)-1}")
    print(f"  Incremental replans        : {replans}")
    print(f"  Obstacle discoveries       : {discoveries}")
    print(f"  Total compute time         : {elapsed*1000:.1f} ms")
    if len(path) > 1:
        ideal = h(path[0], path[-1])
        actual = len(path) - 1
        print(f"  Ideal (octile) distance    : {ideal:.2f} km")
        print(f"  Actual distance travelled  : {actual:.2f} km")
        print(f"  Navigation overhead        : {(actual/ideal - 1)*100:.1f}%")
        print(f"  Avg replan frequency       : every {actual/(replans or 1):.1f} steps")
    print("="*55)


# ── Main simulation ────────────────────────────────────────────────────────
def run(seed=7):
    start = (0, 0)
    goal  = (GRID-1, GRID-1)

    print("="*60)
    print("  UGV DYNAMIC OBSTACLE NAVIGATION  (D* Lite)")
    print("="*60)
    print(f"  Battlefield : {GRID}×{GRID} km²")
    print(f"  Start       : {start}       Goal : {goal}")
    print(f"  Sensor range          : {S_RANGE} km")
    print(f"  True obstacle density : {T_DENS*100:.0f}%  (unknown to UGV)")
    print(f"  Initial known density : {K_DENS*100:.0f}%")

    # Build maps
    true_map  = make_grid(GRID, T_DENS, start, goal, seed)
    known_map = make_grid(GRID, K_DENS, start, goal, seed ^ 0xABCD)
    for mp in (true_map, known_map):
        mp[start[0]][start[1]] = FREE
        mp[goal[0]][goal[1]]   = FREE

    # Initial plan
    print("\n  [Init] Running initial D* Lite from goal...")
    t_init = time.perf_counter()
    planner = DStarLite(known_map, start, goal)
    planner.compute_shortest_path()
    print(f"  [Init] Done in {(time.perf_counter()-t_init)*1000:.1f} ms  "
          f"|  g(start)={planner.g[start]:.2f} km")

    if planner.g[start] == INF:
        print("  Initial map already blocks start→goal. Try a different seed.")
        return

    # Navigation loop
    current      = start
    path         = [current]
    replans      = 0
    discoveries  = 0
    t0           = time.perf_counter()

    for step in range(1, MAX_STEPS+1):
        if current == goal:
            print(f"\n  Goal reached in {step-1} steps!")
            break

        # Sense surroundings
        found = scan(true_map, known_map, current, S_RANGE, planner)
        if found:
            discoveries += found
            # Shift heuristic baseline to current robot position
            planner.km   += h(planner.s, current)
            planner.s     = current
            planner.compute_shortest_path()
            replans += 1

        # Move one step along current best path
        nxt = planner.best_move(current)
        if nxt is None or planner.g[current] >= INF:
            print(f"\n  Path fully blocked at step {step}. Goal unreachable.")
            break

        current = nxt
        path.append(current)

        if step % DISP_INT == 0 or current == goal:
            show(known_map, path, current, start, goal, step)

    else:
        print(f"\n  Safety limit reached ({MAX_STEPS} steps).")

    elapsed = time.perf_counter() - t0
    report_moe(path, replans, discoveries, elapsed, reached=(current == goal))


if __name__ == "__main__":
    run(seed=7)
