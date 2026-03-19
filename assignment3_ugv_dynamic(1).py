# Assignment 3 - UGV navigation with dynamic/unknown obstacles
# using D* Lite algorithm for this one
#
# the idea is that the robot doesn't know the full map beforehand
# it uses a sensor (like lidar) to discover obstacles as it moves
# and replans only the parts of the path that are affected
# much faster than re-running A* from scratch every single step

import heapq
import random
import time
import math

GRID = 70
FREE = 0
WALL = 1
SENSOR = 5        # how far the UGV can see around itself (km)
TRUE_OBS = 0.18   # actual obstacle density (the robot doesn't know this)
INIT_OBS = 0.06   # what the robot knows at the start
MAX_STEPS = 15000
SHOW_EVERY = 60   # print the map every 60 steps
MAP_DISPLAY = 28  # how much of the 70x70 to print (just top-left corner)

SQRT2 = math.sqrt(2)
INF = float('inf')

# 8 directions: up, down, left, right, and 4 diagonals
DIRS = [
    (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)
]


def octile(a, b):
    # admissible heuristic for 8-connected grids
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1) * min(dr, dc)


class DStarLite:
    """
    D* Lite plans from goal back to start.
    That way when the robot moves and the map changes,
    it can patch only what's broken instead of starting over.
    """

    def __init__(self, grid, start, goal):
        self.N = len(grid)
        self.grid = [row[:] for row in grid]
        self.pos = start   # where the robot currently is
        self.goal = goal
        self.km = 0.0      # accumulates as the robot moves (heuristic fix)

        # g = cost from node to goal (best known)
        # rhs = one-step lookahead estimate
        self.g = {(r, c): INF for r in range(self.N) for c in range(self.N)}
        self.rhs = {(r, c): INF for r in range(self.N) for c in range(self.N)}
        self.rhs[goal] = 0.0

        # using a lazy-deletion heap so we dont have to remove stale entries
        self.in_open = {}   # tracks what's actually "active" in the heap
        self.heap = []
        self._add(goal)

    def _neighbors(self, node):
        r, c = node
        for dr, dc, cost in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.N and 0 <= nc < self.N:
                yield (nr, nc), cost

    def _edge_cost(self, to_node, move_cost):
        # cant enter a wall cell
        return INF if self.grid[to_node[0]][to_node[1]] == WALL else move_cost

    def _priority(self, node):
        v = min(self.g[node], self.rhs[node])
        return (v + octile(self.pos, node) + self.km, v)

    def _add(self, node):
        k = self._priority(node)
        self.in_open[node] = k
        heapq.heappush(self.heap, (k, node))

    def _peek_top(self):
        # clean out stale entries while we're at it
        while self.heap:
            k, node = self.heap[0]
            if self.in_open.get(node) == k:
                return k
            heapq.heappop(self.heap)
        return (INF, INF)

    def _pop_top(self):
        while self.heap:
            k, node = heapq.heappop(self.heap)
            if self.in_open.get(node) == k:
                del self.in_open[node]
                return node
        return None

    def _drop(self, node):
        self.in_open.pop(node, None)

    def _update_node(self, node):
        if node != self.goal:
            # recalculate rhs as min over all successors
            self.rhs[node] = min(
                self._edge_cost(nbr, cost) + self.g[nbr]
                for nbr, cost in self._neighbors(node)
            )
        if node in self.in_open:
            self._drop(node)
        # only add back if inconsistent
        if self.g[node] != self.rhs[node]:
            self._add(node)

    def replan(self, max_iter=500000):
        # main D* Lite loop — runs until start node is consistent
        for _ in range(max_iter):
            top = self._peek_top()
            curr_key = self._priority(self.pos)
            if top >= curr_key and self.rhs[self.pos] == self.g[self.pos]:
                break   # converged, path is ready

            u = self._pop_top()
            if u is None:
                break

            if top < self._priority(u):
                self._add(u)
            elif self.g[u] > self.rhs[u]:
                # overconsistent: update g and propagate
                self.g[u] = self.rhs[u]
                for nbr, _ in self._neighbors(u):
                    self._update_node(nbr)
            else:
                # underconsistent: set g to inf and re-evaluate
                self.g[u] = INF
                self._update_node(u)
                for nbr, _ in self._neighbors(u):
                    self._update_node(nbr)

    def mark_change(self, cell, new_state):
        # call this whenever the UGV discovers a cell is different from what it thought
        if self.grid[cell[0]][cell[1]] == new_state:
            return False
        self.grid[cell[0]][cell[1]] = new_state
        self._update_node(cell)
        for nbr, _ in self._neighbors(cell):
            self._update_node(nbr)
        return True

    def next_move(self, cur):
        # greedily pick the neighbor with lowest cost-to-go
        best_val = INF
        best_nbr = None
        for nbr, cost in self._neighbors(cur):
            val = self._edge_cost(nbr, cost) + self.g[nbr]
            if val < best_val:
                best_val = val
                best_nbr = nbr
        return best_nbr


def build_map(size, density, start, goal, seed):
    random.seed(seed)
    grid = [[FREE] * size for _ in range(size)]
    for r in range(size):
        for c in range(size):
            if (r, c) not in (start, goal) and random.random() < density:
                grid[r][c] = WALL
    return grid


def sense_area(true_map, known_map, pos, radius, planner):
    # simulate the UGV's sensor scanning a square around its position
    r, c = pos
    found = 0
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            nr, nc = r + dr, c + dc
            if 0 <= nr < len(true_map) and 0 <= nc < len(true_map[0]):
                if true_map[nr][nc] != known_map[nr][nc]:
                    known_map[nr][nc] = true_map[nr][nc]
                    if planner.mark_change((nr, nc), true_map[nr][nc]):
                        found += 1
    return found


def draw_map(known, trail, cur, start, goal, step_num):
    n = min(MAP_DISPLAY, len(known))
    recent = set(trail[-400:])  # only show recent trail so it doesnt clutter
    print(f"\n[step {step_num}]  robot at {cur}")
    print("   " + "".join(str(c % 10) for c in range(n)))
    for r in range(n):
        line = f"{r % 10:2}|"
        for c in range(n):
            if (r, c) == cur:     line += "R"
            elif (r, c) == start: line += "S"
            elif (r, c) == goal:  line += "G"
            elif (r, c) in recent: line += "."
            elif known[r][c]:     line += "#"
            else:                 line += " "
        print(line)
    print("   R=robot  S=start  G=goal  .=trail  #=obstacle")
    if len(known) > MAP_DISPLAY:
        print(f"   (showing {MAP_DISPLAY}x{MAP_DISPLAY} of full {len(known)}x{len(known)} map)")


def results(trail, replans, found_obs, elapsed, success):
    print("\n" + "=" * 52)
    print("  Results / Measures of Effectiveness")
    print("=" * 52)
    print(f"  reached goal       : {'yes' if success else 'no'}")
    print(f"  steps taken        : {len(trail) - 1}")
    print(f"  replanning count   : {replans}")
    print(f"  obstacles found    : {found_obs}")
    print(f"  total runtime      : {elapsed * 1000:.1f} ms")
    if len(trail) > 1:
        ideal = octile(trail[0], trail[-1])
        actual = len(trail) - 1
        overhead = ((actual / ideal) - 1) * 100 if ideal > 0 else 0
        print(f"  straight-line dist : {ideal:.1f} km")
        print(f"  actual dist taken  : {actual} km")
        print(f"  path overhead      : {overhead:.1f}% longer than ideal")
        avg_freq = actual / replans if replans > 0 else actual
        print(f"  avg steps/replan   : {avg_freq:.1f}")
    print("=" * 52)


def run(seed=7):
    start = (0, 0)
    goal = (GRID - 1, GRID - 1)

    print("=" * 58)
    print("  UGV Dynamic Navigation  (D* Lite)")
    print("=" * 58)
    print(f"  map size      : {GRID}x{GRID} km")
    print(f"  start -> goal : {start} -> {goal}")
    print(f"  sensor range  : {SENSOR} km")
    print(f"  true obs %    : {TRUE_OBS*100:.0f}%  (unknown to robot)")
    print(f"  known obs %   : {INIT_OBS*100:.0f}%  (starting map)")

    true_map  = build_map(GRID, TRUE_OBS, start, goal, seed)
    known_map = build_map(GRID, INIT_OBS, start, goal, seed ^ 0xABCD)

    # make absolutely sure start and goal are free in both maps
    for mp in (true_map, known_map):
        mp[start[0]][start[1]] = FREE
        mp[goal[0]][goal[1]] = FREE

    print("\n  computing initial path from goal...")
    t0 = time.perf_counter()
    planner = DStarLite(known_map, start, goal)
    planner.replan()
    init_time = time.perf_counter() - t0
    print(f"  done in {init_time*1000:.1f} ms  |  estimated dist = {planner.g[start]:.1f} km")

    if planner.g[start] == INF:
        print("  start is already cut off from goal on the initial map, try a different seed")
        return

    # navigation loop
    cur = start
    trail = [cur]
    replans = 0
    found_obs = 0
    nav_start = time.perf_counter()

    for step in range(1, MAX_STEPS + 1):
        if cur == goal:
            print(f"\n  goal reached after {step - 1} steps!")
            break

        # scan the area around the robot
        new_stuff = sense_area(true_map, known_map, cur, SENSOR, planner)
        if new_stuff > 0:
            found_obs += new_stuff
            # tell the planner the robot has moved (adjusts heuristic)
            planner.km += octile(planner.pos, cur)
            planner.pos = cur
            planner.replan()
            replans += 1

        # take one step toward goal
        nxt = planner.next_move(cur)
        if nxt is None or planner.g[cur] >= INF:
            print(f"\n  all paths blocked at step {step}, goal unreachable")
            break

        cur = nxt
        trail.append(cur)

        if step % SHOW_EVERY == 0 or cur == goal:
            draw_map(known_map, trail, cur, start, goal, step)

    else:
        print(f"\n  hit the step limit ({MAX_STEPS}), stopping")

    elapsed = time.perf_counter() - nav_start
    results(trail, replans, found_obs, elapsed, success=(cur == goal))


if __name__ == "__main__":
    run(seed=7)
