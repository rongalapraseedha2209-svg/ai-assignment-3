# Programming Assignment — AI Search Algorithms

Three standalone Python programs implementing state-based search in different
real-world scenarios. No external libraries are required — only the Python
standard library (`heapq`, `random`, `math`, `time`).

---

## Requirements

- Python 3.7 or higher
- No third-party packages needed

---

## Assignment 1 — Dijkstra's Algorithm on the Indian Road Network

**File:** `assignment1_dijkstra.py`

### Problem
When actions have different costs in a state-based search space, the
evaluation function is set to the cost of the path from the root to the
current node. This is called Dijkstra's algorithm (or Uniform-Cost Search
in the AI community). The task is to apply it to the cities of India using
real road distances.

### Algorithm
Dijkstra's algorithm explores nodes in order of increasing path cost using
a min-heap priority queue. It guarantees the shortest path to every reachable
node from the source city.

- **State space:** Graph of ~50 major Indian cities
- **Edge weights:** Approximate road distances in kilometres (sourced from
  publicly available road data)
- **Data structure:** Min-heap (Python `heapq`)
- **Complexity:** O((V + E) log V)

### How to Run

```bash
python assignment1_dijkstra.py
```

### What the Program Does

1. Builds a bidirectional road-distance graph covering cities including Delhi,
   Mumbai, Bangalore, Chennai, Hyderabad, Kolkata, and ~45 others.
2. Runs Dijkstra from **Delhi** and prints the shortest distance and full
   route to every other reachable city.
3. Solves four point-to-point queries (Mumbai→Kolkata, Bangalore→Delhi, etc.).
4. Runs again from **Bangalore** as a second demonstration.

### Sample Output

```
Shortest road distances from Delhi:
Destination               Distance (km)   Path
Agra                      233             Delhi -> Agra
Chandigarh                274             Delhi -> Chandigarh
Jaipur                    282             Delhi -> Jaipur
...
Chennai                   2261            Delhi -> Agra -> Gwalior -> Bhopal -> Nagpur -> Hyderabad -> Chennai
```

### Extending the Program

To query any two cities interactively, call:

```python
shortest_path_between("Mumbai", "Guwahati")
```

To see all shortest paths from any city:

```python
print_all_shortest_paths("Kolkata")
```

---

## Assignment 2 — UGV Navigation with Static Obstacles (A\*)

**File:** `assignment2_ugv_static.py`

### Problem
An Unmanned Ground Vehicle (UGV) must find the optimal path from a
user-specified start node to a goal node on a 70×70 km battlefield grid.
Obstacles are placed randomly with three different density levels and are
known entirely before the UGV begins moving.

### Algorithm
A\* Search with an octile-distance heuristic — the admissible heuristic
for 8-connected grids where diagonal moves cost √2 km and straight moves
cost 1 km.

- **State space:** 70×70 grid (4 900 cells), each cell = 1 km²
- **Movement:** 8-directional (N, S, E, W, NW, NE, SW, SE)
- **Heuristic:** Octile distance — `max(dr,dc) + (√2−1)·min(dr,dc)`
- **Obstacle densities:** Low (10%), Medium (25%), High (40%)

### How to Run

```bash
python assignment2_ugv_static.py
```

### What the Program Does

1. Generates a random 70×70 obstacle grid for each of the three density
   levels using a fixed seed (reproducible results).
2. Runs A\* from corner `(0,0)` to corner `(69,69)`.
3. Prints an ASCII map (top-left 30×30 shown) with the path marked.
4. Prints a **Measures of Effectiveness** table for each density level.

### Measures of Effectiveness (MoE)

| Metric | Description |
|---|---|
| Path found | Whether a valid path exists |
| Path length | Number of waypoints in the solution |
| Shortest distance (km) | True g-cost from A\* |
| Path efficiency ratio | Actual / straight-line distance (1.0 = perfect) |
| Nodes expanded | How many cells were visited during search |
| Search time | Wall-clock time in milliseconds |

### Sample Output

```
UGV STATIC OBSTACLE NAVIGATION — density: LOW
Path found! Distance = 99.34 km, waypoints = 73
Nodes expanded: 221   Search time: 1.5 ms
Path efficiency ratio: 0.738
```

At **40% obstacle density** the grid becomes too fragmented and the program
correctly reports that the goal is unreachable.

### Changing Parameters

Edit the constants at the top of the file:

```python
GRID_SIZE = 70          # map size in km
OBSTACLE_DENSITIES = {
    "low":    0.10,
    "medium": 0.25,
    "high":   0.40,
}
```

Call a single density run and capture the result:

```python
path, grid = run_ugv_static(density_level="medium", seed=123)
```

---

## Assignment 3 — UGV Navigation with Dynamic Obstacles (D\* Lite)

**File:** `assignment3_ugv_dynamic.py`

### Problem
The static-obstacle condition is relaxed. In a real battlefield, obstacles
are dynamic and not known a-priori. The UGV must navigate and find the
optimal path while discovering obstacles in real time.

### Algorithm
**D\* Lite** (Koenig & Likhachev, 2002) — the standard algorithm for
online robotic navigation in partially-known, changing environments.

Unlike A\* (which would replan the entire path from scratch on every new
discovery), D\* Lite plans **backward from goal to start** and repairs only
the affected portion of the search graph when changes are found. This makes
incremental replanning orders of magnitude faster than full replanning.

- **State space:** 70×70 km grid
- **Sensor model:** LiDAR simulation — the UGV reveals all cells within a
  5 km radius at each step
- **True obstacle density:** 18% (hidden from UGV initially)
- **Initially known density:** 6% (UGV's starting map)
- **Movement:** 8-directional, costs 1 km (straight) or √2 km (diagonal)

### How it Works (Step by Step)

```
1. Build a partial known map (6% obstacles revealed)
2. Run D* Lite backward from GOAL → START on the known map
3. UGV begins moving toward goal:
      a. Sense 5 km radius around current position
      b. If any new obstacles found → trigger incremental D* Lite replan
      c. Move one step along the repaired shortest path
      d. Repeat until goal reached or path fully blocked
```

### How to Run

```bash
python assignment3_ugv_dynamic.py
```

### What the Program Does

1. Generates the true battlefield map (hidden) and the UGV's initial
   partial map (visible).
2. Runs the initial D\* Lite plan from goal → start.
3. Simulates step-by-step navigation with live sensing and replanning.
4. Prints ASCII maps at every 60 steps showing the UGV's trail.
5. Prints a full **Measures of Effectiveness** report at the end.

### Measures of Effectiveness (MoE)

| Metric | Description |
|---|---|
| Goal reached | Whether the UGV successfully reached the goal |
| Steps travelled | Total km-steps taken |
| Incremental replans | How many times D\* Lite was triggered |
| Obstacle discoveries | Total new obstacle cells found during navigation |
| Compute time | Total wall-clock time in milliseconds |
| Ideal distance | Octile straight-line distance from start to goal |
| Actual distance | True path length taken |
| Navigation overhead | How much longer the actual path was vs. ideal |

### Sample Output

```
[Init] Running initial D* Lite from goal...
[Init] Done in 17.2 ms  |  g(start)=98.17 km

Goal reached in 80 steps!

MEASURES OF EFFECTIVENESS — Dynamic Navigation
  Goal reached               : Yes
  Steps travelled            : 80
  Incremental replans        : 74
  Obstacle discoveries       : 302
  Total compute time         : 24.2 ms
  Ideal (octile) distance    : 97.58 km
  Navigation overhead        : -18.0%
  Avg replan frequency       : every 1.1 steps
```

### Changing Parameters

Edit the constants at the top of the file:

```python
GRID      = 70      # map size in km
S_RANGE   = 5       # sensor radius in km
T_DENS    = 0.18    # true obstacle density (hidden)
K_DENS    = 0.06    # initially known obstacle density
```

Change the random seed for a different map layout:

```python
run(seed=42)    # any integer
```

---

## File Summary

| File | Assignment | Algorithm | Key Concept |
|---|---|---|---|
| `assignment1_dijkstra.py` | 1 | Dijkstra / UCS | Shortest paths, weighted graph |
| `assignment2_ugv_static.py` | 2 | A\* Search | Heuristic search, static obstacles |
| `assignment3_ugv_dynamic.py` | 3 | D\* Lite | Incremental replanning, dynamic obstacles |

## Concepts Covered

- **Uniform Cost Search (Dijkstra):** Optimal for weighted graphs when all
  edge costs are non-negative.
- **A\* Search:** Extends Dijkstra with a heuristic to guide search toward
  the goal — faster but requires an admissible heuristic.
- **D\* Lite:** Extends A\* to handle unknown and changing environments by
  replanning incrementally rather than from scratch.
- **Measures of Effectiveness:** Quantitative metrics used to evaluate
  algorithm performance — path length, nodes expanded, compute time,
  efficiency ratio, and replan frequency.
