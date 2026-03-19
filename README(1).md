# AI Assignment - Search Algorithms

Three Python programs for the programming assignment on state-based search.
No external libraries needed, just standard Python (heapq, random, math, time).

Tested on Python 3.9 but should work on 3.7 and above.

---

## How to run

Just run each file directly:

```
python assignment1_dijkstra.py
python assignment2_ugv_static.py
python assignment3_ugv_dynamic.py
```

---

## Assignment 1 — Dijkstra's Algorithm (Indian Road Network)

**File:** `assignment1_dijkstra.py`

### What it does

Implements Dijkstra's algorithm (also called Uniform Cost Search in AI) to find
the shortest road distance between cities in India. The graph has around 50 major
cities with approximate road distances taken from open sources (Google Maps, travel
sites etc).

The algorithm uses a min-heap (priority queue) to always expand the cheapest node
first. It's basically BFS but with actual distances instead of hop count.

Since roads go both ways, the graph is converted to bidirectional before running.

### Main functions

- `show_all_from("Delhi")` — prints shortest path from Delhi to every other city
- `find_route("Mumbai", "Kolkata")` — finds shortest path between two specific cities

### Output format

```
City                      km           Route
Agra                      233          Delhi -> Agra
Chandigarh                274          Delhi -> Chandigarh
Jaipur                    282          Delhi -> Jaipur
...
Chennai                   2261         Delhi -> Agra -> Gwalior -> Bhopal -> Nagpur -> Hyderabad -> Chennai
```

### To add more cities

Just add entries to the `city_roads` dictionary at the top of the file.
The function `make_bidirectional` handles making both directions automatically.

---

## Assignment 2 — UGV Static Obstacle Navigation (A\*)

**File:** `assignment2_ugv_static.py`

### What it does

A UGV (robot) has to navigate a 70x70 km battlefield grid from corner (0,0) to
corner (69,69). Obstacles are randomly placed and the robot knows all of them
before it starts moving (static case).

Uses A\* search with octile distance as the heuristic. Octile distance works for
8-directional movement where diagonal steps cost sqrt(2) and straight steps cost 1.

The program runs three times with different obstacle densities: 10%, 25% and 40%.

### How the grid works

- each cell = 1 sq km
- 0 = free, 1 = obstacle
- robot can move in 8 directions (including diagonals)
- start and goal cells are never blocked

### Output

Prints an ASCII version of the map (just the top-left 30x30 since 70x70 is huge)
and a stats table at the end:

```
Measures of Effectiveness:
  density setting    : medium
  obstacle cells     : 1291 / 4900  (26.3%)
  path found         : yes
  actual dist (km)   : 102.85
  efficiency ratio   : 0.799
  nodes expanded     : 482
  search time        : 1.40 ms
```

At 40% density the grid usually gets too blocked and no path exists — the program
handles this and reports it correctly.

### Changing things

At the top of the file:

```python
GRID_SIZE = 70        # change map size
densities = {
    "low": 0.10,
    "medium": 0.25,
    "high": 0.40
}
```

To run just one density level:

```python
run(level="medium", seed=42)
```

Changing the seed gives a different random map layout.

---

## Assignment 3 — UGV Dynamic Obstacle Navigation (D\* Lite)

**File:** `assignment3_ugv_dynamic.py`

### What it does

Same setup as assignment 2 but now obstacles are NOT known in advance. The robot
only sees what's within its sensor range (5 km radius) at each step. When it
discovers a new obstacle, it replans on the fly using D\* Lite.

D\* Lite is designed for exactly this — instead of rerunning A\* from scratch every
time something changes (which would be slow), it only fixes the parts of the path
that are actually affected. The robot was able to reach the goal in 80 steps with
74 replans in about 50ms total.

### How the simulation works

```
1. Build the true map (robot can't see this)
2. Build a partial known map with only 6% obstacles visible
3. Run initial D* Lite plan from goal back to start
4. Loop:
     a. Scan 5km radius around current position
     b. If new obstacles found -> incremental replan
     c. Move one step along best known path
     d. Repeat until goal reached or all paths blocked
```

### Why D\* Lite plans backward

It plans from goal → start instead of start → goal. This means when the robot
moves forward, the search tree doesn't need to be rebuilt from a new root each
time — only edge changes need to be handled.

### Output

Prints the robot's position on the map every 60 steps, then a final stats table:

```
Results / Measures of Effectiveness
  reached goal       : yes
  steps taken        : 80
  replanning count   : 74
  obstacles found    : 302
  total runtime      : 53.8 ms
  straight-line dist : 97.6 km
  path overhead      : -18.0% longer than ideal
  avg steps/replan   : 1.1
```

### Adjustable settings (top of file)

```python
GRID = 70        # map size
SENSOR = 5       # how far the robot can see
TRUE_OBS = 0.18  # actual obstacle density (hidden)
INIT_OBS = 0.06  # what the robot knows at start
```

---

## Summary

| File | Algorithm | Obstacle type |
|---|---|---|
| assignment1_dijkstra.py | Dijkstra / UCS | road graph (not a grid) |
| assignment2_ugv_static.py | A\* | static, fully known |
| assignment3_ugv_dynamic.py | D\* Lite | dynamic, discovered while moving |

---

## Notes

- All distances are in km
- The 70x70 grid maps print only the top-left corner in the terminal since the full grid is too wide
- Random seeds are fixed so results are the same every time you run (change the seed to get a different map)
- At very high obstacle densities (40%+) the path may not exist — both A\* and D\* Lite detect and report this
