# Pathfinding Visualizer (C++17 + raylib)

Step-by-step visualization of **BFS, DFS, Dijkstra and A\*** on a grid with walls
and weighted terrain ("swamp", cost 5).

## Build

You need a C++17 compiler and CMake >= 3.14. raylib is downloaded automatically
the first time (or an installed raylib >= 5.0 is used).

```bash
cmake -S . -B build
cmake --build build -j
./build/pathfinding_visualizer      # Windows: build\Release\pathfinding_visualizer.exe
```

Run the correctness tests (no window needed):

```bash
cd build && ctest --output-on-failure
```

**Linux note:** raylib needs the X11/OpenGL development packages, e.g. on
Debian/Ubuntu:

```bash
sudo apt install build-essential cmake git libgl1-mesa-dev libx11-dev \
     libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

## Controls

| Input | Action |
|---|---|
| Left mouse (drag) | draw walls |
| Shift + left mouse (drag) | draw swamp (cost 5) |
| Right mouse (drag) | erase |
| `S` / `G` (mouse over a cell) | move the start / goal there |
| `1` `2` `3` `4` | BFS / DFS / Dijkstra / A\* |
| `Space` | run / pause (also restarts after a finished search) |
| `N` | single step |
| `R` | reset the search (keeps walls) |
| `Up` / `Down` | faster / slower |
| `M` | random maze (with loops) |
| `T` | sprinkle random swamp |
| `C` | clear the board |

Editing the board while a search is running resets the search.

## Project layout

```
src/grid.hpp        Grid, cell types, search states, neighbour iteration
src/algorithms.hpp  Algorithm base class + BFS, DFS, Dijkstra, A*
src/maze.hpp        maze + swamp generators
src/main.cpp        raylib front-end (input, timing, drawing)
tests/              headless tests against reference implementations
```

## How it works (the short version)

* **Algorithms never draw.** Each one keeps its state in members and exposes
  `step()`, which performs exactly one expansion. The UI calls `step()` as
  often as the chosen speed demands, so pause / single-step / speed control are
  trivial and the window never freezes.
* **Nodes are ints** (`row * cols + col`), so per-node data (parent, distance)
  lives in plain `std::vector<int>`.
* **Dijkstra and A\*** are the same class (`BestFirst`); they differ only in the
  heuristic (`0` vs Manhattan distance).
* **BFS minimizes the number of moves, Dijkstra/A\* minimize total cost, DFS
  just finds some path.** Add swamp (`T`) and compare the "Path moves" and
  "Path cost" numbers to see the difference.

## Ideas to extend it

* 8-directional movement (then use the octile distance as the heuristic)
* greedy best-first search (A\* with only `h`), bidirectional BFS, JPS
* a step-back button (record the order of expansions, replay them)
* dragging the start/goal with the mouse, saving/loading maps
