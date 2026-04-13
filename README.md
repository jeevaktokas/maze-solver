# Maze Solver — AI Pathfinding Visualizer

A Python + Pygame project that generates random mazes and visualizes three classic pathfinding algorithms in real time.

![Python](https://img.shields.io/badge/Python-3.10+-blue) ![Pygame](https://img.shields.io/badge/Pygame-2.x-green) ![Algorithms](https://img.shields.io/badge/Algorithms-BFS%20%7C%20DFS%20%7C%20A*-orange)

## Demo

> Press `G` to generate → pick an algorithm → press `SPACE` to watch it solve.

## Algorithms Implemented

| Algorithm | Shortest Path? | Time Complexity | Space Complexity |
|-----------|---------------|-----------------|-----------------|
| BFS       | ✅ Yes         | O(V + E)        | O(V)            |
| DFS       | ❌ No          | O(V + E)        | O(V)            |
| A\*       | ✅ Yes         | O(E log V)      | O(V)            |

**A\*** uses **Manhattan distance** as its admissible heuristic, making it optimal for grid-based mazes.

## Features

- 🧱 Random maze generation via **recursive backtracking** (Wilson's-style DFS carving)
- 🎨 Real-time visualization of visited cells, frontier, and final path
- ⚡ Adjustable animation speed
- 📊 Live stats: cells visited, path length, selected algorithm
- 🎮 Keyboard-driven interface

## Getting Started

```bash
# 1. Clone the repo
git clone https://github.com/YOUR_USERNAME/maze-solver.git
cd maze-solver

# 2. Install dependency
pip install pygame

# 3. Run
python maze_solver.py
```

**Python 3.10+** required (uses `match`-style type hints).

## Controls

| Key       | Action               |
|-----------|----------------------|
| `G`       | Generate new maze    |
| `1`       | Select BFS           |
| `2`       | Select DFS           |
| `3`       | Select A*            |
| `SPACE`   | Solve maze           |
| `R`       | Reset (keep maze)    |
| `+` / `-` | Increase/decrease speed |
| `ESC`     | Quit                 |

## Project Structure

```
maze_solver/
├── maze_solver.py   # Main application (single-file, well-documented)
└── README.md
```

## Key Concepts Demonstrated

- **OOP design** — `Cell`, `Maze`, `Renderer`, `App` classes with clear responsibilities
- **Graph traversal** — BFS with deque, DFS with explicit stack, A* with a binary heap (`heapq`)
- **Heuristic search** — admissible Manhattan distance heuristic in A*
- **Path reconstruction** — parent-pointer tracing from end → start
- **Animation loop** — tick-based frame stepping decoupled from FPS

## What I Learned / Built This For

This project was built as a portfolio piece to demonstrate:
- Algorithm design and complexity analysis
- Clean, documented Python code
- OOP principles applied to a real interactive program
- Understanding of AI search strategies

## License

MIT
