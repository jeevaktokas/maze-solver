"""
Maze Solver — AI Pathfinding Visualizer
========================================
Algorithms: BFS (Breadth-First Search), DFS (Depth-First Search), A* (A-Star)
Visualization: Pygame

Author: [Your Name]
GitHub: [Your GitHub]
LinkedIn: [Your LinkedIn]

How to run:
    pip install pygame
    python maze_solver.py

Controls:
    G       — Generate a new random maze
    1       — Select BFS
    2       — Select DFS
    3       — Select A*
    SPACE   — Solve the maze
    R       — Reset (clear visited cells)
    +/-     — Increase / decrease animation speed
    ESC     — Quit
"""

from __future__ import annotations
import pygame
import sys
import random
import heapq
from collections import deque
from enum import Enum


# ─────────────────────────────────────────────
#  Configuration
# ─────────────────────────────────────────────

WINDOW_W, WINDOW_H = 900, 680
PANEL_W = 240
MAZE_W = WINDOW_W - PANEL_W

COLS, ROWS = 41, 31          # Must be odd numbers for maze generation
CELL_W = MAZE_W // COLS
CELL_H = (WINDOW_H - 60) // ROWS   # Leave room for status bar

FPS = 60
DEFAULT_SPEED = 5            # Frames between animation steps (lower = faster)

# Colour palette
BLACK   = (15,  23,  42)
WHITE   = (248, 250, 252)
GRAY    = (100, 116, 139)
WALL    = (30,  41,  59)
OPEN    = (241, 245, 249)
VISITED = (147, 197, 253)
FRONTIER= (196, 181, 253)
PATH    = (245, 158,  11)
START   = (34,  197,  94)
END     = (239,  68,  68)
PANEL_BG= (248, 250, 252)
ACCENT  = (99,  102, 241)


class Algorithm(Enum):
    BFS   = "BFS"
    DFS   = "DFS"
    ASTAR = "A*"


# ─────────────────────────────────────────────
#  Cell & Grid
# ─────────────────────────────────────────────

class Cell:
    """Represents a single cell in the maze grid."""

    def __init__(self, row: int, col: int):
        self.row = row
        self.col = col
        self.wall = True
        self.visited = False
        self.parent: "Cell | None" = None

    def __lt__(self, other):
        # Needed for heapq comparisons in A*
        return (self.row, self.col) < (other.row, other.col)

    def __eq__(self, other):
        return isinstance(other, Cell) and self.row == other.row and self.col == other.col

    def __hash__(self):
        return hash((self.row, self.col))


class Maze:
    """
    Holds the grid and generates mazes using recursive backtracking.
    Provides neighbour lookup for pathfinding algorithms.
    """

    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols
        self.grid: list[list[Cell]] = []
        self.start: Cell = None
        self.end: Cell = None
        self._init_grid()

    def _init_grid(self):
        self.grid = [[Cell(r, c) for c in range(self.cols)] for r in range(self.rows)]

    def cell(self, r: int, c: int) -> Cell:
        return self.grid[r][c]

    def generate(self):
        """Recursive backtracking maze generation (DFS-based)."""
        self._init_grid()

        def carve(r: int, c: int):
            self.grid[r][c].wall = False
            directions = [(0, 2), (2, 0), (0, -2), (-2, 0)]
            random.shuffle(directions)
            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if 0 < nr < self.rows - 1 and 0 < nc < self.cols - 1:
                    if self.grid[nr][nc].wall:
                        self.grid[r + dr // 2][c + dc // 2].wall = False
                        carve(nr, nc)

        carve(1, 1)
        self.start = self.grid[1][1]
        self.end   = self.grid[self.rows - 2][self.cols - 2]
        self.end.wall = False

    def reset_search(self):
        """Clear visited state and parents without regenerating walls."""
        for row in self.grid:
            for cell in row:
                cell.visited = False
                cell.parent  = None

    def neighbors(self, cell: Cell) -> list[Cell]:
        """Return passable (non-wall) orthogonal neighbours."""
        result = []
        for dr, dc in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nr, nc = cell.row + dr, cell.col + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if not self.grid[nr][nc].wall:
                    result.append(self.grid[nr][nc])
        return result


# ─────────────────────────────────────────────
#  Pathfinding Algorithms
# ─────────────────────────────────────────────

class PathfinderResult:
    """Container for algorithm output."""
    def __init__(self, frames: list, path: list[Cell], visited_count: int):
        self.frames = frames          # list of (visited_set, frontier_set) snapshots
        self.path   = path            # final shortest path
        self.visited_count = visited_count


def bfs(maze: Maze) -> PathfinderResult:
    """
    Breadth-First Search — guarantees the shortest path.
    Time:  O(V + E)
    Space: O(V)
    """
    start, end = maze.start, maze.end
    queue   = deque([start])
    visited = {start}
    parent  = {start: None}
    frames  = []

    while queue:
        frontier = set()
        next_level = deque()

        for _ in range(len(queue)):
            cell = queue.popleft()
            if cell == end:
                return PathfinderResult(frames, _trace_path(parent, end), len(visited))

            for nb in maze.neighbors(cell):
                if nb not in visited:
                    visited.add(nb)
                    parent[nb] = cell
                    next_level.append(nb)
                    frontier.add(nb)

        frames.append((frozenset(visited), frozenset(frontier)))
        queue = next_level

    return PathfinderResult(frames, [], len(visited))


def dfs(maze: Maze) -> PathfinderResult:
    """
    Depth-First Search — finds A path, not necessarily the shortest.
    Time:  O(V + E)
    Space: O(V)  (recursion / stack)
    """
    start, end = maze.start, maze.end
    stack   = [start]
    visited = set()
    parent  = {start: None}
    frames  = []

    while stack:
        cell = stack.pop()
        if cell in visited:
            continue
        visited.add(cell)

        if cell == end:
            return PathfinderResult(frames, _trace_path(parent, end), len(visited))

        frontier = set()
        for nb in maze.neighbors(cell):
            if nb not in visited:
                parent.setdefault(nb, cell)
                stack.append(nb)
                frontier.add(nb)

        frames.append((frozenset(visited), frozenset(frontier)))

    return PathfinderResult(frames, [], len(visited))


def astar(maze: Maze) -> PathfinderResult:
    """
    A* Search — optimal and efficient using a heuristic.
    Heuristic: Manhattan distance (admissible for grid mazes).
    Time:  O(E log V)  (with binary heap)
    Space: O(V)
    """
    def h(cell: Cell) -> int:
        """Manhattan distance heuristic."""
        return abs(cell.row - maze.end.row) + abs(cell.col - maze.end.col)

    start, end = maze.start, maze.end
    g_score = {start: 0}
    f_score = {start: h(start)}
    open_heap = [(f_score[start], start)]
    came_from: dict[Cell, Cell | None] = {start: None}
    closed = set()
    frames = []

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)

        if current == end:
            return PathfinderResult(frames, _trace_path(came_from, end), len(closed))

        open_set = {cell for _, cell in open_heap}
        frames.append((frozenset(closed), frozenset(open_set)))

        for nb in maze.neighbors(current):
            if nb in closed:
                continue
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(nb, float('inf')):
                came_from[nb] = current
                g_score[nb]   = tentative_g
                f_score[nb]   = tentative_g + h(nb)
                heapq.heappush(open_heap, (f_score[nb], nb))

    return PathfinderResult(frames, [], len(closed))


def _trace_path(parent: dict, end: Cell) -> list[Cell]:
    """Walk the parent map back from end to reconstruct the path."""
    path, cur = [], end
    while cur is not None:
        path.append(cur)
        cur = parent.get(cur)
    path.reverse()
    return path


# ─────────────────────────────────────────────
#  Renderer
# ─────────────────────────────────────────────

class Renderer:
    """Handles all Pygame drawing."""

    def __init__(self, screen: pygame.Surface, maze: Maze):
        self.screen = screen
        self.maze   = maze
        self.font_sm  = pygame.font.SysFont("monospace", 13)
        self.font_med = pygame.font.SysFont("monospace", 15, bold=True)
        self.font_lg  = pygame.font.SysFont("monospace", 18, bold=True)

    def draw_all(
        self,
        visited:  frozenset | None,
        frontier: frozenset | None,
        path:     list[Cell],
        algo:     Algorithm,
        stats:    dict,
        status:   str,
    ):
        self.screen.fill(OPEN)
        self._draw_maze(visited, frontier, path)
        self._draw_panel(algo, stats)
        self._draw_status(status)
        pygame.display.flip()

    def _cell_rect(self, cell: Cell) -> pygame.Rect:
        x = cell.col * CELL_W
        y = cell.row * CELL_H
        return pygame.Rect(x, y, CELL_W, CELL_H)

    def _draw_maze(self, visited, frontier, path):
        path_set = set(path)
        for row in self.maze.grid:
            for cell in row:
                rect = self._cell_rect(cell)
                if cell == self.maze.start:
                    color = START
                elif cell == self.maze.end:
                    color = END
                elif cell in path_set:
                    color = PATH
                elif visited and cell in visited:
                    color = VISITED
                elif frontier and cell in frontier:
                    color = FRONTIER
                elif cell.wall:
                    color = WALL
                else:
                    color = OPEN
                pygame.draw.rect(self.screen, color, rect)

    def _draw_panel(self, algo: Algorithm, stats: dict):
        panel_rect = pygame.Rect(MAZE_W, 0, PANEL_W, WINDOW_H)
        pygame.draw.rect(self.screen, PANEL_BG, panel_rect)
        pygame.draw.line(self.screen, GRAY, (MAZE_W, 0), (MAZE_W, WINDOW_H), 1)

        x = MAZE_W + 16
        y = 20

        title = self.font_lg.render("Maze Solver", True, BLACK)
        self.screen.blit(title, (x, y)); y += 32

        # Algorithm selector
        self._label("ALGORITHM", x, y); y += 20
        for a in Algorithm:
            active = a == algo
            bg = ACCENT if active else WHITE
            fg = WHITE  if active else BLACK
            rect = pygame.Rect(x, y, PANEL_W - 32, 26)
            pygame.draw.rect(self.screen, bg, rect, border_radius=4)
            if not active:
                pygame.draw.rect(self.screen, GRAY, rect, 1, border_radius=4)
            lbl = self.font_sm.render(a.value, True, fg)
            self.screen.blit(lbl, (x + 8, y + 6))
            y += 32

        y += 8
        # Stats
        self._label("STATS", x, y); y += 20
        for key, val in stats.items():
            self._stat_row(key, str(val), x, y); y += 24

        y += 12
        # Legend
        self._label("LEGEND", x, y); y += 20
        legend = [
            (START,    "Start"),
            (END,      "End"),
            (VISITED,  "Visited"),
            (FRONTIER, "Frontier"),
            (PATH,     "Path"),
            (WALL,     "Wall"),
        ]
        for color, name in legend:
            pygame.draw.rect(self.screen, color, (x, y + 3, 12, 12), border_radius=2)
            lbl = self.font_sm.render(name, True, GRAY)
            self.screen.blit(lbl, (x + 18, y))
            y += 22

        y += 12
        # Controls hint
        self._label("CONTROLS", x, y); y += 20
        controls = ["G  New maze", "1/2/3  BFS/DFS/A*",
                    "SPACE  Solve", "R  Reset", "+/-  Speed", "ESC  Quit"]
        for c in controls:
            lbl = self.font_sm.render(c, True, GRAY)
            self.screen.blit(lbl, (x, y)); y += 18

    def _draw_status(self, status: str):
        bar = pygame.Rect(0, WINDOW_H - 30, MAZE_W, 30)
        pygame.draw.rect(self.screen, BLACK, bar)
        lbl = self.font_sm.render(status, True, WHITE)
        self.screen.blit(lbl, (10, WINDOW_H - 22))

    def _label(self, text: str, x: int, y: int):
        lbl = self.font_sm.render(text, True, GRAY)
        self.screen.blit(lbl, (x, y))

    def _stat_row(self, key: str, val: str, x: int, y: int):
        k = self.font_sm.render(key, True, GRAY)
        v = self.font_med.render(val, True, BLACK)
        self.screen.blit(k, (x, y))
        self.screen.blit(v, (x + 120, y))


# ─────────────────────────────────────────────
#  Main Application
# ─────────────────────────────────────────────

class App:
    """
    Main application class — ties together Maze, Algorithms, and Renderer.
    Manages animation state and user input.
    """

    def __init__(self):
        pygame.init()
        self.screen   = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("Maze Solver — AI Pathfinding Visualizer")
        self.clock    = pygame.time.Clock()

        self.maze     = Maze(ROWS, COLS)
        self.renderer = Renderer(self.screen, self.maze)

        self.algo     = Algorithm.BFS
        self.speed    = DEFAULT_SPEED   # ticks between animation steps
        self.tick     = 0

        # Animation state
        self.frames:   list            = []
        self.path:     list[Cell]      = []
        self.frame_idx: int            = 0
        self.animating: bool           = False
        self.visited_snap: frozenset   = frozenset()
        self.frontier_snap: frozenset  = frozenset()

        self.stats  = {"Algorithm": "—", "Visited": "—", "Path len": "—"}
        self.status = "Press G to generate a maze."

        self.maze.generate()
        self.status = "Maze ready. Press SPACE to solve."

    # ── Input ──────────────────────────────────

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN:
                self._on_key(event.key)

    def _on_key(self, key: int):
        if key == pygame.K_ESCAPE:
            pygame.quit(); sys.exit()

        elif key == pygame.K_g:
            self._generate()

        elif key == pygame.K_r:
            self._reset()

        elif key == pygame.K_SPACE:
            self._solve()

        elif key == pygame.K_1:
            self.algo = Algorithm.BFS;   self.status = "Selected: BFS"
        elif key == pygame.K_2:
            self.algo = Algorithm.DFS;   self.status = "Selected: DFS"
        elif key == pygame.K_3:
            self.algo = Algorithm.ASTAR; self.status = "Selected: A*"

        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.speed = max(1, self.speed - 1)
            self.status = f"Speed: {11 - self.speed}/10"
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.speed = min(10, self.speed + 1)
            self.status = f"Speed: {11 - self.speed}/10"

    # ── Actions ────────────────────────────────

    def _generate(self):
        self.animating = False
        self.maze.generate()
        self._clear_anim()
        self.status = "New maze generated. Press SPACE to solve."

    def _reset(self):
        self.animating = False
        self.maze.reset_search()
        self._clear_anim()
        self.status = "Reset. Press SPACE to solve again."

    def _clear_anim(self):
        self.frames = []; self.path = []
        self.frame_idx = 0
        self.visited_snap = frozenset()
        self.frontier_snap = frozenset()
        self.stats = {"Algorithm": self.algo.value, "Visited": "—", "Path len": "—"}

    def _solve(self):
        if self.animating:
            return
        self.maze.reset_search()
        self._clear_anim()

        algo_map = {
            Algorithm.BFS:   bfs,
            Algorithm.DFS:   dfs,
            Algorithm.ASTAR: astar,
        }
        result = algo_map[self.algo](self.maze)
        self.frames      = result.frames
        self.path        = result.path
        self.frame_idx   = 0
        self.animating   = True
        self.stats["Algorithm"] = self.algo.value
        self.status = f"Running {self.algo.value}..."

    # ── Main Loop ──────────────────────────────

    def run(self):
        while True:
            self.handle_events()
            self._tick_animation()
            self.renderer.draw_all(
                visited  = self.visited_snap,
                frontier = self.frontier_snap,
                path     = self.path if not self.animating else [],
                algo     = self.algo,
                stats    = self.stats,
                status   = self.status,
            )
            self.clock.tick(FPS)

    def _tick_animation(self):
        if not self.animating:
            return
        self.tick += 1
        if self.tick < self.speed:
            return
        self.tick = 0

        if self.frame_idx < len(self.frames):
            self.visited_snap, self.frontier_snap = self.frames[self.frame_idx]
            self.frame_idx += 1
        else:
            # Animation complete — show final path
            self.animating = False
            self.stats["Visited"]  = len(self.visited_snap)
            self.stats["Path len"] = len(self.path) if self.path else "No path"
            self.status = (
                f"{self.algo.value} done — "
                f"visited {len(self.visited_snap)} cells, "
                f"path: {len(self.path)} steps"
                if self.path else
                f"{self.algo.value} done — no path found."
            )


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────

if __name__ == "__main__":
    App().run()
