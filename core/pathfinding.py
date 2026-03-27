import math
import heapq
import numpy as np

CELL_SIZE = 50.0  # 50cm per grid cell
GRID_SIZE = 400   # 400x400 cells (200m x 200m)
OFFSET = GRID_SIZE // 2

class OccupancyGrid:
    RESOLUTION = CELL_SIZE
    OFFSET = OFFSET
    
    def __init__(self):
        self.RESOLUTION = CELL_SIZE
        self.OFFSET = OFFSET
        # 0.0 = Unknown (0.5 in logic), but here we initialization with 0.5 (Gray RViz)
        # 0.0 = Free, 1.0 = Obstacle
        self.grid = np.full((GRID_SIZE, GRID_SIZE), 0.5, dtype=np.float32)

    def world_to_cell(self, x, z):
        cx = int(round(x / CELL_SIZE)) + OFFSET
        cz = int(round(z / CELL_SIZE)) + OFFSET
        return (cx, cz)

    def cell_to_world(self, cx, cz):
        x = (cx - OFFSET) * CELL_SIZE
        z = (cz - OFFSET) * CELL_SIZE
        return (x, z)

    def get_value(self, x, z):
        cx, cz = self.world_to_cell(x, z)
        if 0 <= cx < GRID_SIZE and 0 <= cz < GRID_SIZE:
            return self.grid[cz, cx]
        return 1.0 # Treat out of bounds as obstacle

    def mark_obstacle(self, x, z, radius=40.0):
        cx, cz = self.world_to_cell(x, z)
        c_rad = int(radius / CELL_SIZE) + 1
        
        y, x_idx = np.ogrid[-c_rad:c_rad+1, -c_rad:c_rad+1]
        mask = x_idx**2 + y**2 <= c_rad**2
        
        r_start, r_end = max(0, cz - c_rad), min(GRID_SIZE, cz + c_rad + 1)
        c_start, c_end = max(0, cx - c_rad), min(GRID_SIZE, cx + c_rad + 1)
        
        # Adjust mask if near boundaries
        m_r_start = max(0, -(cz - c_rad))
        m_r_end = m_r_start + (r_end - r_start)
        m_c_start = max(0, -(cx - c_rad))
        m_c_end = m_c_start + (c_end - c_start)
        
        self.grid[r_start:r_end, c_start:c_end] = np.maximum(
            self.grid[r_start:r_end, c_start:c_end], 
            mask[m_r_start:m_r_end, m_c_start:m_c_end].astype(np.float32)
        )

    def draw_line(self, x1, z1, x2, z2, value=0.8):
        """Draws a line on the grid by setting cell values."""
        dist = math.hypot(x2 - x1, z2 - z1)
        steps = int(dist / (self.RESOLUTION / 2))
        for i in range(steps + 1):
            t = i / max(1, steps)
            lx = x1 + (x2 - x1) * t
            lz = z1 + (z2 - z1) * t
            cx, cz = self.world_to_cell(lx, lz)
            if 0 <= cx < GRID_SIZE and 0 <= cz < GRID_SIZE:
                # Draw a bit thicker line (3x3)
                for dx in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        nx, nz = cx + dx, cz + dz
                        if 0 <= nx < GRID_SIZE and 0 <= nz < GRID_SIZE:
                            self.grid[nz, nx] = value

    def update_raycast(self, sx, sz, ex, ez, hit: bool):
        cx0, cz0 = self.world_to_cell(sx, sz)
        cx1, cz1 = self.world_to_cell(ex, ez)
        
        # Bresenham's line algorithm (NumPy optimized or just better implementation)
        # For now, keeping the logic but applying to NumPy array
        dx = abs(cx1 - cx0)
        sx_step = 1 if cx0 < cx1 else -1
        dz = -abs(cz1 - cz0)
        sz_step = 1 if cz0 < cz1 else -1
        err = dx + dz
        
        curr_x, curr_z = cx0, cz0
        while True:
            if 0 <= curr_x < GRID_SIZE and 0 <= curr_z < GRID_SIZE:
                is_end = (curr_x == cx1 and curr_z == cz1)
                if is_end and hit:
                    self.grid[curr_z, curr_x] = 1.0
                else:
                    self.grid[curr_z, curr_x] = 0.0
                    
                if is_end: break
                
                e2 = 2 * err
                if e2 >= dz:
                    err += dz
                    curr_x += sx_step
                if e2 <= dx:
                    err += dx
                    curr_z += sz_step
            else:
                break
                
        if hit:
            self.mark_obstacle(ex, ez, radius=25.0)

    def is_free(self, cx, cz):
        if 0 <= cx < GRID_SIZE and 0 <= cz < GRID_SIZE:
            return self.grid[cz, cx] < 0.9
        return False

def smooth_path(grid: OccupancyGrid, path):
    """Simple path smoothing using line-of-sight shortcuts."""
    if len(path) <= 2:
        return path
        
    smoothed = [path[0]]
    curr_idx = 0
    
    while curr_idx < len(path) - 1:
        # Try to find the furthest point we can reach in a straight line
        best_idx = curr_idx + 1
        for next_idx in range(len(path) - 1, curr_idx + 1, -1):
            if has_line_of_sight(grid, path[curr_idx], path[next_idx]):
                best_idx = next_idx
                break
        smoothed.append(path[best_idx])
        curr_idx = best_idx
        
    return smoothed

def has_line_of_sight(grid: OccupancyGrid, start_pos, end_pos):
    cx0, cz0 = grid.world_to_cell(*start_pos)
    cx1, cz1 = grid.world_to_cell(*end_pos)
    
    dx = abs(cx1 - cx0)
    sx = 1 if cx0 < cx1 else -1
    dz = -abs(cz1 - cz0)
    sy = 1 if cz0 < cz1 else -1
    err = dx + dz
    
    curr_x, curr_z = cx0, cz0
    while True:
        if not grid.is_free(curr_x, curr_z):
            return False
        if curr_x == cx1 and curr_z == cz1:
            break
        e2 = 2 * err
        if e2 >= dz:
            err += dz
            curr_x += sx
        if e2 <= dx:
            err += dx
            curr_z += sy
    return True

def a_star_search(grid: OccupancyGrid, start_pos, goal_pos):
    start_cell = grid.world_to_cell(*start_pos)
    goal_cell = grid.world_to_cell(*goal_pos)

    if not (0 <= start_cell[0] < GRID_SIZE and 0 <= start_cell[1] < GRID_SIZE):
        return []

    open_set = []
    heapq.heappush(open_set, (0, start_cell))
    came_from = {}
    g_score = {start_cell: 0}

    def heuristic(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    f_score = {start_cell: heuristic(start_cell, goal_cell)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_cell:
            path = []
            while current in came_from:
                path.append(grid.cell_to_world(*current))
                current = came_from[current]
            path.reverse()
            
            # Apply path smoothing
            return smooth_path(grid, [start_pos] + path)

        for dx, dz in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            neighbor = (current[0] + dx, current[1] + dz)
            
            if not grid.is_free(*neighbor):
                continue
                
            cost = 1.414 if dx != 0 and dz != 0 else 1.0
            tentative_g = g_score[current] + cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal_cell)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
                
    return []

