import pygame
import json 
import math 
from pathlib import Path
import numpy as np
from queue import PriorityQueue

ROWS = 55
CELL_PIXELS = 16
GRID_CENTER = ROWS // 2 # 27 
METERS_PER_CELL = 1.0 
WINDOW_SIZE = ROWS * CELL_PIXELS

X_GEOFENCE_LIMIT = 25.0 # North limit 
Y_GEOFENCE_LIMIT = 25.0 # East limit 

# Bezier smoothing commands 
TURN_RADIUS_CELLS = 2.0   # How far before/after a corner the curve starts/ends
MIN_RADIUS_CELLS = 0.6    # Below this, keep the corner sharp
CURVE_POINTS = 4          # Points generated inside each quadratic curve

# Epsilon (RDP)
RDP_EPSILON = 0.63

# Minimum-snap trajectory settings.  Coordinates remain in grid-cell units until
# after collision validation, so the trajectory is checked against the same map
# that A* used.
MIN_SNAP_SPEED_CELLS_PER_SEC = 1.0
MIN_SNAP_SAMPLE_SPACING_CELLS = 0.10

#COLORS 
RED = (255,0,0) # A* search
GREEN = (0,255,0) # A* search available 
SMOOTH_BLUE = (0, 120, 255)
YELLOW = (255, 255,0)
WHITE = (255, 255, 255)
BLACK = (0,0,0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0) # start 
GREY = (128, 128, 128) 
TURQUOISE = (55, 255, 245) # Way point 1
CYAN = (0, 240, 229) # Way point 2 
TOURMALINE = (0, 180, 171) # Waypoint 3 
MAP_FILE = Path(__file__).resolve().with_name("geofence_map.json")
WAYPOINT_COLORS = [TURQUOISE, CYAN, TOURMALINE]
# PYGAME WINDOW 
pygame.init()
WIN = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("UAV Geofence Planner")

class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = col * width
        self.y = row * width
        self.color = WHITE 
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
    def get_pos(self):
        return self.row, self.col 
    def is_close(self):
        return self.color == RED
    def is_open(self):
        return self.color == GREEN 
    def is_barrier(self):
        return self.color == BLACK
    def is_start(self):
        return self.color == ORANGE 
    def is_waypoint(self): # avoids indiviudal putting all the def == colors
        return self.color in WAYPOINT_COLORS
    def make_waypoint(self, waypoint_index):
        self.color = WAYPOINT_COLORS[waypoint_index]
    def reset(self):
        self.color = WHITE 
    def make_start(self):
        self.color = ORANGE
    def make_close(self):
        self.color = RED
    def make_open(self):
        self.color = GREEN
    def make_barrier(self):
        self.color = BLACK 
    
    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(
            win,
            self.color,
            (self.x, self.y, self.width, self.width)
        )

    def is_too_close_to_barrier(self, grid):
        for row_offset in (-1, 0, 1):
            for col_offset in (-1, 0, 1):
                check_row = self.row + row_offset
                check_col = self.col + col_offset

                if 0 <= check_row < self.total_rows and 0 <= check_col < self.total_rows:
                    if grid[check_row][check_col].is_barrier():
                        return True
        return False
    def update_neighbors(self, grid):
        self.neighbors = []

        # Straight Directions 
        straight_dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for dr, dc in straight_dirs:
            row, col = self.row + dr, self.col + dc 
            if 0 <= row < self.total_rows and 0 <= col < self.total_rows:
                candiate = grid[row][col]
                if not candiate.is_barrier() and not candiate.is_too_close_to_barrier(
                    grid
                ):
                    self.neighbors.append((candiate, 1))

        # Diagonal Directions
        diagonal_dirs = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dr, dc in diagonal_dirs:
            row, col = self.row + dr, self.col + dc

            if 0 <= row < self.total_rows and 0 <= col < self.total_rows:
                candidate = grid[row][col]

                side_row = grid[self.row + dr][self.col]
                side_col = grid[self.row][self.col + dc]

                if (
                    candidate.is_barrier()
                    or candidate.is_too_close_to_barrier(grid)
                    or side_row.is_barrier()
                    or side_row.is_too_close_to_barrier(grid)
                    or side_col.is_barrier()
                    or side_col.is_too_close_to_barrier(grid)
                ):
                    continue 
                self.neighbors.append((candidate, math.sqrt(2)))

    def __lt__(self, other):
        return False

def make_grid(rows, cell_pixels):
    return [
        [Spot(row, col, cell_pixels, rows) for col in range(rows)]
        for row in range(rows)
    ]

FENCE_TOP_ROW = GRID_CENTER - int(X_GEOFENCE_LIMIT) # North +25m
FENCE_BOTTOM_ROW = GRID_CENTER + int(X_GEOFENCE_LIMIT) # North -25m
FENCE_LEFT_COL = GRID_CENTER - int(Y_GEOFENCE_LIMIT) # East -25m 
FENCE_RIGHT_COL = GRID_CENTER + int(Y_GEOFENCE_LIMIT) # East +25m

# GEOFENCE LIMIT BOUNDARY 
def set_geofence_boundary(grid):
    # North and South fence lines 
    for col in range(FENCE_LEFT_COL, FENCE_RIGHT_COL+1):
        grid[FENCE_TOP_ROW][col].make_barrier()
        grid[FENCE_BOTTOM_ROW][col].make_barrier()

    # West and East Fence Limit 
    for row in range(FENCE_TOP_ROW, FENCE_BOTTOM_ROW+1):
        grid[row][FENCE_LEFT_COL].make_barrier()
        grid[row][FENCE_RIGHT_COL].make_barrier()

def is_geofence_boundary(row, col):
    return (
        (row in (FENCE_TOP_ROW, FENCE_BOTTOM_ROW)
         and FENCE_LEFT_COL <= col <= FENCE_RIGHT_COL)
        or
        (col in (FENCE_LEFT_COL, FENCE_RIGHT_COL)
         and FENCE_TOP_ROW <= row <= FENCE_BOTTOM_ROW)
    )

# Flyable cells strictly inside the fence boundary.
def is_inside_geofence(spot):
    return(
        FENCE_TOP_ROW < spot.row < FENCE_BOTTOM_ROW
        and FENCE_LEFT_COL < spot.col < FENCE_RIGHT_COL
    )

# COORDINATE SYSTEM 
def path_NorthEast(path):
    waypoints = []
    for spot in path:
        north_m = GRID_CENTER - spot.row  # row decreases upward = North
        east_m = spot.col - GRID_CENTER   # col increases rightward = East
        waypoints.append((north_m, east_m))
    return waypoints

# Simplified path output
def simplify_path(path):
    if len(path) <= 2:
        return path
    simplified = [path[0]]

    previous_direction = (
        path[1].row - path[0].row, 
        path[1].col - path[0].col 
    )

    for i in range(1, len(path) - 1):
        next_direction = (
            path[i +1].row - path[i].row, 
            path[i +1].col - path[i].col 
        )
        if next_direction != previous_direction:
            simplified.append(path[i])
        previous_direction = next_direction

    simplified.append(path[-1])
    return simplified 

""" ====== RDP and Minimum Snap ===== """
def prep_distance(point, line_start, line_end):
    """ Finds the shortest distance of the line segment"""
    px, py = point 
    x1, y1 = line_start 
    x2, y2 = line_end 

    dx = x2 - x1
    dy = y2 - y1 

# If the segment has zero length, return the distance to its single point.
    if dx == 0 and dy == 0:
        return math.hypot(px-x1, py-y1)
    # Project the point onto the segment; clamp t so it stays from start (0) to end (1).
    t = ((px - x1)* dx + (py-y1)* dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))

    closest_x = x1 + t * dx 
    closest_y = y1 + t * dy 
    return math.hypot(px - closest_x, py - closest_y)

def rdp(points, epsilon):
    """simplfy the waypoints points using RDP"""
    if len(points) < 3:
        return points 
    # Find the point with the maximum distance 
    max_dist = 0.0 
    index = 0 
    for i in range(1, len(points) -1):
        dist = prep_distance(points[i], points[0], points[-1])
        if dist > max_dist:
            max_dist = dist 
            index = i 
    # If distance > epsilon, keep the points and half split 
    if max_dist > epsilon:
        left_half = rdp(points[:index + 1], epsilon)
        right_half = rdp(points[index:], epsilon)
        # combine both half without any duplicants 
        return left_half[:-1] + right_half
    # Otherwise, delete all middle points 
    return [points[0], points[-1]]

def polynomial_basis(time_s, derivative=0):
    """Return the 7th-degree polynomial basis for one derivative order.
    p(0)    = c0
    p'(0)   = c1
    p''(0)  = 2c2
    p'''(0) = 6c3
    """
    basis = np.zeros(8)
    for power in range(derivative, 8):
        coefficient = math.factorial(power) / math.factorial(power - derivative)
        basis[power] = coefficient * time_s ** (power - derivative)
    return basis

def minimum_snap_hessian(durations):
    """Build the integrated-squared-snap cost for 7th-degree segments."""
    segment_count = len(durations)
    hessian = np.zeros((8 * segment_count, 8 * segment_count))

    for segment, duration in enumerate(durations):
        offset = 8 * segment
        for row in range(4, 8):
            for col in range(4, 8):
                row_factor = math.factorial(row) / math.factorial(row - 4)
                col_factor = math.factorial(col) / math.factorial(col - 4)
                exponent = row + col - 7
                hessian[offset + row, offset + col] = (
                    row_factor * col_factor * duration ** exponent / exponent
                )
    return hessian

def minimum_snap_constraints(waypoint_values, durations):
    """Constrain waypoint positions
    polynomial_basis(0.0, 0)  # position at t = 0 -> Start position= first waypoint
    polynomial_basis(0.0, 1)  # velocity at t = 0 -> start velo=0
    polynomial_basis(0.0, 2)  # acceleration at t = 0 -> start accel =0
    polynomial_basis(0.0, 3)  # jerk at t = 0 -> start jerk =0
    """
    segment_count = len(durations)
    variable_count = 8 * segment_count
    rows = []
    values = []

    # Start and finish at rest through jerk, plus their fixed positions.
    for derivative in range(4): # t = 0 
        row = np.zeros(variable_count)
        row[:8] = polynomial_basis(0.0, derivative) #8 coeff of first segment
        rows.append(row)
        values.append(waypoint_values[0] if derivative == 0 else 0.0)

        row = np.zeros(variable_count) # t = T
        row[-8:] = polynomial_basis(durations[-1], derivative) #8 coeff of final segment
        rows.append(row)
        values.append(waypoint_values[-1] if derivative == 0 else 0.0)

# Pin each interior position and make position, velocity, acceleration,
# and jerk continuous across its two neighbouring segments.
    for segment in range(segment_count - 1):
        current = slice(8 * segment, 8 * (segment + 1))
        following = slice(8 * (segment + 1), 8 * (segment + 2))

    # Current segment must end at this anchor position.
        row = np.zeros(variable_count)
        row[current] = polynomial_basis(durations[segment], 0)
        rows.append(row)
        values.append(waypoint_values[segment + 1])

    # Next segment must start at the same anchor position.
        row = np.zeros(variable_count)
        row[current] = polynomial_basis(durations[segment], 0)
        row[following] = -polynomial_basis(0.0, 0)
        rows.append(row)
        values.append(0.0)

    # Both segment stop at this anchor point 
    # Both segment ends/begin with velocity/acceleration/jerk = 0.
        for derivative in range(1, 4): # (0 position,1 velocity,2 accel,3 jerk)  
            row = np.zeros(variable_count)
            row[current] = polynomial_basis(durations[segment], derivative)
            rows.append(row)
            values.append(0.0)

            row = np.zeros(variable_count)
            row[following] = polynomial_basis(0.0, derivative)
            rows.append(row)
            values.append(0.0)

    return np.asarray(rows), np.asarray(values)

def solve_minimum_snap(route_grid, speed=MIN_SNAP_SPEED_CELLS_PER_SEC):
    """Solve a local, unconstrained 7th-degree minimum-snap trajectory.

    ``route_grid`` is a sequence of (row, col) coordinates.  This deliberately
    has no dependency on snap.py or minimum_snap.py.
    """
    waypoints = np.asarray(route_grid, dtype=float)
    if len(waypoints) < 2:
        return waypoints, np.empty(0)

    distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    durations = np.maximum(distances / speed, 0.1)
    hessian = minimum_snap_hessian(durations)
    constraint_matrix, _ = minimum_snap_constraints(waypoints[:, 0], durations)
    variable_count = hessian.shape[0]

    # KKT solve: min 1/2 c'Qc subject to Ac=b.  A tiny diagonal term only
    # resolves the null space of the snap objective; equality constraints still
    # determine the physical trajectory.
    kkt = np.block([
        [hessian + np.eye(variable_count) * 1e-9, constraint_matrix.T],
        [constraint_matrix, np.zeros((len(constraint_matrix), len(constraint_matrix)))],
    ])
    coefficients = []
    for dimension in range(2):
        _, constraint_values = minimum_snap_constraints(
            waypoints[:, dimension], durations
        )
        rhs = np.concatenate((np.zeros(variable_count), constraint_values))
        try:
            solution = np.linalg.solve(kkt, rhs)
        except np.linalg.LinAlgError as error:
            raise RuntimeError("Minimum-snap KKT system is singular") from error
        coefficients.append(solution[:variable_count].reshape(-1, 8))

    return np.stack(coefficients, axis=-1), durations

def sample_minimum_snap(coefficients, durations):
    """Sample each polynomial densely enough for grid collision validation."""
    samples = []
    for segment, duration in enumerate(durations):
        distance_scale = max(duration * MIN_SNAP_SPEED_CELLS_PER_SEC, 1.0)
        count = max(2, math.ceil(distance_scale / MIN_SNAP_SAMPLE_SPACING_CELLS))
        for time_s in np.linspace(0.0, duration, count, endpoint=False):
            samples.append(tuple(polynomial_basis(time_s) @ coefficients[segment]))

    samples.append(tuple(polynomial_basis(durations[-1]) @ coefficients[-1]))
    return samples

def get_grid_line_cells(start_point, end_point):
    row, col = start_point 
    end_row, end_col = end_point

    row_distance = abs(end_row - row)
    col_distance = abs(end_col - col)

    row_step = 1 if row < end_row else -1
    col_step = 1 if col < end_col else -1 
    error = col_distance - row_distance
    cells = []

    while True: 
        cells.append((row, col))
        if row == end_row and col == end_col:
            return cells 
        twice_error = 2 * error 
        if twice_error > -row_distance:
            error -= row_distance
            col += col_step
        if twice_error < col_distance:
            error += col_distance
            row += row_step

def is_safe_grid_point(point, grid):
    """Check a floating-point grid coordinate against obstacles and geofence."""
    row, col = point
    check_row = round(row)
    check_col = round(col)
    if not (0 <= check_row < len(grid) and 0 <= check_col < len(grid)):
        return False

    spot = grid[check_row][check_col]
    return is_inside_geofence(spot) and not spot.is_too_close_to_barrier(grid)

def is_safe_continuous_segment(start_point, end_point, grid):
    for cell in get_supercover_line_cells(start_point, end_point):
        if not is_safe_grid_point(cell, grid):
            return False

    return True

def is_safe_minimum_snap_path(samples, grid):
    """Validate all samples and the grid cells crossed between samples."""
    for previous, current in zip(samples, samples[1:]):
        if not is_safe_grid_point(previous, grid):
            return False, previous

        if not is_safe_continuous_segment(previous, current, grid):
            return False, current

    if samples and not is_safe_grid_point(samples[-1], grid):
        return False, samples[-1]
    return True, None

def get_supercover_line_cells(start_point, end_point):
    """Return every grid cell touched by a continuous line segment."""
    
    start_row, start_col = start_point 
    end_row, end_col = end_point 

    row_delta = end_row - start_row
    col_delta = end_col - start_col 
# Determine which direction the line moves through rows and columns.
# row_step = up/down/no row movement
    if row_delta > 0:
        row_step = 1
    elif row_delta < 0:
        row_step = -1
    else: 
        row_step = 0
# col_step = left/right/no column movement
    if col_delta > 0:
        col_step = 1
    elif col_delta < 0:
        col_step = -1
    else:
        col_step = 0

    current_row = math.floor(start_row + 0.5)
    current_col = math.floor(start_col + 0.5)

    end_cell_row = math.floor(end_row + 0.5)
    end_cell_col = math.floor(end_col + 0.5)
    cells = [(current_row, current_col)]

    if row_step != 0:
        next_row_boundary = current_row + 0.5 * row_step
        t_max_row = (
            (next_row_boundary - start_row) / row_delta
        )
        t_delta_row = 1.0 / abs(row_delta) 
    else: 
        t_max_row = math.inf 
        t_delta_row = math.inf 

    if col_step != 0:
        next_col_boundary = current_col + 0.5 * col_step 
        t_max_col = (
            (next_col_boundary - start_col) / col_delta
        )
        t_delta_col = 1.0 / abs(col_delta)
    else: 
        t_max_col = math.inf 
        t_delta_col = math.inf  #math.inf means positive infinity.

    # which boundary does the line reach first
    while ( 
        current_row != end_cell_row 
        or current_col != end_cell_col
    ):
        if t_max_row < t_max_col:
            # The line reaches a row boundary first 
            current_row += row_step 
            cells.append((current_row, current_col))
            t_max_row += t_delta_row 
        elif t_max_col < t_max_row:
            # THe line reaches a column boundary first 
            current_col += col_step 
            cells.append((current_row, current_col))
            t_max_col += t_delta_col 
        else: #The line hits both at the same time 
            # include every box that it touches 
            next_row = current_row + row_step 
            next_col = current_col + col_step 

            cells.append((next_row, current_col))
            cells.append((current_row, next_col))

            current_row = next_row 
            current_col = next_col 
            cells.append((current_row, current_col))

            t_max_row += t_delta_row
            t_max_col += t_delta_col
    return cells 

""" ===== END OF RDP AND minimum SNAP ====== """
# North/East waypoints in meters; center as (0, 0).
def smooth_path_NorthEast(smooth_path):
    waypoints = []

    for row, col in smooth_path:
        north_m = GRID_CENTER - row
        east_m = col - GRID_CENTER
        waypoints.append((
            float(round(north_m, 2)),
            float(round(east_m, 2)),
        ))
    return waypoints

# Draws the grey 1-meter grid lines over the Pygame window.
def draw_grid(win):
    for i in range(ROWS + 1):
        pygame.draw.line(
            win, GREY,
            (0, i * CELL_PIXELS),
            (WINDOW_SIZE, i * CELL_PIXELS)
        )
        pygame.draw.line(
            win, GREY, 
            (i * CELL_PIXELS, 0),
            (i * CELL_PIXELS, WINDOW_SIZE)
        )

# Clears the screen, draw grid plot 
def draw_smoothed_paths(win, smooth_paths):
    for smooth_path in smooth_paths:
        if len(smooth_path) < 2:
            continue

        pixel_points = [
            (
                int(col * CELL_PIXELS + CELL_PIXELS / 2),
                int(row * CELL_PIXELS + CELL_PIXELS / 2)
            )
            for row, col in smooth_path
        ]

        pygame.draw.lines(win, SMOOTH_BLUE, False, pixel_points, 4)


def draw(win, grid, smooth_paths=None):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win)

    if smooth_paths:
        draw_smoothed_paths(win, smooth_paths)

    pygame.display.update()

# Saves the cells to the JSON map file.
def save_map(grid, start, waypoints, mission_waypoints_ne=None):
    barriers = []
    for row in grid:
        for spot in row:
            if spot.is_barrier():
                barriers.append(spot.get_pos())

    data = {
        "start": start.get_pos(),
        "waypoints": [waypoint.get_pos() for waypoint in waypoints],
        "barriers": barriers
    }

    if mission_waypoints_ne is not None:
        data["mission_waypoints_ne"] = [
            [float(north), float(east)]
            for north, east in mission_waypoints_ne
        ]

    with open(MAP_FILE, "w") as file:
        json.dump(data, file)
    print("Mission map saved.")

# Loads previous cells from the JSON map file
def load_map(rows, cell_pixels):
    grid = make_grid(rows, cell_pixels)

    with open(MAP_FILE, "r") as file:
        data = json.load(file)

    start_row, start_col = data["start"]
    start = grid[start_row][start_col]
    start.make_start()

    saved_waypoints = data.get("waypoints")

    if saved_waypoints is None:
        saved_waypoints = [data["end"]]  # supports an old single-goal save file

    waypoints = []
    for index, (row, col) in enumerate(saved_waypoints):
        if index >= len(WAYPOINT_COLORS):
            break

        waypoint = grid[row][col]
        waypoint.make_waypoint(index)
        waypoints.append(waypoint)

    for row, col in data["barriers"]:
        grid[row][col].make_barrier()

    set_geofence_boundary(grid)

    # Restore colors in case a saved barrier/search color overwrote them.
    start.make_start()
    for index, waypoint in enumerate(waypoints):
        waypoint.make_waypoint(index)

    print("Mission map loaded.")
    return grid, start, waypoints

# Converts mouse position into a grid row and column.
def get_clicked_pos(pos, rows, width):
    gap = width // rows 
    x,y  = pos

    row = y // gap 
    col = x // gap 
    return row, col 

# Manhattan-dist heuristic
def h(p1, p2):
    row1, col1 = p1 
    row2, col2 = p2 
    dr = abs(row1 - row2)
    dc = abs(col1 - col2)

    diagonal_steps = min(dr, dc) 
    straight_steps = max(dr, dc) - diagonal_steps 

    return straight_steps + math.sqrt(2) * diagonal_steps

# Traces the route from goal back to start
def reconstruct_path(came_from , current, draw_callback):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()

    for spot in path[1:-1]:
        spot.make_path()
        draw_callback()

    return path

# Finds a collision-free path from start to end;  core of A* algorithm
def algorithm(draw_callback, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))

    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0

    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False
        current = open_set.get()[2]

        if current == end:
            path = reconstruct_path(came_from, current, draw_callback)
            return path

        for neighbor, movement_cost in current.neighbors:
            tentative_g_score = g_score[current] + movement_cost
            
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + h(
                    neighbor.get_pos(),
                    end.get_pos()
                )
                count += 1
                open_set.put((f_score[neighbor], count, neighbor))
                neighbor.make_open()

        draw_callback()
        if current != start:
            current.make_close()

    return False

# Returns True when a curve point is inside the grid and safely clear of obstacles.
def is_safe_curve_point(point, grid):
    row, col = point
    check_row = round(row)
    check_col = round(col)

    if not (0 <= check_row < len(grid) and 0 <= check_col < len(grid)):
        return False
    spot = grid[check_row][check_col]
    return (
        not spot.is_barrier()
        and not spot.is_too_close_to_barrier(grid)
    )

# Quadratic Bezier curve: (3-point curve)
def generate_smooth_turn(p0, p1, p2, num_points):
    smooth_points = []

    for i in range(1, num_points + 1):
        t = i / (num_points + 1)

        row = (
            (1 - t) ** 2 * p0[0]
            + 2 * (1 - t) * t * p1[0]
            + t ** 2 * p2[0]
        )
        col = (
            (1 - t) ** 2 * p0[1]
            + 2 * (1 - t) * t * p1[1]
            + t ** 2 * p2[1]
        )

        smooth_points.append((row, col))
    return smooth_points

# conver every sharp corner into a bezier turn 
def make_smoothed_path(simplified_path, grid):
    if len(simplified_path) <= 2:
        return [(spot.row, spot.col) for spot in simplified_path]

    smooth_path = [
        (simplified_path[0].row, simplified_path[0].col)
    ]

    for i in range(1, len(simplified_path) - 1):
        prev = simplified_path[i - 1]
        corner = simplified_path[i]
        next_spot = simplified_path[i + 1]

        in_row = corner.row - prev.row
        in_col = corner.col - prev.col
        out_row = next_spot.row - corner.row
        out_col = next_spot.col - corner.col

        # No direction change: keep moving straight.
        cross_product = in_row * out_col - in_col * out_row
        if cross_product == 0:
            smooth_path.append((corner.row, corner.col))
            continue

        in_length = math.hypot(in_row, in_col)
        out_length = math.hypot(out_row, out_col)

        radius = min(
            TURN_RADIUS_CELLS,
            0.4 * in_length,
            0.4 * out_length
        )

        if radius < MIN_RADIUS_CELLS:
            smooth_path.append((corner.row, corner.col))
            continue

        entry = (
            corner.row - radius * in_row / in_length,
            corner.col - radius * in_col / in_length
        )

        exit_point = (
            corner.row + radius * out_row / out_length,
            corner.col + radius * out_col / out_length
        )

        curve = generate_smooth_turn(
            entry,
            (corner.row, corner.col),
            exit_point,
            CURVE_POINTS
        )

        candidate_curve = [entry] + curve + [exit_point]

        # Only accept a curve if every sample remains safely clear.
        if all(is_safe_curve_point(point, grid) for point in candidate_curve):
            smooth_path.extend(candidate_curve)
        else:
            smooth_path.append((corner.row, corner.col))

    smooth_path.append(
        (simplified_path[-1].row, simplified_path[-1].col)
    )
    return smooth_path

# does the WP in order, 
def plan_mission(draw_callback, grid, start, waypoints):

    mission_wp_ne = []
    display_paths = []
    all_rdp_clear = True
    all_minimum_snap_clear = True
    mission_complete = True

    current_start = start

    for waypoint_number, goal in enumerate(waypoints, start=1):
        if not is_inside_geofence(goal):
            print(f"Waypoint {waypoint_number} is outside the geofence. Skipped.")
            mission_complete = False
            continue

        path = algorithm(draw_callback, grid, current_start, goal)

        if not path:
            print(f"Waypoint {waypoint_number} cannot be reached. Mission stopped.")
            mission_complete = False
            break

        path_points = [(spot.row, spot.col) for spot in path]
        rdp_points = rdp(path_points, RDP_EPSILON)

        rdp_is_clear = all(
            is_safe_continuous_segment(a, b, grid)
            for a, b in zip(rdp_points, rdp_points[1:])
        )

        if rdp_is_clear:
            route_grid = rdp_points
            mission_route_grid = rdp_points
            try:
                coefficients, durations = solve_minimum_snap(route_grid)
                minimum_snap_path = sample_minimum_snap(coefficients, durations)
                minimum_snap_is_safe, unsafe_point = is_safe_minimum_snap_path(
                    minimum_snap_path, grid
                )
            except RuntimeError as error:
                minimum_snap_is_safe = False
                unsafe_point = None
                print(f"Leg {waypoint_number}: minimum-snap solver failed: {error}")

            if minimum_snap_is_safe:
                route_grid = minimum_snap_path
                print(
                    f"Leg {waypoint_number}: using safe 7th-degree minimum-snap "
                    f"trajectory ({len(route_grid)} samples)."
                )
            else:
                # An unconstrained polynomial may round an obstacle even though
                # every straight RDP segment is clear.  The RDP route remains
                # the trusted, collision-free fallback.
                all_minimum_snap_clear = False
                if unsafe_point is not None:
                    print(
                        f"Leg {waypoint_number}: minimum-snap leaves the safe "
                        f"corridor near row={unsafe_point[0]:.2f}, "
                        f"col={unsafe_point[1]:.2f}; using RDP fallback."
                    )
                else:
                    print(
                        f"Leg {waypoint_number}: minimum-snap unavailable; "
                        "using RDP fallback."
                    )
        else:
            # Keep the already-safe A* route, with Bézier turns only where safe.
            simplified_path = simplify_path(path)
            route_grid = make_smoothed_path(simplified_path, grid)
            mission_route_grid = route_grid
            all_rdp_clear = False
            print(
                f"Leg {waypoint_number}: RDP shortcut unsafe; "
                "using safe A*/Bézier fallback."
            )

        display_paths.append(route_grid)
        segment_ne = smooth_path_NorthEast(mission_route_grid)

        if mission_wp_ne:
            mission_wp_ne.extend(segment_ne[1:])
        else:
            mission_wp_ne.extend(segment_ne)

        current_start = goal

    start.make_start()
    for index, waypoint in enumerate(waypoints):
        waypoint.make_waypoint(index)

    return (
        mission_wp_ne,
        display_paths,
        all_rdp_clear,
        all_minimum_snap_clear,
        mission_complete,
    )

def main(win, width):
    grid = make_grid(ROWS, CELL_PIXELS)
    set_geofence_boundary(grid)

    start = None
    waypoints = []
    run = True
    smooth_paths = []
    while run:
        draw(win, grid, smooth_paths)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # left-click / drag
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)

                spot = grid[row][col]
                if is_inside_geofence(spot):
                    if not start:
                        start = spot
                        start.make_start()
                    elif (
                        len(waypoints) < len(WAYPOINT_COLORS)
                        and spot != start
                        and spot not in waypoints
                    ):
                        waypoint_index = len(waypoints)
                        waypoints.append(spot)
                        spot.make_waypoint(waypoint_index)
                    elif spot != start and spot not in waypoints:
                        spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # right-click / drag
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)

                spot = grid[row][col]
                if is_inside_geofence(spot):

                    if spot == start:
                        spot.reset()
                        start = None
                    elif spot in waypoints:
                        waypoints.remove(spot)
                        spot.reset()
                        # Re-label remaining waypoints in sequence.
                        for index, waypoint in enumerate(waypoints):
                            waypoint.make_waypoint(index)
                    else:
                        spot.reset()

            # S: SAVE, L:LOADS, C:CLEARS
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and start:
                    save_map(grid, start, waypoints)
                # In the L branch
                elif event.key == pygame.K_l:
                    try:
                        grid, start, waypoints = load_map(ROWS, CELL_PIXELS)
                        smooth_paths = []
                    except FileNotFoundError:
                        print("No saved geofence map found yet.")
                # In the C branch:
                elif event.key == pygame.K_c:
                    grid = make_grid(ROWS, CELL_PIXELS)
                    set_geofence_boundary(grid)
                    start = None
                    waypoints = []
                    smooth_paths = []
                elif event.key == pygame.K_SPACE and start and waypoints:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)

                    (
                        mission_wp_ne,
                        smooth_paths,
                        all_rdp_clear,
                        all_minimum_snap_clear,
                        mission_complete,
                    ) = plan_mission(
                        lambda: draw(win, grid),
                        grid,
                        start,
                        waypoints
                    )

                    if all_rdp_clear:
                        print("All mission legs use safe RDP shortcuts.")
                    else:
                        print("One or more RDP shortcuts were unsafe; safe fallback used.")

                    if not all_minimum_snap_clear:
                        print("One or more minimum-snap legs were unsafe; RDP fallback used.")

                    print(f"Mission waypoints: {len(mission_wp_ne)} anchors")
                    print(f"Start: {mission_wp_ne[0]}")
                    print(f"End:   {mission_wp_ne[-1]}")

                    if mission_complete:
                        save_map(grid, start, waypoints, mission_wp_ne)
                        print("Generated mission route saved for flight.")
                    else:
                        print("Generated route was incomplete; flight route was not saved.")

    pygame.quit()

main(WIN, WINDOW_SIZE)
