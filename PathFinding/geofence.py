import pygame
import json 
import math 
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
MAP_FILE = "geofence_map.json"
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
        #DOWN
        if self.row < self.total_rows - 1:
            candidate = grid[self.row + 1][self.col]
            if not candidate.is_barrier() and not candidate.is_too_close_to_barrier(grid):
                self.neighbors.append(candidate)
        #UP
        if self.row > 0:
            candidate = grid[self.row -1][self.col]
            if not candidate.is_barrier() and not candidate.is_too_close_to_barrier(grid):
                self.neighbors.append(candidate)
        #RIGHT 
        if self.col < self.total_rows - 1: 
            candidate = grid[self.row][self.col + 1]
            if not candidate.is_barrier() and not candidate.is_too_close_to_barrier(grid):
                self.neighbors.append(candidate)
        #LEFT
        if self.col > 0:
            candidate = grid[self.row][self.col - 1]
            if not candidate.is_barrier() and not candidate.is_too_close_to_barrier(grid):
                self.neighbors.append(candidate)
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

# North/East waypoints in meters; center as (0, 0).
def smooth_path_NorthEast(smooth_path):
    waypoints = []

    for row, col in smooth_path:
        north_m = GRID_CENTER - row
        east_m = col - GRID_CENTER
        waypoints.append((
            round(north_m, 2),
            round(east_m, 2),
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
def save_map(grid, start, waypoints):
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
    return abs(row1 - row2) + abs(col1 - col2)

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

        for neighbor in current.neighbors:
            tentative_g_score = g_score[current] + 1
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
    current_start = start
    mission_wp_ne = []
    mission_smooth_paths = []

    for waypoints_number, goal in enumerate(waypoints, start=1):
        if not is_inside_geofence(goal):
            print(f"Waypoint {waypoints_number} is outside the geofence. Skipped")
            continue 

        path = algorithm(draw_callback, grid, current_start, goal)

        if not path:
            print(f"Waypoint {waypoints_number} cannot be reached. Mission stopped.")
            break

        simplified_path = simplify_path(path)

        smooth_path = make_smoothed_path(simplified_path, grid)
        mission_smooth_paths.append(smooth_path)
        segment_ne = smooth_path_NorthEast(smooth_path)

        if mission_wp_ne:
            mission_wp_ne.extend(segment_ne[1:])
        else:
            mission_wp_ne.extend(segment_ne) 

        print(f"Waypoint {waypoints_number} route: ")
        print(segment_ne) 

        current_start = goal 

    start.make_start()
    for index, waypoint in enumerate(waypoints):
        waypoint.make_waypoint(index)

    return mission_wp_ne, mission_smooth_paths

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

                if not is_geofence_boundary(row, col):
                    spot = grid[row][col]
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

                if not is_geofence_boundary(row, col):
                    spot = grid[row][col]

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
                    mission_wp_ne, smooth_paths = plan_mission(
                        lambda: draw(win, grid),
                        grid,
                        start,
                        waypoints
                    )

                    print("Full mission NE waypoints (m)")
                    print(mission_wp_ne)
    pygame.quit()

main(WIN, WINDOW_SIZE)