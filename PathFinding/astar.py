import pygame
import json 
import math 
from queue import PriorityQueue

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

TURN_RADIUS_CELLS = 2.0
MIN_RADIUS_CELLS = 0.6
CURVE_POINTS = 4
SMOOTH_BLUE = (0, 120, 255) # this is the drone path 
LAMBDA_TURN = 2

RED = (255,0,0) # A* search
GREEN = (0,255,0) # A* search available 
BLUE = (0, 255, 0)
YELLOW = (255, 255,0)
WHITE = (255, 255, 255)
BLACK = (0,0,0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0) # start 
GREY = (128, 128, 128) 
TURQUOISE = (64, 224, 208) # end 
MAP_FILE = "fixed_test_map.json"

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

    def is_end(self):
        return self.color == TURQUOISE 

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

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))
    
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

    def __it__(self, other):
        return False

def h(p1, p2):
    x1, y1 = p1 
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 -y2)

def reconstruct_path(came_from, current, draw):
    path = [current]  # starts at the goal

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()  # now: start -> goal

    for spot in path[1:-1]:
        spot.make_path()
        draw()

    return path
def reconstruct_state_path(came_from, current_state, draw):
    path = []

    while current_state in came_from:
        path.append(current_state[0])
        current_state = came_from[current_state]

    path.append(current_state[0])
    path.reverse()

    for spot in path[1:-1]:
        spot.make_path()
        draw()

    return path

def get_turn_cost(previous_direction, new_direction):
    if previous_direction is None:
        return 0.0

    if previous_direction == new_direction:
        return 0.0

    # All normal grid direction changes are 90 degrees.
    return LAMBDA_TURN * (90 / 180) ** 2

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
            path[i + 1].row - path[i].row,
            path[i + 1].col - path[i].col
        )
        if next_direction != previous_direction:
            simplified.append(path[i])
        previous_direction = next_direction
    simplified.append(path[-1])
    return simplified

CELL_SIZE_M = 1.0

def calculate_motion_cost(path, lambda_turn):
    path_length_m = 0.0
    turn_penalty = 0.0
    turns = []

    # Total route distance
    for i in range(len(path) - 1):
        row_change = path[i + 1].row - path[i].row
        col_change = path[i + 1].col - path[i].col

        segment_length_cells = math.sqrt(
            row_change ** 2 + col_change ** 2
        )
        path_length_m += segment_length_cells * CELL_SIZE_M
    # Turn angle at every middle waypoint
    for i in range(1, len(path) - 1):
        previous = path[i - 1]
        corner = path[i]
        next_spot = path[i + 1]

        vector_in = (
            corner.row - previous.row,
            corner.col - previous.col
        )
        vector_out = (
            next_spot.row - corner.row,
            next_spot.col - corner.col
        )

        length_in = math.sqrt(
            vector_in[0] ** 2 + vector_in[1] ** 2
        )
        length_out = math.sqrt(
            vector_out[0] ** 2 + vector_out[1] ** 2
        )
        dot_product = (
            vector_in[0] * vector_out[0]
            + vector_in[1] * vector_out[1]
        )
        cos_angle = dot_product / (length_in * length_out)
        cos_angle = max(-1, min(1, cos_angle))

        angle_degrees = math.degrees(math.acos(cos_angle))

        normalized_turn_cost = (angle_degrees / 180) ** 2
        turn_penalty += normalized_turn_cost

        turns.append((corner.get_pos(), angle_degrees))
    motion_cost = path_length_m + lambda_turn * turn_penalty

    return path_length_m, turn_penalty, motion_cost, turns

def path_to_north_east(path, start):
    waypoints = []

    for spot in path:
        north_m = (start.row - spot.row) * CELL_SIZE_M
        east_m = (spot.col - start.col) * CELL_SIZE_M
        waypoints.append((north_m, east_m))

    return waypoints

def smooth_path_to_north_east(smooth_path, start):
    waypoints = []

    for row, col in smooth_path:
        north_m = (start.row - row) * CELL_SIZE_M
        east_m = (col - start.col) * CELL_SIZE_M

        waypoints.append((
            round(north_m, 2),
            round(east_m, 2)
        ))

    return waypoints

def is_safe_curve_point(point, grid):
    row, col = point 
    check_row = round(row)
    check_col = round(col)

    if not (0 <= check_row < len(grid) and 0 <= check_col < len(grid)):
        return False 

    spot = grid[check_row][check_col]

    return( 
        not spot.is_barrier()
        and not spot.is_too_close_to_barrier(grid)
    )

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

def make_smoothed_path(simplified_path, grid):
    smooth_path = [
        (simplified_path[0].row, simplified_path[0].col)
    ]

    for i in range(1, len(simplified_path) - 1):
        previous = simplified_path[i - 1]
        corner = simplified_path[i]
        next_spot = simplified_path[i + 1]

        in_row = corner.row - previous.row
        in_col = corner.col - previous.col
        out_row = next_spot.row - corner.row
        out_col = next_spot.col - corner.col

        in_length = math.hypot(in_row, in_col)
        out_length = math.hypot(out_row, out_col)

        radius = min(
            TURN_RADIUS_CELLS,
            0.4 * in_length,
            0.4 * out_length
        )

        # Not enough physical space: leave this corner sharp.
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

        if all(is_safe_curve_point(point, grid) for point in candidate_curve):
            smooth_path.extend(candidate_curve)
        else:
            smooth_path.append((corner.row, corner.col))

    smooth_path.append(
        (simplified_path[-1].row, simplified_path[-1].col)
    )

    return smooth_path
def draw_smoothed_path(win, smooth_path, rows, width):
    if len(smooth_path) < 2:
        return

    gap = width // rows

    pixel_points = [
        (
            int(col * gap + gap / 2),
            int(row * gap + gap / 2)
        )
        for row, col in smooth_path
    ]

    pygame.draw.lines(win, SMOOTH_BLUE, False, pixel_points, 4)

def algorithm(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()

    # State = (spot, direction used to arrive at that spot)
    start_state = (start, None)
    open_set.put((h(start.get_pos(), end.get_pos()), count, start_state))

    came_from = {}
    g_score = {start_state: 0}
    closed_states = set()

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current_state = open_set.get()[2]

        if current_state in closed_states:
            continue

        current, previous_direction = current_state

        if current == end:
            raw_path = reconstruct_state_path(
                came_from,
                current_state,
                draw
            )
            end.make_end()

            simplified_path = simplify_path(raw_path)
            smooth_path = make_smoothed_path(simplified_path, grid)

            path_length_m, turn_penalty, motion_cost, turns = calculate_motion_cost(
                simplified_path,
                LAMBDA_TURN
            )
            print("Lambda:", LAMBDA_TURN)
            print("Path length (m):", round(path_length_m, 2))
            print("Turn penalty:", round(turn_penalty, 3))
            print("Motion cost M:", round(motion_cost, 2))

            print("Turns:")
            for position, angle in turns:
                print(f"  Corner {position}: {angle:.1f} degrees")

            north_east_waypoints = path_to_north_east(
                simplified_path,
                start
            )

            smoothed_north_east_waypoints = smooth_path_to_north_east(
                smooth_path,
                start
            )
            print("Smoothed North/East waypoints (m):")
            print(smoothed_north_east_waypoints)

            return smooth_path

        closed_states.add(current_state)

        for neighbor in current.neighbors:
            new_direction = (
                neighbor.row - current.row,
                neighbor.col - current.col
            )

            neighbor_state = (neighbor, new_direction)

            movement_cost = 1 + get_turn_cost(
                previous_direction,
                new_direction
            )

            temp_g_score = g_score[current_state] + movement_cost

            if temp_g_score < g_score.get(neighbor_state, float("inf")):
                came_from[neighbor_state] = current_state
                g_score[neighbor_state] = temp_g_score

                count += 1
                f_score = temp_g_score + h(
                    neighbor.get_pos(),
                    end.get_pos()
                )

                open_set.put((f_score, count, neighbor_state))
                neighbor.make_open()

        draw()

        if current != start:
            current.make_close()

    return False

def make_grid(rows, width):
    grid = []
    gap = width // rows 
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)
    return grid
def save_map(grid, start, end):
    barriers = []

    for row in grid:
        for spot in row:
            if spot.is_barrier():
                barriers.append(spot.get_pos())

    data = {
        "start": start.get_pos(),
        "end": end.get_pos(),
        "barriers": barriers
    }

    with open(MAP_FILE, "w") as file:
        json.dump(data, file)

    print("Fixed map saved.")


def load_map(rows, width):
    grid = make_grid(rows, width)

    with open(MAP_FILE, "r") as file:
        data = json.load(file)

    start_row, start_col = data["start"]
    start = grid[start_row][start_col]
    start.make_start()

    end_row, end_col = data["end"]
    end = grid[end_row][end_col]
    end.make_end()

    for row, col in data["barriers"]:
        grid[row][col].make_barrier()

    print("Fixed map loaded.")
    return grid, start, end

def draw_grid(win, rows, width):
    gap = width // rows 
    for i in range(rows):
        pygame.draw.line(win, GREY, (0,i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j* gap, 0), (j * gap, width))

def draw(win, grid, rows, width, smooth_path=None):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)

    if smooth_path:
        draw_smoothed_path(win, smooth_path, rows, width)

    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    gap = width // rows 
    x,y  = pos

    row = y // gap 
    col = x // gap 
    return row, col 

def main(win, width):
    ROWS = 50 
    try:
        grid, start, end = load_map(ROWS, width)
    except FileNotFoundError:
        grid = make_grid(ROWS, width)
        start = None
        end = None
    smooth_path = []

    run = True 
    started = False 
    while run: 
        draw(win, grid, ROWS, width, smooth_path)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False 

            if started:
                continue 

            if pygame.mouse.get_pressed()[0]:  # left click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                # this is make the start, end and barrier: 
                if not start and spot != end:
                    start = spot
                    start.make_start()
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # right click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()

                if spot == start:
                    start = None
                elif spot == end:
                    end = None
                        
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started and start and end:
                    for row in grid: 
                        for spot in row: 
                            spot.update_neighbors(grid)

                    smooth_path = algorithm(
                        lambda: draw(win, grid, ROWS, width),
                        grid,
                        start,
                        end
                    ) or []
                if event.key == pygame.K_c:
                    start = None 
                    end = None
                    grid = make_grid(ROWS, width)
                    smooth_path = []
                if event.key == pygame.K_s and start and end:
                    save_map(grid, start, end)

                if event.key == pygame.K_l:
                    grid, start, end = load_map(ROWS, width)
                    smooth_path = []
    pygame.quit()

main(WIN, WIDTH)