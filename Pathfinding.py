import pygame
import pygame_gui
import sys
import random
from queue import PriorityQueue
from time import perf_counter

pygame.init()

WIDTH = 780
screen = pygame.display.set_mode((WIDTH + 300, WIDTH))
manager = pygame_gui.UIManager((WIDTH + 300, WIDTH))
pygame.display.set_caption('Pathfinding Visualiser')

clock = pygame.time.Clock()
start = None
end = None
dijkstra_check, BFS_check, DFS_check, astar_check = False, False, False, True
astar_result, dijkstra_result, BFS_result, DFS_result = None, None, None, None
random_obstacles_toggle = False
stats = ""
hueristic = 1
percentage = 0

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

TOTAL_ROWS = 52

class Node():
    def __init__(self, col, row):
        self.row = row
        self.col = col
        self.width = WIDTH // TOTAL_ROWS
        self.x = row * self.width
        self.y = col * self.width
        self.prev = None
        self.neighbours = []
        self.colour = BLACK
        self.visited = False

        self.f = 0
        self.g = 0
        self.h = 0

    def pos(self):
        return (self.row, self.col)

    def reset(self):
        self.colour = BLACK

    def make_start(self):
        self.colour = YELLOW
    
    def make_end(self):
        self.colour = TURQUOISE

    def make_path(self):
        self.colour = PURPLE

    def make_barrier(self):
        self.colour = WHITE

    def is_start(self):
        return self.colour == YELLOW

    def is_end(self):
        return self.colour == TURQUOISE

    def is_barrier(self):
        return self.colour == WHITE

    def draw(self):
        pygame.draw.rect(screen, self.colour, (self.x, self.y, self.width, self.width))

    def get_neighbours(self, maze):
        self.neighbours = []
        if self.row < TOTAL_ROWS - 1:
            if not maze[self.row + 1][self.col].is_barrier():
                self.neighbours.append(maze[self.row + 1][self.col])
        if self.col < TOTAL_ROWS - 1:
            if not maze[self.row][self.col + 1].is_barrier():
                self.neighbours.append(maze[self.row][self.col + 1])
        if self.col != 0:
            if not maze[self.row][self.col - 1].is_barrier():
                self.neighbours.append(maze[self.row][self.col - 1])
        if self.row != 0:
            if not maze[self.row - 1][self.col].is_barrier():
                self.neighbours.append(maze[self.row - 1][self.col])
        return self.neighbours

    def __lt__(self, other):
        return False

def euclidean_distance(start, end):
    x = (start[0] - end[0]) ** 2
    y = (start[1] - end[1]) ** 2

    return x + y

def manhattan_distance(start, end):
    return abs(start[0] - end[0]) + abs(start[1] - end[1])

def reconstruct_path(current):
    count = 0
    while current.prev:
        count += 1
        current = current.prev
        current.make_path()
        current.draw()
    return count

maze = [[Node(i, j) for i in range(TOTAL_ROWS)] for j in range(TOTAL_ROWS)]

def astar(maze, start, end, screen, h):
    start_time = perf_counter()
    iter = 0
    expanded = 0
    open_list = PriorityQueue()
    closed_list = PriorityQueue()
    open_set = {start}
    closed_set = {start}

    start.g = start.h = start.f = 0
    open_list.put((start.f, iter, start))

    while not open_list.empty():
        current = open_list.get()[2]
        open_set.remove(current)
        current_neighbours = current.neighbours

        for neighbour in current_neighbours:
            expanded += 1
            if current == end:
                path = reconstruct_path(end)
                end.make_end()
                start.make_start()
                start.draw()
                end.draw()
                return (perf_counter() - start_time, expanded, path)

            temp_g = current.g + 1
            if temp_g < neighbour.g:
                if neighbour not in open_set and neighbour not in closed_set:
                    iter += 1
                    neighbour.prev = current
                    neighbour.g = temp_g
                    if h:
                        neighbour.h = manhattan_distance(neighbour.pos(), end.pos())  
                    else:
                        neighbour.h = euclidean_distance(neighbour.pos(), end.pos())
                    neighbour.f = neighbour.g + neighbour.h
                    open_list.put((neighbour.f, iter, neighbour))
                    open_set.add(neighbour)
                    neighbour.colour = GREEN
                    neighbour.draw()

        closed_list.put(current, iter, current.f)
        closed_set.add(current)
        current.colour = RED
        current.draw()
        pygame.display.update()

    return False

def BFS(maze, start, end, screen):
    global start_time

    start.visited = True
    expanded = 0
    queue = [start]

    while queue:
        current = queue.pop(0)
        current.colour = RED
        current.draw()
        neighbours = current.neighbours
        for neighbour in neighbours:
            expanded += 1
            if neighbour.visited:
                continue

            neighbour.visited = True
            neighbour.prev = current

            if neighbour == end:
                path = reconstruct_path(end)
                start.make_start()
                start.draw()
                return (perf_counter() - start_time, expanded, path)
            
            neighbour.colour = GREEN
            neighbour.draw()
            
            queue.append(neighbour)

        pygame.display.update()
    
    return False


def dijkstras(maze, start, end, screen):
    global start_time

    iter = 0
    expanded = 0
    open_list = PriorityQueue()
    open_list_hash = {start}
    closed_list = []

    start.g = 0

    open_list.put((start.g, expanded, start))

    while not open_list.empty():
        current = open_list.get()[2]
        open_list_hash.remove(current)
        current_neighbours = current.neighbours

        for neighbour in current_neighbours:
            expanded += 1
            if current == end:
                path = reconstruct_path(end)
                end.make_end()
                start.make_start()
                start.draw()
                end.draw()
                return (perf_counter() - start_time, iter, path)

            if current.g + 1 < neighbour.g:
                if neighbour not in open_list_hash and neighbour not in closed_list:
                    iter += 1
                    neighbour.prev = current
                    neighbour.g = current.g + 1
                    open_list.put((neighbour.g, iter, neighbour))
                    open_list_hash.add(neighbour)
                    neighbour.colour = GREEN
                    neighbour.draw()

        closed_list.append(current)
        current.colour = RED
        current.draw()
        pygame.display.update()
    
    return False

def DFS(maze, start, end, screen):
    global start_time

    stack = [start]
    start.visited = True
    expanded = 0

    while stack:
        current = stack.pop()
        neighbours = current.neighbours
        current.colour = RED
        current.draw()
        for neighbour in neighbours:
            expanded += 1
            if neighbour.visited:
                continue

            neighbour.visited = True
            neighbour.prev = current

            if neighbour == end:
                path = reconstruct_path(end)
                start.make_start()
                start.draw()
                return (perf_counter() - start_time, expanded, path)

            neighbour.colour = GREEN
            neighbour.draw()

            stack.append(neighbour)
        pygame.display.update()

    return False

def draw_grid(screen):
    for i in range(TOTAL_ROWS):
        pygame.draw.line(screen, GREY, (0, i * WIDTH//TOTAL_ROWS), (WIDTH, i * WIDTH//TOTAL_ROWS))
        for j in range(TOTAL_ROWS):
            pygame.draw.line(screen, GREY, (j * WIDTH//TOTAL_ROWS, 0), (j * WIDTH//TOTAL_ROWS, WIDTH))

def find_tile(pos, rows, width):
    space = width // rows
    col = pos[0] // space
    row = pos[1] // space

    return row, col

def reset(screen, type):
    global start, end, maze, astar_result, dijkstra_result, BFS_result, DFS_result

    if type == "soft":
        for row in maze: 
            for node in row:
                if node.colour != WHITE and not node.is_start() and not node.is_end():
                    node.colour = BLACK
                    node.draw()
                node.get_neighbours(maze)
                node.prev = None
                node.visited = False
        start.draw()
        end.draw()
    else:
        for row in maze: 
            for node in row:
                node.colour = BLACK
                node.draw()
                node.prev = None
                node.visited = False
        start = None
        end = None
    
    draw_grid(screen)

def generate_random_maze(percentage=0):
    global maze
    
    for row in maze:
        for node in row:
            if not node.is_start() and not node.is_end():
                node.colour == BLACK
                node.draw()
                if random.uniform(0, 1) < percentage/100:
                    node.prev = None
                    node.visited = False
                    node.make_barrier()
                    node.draw()
    return maze

astar_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, 0), (300, 50)),
                                             text='Use A*',
                                             manager=manager)
dijkstra_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, 70), (300, 50)),
                                             text='Use Dijkstra\'s',
                                             manager=manager)
DFS_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, 120), (300, 50)),
                                             text='Use DFS',
                                             manager=manager)
BFS_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, 170), (300, 50)),
                                             text='Use BFS',
                                             manager=manager)
hueristic_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, 45), (300, 25)),
                                             text='Change Hueristic',
                                             manager=manager)
soft_reset = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, WIDTH-30), (300, 30)),
                                             text='Soft Reset',
                                             manager=manager)
hard_reset = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, WIDTH-55), (300, 30)),
                                             text='Hard Reset',
                                             manager=manager)
random_obstacles = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((WIDTH, WIDTH-140), (300, 60)),
                                             text='Random Obstacles',
                                             manager=manager)
slider = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((WIDTH, WIDTH-85), (300, 30)), 
                                             start_value=0.0, 
                                             value_range=[0,100],
                                             manager=manager)
comparison = pygame_gui.elements.UITextBox(stats,
                                             relative_rect=pygame.Rect((WIDTH, 220), (300, WIDTH-360)),
                                             manager=manager)

def show_stats():
    global comparison

    stats = f'''
    <b>A* (manhattan hueristic)</b>
    Elapsed time (seconds): {round(astar_result[0], 3) if hueristic and astar_result else 'Nil'}
    Expanded nodes: {astar_result[1] if hueristic and astar_result else 'Nil'}
    Distance to goal: {astar_result[2] if hueristic and astar_result else 'Nil'}

    <b>A* (euclidean hueristic)</b>
    Elapsed time (seconds): {round(astar_result[0], 3) if not hueristic and astar_result else 'Nil'}
    Expanded nodes: {astar_result[1] if not hueristic and astar_result else 'Nil'}
    Distance to goal: {astar_result[2] if not hueristic and astar_result else 'Nil'}

    <b>Dijkstra</b>
    Elapsed time (seconds): {round(dijkstra_result[0], 3) if dijkstra_result else 'Nil'}
    Expanded nodes: {dijkstra_result[1] if dijkstra_result else 'Nil'}
    Distance to goal: {dijkstra_result[2] if dijkstra_result else 'Nil'}

    <b>DFS</b>
    Elapsed time (seconds): {round(DFS_result[0], 3) if DFS_result else 'Nil'}
    Expanded nodes: {DFS_result[1] if DFS_result else 'Nil'}
    Distance to goal: {DFS_result[2] if DFS_result else 'Nil'}

    <b>BFS</b>
    Elapsed time (seconds): {round(BFS_result[0], 3) if BFS_result else 'Nil'}
    Expanded nodes: {BFS_result[1] if BFS_result else 'Nil'}
    Distance to goal: {BFS_result[2] if BFS_result else 'Nil'}
    '''

    comparison = pygame_gui.elements.UITextBox(stats,
                                    relative_rect=pygame.Rect((WIDTH, 220), (300, WIDTH-360)),
                                    manager=manager)

while True:
    time_delta = clock.tick(60)/1000.0
    draw_grid(screen)
    manager.update(time_delta)

    for event in pygame.event.get():
        manager.process_events(event)
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if pygame.mouse.get_pressed()[0]:
            pos = pygame.mouse.get_pos()
            if (0 <= pos[0] <= WIDTH and 0 <= pos[1] <= WIDTH):
                row, col = find_tile(pos, TOTAL_ROWS, WIDTH)
                spot = maze[col][row]

                if not start and spot != end:
                    start = spot
                    start.make_start()
                    start.draw()

                elif not end and spot != start:
                    end = spot
                    end.make_end()
                    end.draw()

                elif spot != end and spot != start:
                    spot.make_barrier()
                    spot.draw()

        elif pygame.mouse.get_pressed()[2]:
            pos = pygame.mouse.get_pos()
            if (0 <= pos[0] <= WIDTH and 0 <= pos[1] <= WIDTH):
                row, col = find_tile(pos, TOTAL_ROWS, 800)
                spot = maze[col][row]
                spot.reset()
                spot.draw()

                if spot == start:
                    start = None
                elif spot == end:
                    end = None

        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == astar_button:
                dijkstra_check, BFS_check, DFS_check, astar_check = False, False, False, True
            if event.ui_element == dijkstra_button:
                dijkstra_check, BFS_check, DFS_check, astar_check = True, False, False, False
            if event.ui_element == BFS_button:
                dijkstra_check, BFS_check, DFS_check, astar_check = False, True, False, False
            if event.ui_element == DFS_button:
                dijkstra_check, BFS_check, DFS_check, astar_check = False, False, True, False
            if event.ui_element == hueristic_button:
                if hueristic:
                    hueristic = 0
                else:
                    hueristic = 1
            if event.ui_element == soft_reset:
                reset(screen, "soft")
            if event.ui_element == hard_reset:
                reset(screen, "hard")
                random_obstacles_toggle = False
            if event.ui_element == random_obstacles:
                if random_obstacles_toggle:
                    random_obstacles_toggle = False
                    maze = [[Node(i, j) for i in range(TOTAL_ROWS)] for j in range(TOTAL_ROWS)]
                else:
                    random_obstacles_toggle = True
                    maze = generate_random_maze(percentage)

        if event.type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
            if event.ui_element == slider:
                random_obstacles_toggle = False
                percentage = slider.current_value

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE and start != None and end != None:
                start_time = perf_counter()
                for row in maze:
                    for node in row:
                        node.f = float("inf")
                        node.g = float("inf")
                        node.get_neighbours(maze)
                if dijkstra_check:
                    dijkstra_result = dijkstras(maze, start, end, screen)
                    comparison.kill()                   
                elif BFS_check:
                    BFS_result = BFS(maze, start, end, screen)
                    comparison.kill()
                elif DFS_check:
                    DFS_result = DFS(maze, start, end, screen)
                    comparison.kill()
                elif astar_check:
                    astar_result = astar(maze, start, end, screen, hueristic)
                    comparison.kill()

            show_stats()

    manager.draw_ui(screen)
    pygame.display.update()