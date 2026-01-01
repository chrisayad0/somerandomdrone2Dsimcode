import pygame
import heapq
import random
import math

# Constants
WIDTH, HEIGHT = 600, 600
GRID_SIZE = 20
CELL_SIZE = WIDTH // GRID_SIZE

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)

class DroneSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("DroneAI Simulator")
        self.clock = pygame.time.Clock()
        self.reset()

    def reset(self):
        self.grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.start = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
        self.goal = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
        while self.goal == self.start:
            self.goal = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
        
        # Add random obstacles (20% density)
        for _ in range(int(GRID_SIZE * GRID_SIZE * 0.2)):
            x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
            if (x, y) != self.start and (x, y) != self.goal:
                self.grid[y][x] = 1  # 1 = obstacle
        
        self.path = self.a_star(self.start, self.goal)
        self.drone_pos = list(self.start)
        self.current_waypoint = 0 if self.path else -1
        self.drone_speed = 0.1  # Cells per frame

    def a_star(self, start, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4 directions
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and self.grid[neighbor[1]][neighbor[0]] == 0:
                    tentative_g = g_score[current] + 1
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path

    def update_drone(self):
        if self.current_waypoint < len(self.path) - 1:
            target = self.path[self.current_waypoint + 1]
            dx = target[0] - self.drone_pos[0]
            dy = target[1] - self.drone_pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            if dist < self.drone_speed:
                self.current_waypoint += 1
            else:
                self.drone_pos[0] += (dx / dist) * self.drone_speed
                self.drone_pos[1] += (dy / dist) * self.drone_speed

    def draw(self):
        self.screen.fill(WHITE)
        
        # Draw grid and obstacles
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                if self.grid[y][x] == 1:
                    pygame.draw.rect(self.screen, BLACK, rect)
                pygame.draw.rect(self.screen, BLACK, rect, 1)
        
        # Draw start and goal
        pygame.draw.circle(self.screen, GREEN, (self.start[0] * CELL_SIZE + CELL_SIZE // 2, self.start[1] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 3)
        pygame.draw.circle(self.screen, RED, (self.goal[0] * CELL_SIZE + CELL_SIZE // 2, self.goal[1] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 3)
        
        # Draw path
        for i in range(len(self.path) - 1):
            start_pos = (self.path[i][0] * CELL_SIZE + CELL_SIZE // 2, self.path[i][1] * CELL_SIZE + CELL_SIZE // 2)
            end_pos = (self.path[i+1][0] * CELL_SIZE + CELL_SIZE // 2, self.path[i+1][1] * CELL_SIZE + CELL_SIZE // 2)
            pygame.draw.line(self.screen, YELLOW, start_pos, end_pos, 3)
        
        # Draw drone
        if self.path:
            drone_x = int(self.drone_pos[0] * CELL_SIZE + CELL_SIZE // 2)
            drone_y = int(self.drone_pos[1] * CELL_SIZE + CELL_SIZE // 2)
            pygame.draw.circle(self.screen, BLUE, (drone_x, drone_y), CELL_SIZE // 4)
        
        pygame.display.flip()

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.reset()
            
            self.update_drone()
            self.draw()
            self.clock.tick(60)
        
        pygame.quit()

if __name__ == "__main__":
    sim = DroneSimulator()
    sim.run()
