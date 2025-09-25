import pygame
import math
from PathFinding.pathfinding import plan_path, missions, other_obstacles, robot, field_size_mm, grid_resolution_mm

# --- Pygame setup ---
WIDTH, HEIGHT = int(field_size_mm[0]//5), int(field_size_mm[1]//5)  # scale down for display
SCALE = WIDTH / field_size_mm[0]

WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255,0,0)
BLUE = (0,0,255)
GREEN = (0,255,0)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("FLL Pathfinding Visualization")
clock = pygame.time.Clock()

# --- Utility functions ---
def mm_to_px(pos):
    return int(pos[0]*SCALE), int(pos[1]*SCALE)

def draw_obstacles():
    for name, m in missions.items():
        x, y = mm_to_px((m['x']-m['w']/2, m['y']-m['h']/2))
        w, h = int(m['w']*SCALE), int(m['h']*SCALE)
        pygame.draw.rect(screen, RED, (x, y, w, h))
    for o in other_obstacles:
        x, y = mm_to_px((o.x - o.w/2, o.y - o.h/2))
        w, h = int(o.w*SCALE), int(o.h*SCALE)
        pygame.draw.rect(screen, BLUE, (x, y, w, h))

def draw_path(path):
    if len(path) < 2:
        return
    for i in range(len(path)-1):
        pygame.draw.line(screen, GREEN, mm_to_px(path[i]), mm_to_px(path[i+1]), 3)

def draw_robot(pos):
    x, y = mm_to_px(pos)
    w = int(robot.robot_width_mm*SCALE/2)
    h = int(robot.robot_length_mm*SCALE/2)
    pygame.draw.rect(screen, BLACK, (x-w, y-h, 2*w, 2*h), 2)

# --- Main loop ---
start_position = (100, 100)
target_mission = 'MissionB'
path_data = plan_path(start_position, target_mission)
path = path_data['smooth_path']

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill(WHITE)
    draw_obstacles()
    draw_path(path)
    draw_robot(start_position)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
