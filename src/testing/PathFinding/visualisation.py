"""Visualise planned paths using pygame (optional dependency)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

try:  # pragma: no cover - optional
    import pygame
except ImportError:  # pragma: no cover - optional
    pygame = None

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.append(str(HERE))

from pathfinding import DEFAULT_FIELD, DEFAULT_ROBOT, PathPlanner

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 200, 0)


def mm_to_px(point, scale):
    return int(point[0] * scale), int(point[1] * scale)


def draw_obstacles(screen, planner, scale):
    for mission in planner.field.missions.values():
        x, y = mm_to_px((mission.x - mission.w / 2, mission.y - mission.h / 2), scale)
        w, h = int(mission.w * scale), int(mission.h * scale)
        pygame.draw.rect(screen, RED, (x, y, w, h), 0)
    for obstacle in planner.field.obstacles:
        x, y = mm_to_px((obstacle.x - obstacle.w / 2, obstacle.y - obstacle.h / 2), scale)
        w, h = int(obstacle.w * scale), int(obstacle.h * scale)
        pygame.draw.rect(screen, BLUE, (x, y, w, h), 0)


def draw_path(screen, path: Sequence, scale):
    if len(path) < 2:
        return
    for i in range(len(path) - 1):
        pygame.draw.line(screen, GREEN, mm_to_px(path[i], scale), mm_to_px(path[i + 1], scale), 3)


def draw_robot(screen, planner, position, scale):
    x, y = mm_to_px(position, scale)
    half_w = int(planner.robot.robot_width_mm * scale / 2)
    half_h = int(planner.robot.robot_length_mm * scale / 2)
    pygame.draw.rect(screen, BLACK, (x - half_w, y - half_h, half_w * 2, half_h * 2), 2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualise a planned path with pygame")
    parser.add_argument("mission", choices=sorted(DEFAULT_FIELD.missions.keys()))
    parser.add_argument("start", nargs=2, type=float, metavar=("X_MM", "Y_MM"))
    parser.add_argument("--scale", type=float, default=0.2, help="Pixels per mm (smaller shrinks window)")
    parser.add_argument("--inflation", type=float, default=10.0, help="Obstacle inflation padding in mm")
    return parser.parse_args()


def main() -> None:  # pragma: no cover - interactive
    if pygame is None:
        raise SystemExit("pygame is not installed. Run `pip install pygame` to enable this visualiser.")
    args = parse_args()
    planner = PathPlanner(DEFAULT_FIELD, DEFAULT_ROBOT, inflation_margin_mm=args.inflation)
    start = (args.start[0], args.start[1])
    result = planner.plan(start, args.mission)

    width = max(200, int(planner.field.size_mm[0] * args.scale))
    height = max(200, int(planner.field.size_mm[1] * args.scale))
    pygame.display.set_caption("FLL Pathfinding Visualisation")
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill(WHITE)
        draw_obstacles(screen, planner, args.scale)
        draw_path(screen, result.smooth_path, args.scale)
        draw_robot(screen, planner, start, args.scale)
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
