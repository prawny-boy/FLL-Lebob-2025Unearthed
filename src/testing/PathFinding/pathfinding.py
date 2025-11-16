"""Path planning helpers for FLL testing rigs."""

from __future__ import annotations

import argparse
import csv
import heapq
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

Point = Tuple[float, float]
GridPoint = Tuple[int, int]


@dataclass
class Obstacle:
    x: float
    y: float
    w: float
    h: float


@dataclass
class Mission:
    name: str
    x: float
    y: float
    w: float
    h: float

    def center(self) -> Point:
        return (self.x, self.y)


@dataclass
class RobotSpec:
    wheel_diameter_mm: float
    axle_track_mm: float
    robot_width_mm: float
    robot_length_mm: float


@dataclass
class FieldConfig:
    size_mm: Tuple[float, float]
    grid_resolution_mm: float
    missions: Dict[str, Mission] = field(default_factory=dict)
    obstacles: List[Obstacle] = field(default_factory=list)


@dataclass
class Instruction:
    action: str
    params: Dict[str, float]


@dataclass
class PlannerResult:
    raw_path: List[Point]
    smooth_path: List[Point]
    instructions: List[Instruction]


class PathPlanner:
    """Plans waypoints and tank-drive instructions with optional smoothing."""

    def __init__(self, field_config: FieldConfig, robot_spec: RobotSpec, inflation_margin_mm: float = 10.0):
        self.field = field_config
        self.robot = robot_spec
        self.grid_w = int(math.ceil(field_config.size_mm[0] / field_config.grid_resolution_mm))
        self.grid_h = int(math.ceil(field_config.size_mm[1] / field_config.grid_resolution_mm))
        self.inflation_margin_mm = inflation_margin_mm

    # ------------------------------------------------------------------
    # Occupancy helpers
    def _rect_cells(self, cx: float, cy: float, w: float, h: float, inflate: float) -> List[GridPoint]:
        half_w = (w + inflate * 2) / 2.0
        half_h = (h + inflate * 2) / 2.0
        x0 = max(0, int((cx - half_w) // self.field.grid_resolution_mm))
        x1 = min(self.grid_w - 1, int((cx + half_w) // self.field.grid_resolution_mm))
        y0 = max(0, int((cy - half_h) // self.field.grid_resolution_mm))
        y1 = min(self.grid_h - 1, int((cy + half_h) // self.field.grid_resolution_mm))
        return [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]

    def _build_occupancy(self) -> List[List[int]]:
        occ = [[0] * self.grid_h for _ in range(self.grid_w)]
        footprint_radius = math.hypot(self.robot.robot_width_mm / 2.0, self.robot.robot_length_mm / 2.0) / 2.0
        inflate = footprint_radius + self.inflation_margin_mm
        for mission in self.field.missions.values():
            for cell in self._rect_cells(mission.x, mission.y, mission.w, mission.h, inflate):
                occ[cell[0]][cell[1]] = 1
        for obs in self.field.obstacles:
            for cell in self._rect_cells(obs.x, obs.y, obs.w, obs.h, inflate):
                occ[cell[0]][cell[1]] = 1
        return occ

    # ------------------------------------------------------------------
    # Path finding
    def _grid_point(self, point_mm: Point) -> GridPoint:
        return (
            int(point_mm[0] // self.field.grid_resolution_mm),
            int(point_mm[1] // self.field.grid_resolution_mm),
        )

    def _center_of_cell(self, cell: GridPoint) -> Point:
        return (
            cell[0] * self.field.grid_resolution_mm + self.field.grid_resolution_mm / 2.0,
            cell[1] * self.field.grid_resolution_mm + self.field.grid_resolution_mm / 2.0,
        )

    @staticmethod
    def _heuristic(a: GridPoint, b: GridPoint) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _neighbors(self, node: GridPoint) -> Iterable[Tuple[GridPoint, float]]:
        x, y = node
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                    yield (nx, ny), math.hypot(dx, dy)

    def _a_star(self, start_mm: Point, goal_mm: Point, occ: List[List[int]]) -> List[Point]:
        start = self._grid_point(start_mm)
        goal = self._grid_point(goal_mm)
        if occ[goal[0]][goal[1]] == 1:
            raise ValueError("Goal is blocked—move the waypoint or shrink inflation")
        queue: List[Tuple[float, float, GridPoint, GridPoint | None]] = []
        heapq.heappush(queue, (self._heuristic(start, goal), 0.0, start, None))
        parents: Dict[GridPoint, GridPoint | None] = {}
        g_score: Dict[GridPoint, float] = {start: 0.0}
        while queue:
            f, g, current, parent = heapq.heappop(queue)
            if current in parents:
                continue
            parents[current] = parent
            if current == goal:
                break
            for neighbor, cost in self._neighbors(current):
                if occ[neighbor[0]][neighbor[1]] == 1:
                    continue
                tentative_g = g + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    heapq.heappush(
                        queue,
                        (tentative_g + self._heuristic(neighbor, goal), tentative_g, neighbor, current),
                    )
        if goal not in parents:
            raise ValueError("Unable to route around current obstacles")
        path: List[Point] = []
        node: GridPoint | None = goal
        while node is not None:
            path.append(self._center_of_cell(node))
            node = parents.get(node)
        path.reverse()
        return path

    # ------------------------------------------------------------------
    # Path smoothing
    def _line_of_sight(self, p1: Point, p2: Point, occ: List[List[int]]) -> bool:
        x0, y0 = self._grid_point(p1)
        x1, y1 = self._grid_point(p2)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2
        for _ in range(n):
            if occ[x][y] == 1:
                return False
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        return True

    def _smooth(self, path: Sequence[Point], occ: List[List[int]]) -> List[Point]:
        if not path:
            return []
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._line_of_sight(path[i], path[j], occ):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    # ------------------------------------------------------------------
    # Instruction generation
    @staticmethod
    def _angle_between(a: Point, b: Point) -> float:
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

    @staticmethod
    def _normalize(angle: float) -> float:
        while angle <= -180:
            angle += 360
        while angle > 180:
            angle -= 360
        return angle

    def _rotation_targets(self, delta_heading: float) -> Dict[str, float]:
        turn_circ = math.pi * self.robot.axle_track_mm
        wheel_travel = turn_circ * abs(delta_heading) / 360.0
        wheel_rot_deg = (wheel_travel / (math.pi * self.robot.wheel_diameter_mm)) * 360.0
        left = -wheel_rot_deg if delta_heading > 0 else wheel_rot_deg
        right = wheel_rot_deg if delta_heading > 0 else -wheel_rot_deg
        return {
            "angle_deg": delta_heading,
            "left_motor_deg": left,
            "right_motor_deg": right,
        }

    def _drive_targets(self, distance_mm: float) -> Dict[str, float]:
        wheel_rot_deg = (distance_mm / (math.pi * self.robot.wheel_diameter_mm)) * 360.0
        return {
            "distance_mm": distance_mm,
            "left_motor_deg": wheel_rot_deg,
            "right_motor_deg": wheel_rot_deg,
        }

    def _instructions(self, path: Sequence[Point]) -> List[Instruction]:
        commands: List[Instruction] = []
        if len(path) < 2:
            return commands
        heading = 0.0
        for idx in range(len(path) - 1):
            start = path[idx]
            end = path[idx + 1]
            target = self._angle_between(start, end)
            delta = self._normalize(target - heading)
            if abs(delta) > 1.0:
                commands.append(Instruction("rotate", self._rotation_targets(delta)))
                heading = target
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            if distance > 1.0:
                commands.append(Instruction("drive", self._drive_targets(distance)))
        return commands

    # ------------------------------------------------------------------
    def plan(self, start_mm: Point, mission_name: str) -> PlannerResult:
        if mission_name not in self.field.missions:
            raise KeyError(f"Mission '{mission_name}' is not defined.")
        occ = self._build_occupancy()
        mission = self.field.missions[mission_name]
        raw = self._a_star(start_mm, mission.center(), occ)
        smooth = self._smooth(raw, occ)
        instructions = self._instructions(smooth)
        return PlannerResult(raw_path=raw, smooth_path=smooth, instructions=instructions)


# ----------------------------------------------------------------------
# Example configuration matching the previous inline data
DEFAULT_FIELD = FieldConfig(
    size_mm=(1500.0, 1000.0),
    grid_resolution_mm=10.0,
    missions={
        key: Mission(name=key, **values)
        for key, values in {
            "MissionA": {"x": 1100.0, "y": 300.0, "w": 100.0, "h": 100.0},
            "MissionB": {"x": 700.0, "y": 700.0, "w": 120.0, "h": 120.0},
            "MissionC": {"x": 300.0, "y": 500.0, "w": 150.0, "h": 150.0},
        }.items()
    },
    obstacles=[
        Obstacle(x=400.0, y=200.0, w=80.0, h=80.0),
        Obstacle(x=900.0, y=400.0, w=60.0, h=200.0),
    ],
)

DEFAULT_ROBOT = RobotSpec(
    wheel_diameter_mm=56.0,
    axle_track_mm=120.0,
    robot_width_mm=150.0,
    robot_length_mm=200.0,
)


# ----------------------------------------------------------------------
# CLI helpers

def write_csv(path: Path, points: Sequence[Point]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["x_mm", "y_mm"])
        for x, y in points:
            writer.writerow([f"{x:.2f}", f"{y:.2f}"])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plan a path and drive instructions for an FLL mission")
    parser.add_argument("mission", choices=sorted(DEFAULT_FIELD.missions.keys()))
    parser.add_argument("start", nargs=2, type=float, metavar=("X_MM", "Y_MM"))
    parser.add_argument("--csv", dest="csv", type=Path, help="Optional CSV path for exporting the smooth waypoints")
    parser.add_argument("--inflation", type=float, default=10.0, help="Extra clearance when inflating obstacles")
    return parser


def _format_instruction(instr: Instruction) -> str:
    pretty = ", ".join(f"{key}={value:.1f}" for key, value in instr.params.items())
    return f"  - {instr.action.upper():6s} {pretty}"


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    planner = PathPlanner(DEFAULT_FIELD, DEFAULT_ROBOT, inflation_margin_mm=args.inflation)
    result = planner.plan((args.start[0], args.start[1]), args.mission)
    print("Smoothed path (mm):")
    for waypoint in result.smooth_path:
        print(f"  - ({waypoint[0]:.1f}, {waypoint[1]:.1f})")
    print("\nTank-drive instructions:")
    for instruction in result.instructions:
        print(_format_instruction(instruction))
    if args.csv:
        write_csv(args.csv, result.smooth_path)
        print(f"\nSaved smoothed path to {args.csv}")


if __name__ == "__main__":
    main()
