# Pathfinding and tank-drive instruction generator for FLL fields
# This is a self-contained demo. Replace `field_config` and `missions` with
# exact coordinates from your season's Field Setup Guide PDF (we found the official PDFs in search results).
#
# Key features:
# - Builds an occupancy grid and inflates obstacles by robot radius.
# - Uses A* to find collision-free path from any start to a chosen mission target.
# - Smooths path with line-of-sight pruning.
# - Outputs explicit tank-drive instructions (rotate in place + drive forward) with motor rotation degrees,
#   based on wheel_diameter_mm and axle_track_mm.
#
# Note: For official mission coordinates and exact mat dimensions, extract values from FIRST's Field Setup Guide PDF.
# Official field setup guides were found during search and should be used to populate `missions` and `field_size_mm`.
# (See references in assistant message.)

import math
import heapq
from dataclasses import dataclass
from typing import List, Tuple

# ---------- Utilities ----------
@dataclass
class Obstacle:
    x: float  # center x (mm)
    y: float  # center y (mm)
    w: float  # width (mm)
    h: float  # height (mm)

@dataclass
class RobotSpec:
    wheel_diameter_mm: float
    axle_track_mm: float
    robot_width_mm: float
    robot_length_mm: float

# Example field config (replace with official values from your season's Field Setup Guide)
field_size_mm = (1500.0, 1000.0)  # (width_x, height_y) in mm - example values
grid_resolution_mm = 10.0  # grid cell resolution; lower = finer planning but slower

# Example mission obstacles (rectangles). In practice, extract these from the field PDF.
missions = {
    "MissionA": {"x": 1100.0, "y": 300.0, "w": 100.0, "h": 100.0},
    "MissionB": {"x": 700.0, "y": 700.0, "w": 120.0, "h": 120.0},
    "MissionC": {"x": 300.0, "y": 500.0, "w": 150.0, "h": 150.0},
}

# Example other obstacles (tableside models, border posts etc.)
other_obstacles = [
    Obstacle(x=400.0, y=200.0, w=80.0, h=80.0),
    Obstacle(x=900.0, y=400.0, w=60.0, h=200.0),
]

robot = RobotSpec(
    wheel_diameter_mm=56.0,  # example SPIKE/EV3 wheel
    axle_track_mm=120.0,     # distance between left/right wheels (wheelbase)
    robot_width_mm=150.0,
    robot_length_mm=200.0
)

# ---------- Grid / Occupancy ----------
grid_w = int(math.ceil(field_size_mm[0] / grid_resolution_mm))
grid_h = int(math.ceil(field_size_mm[1] / grid_resolution_mm))

def rect_cells(cx, cy, w, h, inflate=0.0):
    """Return grid cell indices covered by rectangle centered at (cx,cy) mm with size w x h plus inflate (mm)."""
    half_w = (w + inflate*2) / 2.0
    half_h = (h + inflate*2) / 2.0
    x0 = max(0, int((cx - half_w) // grid_resolution_mm))
    x1 = min(grid_w-1, int((cx + half_w) // grid_resolution_mm))
    y0 = max(0, int((cy - half_h) // grid_resolution_mm))
    y1 = min(grid_h-1, int((cy + half_h) // grid_resolution_mm))
    cells = [(x,y) for x in range(x0, x1+1) for y in range(y0, y1+1)]
    return cells

def build_occupancy(robot_spec: RobotSpec, missions_dict, other_obs: List[Obstacle], inflation_margin_mm=10.0):
    """Create occupancy grid and mark inflated obstacles for collision checking."""
    occ = [[0]*grid_h for _ in range(grid_w)]
    # Inflate obstacles by robot footprint radius (approx half diagonal)
    robot_radius = math.hypot(robot_spec.robot_width_mm/2.0, robot_spec.robot_length_mm/2.0) / 2.0
    inflate = robot_radius + inflation_margin_mm
    # Missions as obstacles
    for name, m in missions_dict.items():
        cells = rect_cells(m["x"], m["y"], m["w"], m["h"], inflate=inflate)
        for (x,y) in cells:
            occ[x][y] = 1
    # Other obstacles
    for o in other_obs:
        cells = rect_cells(o.x, o.y, o.w, o.h, inflate=inflate)
        for (x,y) in cells:
            occ[x][y] = 1
    # Table border (assume robot cannot leave field)
    # Already limited by grid indices.
    return occ

# ---------- A* pathfinding ----------
def heuristic(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def neighbors(node):
    x,y = node
    nb = []
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            if dx==0 and dy==0:
                continue
            nx, ny = x+dx, y+dy
            if 0<=nx<grid_w and 0<=ny<grid_h:
                cost = math.hypot(dx,dy)
                nb.append(((nx,ny), cost))
    return nb

def a_star(start_mm, goal_mm, occ):
    start = (int(start_mm[0]//grid_resolution_mm), int(start_mm[1]//grid_resolution_mm))
    goal = (int(goal_mm[0]//grid_resolution_mm), int(goal_mm[1]//grid_resolution_mm))
    if occ[goal[0]][goal[1]]==1:
        raise ValueError("Goal is occupied (or too close) — choose a different target or reduce inflation.")
    open_set = []
    heapq.heappush(open_set, (0+heuristic(start,goal), 0, start, None))
    came_from = {}
    gscore = {start:0}
    while open_set:
        f, g, current, parent = heapq.heappop(open_set)
        if current in came_from:
            continue
        came_from[current] = parent
        if current == goal:
            break
        for (n, cost) in neighbors(current):
            if occ[n[0]][n[1]]==1: continue
            ng = g + cost
            if n not in gscore or ng < gscore[n]:
                gscore[n] = ng
                heapq.heappush(open_set, (ng + heuristic(n, goal), ng, n, current))
    if goal not in came_from:
        raise ValueError("No path found to goal (blocked).")
    # Reconstruct path
    path = []
    cur = goal
    while cur is not None:
        path.append((cur[0]*grid_resolution_mm + grid_resolution_mm/2.0,
                     cur[1]*grid_resolution_mm + grid_resolution_mm/2.0))
        cur = came_from.get(cur)
    path.reverse()
    return path

# ---------- Path smoothing (line-of-sight) ----------
def line_of_sight(p1, p2, occ):
    # Bresenham-like check along segment between p1,p2
    x0 = int(p1[0]//grid_resolution_mm); y0 = int(p1[1]//grid_resolution_mm)
    x1 = int(p2[0]//grid_resolution_mm); y1 = int(p2[1]//grid_resolution_mm)
    dx = abs(x1-x0); dy = abs(y1-y0)
    x,y = x0,y0
    n = 1 + dx + dy
    x_inc = 1 if x1>x0 else -1
    y_inc = 1 if y1>y0 else -1
    error = dx - dy
    dx *= 2; dy *= 2
    for _ in range(n):
        if occ[x][y]==1:
            return False
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return True

def smooth_path(path, occ):
    if not path: return path
    smoothed = [path[0]]
    i = 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1:
            if line_of_sight(path[i], path[j], occ):
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed

# ---------- Tank-drive instruction generation ----------
def angle_between(a,b):
    return math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))

def normalize_angle_deg(a):
    while a <= -180: a += 360
    while a > 180: a -= 360
    return a

def generate_tank_instructions(path, robot_spec: RobotSpec):
    """Returns a list of (action, params) where action is 'rotate' or 'drive'.
       Rotate uses degrees; drive uses mm distance.
    """
    cmds = []
    if len(path) < 2:
        return cmds
    heading = angle_between(path[0], path[1])
    # initial rotate from assumed starting heading 0 -> we assume robot faces +x initially
    current_heading = 0.0
    for i in range(len(path)-1):
        p = path[i]; q = path[i+1]
        target_heading = angle_between(p,q)
        delta = normalize_angle_deg(target_heading - current_heading)
        if abs(delta) > 1.0:  # rotate in place
            # compute wheel travel for rotate-in-place
            turn_circumference = math.pi * robot_spec.axle_track_mm
            # fraction of full circle:
            fraction = abs(delta) / 360.0
            wheel_travel_mm = turn_circumference * fraction
            wheel_rot_deg = (wheel_travel_mm / (math.pi * robot_spec.wheel_diameter_mm)) * 360.0
            # left and right motor degrees (signs)
            left_deg = -wheel_rot_deg if delta>0 else wheel_rot_deg
            right_deg = wheel_rot_deg if delta>0 else -wheel_rot_deg
            cmds.append(("rotate", {"angle_deg": delta, "left_motor_deg": left_deg, "right_motor_deg": right_deg}))
            current_heading = target_heading
        # drive forward
        dist = math.hypot(q[0]-p[0], q[1]-p[1])
        if dist > 1.0:
            wheel_rot_deg_drive = (dist / (math.pi * robot_spec.wheel_diameter_mm)) * 360.0
            cmds.append(("drive", {"distance_mm": dist, "left_motor_deg": wheel_rot_deg_drive, "right_motor_deg": wheel_rot_deg_drive}))
    return cmds

# ---------- Demo: plan from arbitrary start to a chosen mission ----------
def plan_path(start_mm, target_mission_name):
    occ = build_occupancy(robot, missions, other_obstacles, inflation_margin_mm=10.0)
    goal = (missions[target_mission_name]["x"], missions[target_mission_name]["y"])
    raw_path = a_star(start_mm, goal, occ)
    smooth = smooth_path(raw_path, occ)
    instr = generate_tank_instructions(smooth, robot)
    return {"raw_path": raw_path, "smooth_path": smooth, "instructions": instr}

# Run demo
start_position = (100.0, 100.0)  # start in mm (replace with your chosen start)
result = plan_path(start_position, "MissionB")

# Pretty-print results
print("Planned smoothed waypoints (mm):")
for p in result["smooth_path"]:
    print(f"  - ({p[0]:.1f}, {p[1]:.1f})")
print("\nTank-drive instructions:")
for step in result["instructions"]:
    action, params = step
    if action=="rotate":
        print(f"  ROTATE in place by {params['angle_deg']:.1f}° -> left motor {params['left_motor_deg']:.1f}°, right motor {params['right_motor_deg']:.1f}°")
    else:
        print(f"  DRIVE forward {params['distance_mm']:.1f} mm -> left {params['left_motor_deg']:.1f}°, right {params['right_motor_deg']:.1f}°")

# If you'd like, save path as CSV for visualization/testing
import csv
csv_path = "/mnt/data/planned_path.csv"
with open(csv_path, "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x_mm","y_mm"])
    for p in result["smooth_path"]:
        writer.writerow([p[0], p[1]])
print(f"\nSaved smoothed path to: {csv_path}")
