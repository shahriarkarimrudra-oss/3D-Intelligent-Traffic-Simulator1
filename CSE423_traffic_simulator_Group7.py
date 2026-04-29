from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import math
import time


camera_pos = (0, 500, 500)
fovY = 86
GRID_LENGTH = 650
rand_var = 423

WINDOW_W = 1000
WINDOW_H = 800

camera_angle = 44.0
camera_radius = 760.0
camera_height = 520.0
camera_mode = "TOP"     
follow_vehicle_index = 0
smooth_eye = None
smooth_look = None

LANE_WIDTH = 38.0
ROAD_HALF = 88.0
SPAWN_DISTANCE = 650.0
INTERSECTION_HALF = 108.0
STOP_LINE = 132.0
SAFE_GAP = 86.0
MAX_VEHICLES = 20

DIR_NAMES = ["EAST", "NORTH", "WEST", "SOUTH"]
DIR_SHORT = ["E", "N", "W", "S"]
DIR_VECTORS = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
AUTO_ORDER = [1, 0, 3, 2]  
BUTTON_DIRS = [("N", 1), ("E", 0), ("S", 3), ("W", 2)]

vehicles = []
completed_count = 0
total_completed_wait = 0.0
simulation_time = 0.0
last_time = None
paused = False
spawn_enabled = True
spawn_timer = 0.0
spawn_interval = 1.85
manual_incident = False
day_mode = True

traffic_auto = True
active_direction = 1
pending_direction = 0
auto_index = 0
signal_phase = "GREEN"     
transition_timer = 0.0
GREEN_DURATION = 8.5
YELLOW_DURATION = 2.0

pedestrian_crossing_active = False
lane_block_active = False
BLOCKAGE_DIR = 0
BLOCKAGE_LANE = 1
BLOCKAGE_LONGITUDINAL = -305.0
BLOCKAGE_S = SPAWN_DISTANCE + BLOCKAGE_LONGITUDINAL

queue_counts = [0, 0, 0, 0]
crash_count = 0

BUTTONS = []


# ------------------------------------------------------------
# Common utility and template-safe drawing helpers
# ------------------------------------------------------------

def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def next_rand():
    global rand_var
    rand_var = (rand_var * 1103515245 + 12345) % 2147483648
    return rand_var


def rand01():
    return (next_rand() % 10000) / 10000.0


def rand_range(a, b):
    return a + (b - a) * rand01()


def draw_text(x, y, text, font=GLUT_BITMAP_HELVETICA_18, color=(1.0, 1.0, 1.0)):
    glColor3f(color[0], color[1], color[2])
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0, WINDOW_W, 0, WINDOW_H)

    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    glRasterPos2f(x, y)
    for ch in text:
        glutBitmapCharacter(font, ord(ch))

    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)


def enter_2d():
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0, WINDOW_W, 0, WINDOW_H)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()


def exit_2d():
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)


def draw_panel_rect(x1, y1, x2, y2, color):

    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    sx = abs(x2 - x1)
    sy = abs(y2 - y1)
    glPushMatrix()
    glColor3f(color[0], color[1], color[2])
    glTranslatef(cx, cy, 0)
    glScalef(sx, sy, 0.40)
    glutSolidCube(1)
    glPopMatrix()


def scene_color(day_color, night_color):
    if day_mode:
        return day_color
    return night_color


def night_tint(color, amount=0.34):
    return (color[0] * amount, color[1] * amount, color[2] * amount)


def draw_flat_rect(x1, y1, x2, y2, z, color):
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    sx = abs(x2 - x1)
    sy = abs(y2 - y1)
    if sx < 0.35:
        sx = 0.35
    if sy < 0.35:
        sy = 0.35
    thickness = 0.65
    draw_box_world(cx, cy, z, sx, sy, thickness, color)


def draw_box_world(x, y, z_bottom, sx, sy, sz, color):
    glPushMatrix()
    glColor3f(color[0], color[1], color[2])
    glTranslatef(x, y, z_bottom + sz * 0.5)
    glScalef(sx, sy, sz)
    glutSolidCube(1)
    glPopMatrix()


def draw_box_local(x, y, z_center, sx, sy, sz, color):
    glPushMatrix()
    glColor3f(color[0], color[1], color[2])
    glTranslatef(x, y, z_center)
    glScalef(sx, sy, sz)
    glutSolidCube(1)
    glPopMatrix()


def draw_cylinder_z(x, y, z, radius, height, color):
    glPushMatrix()
    glColor3f(color[0], color[1], color[2])
    glTranslatef(x, y, z)
    gluCylinder(gluNewQuadric(), radius, radius, height, 10, 8)
    glPopMatrix()


def draw_cone_z(x, y, z, base_radius, top_radius, height, color):
    """Draw an opaque tapered 3D cylinder/cone along the z-axis."""
    glPushMatrix()
    glColor3f(color[0], color[1], color[2])
    glTranslatef(x, y, z)
    gluCylinder(gluNewQuadric(), base_radius, top_radius, height, 12, 8)
    glPopMatrix()


def draw_sphere_world(x, y, z, radius, color):
    glPushMatrix()
    glColor3f(color[0], color[1], color[2])
    glTranslatef(x, y, z)
    gluSphere(gluNewQuadric(), radius, 10, 10)
    glPopMatrix()


# ------------------------------------------------------------
# // Member 1: Vehicle Behavior System
# ------------------------------------------------------------

def right_vector(direction):
    dx, dy = DIR_VECTORS[direction]
    return (dy, -dx)


def lane_offset(lane_index):
    return LANE_WIDTH * 0.5 + lane_index * LANE_WIDTH


def lane_center_point(direction, lane_index, longitudinal):
    dx, dy = DIR_VECTORS[direction]
    rx, ry = right_vector(direction)
    off = lane_offset(lane_index)
    return (dx * longitudinal + rx * off,
            dy * longitudinal + ry * off)


def quadratic_bezier(p0, p1, p2, t):
    a = (1.0 - t) * (1.0 - t)
    b = 2.0 * (1.0 - t) * t
    c = t * t
    return (a * p0[0] + b * p1[0] + c * p2[0],
            a * p0[1] + b * p1[1] + c * p2[1])


def turn_exit_direction(start_direction, turn_type):
    if turn_type == "left":
        return (start_direction + 1) % 4
    if turn_type == "right":
        return (start_direction + 3) % 4
    return start_direction


def build_path(start_direction, turn_type, lane_index):
    points = [lane_center_point(start_direction, lane_index, -SPAWN_DISTANCE)]

    if turn_type == "straight":
        points.append(lane_center_point(start_direction, lane_index, -INTERSECTION_HALF))
        points.append(lane_center_point(start_direction, lane_index, INTERSECTION_HALF))
        points.append(lane_center_point(start_direction, lane_index, SPAWN_DISTANCE))
    else:
        exit_direction = turn_exit_direction(start_direction, turn_type)
        entry = lane_center_point(start_direction, lane_index, -INTERSECTION_HALF)
        exit_point = lane_center_point(exit_direction, lane_index, INTERSECTION_HALF)

        incoming_right = right_vector(start_direction)
        outgoing_right = right_vector(exit_direction)
        off = lane_offset(lane_index)
        control = (incoming_right[0] * off + outgoing_right[0] * off,
                   incoming_right[1] * off + outgoing_right[1] * off)

        points.append(entry)
        samples = 28
        for i in range(1, samples + 1):
            t = i / float(samples)
            points.append(quadratic_bezier(entry, control, exit_point, t))
        points.append(lane_center_point(exit_direction, lane_index, SPAWN_DISTANCE))

    clean = []
    for p in points:
        if len(clean) == 0:
            clean.append(p)
        else:
            dx = p[0] - clean[-1][0]
            dy = p[1] - clean[-1][1]
            if dx * dx + dy * dy > 0.0001:
                clean.append(p)

    cumulative = [0.0]
    for i in range(1, len(clean)):
        dx = clean[i][0] - clean[i - 1][0]
        dy = clean[i][1] - clean[i - 1][1]
        cumulative.append(cumulative[-1] + math.sqrt(dx * dx + dy * dy))

    return {"points": clean, "cum": cumulative, "length": cumulative[-1]}


def sample_path(path, distance):
    points = path["points"]
    cumulative = path["cum"]
    if distance <= 0.0:
        i = 0
    elif distance >= path["length"]:
        i = len(points) - 2
    else:
        i = 0
        while i < len(cumulative) - 2 and cumulative[i + 1] < distance:
            i += 1

    seg_len = cumulative[i + 1] - cumulative[i]
    if seg_len <= 0.0001:
        t = 0.0
    else:
        t = clamp((distance - cumulative[i]) / seg_len, 0.0, 1.0)

    x = points[i][0] + (points[i + 1][0] - points[i][0]) * t
    y = points[i][1] + (points[i + 1][1] - points[i][1]) * t
    heading = math.atan2(points[i + 1][1] - points[i][1],
                         points[i + 1][0] - points[i][0])
    return x, y, heading


def angle_difference_degrees(a, b):
    diff = a - b
    while diff > 180.0:
        diff -= 360.0
    while diff < -180.0:
        diff += 360.0
    return diff


class Vehicle:
    def __init__(self, start_direction, turn_type, lane_index, initial_distance=0.0):
        self.start_direction = start_direction
        self.turn_type = turn_type
        self.exit_direction = turn_exit_direction(start_direction, turn_type)
        self.lane_index = lane_index
        self.path = build_path(start_direction, turn_type, lane_index)

        self.dist = initial_distance
        self.stop_s = SPAWN_DISTANCE - STOP_LINE

        
        self.kind = "bus" if rand01() < 0.15 else "car"
        if self.kind == "bus":
            self.length = 82.0
            self.width = 32.0
            self.height = 28.0
            self.max_speed = 48.0 + rand01() * 11.0
            self.accel = 27.0
            self.brake = 72.0
        else:
            self.length = 54.0
            self.width = 28.0
            self.height = 18.0
            self.max_speed = 58.0 + rand01() * 18.0
            self.accel = 39.0 + rand01() * 8.0
            self.brake = 88.0

        self.speed = 10.0 + rand01() * 18.0
        self.driver_factor = 0.92 + rand01() * 0.18
        self.color = self.pick_color()
        self.wait_time = 0.0
        self.target_speed = self.max_speed
        self.braking = False
        self.stopped = False
        self.steer = 0.0
        self.roll = 0.0
        self.completed = False
        self.crashed = False
        self.removed_by_crash = False
        self.crash_timer = 0.0
        self.shake_timer = 0.0
        self.shake_amp = 0.0
        self.push_x = 0.0
        self.push_y = 0.0
        self.idle_phase = rand01() * 6.28
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.heading_deg = 0.0
        update_vehicle_pose(self)

    def pick_color(self):
        palette = [
            (0.86, 0.08, 0.08), (0.05, 0.32, 0.96),
            (0.94, 0.70, 0.08), (0.10, 0.72, 0.28),
            (0.75, 0.18, 0.88), (0.92, 0.92, 0.92),
            (0.08, 0.75, 0.78), (0.95, 0.42, 0.10)
        ]
        if self.kind == "bus":
            return (0.95, 0.78, 0.18)
        return palette[next_rand() % len(palette)]


def update_vehicle_pose(vehicle):
    x, y, heading = sample_path(vehicle.path, vehicle.dist)
    x2, y2, heading2 = sample_path(vehicle.path, min(vehicle.path["length"], vehicle.dist + 22.0))
    h_deg = math.degrees(heading)
    h2_deg = math.degrees(heading2)
    steer = angle_difference_degrees(h2_deg, h_deg) * 2.2
    vehicle.x = x
    vehicle.y = y
    vehicle.heading = heading
    vehicle.heading_deg = h_deg
    vehicle.steer = clamp(steer, -28.0, 28.0)
    vehicle.roll = (vehicle.dist * 7.2) % 360.0


def same_approach_lane(a, b):
    return (a.start_direction == b.start_direction and
            a.lane_index == b.lane_index and
            a.dist < a.stop_s + 185.0 and
            b.dist < b.stop_s + 185.0)


def is_prepare_yellow(direction):
    return signal_phase == "YELLOW_SWITCH" and pending_direction == direction


def desired_vehicle_speed(vehicle):
    if vehicle.crash_timer > 0.0:
        return 0.0, None

    target = vehicle.max_speed * vehicle.driver_factor
    stop_at = None
    front_s = vehicle.dist + vehicle.length * 0.48
    dist_to_stop = vehicle.stop_s - front_s
    color = get_signal_color(vehicle.start_direction)


    if dist_to_stop > -5.0 and dist_to_stop < 230.0:
        must_stop = False
        if color == "RED":
            must_stop = True
        elif color == "YELLOW":
            if is_prepare_yellow(vehicle.start_direction):
                target = min(target, 5.5 if dist_to_stop > 12.0 else 0.0)
            else:
                if dist_to_stop > 42.0:
                    must_stop = True

        if pedestrian_crossing_active:
            must_stop = True

        if must_stop:
            target = min(target, max(0.0, dist_to_stop * 1.10))
            stop_at = vehicle.stop_s

  
    if lane_block_active and vehicle.start_direction == BLOCKAGE_DIR and vehicle.lane_index == BLOCKAGE_LANE:
        blockage_stop_s = BLOCKAGE_S - 42.0
        dist_to_block = blockage_stop_s - front_s
        if dist_to_block > -4.0 and dist_to_block < 210.0:
            target = min(target, max(0.0, dist_to_block * 1.05))
            if stop_at is None or blockage_stop_s < stop_at:
                stop_at = blockage_stop_s

  
    for other in vehicles:
        if other is vehicle:
            continue
        if other.crashed and other.start_direction == vehicle.start_direction and other.lane_index == vehicle.lane_index:
            pass
        if same_approach_lane(vehicle, other) and other.dist > vehicle.dist:
            gap = other.dist - vehicle.dist - (vehicle.length + other.length) * 0.50
            desired_gap = SAFE_GAP + vehicle.speed * 0.24
            if gap < desired_gap:
                target = min(target, max(0.0, gap * 1.08))
                if gap < 10.0:
                    stop_at = vehicle.dist + max(0.0, gap)

        dx = other.x - vehicle.x
        dy = other.y - vehicle.y
        forward = math.cos(vehicle.heading) * dx + math.sin(vehicle.heading) * dy
        side = -math.sin(vehicle.heading) * dx + math.cos(vehicle.heading) * dy
        near_center = (abs(vehicle.x) < 155.0 and abs(vehicle.y) < 155.0 and
                       abs(other.x) < 155.0 and abs(other.y) < 155.0)
        if near_center and forward > 0.0 and forward < 72.0 and abs(side) < 32.0:
            target = min(target, max(0.0, (forward - 35.0) * 1.10))

    return clamp(target, 0.0, vehicle.max_speed), stop_at


def add_vehicle(start_direction, turn_type, lane_index, initial_distance=0.0):
    if len(vehicles) >= MAX_VEHICLES:
        return False
    new_vehicle = Vehicle(start_direction, turn_type, lane_index, initial_distance)
    for old_vehicle in vehicles:
        if old_vehicle.start_direction == start_direction and old_vehicle.lane_index == lane_index:
            if abs(old_vehicle.dist - new_vehicle.dist) < 115.0:
                return False
    vehicles.append(new_vehicle)
    return True


def choose_turn_and_lane():
    r = next_rand() % 10
    if r < 2:
        return "left", 0
    if r < 7:
        lane = 0 if next_rand() % 3 else 1
        return "straight", lane
    return "right", 1


def try_spawn_vehicle():
    direction = next_rand() % 4
    turn, lane = choose_turn_and_lane()
    for v in vehicles:
        if v.start_direction == direction and v.lane_index == lane and v.dist < 145.0:
            return False
    return add_vehicle(direction, turn, lane, 0.0)


def apply_collision_detection():
    global crash_count
    for i in range(len(vehicles)):
        a = vehicles[i]
        if a.crashed or a.removed_by_crash:
            continue
        for j in range(i + 1, len(vehicles)):
            b = vehicles[j]
            if b.crashed or b.removed_by_crash:
                continue
            dx = a.x - b.x
            dy = a.y - b.y
            d2 = dx * dx + dy * dy
            min_dist = (a.width + b.width) * 0.55
            if d2 < min_dist * min_dist and d2 > 0.01:
                if a.shake_timer <= 0.0 and b.shake_timer <= 0.0:
                    d = math.sqrt(d2)
                    nx = dx / d
                    ny = dy / d
                    impact = (a.speed + b.speed) * 0.5
                    amp = clamp(impact * 0.13, 2.5, 10.0)
                    a.shake_amp = amp
                    b.shake_amp = amp
                    a.shake_timer = 1.5
                    b.shake_timer = 1.5
                    a.crash_timer = 3.0
                    b.crash_timer = 3.0
                    a.speed *= 0.12
                    b.speed *= 0.12
                    a.push_x += nx * 9.0
                    a.push_y += ny * 9.0
                    b.push_x -= nx * 9.0
                    b.push_y -= ny * 9.0
                    a.crashed = True
                    b.crashed = True
                    crash_count += 1


def draw_crash_smoke(vehicle):
    if vehicle.crash_timer <= 0.0:
        return
    progress = clamp(1.0 - vehicle.crash_timer / 3.0, 0.0, 1.0)
    base_x = vehicle.x + vehicle.push_x
    base_y = vehicle.y + vehicle.push_y
    for i in range(6):
        drift = simulation_time * (1.4 + i * 0.18) + i * 1.9
        ox = math.sin(drift) * (5.0 + i * 2.0) + (i - 2) * 5.5
        oy = math.cos(drift * 0.73) * (4.0 + i * 1.6)
        oz = 18.0 + progress * 42.0 + i * 5.5
        radius = 6.5 + progress * 10.0 + i * 1.6
        shade = clamp(0.42 + progress * 0.18 - i * 0.025, 0.26, 0.62)
        draw_sphere_world(base_x + ox, base_y + oy, oz, radius, (shade, shade, shade))


def draw_wheel(wx, wy, vehicle, is_front):
    glPushMatrix()
    glTranslatef(wx, wy, 7.0)
    if is_front:
        glRotatef(vehicle.steer, 0, 0, 1)
    glRotatef(vehicle.roll, 0, 1, 0)
    draw_box_local(0.0, 0.0, 0.0, 2.2, 3.0, 12.0, (0.78, 0.78, 0.78))
    glRotatef(90, 1, 0, 0)
    glTranslatef(0, 0, -3.0)
    glColor3f(0.02, 0.02, 0.02)
    gluCylinder(gluNewQuadric(), 6.5, 6.5, 6.0, 10, 5)
    glPopMatrix()


def draw_vehicle(vehicle):
    """Draw a vehicle with painter-safe wheel occlusion.

    The template does not enable depth testing, so drawing all wheels after the
    body makes the far-side wheels appear through the car. To keep the car
    visually realistic, only the camera-facing wheel pair is drawn.
    """
    shake_x = 0.0
    shake_y = 0.0
    shake_z = 0.0
    if vehicle.shake_timer > 0.0:
        decay = vehicle.shake_timer / 1.5
        t = simulation_time * 28.0
        shake_x = math.sin(t * 1.7) * vehicle.shake_amp * decay
        shake_y = math.cos(t * 2.1) * vehicle.shake_amp * decay
        shake_z = abs(math.sin(t * 2.8)) * vehicle.shake_amp * 0.35 * decay

    idle_z = 0.0
    idle_x = 0.0
    if vehicle.speed < 2.0 and not vehicle.crashed:
        idle_z = math.sin(vehicle.idle_phase) * 0.28
        idle_x = math.cos(vehicle.idle_phase * 0.7) * 0.12

   
    center_x = vehicle.x + vehicle.push_x
    center_y = vehicle.y + vehicle.push_y
    rel_x = camera_pos[0] - center_x
    rel_y = camera_pos[1] - center_y
    local_camera_y = -math.sin(vehicle.heading) * rel_x + math.cos(vehicle.heading) * rel_y
    visible_side = 1.0 if local_camera_y >= 0.0 else -1.0

    glPushMatrix()
    glTranslatef(vehicle.x + shake_x + idle_x + vehicle.push_x,
                 vehicle.y + shake_y + vehicle.push_y,
                 8.0 + shake_z + idle_z)
    glRotatef(vehicle.heading_deg, 0, 0, 1)

    body = vehicle.color
    roof = (min(body[0] + 0.13, 1.0), min(body[1] + 0.13, 1.0), min(body[2] + 0.13, 1.0))

    if vehicle.kind == "bus":
        wheel_y = 18.0


        draw_box_local(0, 0, 15, vehicle.length, vehicle.width, vehicle.height, body)
        draw_box_local(3, 0, 31, vehicle.length * 0.72, vehicle.width * 0.82, 9, roof)

 
        for xw in [-26, -8, 10, 28]:
            draw_box_local(xw, visible_side * 17, 28, 10, 2, 8, (0.45, 0.76, 0.96))

        head_color = (0.55, 0.50, 0.30)
        draw_box_local(43, -9, 19, 3, 5, 4, head_color)
        draw_box_local(43, 9, 19, 3, 5, 4, head_color)
        brake_color = (1.0, 0.0, 0.0) if vehicle.braking else (0.35, 0.0, 0.0)
        draw_box_local(-43, -9, 19, 3, 5, 4, brake_color)
        draw_box_local(-43, 9, 19, 3, 5, 4, brake_color)

        draw_wheel(28, visible_side * wheel_y, vehicle, True)
        draw_wheel(-30, visible_side * wheel_y, vehicle, False)

    else:
        wheel_y = 16.0

        draw_box_local(0, 0, 11, vehicle.length, vehicle.width, 16, body)
        draw_box_local(-5, 0, 24, 29, 22, 14, roof)
        draw_box_local(12, 0, 26, 7, 23, 5, (0.35, 0.70, 0.95))
        draw_box_local(-22, 0, 23, 5, 20, 4, (0.25, 0.55, 0.85))

        head_color = (0.55, 0.50, 0.30)
        draw_box_local(28, -8, 15, 2.5, 5, 3, head_color)
        draw_box_local(28, 8, 15, 2.5, 5, 3, head_color)

        brake_color = (1.0, 0.0, 0.0) if vehicle.braking else (0.35, 0.0, 0.0)
        draw_box_local(-28, -8, 15, 2.5, 5, 3, brake_color)
        draw_box_local(-28, 8, 15, 2.5, 5, 3, brake_color)


        draw_wheel(18, visible_side * wheel_y, vehicle, True)
        draw_wheel(-18, visible_side * wheel_y, vehicle, False)

    glPopMatrix()
    draw_crash_smoke(vehicle)

# ------------------------------------------------------------
# // Member 2: Traffic Control System
# ------------------------------------------------------------

def get_signal_color(direction):
    if signal_phase == "GREEN":
        return "GREEN" if direction == active_direction else "RED"
    if signal_phase == "YELLOW_SWITCH":
        if direction == active_direction or direction == pending_direction:
            return "YELLOW"
        return "RED"
    return "RED"


def request_manual_green(direction):
    global traffic_auto, pending_direction, signal_phase, transition_timer, green_timer
    traffic_auto = False
    if direction == active_direction and signal_phase == "GREEN":
        return
    pending_direction = direction
    signal_phase = "YELLOW_SWITCH"
    transition_timer = 0.0
    green_timer = 0.0


def toggle_auto_mode():
    global traffic_auto, signal_phase, green_timer, transition_timer, pending_direction
    global auto_index
    traffic_auto = not traffic_auto
    if traffic_auto:
        if active_direction in AUTO_ORDER:
            auto_index = AUTO_ORDER.index(active_direction)
        pending_direction = AUTO_ORDER[(auto_index + 1) % len(AUTO_ORDER)]
        signal_phase = "GREEN"
        green_timer = 0.0
        transition_timer = 0.0


def update_signal(dt):
    global active_direction, pending_direction, auto_index
    global signal_phase, green_timer, transition_timer

    if signal_phase == "GREEN":
        green_timer += dt
        if traffic_auto and green_timer >= GREEN_DURATION:
            pending_direction = AUTO_ORDER[(auto_index + 1) % len(AUTO_ORDER)]
            signal_phase = "YELLOW_SWITCH"
            transition_timer = 0.0

    elif signal_phase == "YELLOW_SWITCH":
        transition_timer += dt
        if transition_timer >= YELLOW_DURATION:
            active_direction = pending_direction
            if active_direction in AUTO_ORDER:
                auto_index = AUTO_ORDER.index(active_direction)
            signal_phase = "GREEN"
            green_timer = 0.0
            transition_timer = 0.0


def signal_remaining_time():
    if signal_phase == "GREEN":
        return max(0.0, GREEN_DURATION - green_timer)
    return max(0.0, YELLOW_DURATION - transition_timer)


def signal_light_colors(direction):
    state = get_signal_color(direction)
    if state == "RED":
        return (1.0, 0.05, 0.03), (0.18, 0.14, 0.02), (0.01, 0.16, 0.03)
    if state == "YELLOW":
        return (0.25, 0.02, 0.02), (1.0, 0.85, 0.08), (0.01, 0.16, 0.03)
    return (0.25, 0.02, 0.02), (0.18, 0.14, 0.02), (0.05, 1.0, 0.10)


def count_traffic_queues():
    global queue_counts
    queue_counts = [0, 0, 0, 0]
    for v in vehicles:
        front_s = v.dist + v.length * 0.48
        dist_to_stop = v.stop_s - front_s
        queued = dist_to_stop > -8.0 and dist_to_stop < 245.0 and v.speed < 10.0
        if queued:
            queue_counts[v.start_direction] += 1


def draw_axis_arm(x1, y1, x2, y2, z_bottom, color):
   
    width = 6.0
    sx = max(width, abs(x2 - x1))
    sy = max(width, abs(y2 - y1))
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    draw_box_world(cx, cy, z_bottom, sx, sy, 5.0, color)


def draw_signal_head(direction, bx, by):

    dx, dy = DIR_VECTORS[direction]
    red, yellow, green = signal_light_colors(direction)
    head_color = (0.025, 0.025, 0.025)
    hood_color = (0.055, 0.055, 0.055)
    back_color = (0.015, 0.015, 0.015)


    if abs(dx) > 0.5:
        draw_box_world(bx, by, 78, 16, 34, 58, head_color)
        draw_box_world(bx + dx * 3.0, by, 75, 4, 40, 64, back_color)
        bulb_x = bx - dx * 9.0
        hood_x = bx - dx * 13.0
        hood_sx, hood_sy = 10, 25
        hood_dx, hood_dy = -dx * 3.0, 0.0
        positions = [(bulb_x, by, 124, red), (bulb_x, by, 107, yellow), (bulb_x, by, 90, green)]
    else:
        draw_box_world(bx, by, 78, 34, 16, 58, head_color)
        draw_box_world(bx, by + dy * 3.0, 75, 40, 4, 64, back_color)
        bulb_y = by - dy * 9.0
        hood_y = by - dy * 13.0
        hood_sx, hood_sy = 25, 10
        hood_dx, hood_dy = 0.0, -dy * 3.0
        positions = [(bx, bulb_y, 124, red), (bx, bulb_y, 107, yellow), (bx, bulb_y, 90, green)]

    # Three separate compartments and visors give the light a real 3D form.
    for px, py, pz, col in positions:
        if abs(dx) > 0.5:
            draw_box_world(hood_x + hood_dx, by, pz - 3.0, hood_sx, hood_sy, 5.0, hood_color)
            draw_box_world(hood_x + hood_dx, by - 12.0, pz - 8.0, hood_sx, 3.0, 15.0, hood_color)
            draw_box_world(hood_x + hood_dx, by + 12.0, pz - 8.0, hood_sx, 3.0, 15.0, hood_color)
        else:
            draw_box_world(bx, hood_y + hood_dy, pz - 3.0, hood_sx, hood_sy, 5.0, hood_color)
            draw_box_world(bx - 12.0, hood_y + hood_dy, pz - 8.0, 3.0, hood_sy, 15.0, hood_color)
            draw_box_world(bx + 12.0, hood_y + hood_dy, pz - 8.0, 3.0, hood_sy, 15.0, hood_color)
        draw_sphere_world(px, py, pz, 6.6, col)


def draw_traffic_light(direction):

    dx, dy = DIR_VECTORS[direction]
    rx, ry = right_vector(direction)

    head_longitudinal = -STOP_LINE - 6.0
    head_offset = LANE_WIDTH
    bx = dx * head_longitudinal + rx * head_offset
    by = dy * head_longitudinal + ry * head_offset

    pole_longitudinal = -STOP_LINE - 36.0
    pole_offset = ROAD_HALF + 64.0
    px = dx * pole_longitudinal + rx * pole_offset
    py = dy * pole_longitudinal + ry * pole_offset

    pole_color = scene_color((0.20, 0.20, 0.20), (0.12, 0.12, 0.14))
    base_color = scene_color((0.38, 0.38, 0.36), (0.18, 0.18, 0.20))

    draw_box_world(px, py, 0.0, 22.0, 22.0, 8.0, base_color)
    draw_cylinder_z(px, py, 7.5, 4.0, 112.0, pole_color)


    z_arm = 112.0
    joint_x = bx
    joint_y = py
    draw_axis_arm(px, py, joint_x, joint_y, z_arm, pole_color)
    draw_axis_arm(joint_x, joint_y, bx, by, z_arm, pole_color)

    draw_cylinder_z(bx, by, 106.0, 2.4, 18.0, pole_color)
    draw_box_world(bx, by, 102.0, 9.0, 9.0, 8.0, pole_color)

    draw_signal_head(direction, bx, by)


# ------------------------------------------------------------
# // Member 3: Environment and Analytics System
# ------------------------------------------------------------

BUILDINGS = [
    (-430, 370, 0, 85, 95, 160, (0.40, 0.42, 0.50)),
    (-300, 360, 0, 70, 120, 110, (0.48, 0.36, 0.30)),
    (-420, -350, 0, 120, 75, 145, (0.35, 0.38, 0.48)),
    (-280, -360, 0, 80, 85, 90, (0.52, 0.42, 0.35)),
    (310, 355, 0, 95, 105, 130, (0.45, 0.45, 0.52)),
    (440, 310, 0, 75, 95, 180, (0.36, 0.40, 0.55)),
    (340, -335, 0, 110, 80, 105, (0.50, 0.38, 0.34)),
    (470, -380, 0, 75, 85, 135, (0.42, 0.44, 0.47)),
]


def draw_dashed_x(y, x_start, x_end, width, color):
    x = x_start
    while x < x_end:
        x2 = min(x + 46.0, x_end)
        mid = (x + x2) * 0.5
        if abs(mid) > INTERSECTION_HALF + 12.0:
            draw_flat_rect(x, y - width * 0.5, x2, y + width * 0.5, 1.0, color)
        x += 80.0


def draw_dashed_y(x, y_start, y_end, width, color):
    y = y_start
    while y < y_end:
        y2 = min(y + 46.0, y_end)
        mid = (y + y2) * 0.5
        if abs(mid) > INTERSECTION_HALF + 12.0:
            draw_flat_rect(x - width * 0.5, y, x + width * 0.5, y2, 1.0, color)
        y += 80.0


def draw_crosswalks():
    white = scene_color((0.92, 0.92, 0.88), (0.62, 0.66, 0.70))
    stripe = 8.0
    gap = 8.0

    x = -ROAD_HALF + 8
    while x < ROAD_HALF - 8:
        draw_flat_rect(x, -STOP_LINE - 22, x + stripe, -STOP_LINE - 4, 1.15, white)
        draw_flat_rect(x, STOP_LINE + 4, x + stripe, STOP_LINE + 22, 1.15, white)
        x += stripe + gap

    y = -ROAD_HALF + 8
    while y < ROAD_HALF - 8:
        draw_flat_rect(-STOP_LINE - 22, y, -STOP_LINE - 4, y + stripe, 1.15, white)
        draw_flat_rect(STOP_LINE + 4, y, STOP_LINE + 22, y + stripe, 1.15, white)
        y += stripe + gap


def draw_roads():
    ground = scene_color((0.08, 0.30, 0.12), (0.018, 0.060, 0.075))
    road = scene_color((0.13, 0.13, 0.13), (0.040, 0.042, 0.052))
    inter = scene_color((0.10, 0.10, 0.10), (0.052, 0.052, 0.065))
    curb = scene_color((0.55, 0.55, 0.55), (0.30, 0.31, 0.34))
    yellow = scene_color((1.0, 0.86, 0.05), (0.78, 0.66, 0.08))
    white = scene_color((0.90, 0.90, 0.90), (0.62, 0.66, 0.70))

    draw_flat_rect(-GRID_LENGTH, -GRID_LENGTH, GRID_LENGTH, GRID_LENGTH, -1.0, ground)
    draw_flat_rect(-GRID_LENGTH, -ROAD_HALF, GRID_LENGTH, ROAD_HALF, 0.0, road)
    draw_flat_rect(-ROAD_HALF, -GRID_LENGTH, ROAD_HALF, GRID_LENGTH, 0.05, road)
    draw_flat_rect(-ROAD_HALF, -ROAD_HALF, ROAD_HALF, ROAD_HALF, 0.10, inter)

    draw_flat_rect(-GRID_LENGTH, ROAD_HALF, GRID_LENGTH, ROAD_HALF + 5, 0.9, curb)
    draw_flat_rect(-GRID_LENGTH, -ROAD_HALF - 5, GRID_LENGTH, -ROAD_HALF, 0.9, curb)
    draw_flat_rect(ROAD_HALF, -GRID_LENGTH, ROAD_HALF + 5, GRID_LENGTH, 0.9, curb)
    draw_flat_rect(-ROAD_HALF - 5, -GRID_LENGTH, -ROAD_HALF, GRID_LENGTH, 0.9, curb)

    draw_dashed_x(0.0, -GRID_LENGTH, GRID_LENGTH, 5.0, yellow)
    draw_dashed_y(0.0, -GRID_LENGTH, GRID_LENGTH, 5.0, yellow)
    draw_dashed_x(-LANE_WIDTH, -GRID_LENGTH, GRID_LENGTH, 3.3, white)
    draw_dashed_x(LANE_WIDTH, -GRID_LENGTH, GRID_LENGTH, 3.3, white)
    draw_dashed_y(-LANE_WIDTH, -GRID_LENGTH, GRID_LENGTH, 3.3, white)
    draw_dashed_y(LANE_WIDTH, -GRID_LENGTH, GRID_LENGTH, 3.3, white)

    draw_flat_rect(-STOP_LINE - 2.5, -ROAD_HALF, -STOP_LINE + 2.5, 0, 1.25, white)
    draw_flat_rect(STOP_LINE - 2.5, 0, STOP_LINE + 2.5, ROAD_HALF, 1.25, white)
    draw_flat_rect(0, -STOP_LINE - 2.5, ROAD_HALF, -STOP_LINE + 2.5, 1.25, white)
    draw_flat_rect(-ROAD_HALF, STOP_LINE - 2.5, 0, STOP_LINE + 2.5, 1.25, white)
    draw_crosswalks()


def draw_buildings_and_trees():
    for b in BUILDINGS:
        color = b[6] if day_mode else night_tint(b[6], 0.34)
        draw_box_world(b[0], b[1], b[2], b[3], b[4], b[5], color)
        z = 35
        window_color = scene_color((0.82, 0.90, 1.0), (1.0, 0.78, 0.25))
        while z < b[5] - 12:
            draw_box_world(b[0], b[1] - b[4] * 0.51, z, b[3] * 0.55, 2, 8, window_color)
            z += 32

    tree_positions = [(-520, 180), (-520, -180), (520, 180), (520, -180),
                      (-180, 520), (180, 520), (-180, -520), (180, -520)]
    for tx, ty in tree_positions:
        trunk = scene_color((0.38, 0.20, 0.08), (0.16, 0.10, 0.05))
        leaves = scene_color((0.04, 0.45, 0.12), (0.015, 0.16, 0.08))
        draw_cylinder_z(tx, ty, 0, 6, 34, trunk)
        draw_sphere_world(tx, ty, 46, 22, leaves)


def draw_sun_moon_and_lamps():
    if day_mode:
        draw_sphere_world(-500, 510, 430, 38, (1.0, 0.86, 0.25))
    else:
        draw_sphere_world(-500, 510, 430, 28, (0.72, 0.82, 1.0))

    lamp_positions = [(-160, -160), (160, -160), (-160, 160), (160, 160),
                      (-470, 105), (470, -105), (105, 470), (-105, -470),
                      (-310, 92), (310, -92), (92, 310), (-92, -310)]
    pole_color = scene_color((0.34, 0.34, 0.36), (0.20, 0.20, 0.24))
    off_bulb = (0.46, 0.46, 0.38)
    on_bulb = (1.0, 0.88, 0.38)
    hood_color = scene_color((0.24, 0.24, 0.25), (0.12, 0.12, 0.15))

    for lx, ly in lamp_positions:
        draw_cylinder_z(lx, ly, 0, 3.0, 58, pole_color)
        draw_box_world(lx, ly, 58, 11, 11, 3, pole_color)
        draw_box_world(lx + 8, ly, 61, 16, 4, 3, pole_color)
        draw_box_world(lx + 16, ly, 58, 12, 10, 6, hood_color)

        if day_mode:
            draw_sphere_world(lx + 16, ly, 55, 4.8, off_bulb)
        else:
            draw_cone_z(lx + 16, ly, 5, 28.0, 4.0, 50.0, (0.42, 0.34, 0.09))
            draw_box_world(lx + 16, ly, 0.7, 68, 34, 0.8, (0.32, 0.26, 0.07))
            draw_box_world(lx + 16, ly, 1.0, 38, 18, 0.9, (0.50, 0.40, 0.10))
            draw_sphere_world(lx + 16, ly, 55, 5.5, on_bulb)

def update_dynamic_events():
    global pedestrian_crossing_active, lane_block_active
    cross_cycle = simulation_time % 30.0
    pedestrian_crossing_active = 13.0 <= cross_cycle <= 16.5
    block_cycle = simulation_time % 48.0
    lane_block_active = manual_incident or (31.0 <= block_cycle <= 39.0)


def draw_pedestrians():
    if not pedestrian_crossing_active:
        return
    phase = (simulation_time % 3.5) / 3.5
    walk_x = -ROAD_HALF + 10.0 + phase * (ROAD_HALF * 2.0 - 20.0)
    for i in range(4):
        px = walk_x - i * 24.0
        if px < -ROAD_HALF + 6.0:
            px += ROAD_HALF * 2.0 - 12.0
        py = STOP_LINE + 15.0 + (i % 2) * 8.0
        draw_cylinder_z(px, py, 0, 3.0, 18.0, (0.15, 0.20, 0.85))
        draw_sphere_world(px, py, 23.0, 5.0, (0.95, 0.72, 0.52))


def draw_lane_blockage():
    if not lane_block_active:
        return
    bx, by = lane_center_point(BLOCKAGE_DIR, BLOCKAGE_LANE, BLOCKAGE_LONGITUDINAL)
    draw_box_world(bx, by, 0, 62, 9, 15, (1.0, 0.25, 0.05))
    draw_box_world(bx, by + 10, 0, 48, 7, 10, (0.95, 0.95, 0.95))
    for offset in [-26, 0, 26]:
        glPushMatrix()
        glColor3f(1.0, 0.40, 0.05)
        glTranslatef(bx + offset, by - 13, 0)
        gluCylinder(gluNewQuadric(), 7, 2, 24, 10, 6)
        glPopMatrix()


def congestion_label(q):
    if q <= 1:
        return "LOW"
    if q <= 4:
        return "MED"
    return "HIGH"


def build_buttons():
    global BUTTONS
    x0 = WINDOW_W - 188
    top = WINDOW_H - 36
    bw = 174
    bh = 28
    gap = 7
    BUTTONS = []
    BUTTONS.append((x0, top, x0 + bw, top + bh, "AUTO/MANUAL", "auto"))
    y = top - bh - gap
    for label, direction in BUTTON_DIRS:
        BUTTONS.append((x0, y, x0 + bw, y + bh, label + " GREEN", direction))
        y -= bh + gap
    BUTTONS.append((x0, y, x0 + 82, y + bh, "+ CAR", "add"))
    BUTTONS.append((x0 + 92, y, x0 + bw, y + bh, "- CAR", "remove"))
    y -= bh + gap
    BUTTONS.append((x0, y, x0 + bw, y + bh, "CAMERA", "camera"))
    y -= bh + gap
    BUTTONS.append((x0, y, x0 + bw, y + bh, "DAY/NIGHT", "daynight"))
    y -= bh + gap
    BUTTONS.append((x0, y, x0 + bw, y + bh, "INCIDENT", "incident"))
    y -= bh + gap
    BUTTONS.append((x0, y, x0 + bw, y + bh, "PAUSE", "pause"))
    y -= bh + gap
    BUTTONS.append((x0, y, x0 + bw, y + bh, "RESET", "reset"))


def button_color(action):
    if action == "auto":
        return (0.08, 0.45, 0.15) if traffic_auto else (0.46, 0.24, 0.05)
    if isinstance(action, int):
        state = get_signal_color(action)
        if state == "GREEN":
            return (0.05, 0.45, 0.08)
        if state == "YELLOW":
            return (0.58, 0.46, 0.05)
        return (0.42, 0.05, 0.05)
    if action == "pause":
        return (0.10, 0.42, 0.10) if paused else (0.42, 0.32, 0.06)
    if action == "camera":
        return (0.10, 0.25, 0.52) if camera_mode == "TOP" else (0.45, 0.25, 0.10)
    if action == "daynight":
        return (0.08, 0.32, 0.54) if day_mode else (0.18, 0.13, 0.34)
    if action == "incident":
        return (0.50, 0.10, 0.05) if manual_incident else (0.12, 0.27, 0.40)
    if action == "reset":
        return (0.48, 0.08, 0.08)
    return (0.12, 0.30, 0.50)


def draw_hud_buttons():
    enter_2d()
    draw_panel_rect(WINDOW_W - 205, WINDOW_H - 412, WINDOW_W - 6, WINDOW_H - 3, (0.04, 0.04, 0.08))
    for b in BUTTONS:
        x0, y0, x1, y1, label, action = b
        draw_panel_rect(x0, y0, x1, y1, button_color(action))
        draw_panel_rect(x0 + 2, y0 + 2, x1 - 2, y1 - 2, button_color(action))
    draw_panel_rect(0, 0, WINDOW_W, 26, (0.02, 0.02, 0.02))
    exit_2d()

    draw_text(WINDOW_W - 184, WINDOW_H - 18, "TRAFFIC CONTROL", GLUT_BITMAP_HELVETICA_12, (0.80, 0.92, 1.0))
    for b in BUTTONS:
        x0, y0, x1, y1, label, action = b
        shown = label
        if action == "auto":
            shown = "MODE: AUTO" if traffic_auto else "MODE: MANUAL"
        elif isinstance(action, int):
            shown = label + "  " + get_signal_color(action)
        elif action == "pause":
            shown = "RESUME" if paused else "PAUSE"
        elif action == "camera":
            shown = "VIEW: " + ("FOLLOW" if camera_mode == "DRIVER" else "TOP")
        elif action == "daynight":
            shown = "LIGHTING: " + ("DAY" if day_mode else "NIGHT")
        elif action == "incident":
            shown = "INCIDENT ON" if manual_incident else "INCIDENT OFF"
        draw_text(x0 + 8, y0 + 8, shown, GLUT_BITMAP_HELVETICA_12, (1.0, 1.0, 1.0))


def draw_analytics():
    moving = 0
    stopped = 0
    crashed = 0
    for v in vehicles:
        if v.speed > 3.0:
            moving += 1
        if v.stopped:
            stopped += 1
        if v.crashed:
            crashed += 1

    flow = 0.0
    if simulation_time > 1.0:
        flow = completed_count / (simulation_time / 60.0)
    avg_wait = 0.0
    if completed_count > 0:
        avg_wait = total_completed_wait / completed_count

    mode = "AUTO" if traffic_auto else "MANUAL"
    draw_text(10, 770, "3D Realistic Traffic Simulator  |  1=N 2=E 3=S 4=W  M mode  N day/night  Right-click follow  TAB next car  ESC top  Arrows/A-D orbit  W/S zoom", GLUT_BITMAP_HELVETICA_12, (0.85, 0.95, 1.0))
    draw_text(10, 746, "Mode " + mode + " | Active " + DIR_NAMES[active_direction] + " | Phase " + signal_phase +
              " | next " + str(round(signal_remaining_time(), 1)) + "s | Ped " + ("ON" if pedestrian_crossing_active else "OFF") +
              " | Block " + ("ON" if lane_block_active else "OFF") +
              " | " + ("DAY" if day_mode else "NIGHT"), GLUT_BITMAP_HELVETICA_12)
    draw_text(10, 722, "Vehicles " + str(len(vehicles)) + " | moving " + str(moving) + " | stopped " + str(stopped) +
              " | crashes " + str(crash_count) + " | active crash shake " + str(crashed), GLUT_BITMAP_HELVETICA_12)
    draw_text(10, 698, "Completed " + str(completed_count) + " | flow/min " + str(round(flow, 1)) +
              " | avg wait " + str(round(avg_wait, 1)) + "s", GLUT_BITMAP_HELVETICA_12)
    draw_text(10, 674, "Queues: E " + str(queue_counts[0]) + " " + congestion_label(queue_counts[0]) +
              " | N " + str(queue_counts[1]) + " " + congestion_label(queue_counts[1]) +
              " | W " + str(queue_counts[2]) + " " + congestion_label(queue_counts[2]) +
              " | S " + str(queue_counts[3]) + " " + congestion_label(queue_counts[3]), GLUT_BITMAP_HELVETICA_12)
    draw_hud_buttons()


def draw_environment_and_analytics():
    draw_roads()
    draw_buildings_and_trees()
    draw_sun_moon_and_lamps()
    draw_lane_blockage()
    draw_pedestrians()


# ------------------------------------------------------------
# Simulation update and reset
# ------------------------------------------------------------

def reset_simulation():
    global vehicles, completed_count, total_completed_wait, simulation_time
    global green_timer, transition_timer, active_direction, pending_direction, auto_index
    global signal_phase, spawn_timer, rand_var, manual_incident, crash_count
    global traffic_auto, last_time, camera_mode, smooth_eye, smooth_look, day_mode

    vehicles = []
    completed_count = 0
    total_completed_wait = 0.0
    simulation_time = 0.0
    green_timer = 0.0
    transition_timer = 0.0
    active_direction = 1
    pending_direction = 0
    auto_index = 0
    signal_phase = "GREEN"
    spawn_timer = 0.0
    rand_var = 423
    manual_incident = False
    traffic_auto = True
    crash_count = 0
    last_time = None
    camera_mode = "TOP"
    day_mode = True
    smooth_eye = None
    smooth_look = None

    initial_routes = [
        (0, "straight", 0, 70), (0, "right", 1, 250),
        (1, "straight", 0, 105), (1, "left", 0, 285),
        (2, "straight", 0, 80), (2, "right", 1, 255),
        (3, "straight", 0, 115), (3, "left", 0, 300),
    ]
    for route in initial_routes:
        add_vehicle(route[0], route[1], route[2], route[3])
    count_traffic_queues()


def update_simulation(dt):
    global spawn_timer, completed_count, total_completed_wait, simulation_time

    if paused:
        return

    dt = clamp(dt, 0.0, 0.055)
    simulation_time += dt
    update_dynamic_events()
    update_signal(dt)

    if spawn_enabled:
        spawn_timer += dt
        if spawn_timer >= spawn_interval:
            spawn_timer = 0.0
            try_spawn_vehicle()

    for v in vehicles:
        update_vehicle_pose(v)

    for v in vehicles:
        if v.crash_timer > 0.0:
            v.crash_timer = max(0.0, v.crash_timer - dt)
            v.speed = max(0.0, v.speed - v.brake * 1.4 * dt)
            if v.crash_timer <= 0.0:
                v.removed_by_crash = True
                v.completed = True
        if v.shake_timer > 0.0:
            v.shake_timer = max(0.0, v.shake_timer - dt)

        target, stop_at = desired_vehicle_speed(v)
        v.target_speed = target
        slowing = target < v.speed - 0.4

        if v.speed > target:
            v.speed -= v.brake * dt
            if v.speed < target:
                v.speed = target
        else:
            v.speed += v.accel * dt
            if v.speed > target:
                v.speed = target

        v.speed = clamp(v.speed, 0.0, v.max_speed)

        old_front = v.dist + v.length * 0.48
        new_dist = v.dist + v.speed * dt
        new_front = new_dist + v.length * 0.48
        if stop_at is not None and old_front <= stop_at and new_front > stop_at:
            new_dist = stop_at - v.length * 0.48
            v.speed = 0.0

        v.dist = max(0.0, new_dist)
        v.braking = slowing or v.speed < 3.0
        v.stopped = v.speed < 2.0 and v.dist < v.path["length"] - 40.0
        if v.stopped:
            v.wait_time += dt
        v.idle_phase += dt * (9.0 if v.stopped else 3.0)
        v.push_x *= max(0.0, 1.0 - 3.0 * dt)
        v.push_y *= max(0.0, 1.0 - 3.0 * dt)

        update_vehicle_pose(v)
        if v.dist >= v.path["length"] - 4.0:
            v.completed = True

    apply_collision_detection()

    survivors = []
    for v in vehicles:
        if v.completed:
            if not v.removed_by_crash:
                completed_count += 1
                total_completed_wait += v.wait_time
        else:
            survivors.append(v)
    vehicles[:] = survivors
    count_traffic_queues()


# ------------------------------------------------------------
# Camera, input callbacks, and template main loop
# ------------------------------------------------------------

def select_follow_vehicle(step=1):
    global follow_vehicle_index
    if len(vehicles) == 0:
        follow_vehicle_index = 0
    else:
        follow_vehicle_index = (follow_vehicle_index + step) % len(vehicles)


def toggle_camera_mode():
    global camera_mode, smooth_eye, smooth_look, follow_vehicle_index
    camera_mode = "DRIVER" if camera_mode == "TOP" else "TOP"
    if camera_mode == "DRIVER" and len(vehicles) > 0:
        for i, v in enumerate(vehicles):
            if not v.crashed:
                follow_vehicle_index = i
                break
    smooth_eye = None
    smooth_look = None


def setupCamera():
    global camera_pos, smooth_eye, smooth_look
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(fovY, 1.25, 0.1, 1600)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    if camera_mode == "DRIVER" and len(vehicles) > 0:
        if follow_vehicle_index >= len(vehicles):
            idx = 0
        else:
            idx = follow_vehicle_index
        v = vehicles[idx]
        back = 145.0 if v.kind == "car" else 175.0
        up = 76.0 if v.kind == "car" else 88.0
        eye_x = v.x - math.cos(v.heading) * back
        eye_y = v.y - math.sin(v.heading) * back
        eye_z = up + math.sin(v.idle_phase) * 0.35
        look_x = v.x + math.cos(v.heading) * 270.0
        look_y = v.y + math.sin(v.heading) * 270.0
        look_z = 28.0
    else:
        rad = math.radians(camera_angle % 360.0)
        eye_x = math.cos(rad) * camera_radius
        eye_y = math.sin(rad) * camera_radius
        eye_z = camera_height
        look_x = 0.0
        look_y = 0.0
        look_z = 18.0

    if camera_mode == "TOP":

        smooth_eye = (eye_x, eye_y, eye_z)
        smooth_look = (look_x, look_y, look_z)
    elif smooth_eye is None:
        smooth_eye = (eye_x, eye_y, eye_z)
        smooth_look = (look_x, look_y, look_z)
    else:
        k = 0.20
        smooth_eye = (smooth_eye[0] + (eye_x - smooth_eye[0]) * k,
                      smooth_eye[1] + (eye_y - smooth_eye[1]) * k,
                      smooth_eye[2] + (eye_z - smooth_eye[2]) * k)
        smooth_look = (smooth_look[0] + (look_x - smooth_look[0]) * k,
                       smooth_look[1] + (look_y - smooth_look[1]) * k,
                       smooth_look[2] + (look_z - smooth_look[2]) * k)

    camera_pos = smooth_eye
    gluLookAt(smooth_eye[0], smooth_eye[1], smooth_eye[2],
              smooth_look[0], smooth_look[1], smooth_look[2],
              0, 0, 1)


def handle_button_click(mx, my):
    global paused, manual_incident, day_mode
    hit_pad = 6
    for b in BUTTONS:
        x0, y0, x1, y1, label, action = b
        if mx >= x0 - hit_pad and mx <= x1 + hit_pad and my >= y0 - hit_pad and my <= y1 + hit_pad:
            if action == "auto":
                toggle_auto_mode()
            elif isinstance(action, int):
                request_manual_green(action)
            elif action == "add":
                try_spawn_vehicle()
            elif action == "remove":
                if len(vehicles) > 0:
                    vehicles.pop()
            elif action == "camera":
                toggle_camera_mode()
            elif action == "daynight":
                day_mode = not day_mode
            elif action == "incident":
                manual_incident = not manual_incident
            elif action == "pause":
                paused = not paused
            elif action == "reset":
                reset_simulation()
            return True
    if mx >= WINDOW_W - 220:
        return True
    return False


def keyboardListener(key, x, y):
    global paused, camera_radius, camera_angle, camera_height
    global camera_mode, manual_incident, spawn_enabled, day_mode, smooth_eye, smooth_look
    k = key.lower()

    if k == b'p' or key == b' ':
        paused = not paused
    elif k == b'r':
        reset_simulation()
    elif k == b'w':
        camera_radius = max(230.0, camera_radius - 40.0)
    elif k == b's':
        camera_radius = min(1300.0, camera_radius + 40.0)
    elif k == b'a':
        camera_angle = (camera_angle + 4.0) % 360.0
    elif k == b'd':
        camera_angle = (camera_angle - 4.0) % 360.0
    elif k == b'm':
        toggle_auto_mode()
    elif k == b'n':
        day_mode = not day_mode
    elif k == b'c':
        toggle_camera_mode()
    elif key == b'\t':
        select_follow_vehicle(1)
        camera_mode = 'DRIVER'
        smooth_eye = None
        smooth_look = None
    elif key == b'\x1b':
        camera_mode = "TOP"
    elif k == b'e':
        manual_incident = not manual_incident
    elif k == b'v':
        spawn_enabled = not spawn_enabled
    elif k == b'+':
        for _ in range(2):
            try_spawn_vehicle()
    elif k == b'-':
        if len(vehicles) > 0:
            vehicles.pop()
    elif k == b'1':
        request_manual_green(1)
    elif k == b'2':
        request_manual_green(0)
    elif k == b'3':
        request_manual_green(3)
    elif k == b'4':
        request_manual_green(2)


def specialKeyListener(key, x, y):
    global camera_angle, camera_height
    if key == GLUT_KEY_LEFT:
        camera_angle = (camera_angle + 4.0) % 360.0
    if key == GLUT_KEY_RIGHT:
        camera_angle = (camera_angle - 4.0) % 360.0
    if key == GLUT_KEY_UP:
        camera_height = min(900.0, camera_height + 25.0)
    if key == GLUT_KEY_DOWN:
        camera_height = max(120.0, camera_height - 25.0)


def mouseListener(button, state, x, y):
    if button == GLUT_LEFT_BUTTON and state == GLUT_DOWN:
        gl_y = WINDOW_H - y
        handle_button_click(x, gl_y)
    if button == GLUT_RIGHT_BUTTON and state == GLUT_DOWN:
        toggle_camera_mode()


def idle():
    global last_time
    now = time.time()
    if last_time is None:
        last_time = now
    dt = now - last_time
    last_time = now
    update_simulation(dt)
    glutPostRedisplay()


def showScreen():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glViewport(0, 0, WINDOW_W, WINDOW_H)
    setupCamera()

    draw_environment_and_analytics()

    ordered = sorted(
        vehicles,
        key=lambda car: (car.x - camera_pos[0]) * (car.x - camera_pos[0]) +
                        (car.y - camera_pos[1]) * (car.y - camera_pos[1]),
        reverse=True
    )
    for car in ordered:
        draw_vehicle(car)

    for direction in range(4):
        draw_traffic_light(direction)

    draw_analytics()

    glutSwapBuffers()


def main():
    build_buttons()
    reset_simulation()

    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(WINDOW_W, WINDOW_H)
    glutInitWindowPosition(0, 0)
    glutCreateWindow(b"3D Intelligent Traffic Simulator - Realistic")

    glutDisplayFunc(showScreen)
    glutKeyboardFunc(keyboardListener)
    glutSpecialFunc(specialKeyListener)
    glutMouseFunc(mouseListener)
    glutIdleFunc(idle)

    glutMainLoop()


if __name__ == "__main__":
    main()
