"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633  # [m/s]
AXLE_LENGTH = 0.4044  # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75  # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = (
    "head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
    "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
    "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint"
)

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts = []

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

range = robot.getDevice('range-finder')
range.enable(timestep)
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x = 0
pose_y = 0
pose_theta = 0

vL = 0
vR = 0
timepassed = 0
timepassed1 = 0
timepassed2 = 0
check_wall = 0

lidar_sensor_readings = []  # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE / 2., +LIDAR_ANGLE_RANGE / 2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets) - 83]  # Only keep lidar readings not blocked by robot chassis

# map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'autonomous'  # Part 1.1: manual mode

# part 1
map = np.zeros(shape=[360, 360], dtype=float)
waypoints = []

MAP_SIZE = 360
PIXELS_PER_METER = 30.0

MAP_INCREMENT = 0.005
MAP_THRESHOLD = 0.12
DISPLAY_GAIN = 18.0
MIN_DISPLAY_CONF = 0.02
DEBUG_PRINT = False

DRIVE_SPEED = 4.0
TURN_SPEED = 2.5

step_count = 0


def clamp(value, low, high):
    return max(low, min(high, value))


def world_to_map(wx, wy):
    mx = 360 - abs(int(wx * PIXELS_PER_METER))
    my = abs(int(wy * PIXELS_PER_METER))
    mx = clamp(mx, 0, MAP_SIZE - 1)
    my = clamp(my, 0, MAP_SIZE - 1)
    return int(mx), int(my)


def gray_to_hex(g):
    g = clamp(g, 0.0, 1.0)
    gray = int(g * 255)
    return (gray << 16) | (gray << 8) | gray


def save_current_map():
    saved_map = (map >= MAP_THRESHOLD)
    np.save("map.npy", saved_map)
    np.save("map_raw.npy", map)
    print("Saved map.npy and map_raw.npy")


def clear_current_map():
    global map
    map = np.zeros(shape=[360, 360], dtype=float)
    display.setColor(0x000000)
    display.fillRectangle(0, 0, MAP_SIZE, MAP_SIZE)
    print("Cleared map")


# ---------- Part 2 helpers ----------
def valid_ranges(ranges):
    vals = []
    for r in ranges:
        if np.isfinite(r) and 0.18 < r < LIDAR_SENSOR_MAX_RANGE:
            vals.append(float(r))
    return vals


def region_min(ranges, start_frac, end_frac, fallback=LIDAR_SENSOR_MAX_RANGE):
    n = len(ranges)
    a = int(start_frac * n)
    b = int(end_frac * n)
    if b <= a:
        return fallback
    vals = valid_ranges(ranges[a:b])
    if len(vals) == 0:
        return fallback
    return min(vals)


def get_lidar_regions(ranges):
    # low indices = right side, middle = front, high indices = left side
    front = region_min(ranges, 0.42, 0.58)
    left = region_min(ranges, 0.68, 0.90)
    right = region_min(ranges, 0.10, 0.32)
    front_left = region_min(ranges, 0.58, 0.72)
    front_right = region_min(ranges, 0.28, 0.42)

    return {
        "front": front,
        "left": left,
        "right": right,
        "front_left": front_left,
        "front_right": front_right
    }


# autonomous states
FIND_WALL = 0
ALIGN_WITH_WALL = 1
FOLLOW_LEFT_WALL = 2
BRAKE_BEFORE_TURN = 3
TURN_RIGHT_FROM_WALL = 4
SCAN_ROTATE = 5
FINISHED = 6

robot_state = FIND_WALL
autonomous_started = False
start_pose = None
wall_start_pose = None
state_start_time_ms = 0
last_scan_time_ms = 0


while robot.step(timestep) != -1 and mode != 'planner':

    step_count += 1

    # Mapping

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2])) - 1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings) - 83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha) * rho
        ry = -math.sin(alpha) * rho

        t = pose_theta + np.pi / 2.
        # Convert detection from robot coordinates into world coordinates
        wx = math.cos(t) * rx - math.sin(t) * ry + pose_x
        wy = math.sin(t) * rx + math.cos(t) * ry + pose_y

        ################ ^ [End] Do not modify ^ ##################

        if wx >= 12:
            wx = 11.999
        if wy >= 12:
            wy = 11.999
        if wx <= -12:
            wx = -11.999
        if wy <= -12:
            wy = -11.999
        if rho < LIDAR_SENSOR_MAX_RANGE:

            # You will eventually REPLACE the following lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            if not np.isfinite(rho):
                continue
            if rho <= 0.18:
                continue
            if rho >= (LIDAR_SENSOR_MAX_RANGE - 0.05):
                continue

            mx, my = world_to_map(wx, wy)

            map[my][mx] = min(1.0, map[my][mx] + MAP_INCREMENT)

            if map[my][mx] >= MIN_DISPLAY_CONF:
                display_gray = min(1.0, map[my][mx] * DISPLAY_GAIN)
                display.setColor(int(gray_to_hex(display_gray)))
                display.drawPixel(mx, my)

    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(360 - abs(int(pose_x * 30)), abs(int(pose_y * 30)))

    if DEBUG_PRINT and step_count % 20 == 0:
        center_idx = len(lidar_sensor_readings) // 2
        left_idx = len(lidar_sensor_readings) // 4
        right_idx = (3 * len(lidar_sensor_readings)) // 4
        print(
            "POSE", round(pose_x, 3), round(pose_y, 3), round(pose_theta, 3),
            "L", round(float(lidar_sensor_readings[left_idx]), 3),
            "C", round(float(lidar_sensor_readings[center_idx]), 3),
            "R", round(float(lidar_sensor_readings[right_idx]), 3)
        )

    # Controllers

    if mode == 'manual':
        # Perform teleoperation and obtain a map of the entire environment and store the map as an npy file
        vL = 0.0
        vR = 0.0

        up_pressed = False
        down_pressed = False
        left_pressed = False
        right_pressed = False
        save_pressed = False
        clear_pressed = False

        key = keyboard.getKey()
        while key != -1:
            if key == Keyboard.UP:
                up_pressed = True
            elif key == Keyboard.DOWN:
                down_pressed = True
            elif key == Keyboard.LEFT:
                left_pressed = True
            elif key == Keyboard.RIGHT:
                right_pressed = True
            elif key == ord('S') or key == ord('s'):
                save_pressed = True
            elif key == ord('C') or key == ord('c'):
                clear_pressed = True
            key = keyboard.getKey()

        if up_pressed:
            vL = DRIVE_SPEED
            vR = DRIVE_SPEED
        elif down_pressed:
            vL = -DRIVE_SPEED
            vR = -DRIVE_SPEED

        if left_pressed:
            if up_pressed:
                vL = DRIVE_SPEED * 0.45
                vR = DRIVE_SPEED
            elif down_pressed:
                vL = -DRIVE_SPEED * 0.45
                vR = -DRIVE_SPEED
            else:
                vL = -TURN_SPEED
                vR = TURN_SPEED

        if right_pressed:
            if up_pressed:
                vL = DRIVE_SPEED
                vR = DRIVE_SPEED * 0.45
            elif down_pressed:
                vL = -DRIVE_SPEED
                vR = -DRIVE_SPEED * 0.45
            else:
                vL = TURN_SPEED
                vR = -TURN_SPEED

        if save_pressed:
            save_current_map()

        if clear_pressed:
            clear_current_map()

    elif mode == 'autonomous':
        current_time_ms = step_count * timestep
        vL = 0.0
        vR = 0.0

        regions = get_lidar_regions(lidar_sensor_readings)
        front = regions["front"]
        left = regions["left"]
        right = regions["right"]
        front_left = regions["front_left"]
        front_right = regions["front_right"]

        # speeds
        AUTO_BASE_SPEED = 2.6
        AUTO_TURN_SPEED = 3.8

        # thresholds
        FRONT_WARN = 0.95
        FRONT_BLOCKED = 0.78
        FRONT_HARD_BLOCKED = 0.62

        LEFT_TARGET = 0.62
        LEFT_PRESENT = 0.82
        LEFT_TOO_CLOSE = 0.44
        LEFT_OPEN = 0.98

        # timing
        STARTUP_WAIT_MS = 1500
        ALIGN_DURATION_MS = 820
        BRAKE_DURATION_MS = 220
        TURN_DURATION_MS = 620
        SCAN_INTERVAL_MS = 9000
        SCAN_DURATION_MS = 2600

        LOOP_MIN_TIME_MS = 26000
        LOOP_CLOSE_DIST = 0.60

        if not autonomous_started:
            autonomous_started = True
            start_pose = (pose_x, pose_y)
            state_start_time_ms = current_time_ms
            last_scan_time_ms = current_time_ms

        if current_time_ms < STARTUP_WAIT_MS:
            vL = 0.0
            vR = 0.0

        else:
            if robot_state == FIND_WALL:
                # drive forward until near a wall
                if front > FRONT_WARN:
                    vL = AUTO_BASE_SPEED
                    vR = AUTO_BASE_SPEED
                else:
                    wall_start_pose = (pose_x, pose_y)
                    robot_state = ALIGN_WITH_WALL
                    state_start_time_ms = current_time_ms

            elif robot_state == ALIGN_WITH_WALL:
                # rotate right so that the first wall becomes the left wall to follow
                vL = AUTO_TURN_SPEED
                vR = -AUTO_TURN_SPEED

                if current_time_ms - state_start_time_ms >= ALIGN_DURATION_MS:
                    robot_state = FOLLOW_LEFT_WALL
                    state_start_time_ms = current_time_ms
                    last_scan_time_ms = current_time_ms

            elif robot_state == FOLLOW_LEFT_WALL:
                if current_time_ms - last_scan_time_ms >= SCAN_INTERVAL_MS:
                    robot_state = SCAN_ROTATE
                    state_start_time_ms = current_time_ms
                else:
                    front_warn = (front < FRONT_WARN) or (front_left < FRONT_WARN * 0.95)
                    front_blocked = (front < FRONT_BLOCKED) or (front_left < FRONT_BLOCKED)
                    hard_front_blocked = (front < FRONT_HARD_BLOCKED) or (front_left < FRONT_HARD_BLOCKED)

                    left_present = left < LEFT_PRESENT
                    left_too_close = left < LEFT_TOO_CLOSE
                    left_open = left > LEFT_OPEN

                    # stop first, then turn, so it does not ram the wall
                    if hard_front_blocked or front_blocked:
                        robot_state = BRAKE_BEFORE_TURN
                        state_start_time_ms = current_time_ms
                        vL = 0.0
                        vR = 0.0

                    else:
                        if left_open:
                            # no wall on left -> search left
                            vL = 1.1
                            vR = AUTO_BASE_SPEED

                        elif left_too_close:
                            # too close to wall -> steer right
                            vL = AUTO_BASE_SPEED
                            vR = 1.3

                        elif left_present:
                            # proportional correction to stay near target wall distance
                            error = LEFT_TARGET - left
                            correction = 2.0 * error
                            vL = AUTO_BASE_SPEED + correction
                            vR = AUTO_BASE_SPEED - correction
                        else:
                            vL = 1.3
                            vR = AUTO_BASE_SPEED

                        if front_warn:
                            vL *= 0.72
                            vR *= 0.72

                        vL = clamp(vL, -MAX_SPEED, MAX_SPEED)
                        vR = clamp(vR, -MAX_SPEED, MAX_SPEED)

                        # close loop near initial wall position
                        if wall_start_pose is not None and current_time_ms > LOOP_MIN_TIME_MS:
                            dx = pose_x - wall_start_pose[0]
                            dy = pose_y - wall_start_pose[1]
                            dist_to_start = math.sqrt(dx * dx + dy * dy)

                            if dist_to_start < LOOP_CLOSE_DIST:
                                save_current_map()
                                robot_state = FINISHED

            elif robot_state == BRAKE_BEFORE_TURN:
                vL = 0.0
                vR = 0.0

                if current_time_ms - state_start_time_ms >= BRAKE_DURATION_MS:
                    robot_state = TURN_RIGHT_FROM_WALL
                    state_start_time_ms = current_time_ms

            elif robot_state == TURN_RIGHT_FROM_WALL:
                vL = AUTO_TURN_SPEED
                vR = -AUTO_TURN_SPEED

                if (front > FRONT_WARN and front_left > FRONT_BLOCKED) or \
                   (current_time_ms - state_start_time_ms >= TURN_DURATION_MS):
                    robot_state = FOLLOW_LEFT_WALL
                    state_start_time_ms = current_time_ms

            elif robot_state == SCAN_ROTATE:
                vL = -AUTO_TURN_SPEED
                vR = AUTO_TURN_SPEED

                if current_time_ms - state_start_time_ms >= SCAN_DURATION_MS:
                    last_scan_time_ms = current_time_ms
                    robot_state = FOLLOW_LEFT_WALL
                    state_start_time_ms = current_time_ms

            elif robot_state == FINISHED:
                vL = 0.0
                vR = 0.0

        if step_count % 15 == 0:
            print(
                "STATE:", robot_state,
                "front:", round(front, 2),
                "front_left:", round(front_left, 2),
                "left:", round(left, 2),
                "right:", round(right, 2),
                "pose:", round(pose_x, 2), round(pose_y, 2)
            )

    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL + vR) / 2 / MAX_SPEED * MAX_SPEED_MS * timestep / 1000.0 * math.cos(pose_theta)
    pose_y -= (vL + vR) / 2 / MAX_SPEED * MAX_SPEED_MS * timestep / 1000.0 * math.sin(pose_theta)
    pose_theta += (vR - vL) / AXLE_LENGTH / MAX_SPEED * MAX_SPEED_MS * timestep / 1000.0

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)

while robot.step(timestep) != -1:
    # there is a bug where webots have to be restarted if the controller exits on Windows
    # this is to keep the controller running
    pass
