# """csci3302_lab4 controller."""
# Copyright (2025) University of Colorado Boulder
# CSCI 3302: Introduction to Robotics

import math
import numpy as np
from controller import Robot

# Change this to anything else to stay in place to test coordinate transform functions
state = "line_follower"

LIDAR_SENSOR_MAX_RANGE = 3.0  # meters
NUM_LIDAR_RAYS = 21
LIDAR_ANGLE_RANGE = 1.5708  # 90 degrees in radians

# Pose in MAP frame (map origin top-left, +x right, +y down)
pose_x = 0.197
pose_y = 0.65
pose_theta = np.pi / 2

# ePuck constants
EPUCK_AXLE_DIAMETER = 0.053
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice("gs0"), robot.getDevice("gs1"), robot.getDevice("gs2")]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Initialize the Display
display = robot.getDevice("display")

# get and enable lidar
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()

##### DO NOT MODIFY ANY CODE ABOVE THIS #####


########################
# Part 1: LiDAR angles
########################

# store LiDAR distance measurements each step (filled in loop)
lidar_sensor_readings = []

# Precompute LiDAR ray angles (alphas), centered at 0
lidar_ray_angles = np.linspace(
    -LIDAR_ANGLE_RANGE / 2.0,
    +LIDAR_ANGLE_RANGE / 2.0,
    NUM_LIDAR_RAYS,
)

########################
# Part 2: Map data + helpers
########################

DISPLAY_WIDTH = 300
DISPLAY_HEIGHT = 300

def world_to_display(x_m: float, y_m: float):
    """Map coordinates (0..1 m) -> display pixels (0..299)."""
    px = int(x_m * DISPLAY_WIDTH)
    py = int(y_m * DISPLAY_HEIGHT)
    px = max(0, min(DISPLAY_WIDTH - 1, px))
    py = max(0, min(DISPLAY_HEIGHT - 1, py))
    return px, py

# Keep memory of visited + occupied pixels for redraw before saving
visited_pixels = set()   # (px, py)
occupied_pixels = set()  # (px, py)

def report():
    print(f"CURRENT xPOSE {pose_x} CURRENT yPOSE {pose_y}")

########################
# Main Control Loop
########################

cnt = 0
while robot.step(SIM_TIMESTEP) != -1:
    report()

    #####################################################
    #                 Sensing                           #
    #####################################################
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    lidar_sensor_readings = lidar.getRangeImage()

    #####################################################
    # Part 2: draw visited in red
    #####################################################
    robot_px, robot_py = world_to_display(pose_x, pose_y)
    visited_pixels.add((robot_px, robot_py))

    # Draw visited pixel (red)
    display.setColor(0xFF0000)
    display.drawPixel(robot_px, robot_py)

    #####################################################
    # Part 3 + Part 4: LiDAR -> robot frame -> map frame -> draw
    #####################################################
    valid_hits = 0

    c = math.cos(pose_theta)
    s = math.sin(pose_theta)

    for i in range(min(NUM_LIDAR_RAYS, len(lidar_sensor_readings))):
        rho = float(lidar_sensor_readings[i])
        alpha = float(lidar_ray_angles[i])

        # Filter out invalid / max-range / too-close (reduces “waves”)
        if not np.isfinite(rho):
            continue
        if rho <= 0.10:
            continue
        if rho >= (LIDAR_SENSOR_MAX_RANGE - 0.05):
            continue

        valid_hits += 1

        # 1) object in ROBOT frame (robot +x forward, +y left)
        rx = rho * math.cos(alpha)
        ry = rho * math.sin(alpha)

        # 2) ROBOT -> MAP (map +y down)
        # mx = pose_x + cos(theta)*rx + sin(theta)*ry
        # my = pose_y - sin(theta)*rx + cos(theta)*ry
        mx = pose_x + c * rx + s * ry
        my = pose_y - s * rx + c * ry

        # Convert obstacle to display pixels
        obs_px, obs_py = world_to_display(mx, my)
        occupied_pixels.add((obs_px, obs_py))

        # Drawing order matters:
        # (1) free space line (white)
        display.setColor(0xFFFFFF)
        display.drawLine(robot_px, robot_py, obs_px, obs_py)

        # (2) obstacle (blue)
        display.setColor(0x0000FF)
        display.drawPixel(obs_px, obs_py)

        # (3) robot (red) so it stays visible
        display.setColor(0xFF0000)
        display.drawPixel(robot_px, robot_py)

    # Optional: keep this if you like
    # if valid_hits > 0:
    #     print("valid lidar hits:", valid_hits)

    #####################################################
    #                 Robot controller                  #
    #####################################################

    if state == "line_follower":
        if (gsr[1] < 350 and gsr[0] > 400 and gsr[2] > 400):
            vL = MAX_SPEED * 0.3
            vR = MAX_SPEED * 0.3

        # Checking for Start Line
        elif (gsr[0] < 310 and gsr[1] < 310 and gsr[2] < 310):
            cnt += 1
            vL = MAX_SPEED * 0.3
            vR = MAX_SPEED * 0.3
            if cnt > 10:
                cnt = 0
                print("Over the line!")

                # Redraw everything cleanly before saving
                # 1) occupied (blue)
                display.setColor(0x0000FF)
                for (px, py) in occupied_pixels:
                    display.drawPixel(px, py)

                # 2) visited (red)
                display.setColor(0xFF0000)
                for (px, py) in visited_pixels:
                    display.drawPixel(px, py)

                display.imageSave(None, "map.png")

        elif (gsr[2] < 650):  # turn right
            vL = 0.2 * MAX_SPEED
            vR = -0.05 * MAX_SPEED
            cnt = 0
        elif (gsr[0] < 650):  # turn left
            vL = -0.05 * MAX_SPEED
            vR = 0.2 * MAX_SPEED
            cnt = 0
        else:
            # if none match, keep moving
            vL = MAX_SPEED * 0.3
            vR = MAX_SPEED * 0.3
    else:
        vL = 0.0
        vR = 0.0

    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

    #####################################################
    #                    Odometry                       #
    #####################################################

    EPUCK_MAX_WHEEL_SPEED = 0.11695 * SIM_TIMESTEP / 1000.0
    dsr = vR / MAX_SPEED * EPUCK_MAX_WHEEL_SPEED
    dsl = vL / MAX_SPEED * EPUCK_MAX_WHEEL_SPEED
    ds = (dsr + dsl) / 2.0

    pose_x += ds * math.cos(pose_theta)
    pose_y -= ds * math.sin(pose_theta)  # map +y is down
    pose_theta += (dsr - dsl) / EPUCK_AXLE_DIAMETER