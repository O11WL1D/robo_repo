# """csci3302_lab4 controller."""
# Copyright (2025) University of Colorado Boulder
# CSCI 3302: Introduction to Robotics

# You may need to import some classes of the controller module. Ex:
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Motor, DistanceSensor

# Change this to anything else to stay in place to test coordinate transform functions
state = "line_follower"

LIDAR_SENSOR_MAX_RANGE = 3  # Meters
NUM_LIDAR_RAYS = 21  # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708  # 90 degrees, 1.5708 radians

# These are your pose values that you will update by solving the odometry equations
pose_x = 0.197
pose_y = 0.65
pose_theta = np.pi/2

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice(
    'gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Initialize the Display
display = robot.getDevice("display")


# get and enable lidar
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()


##### DO NOT MODIFY ANY CODE ABOVE THIS #####















# TODO Part 1: Setup Data structures for managing LiDAR data
#
# 1. Initialize an empty list to store LiDAR distance measurements.
# 2. Precompute lidar_ray_angles: the angles of each LiDAR ray (in radians):
#   The LiDAR has a total field of view of LIDAR_ANGLE_RANGE across
#   NUM_LIDAR_RAYS rays, evenly spaced with the middle ray at angle 0.
#   Hint: use np.linspace(start, stop, num=NUM_LIDAR_RAYS)





lidar_sensor_readings = []

# Precompute lidar ray angles (centered at 0)
lidar_ray_angles = np.linspace(
    -LIDAR_ANGLE_RANGE/2,
    LIDAR_ANGLE_RANGE/2,
    NUM_LIDAR_RAYS
)

print(lidar_sensor_readings)

print("Middle Ray Angle (should be 0.0):", lidar_ray_angles[10])







def report():
    print("CURRENT xPOSE " + str(pose_x) + " CURRENT yPOSE " +str(pose_y))

DISPLAY_WIDTH = 300
DISPLAY_HEIGHT = 300

def world_to_display(x, y):
    px = int(x * DISPLAY_WIDTH)
    py = int(y * DISPLAY_HEIGHT)

    px = max(0, min(DISPLAY_WIDTH - 1, px))
    py = max(0, min(DISPLAY_HEIGHT - 1, py))

    return px, py



#### End of Part 1 #####

########################
# Part 2
########################
# TODO Part 2:Initialize the data structure for your map here

# Main Control Loop:
cnt = 0
visited_pixels = set()
occupied_pixels = set()
while robot.step(SIM_TIMESTEP) != -1:

    report()



    #####################################################
    #                 Sensing                           #
    #####################################################

    # Read ground sensors
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read Lidar
    lidar_sensor_readings = lidar.getRangeImage()

    

    # print(tuple(zip(lidar_sensor_readings, lidar_ray_angles)))



    # TODO Part 2: Transform the continuous map coordinates into the discrete locations on the display
    #
    # Come up with a way to transform the robot pose (in map coordinates)
    # into discrete locations on the display. Draw a red dot using display.drawPixel()
    # where the robot moves.

    display.setColor(0xFF0000)
    display.drawPixel(pose_x, pose_y)


    # Convert robot world pose to display pixel
    robot_px, robot_py = world_to_display(pose_x, pose_y)

    # Store visited pixel
    #visited_pixels.add((robot_px, robot_py))

    # Draw robot in red
    display.setColor(0xFF0000)
    display.drawPixel(robot_px, robot_py)




    # TODO Part 3: Convert Lidar data into world coordinates
    # For each LiDAR ray, if the distance measurement is less than the maximum range:
    #   1. compute the the object's coordinates in the robot's frame.
    #       - lidar_sensor_readings stores the distance measurements of each LiDAR ray (rhos).
    #       - lidar_ray_angles should store the angles of each LiDAR ray (alphas).
    #       - Please also refer to the instruction document to see how the LiDAR rays are oriented in the robot's frame.
    #   2. use the homogeneous transformation matrix to convert the object's coordinates from the robot's frame to the map coordinates.
    #       - The map coordinate system has its origin (0,0) at the top-left corner of the arena,
    #         and its x-axis increases to the right, and its y-axis increases downward.
    #       - In the robot frame, forward motion aligns with the +x-axis, and left corresponds to the +y-axis.
    #       - Please also refer to the instructions document to see how the robot's pose is defined in the map coordinate system.
    
    cos_t = math.cos(pose_theta)
    sin_t = math.sin(pose_theta)
    


    # construct homogenous transformation matrix
    T = np.array([
        [cos_t, -sin_t, pose_x], 
        [-sin_t, -cos_t, pose_y], 
        [0,      0,      1]      
    ])
    

    for i in range(NUM_LIDAR_RAYS):
        rho = lidar_sensor_readings[i]
        alpha = lidar_ray_angles[i]

        if rho < LIDAR_SENSOR_MAX_RANGE:
            # B_Q = relative object location in Robot Frame {XB, YB}
            rx = rho * math.cos(alpha)
            ry = -rho * math.sin(alpha)  
            
            # Form the augmented vector [B_Q | 1]
            B_Q = np.array([rx, ry, 1])

            # calculate for A_Q = location in global reference frame {XA, YA}
            # matrix multiplication [A_Q; 1] = T * [B_Q; 1]
            A_Q = T @ B_Q
            
            # Extract coordinates from the resulting vector
            mx = A_Q[0]
            my = A_Q[1]

            
            

    # TODO Part 4: Draw the obstacle, the robot's path, and free spaces on the map
    # Draw the occupied pixel in blue.
    # Draw the current robot’s visited pixel in red.
    # Draw a white line between the occupied pixel and the robot’s current visited pixel.
    # Please also refer to the instruction document for more details.
    # Obstacles
    
    # Convert world coordinates to display pixels
            obj_px, obj_py = world_to_display(mx, my)
            
            #needed for final redraw 
            occupied_pixels.add((obj_px, obj_py))

            #drawing sequence
            
            #white line for free space between robot and wall
            display.setColor(0xFFFFFF)
            display.drawLine(robot_px, robot_py, obj_px, obj_py)
            
            #blue pixel for walls and obstacles
            display.setColor(0x0000FF)
            display.drawPixel(obj_px, obj_py)
            
            #red pixels for robot current pos
            display.setColor(0xFF0000)
            display.drawPixel(robot_px, robot_py)
    

    # Except for Part 0A (as noted in the instructions) and the "TODO Part 4", PLEASE DO NOT MODIFY ANY CODE BELOW THIS LINE
    #####################################################
    #                 Robot controller                  #
    #####################################################

    if state == "line_follower":
        if (gsr[1] < 350 and gsr[0] > 400 and gsr[2] > 400):
            vL = MAX_SPEED*0.3
            vR = MAX_SPEED*0.3
        # Checking for Start Line
        elif (gsr[0] < 310 and gsr[1] < 310 and gsr[2] < 310):
            cnt += 1
            vL = MAX_SPEED*0.3
            vR = MAX_SPEED*0.3
            if cnt > 10:
                cnt = 0
                # Feel free to comment this to make your terminal cleaner
                print("Over the line!")
                # TODO Part 4: Save the map clearly showing the robot's path and the obstacles.
                # Before saving the map,
                # 1. draw ALL the robot's visited pixels again
                # 2. draw ALL the occupied pixels in blue again

                display.imageSave(None, "map.png")

        elif (gsr[2] < 650):  # turn right
            vL = 0.2*MAX_SPEED
            vR = -0.05*MAX_SPEED
            cnt = 0
        elif (gsr[0] < 650):  # turn left
            vL = -0.05*MAX_SPEED
            vR = 0.2*MAX_SPEED
            cnt = 0

    else:
        # Stationary State
        vL = 0
        vR = 0

    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)


    #  PLEASE DO NOT MODIFY THE FOLLOWING CODE
    #####################################################
    #                    Odometry                       #
    #####################################################

    EPUCK_MAX_WHEEL_SPEED = 0.11695*SIM_TIMESTEP/1000.0
    dsr = vR/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    dsl = vL/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    ds = (dsr+dsl)/2.0

    pose_x += ds*math.cos(pose_theta)
    pose_y -= ds*math.sin(pose_theta)
    pose_theta += (dsr-dsl)/EPUCK_AXLE_DIAMETER

    # Feel free to uncomment this for debugging
    # print("X: %f Y: %f Theta: %f " % (pose_x, pose_y, pose_theta))   