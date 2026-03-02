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




# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = timestep/1000.0    # [s]








# Robot pose
# Adjust the initial values to match the initial robot pose in your simulation
x = -0.06    # position in x [m]
y = 0.436    # position in y [m]
phi = 0.0531  # orientation [rad]

# Robot velocity and acceleration
dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]


# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]


# e-puck Physical parameters for the kinematics model (constants)
R = 0.020    # radius of the wheels: 20.5mm [m]
D = 0.057    # distance between the wheels: 52mm [m]
# distance from the center of the wheels to the point of interest [m]
A = 0.05





# encoders
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

oldEncoderValues = []





def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    """Computes speed of the wheels based on encoder readings
    """
    # Calculate the change in angular position of the wheels:
    ang_diff_l = 2*np.pi*(encoderValues[0] - oldEncoderValues[0])/pulses_per_turn
    ang_diff_r = 2*np.pi*(encoderValues[1] - oldEncoderValues[1])/pulses_per_turn

    # Calculate the angular speeds:
    wl = ang_diff_l/delta_t
    wr = ang_diff_r/delta_t

    return wl, wr




def get_robot_speeds(wl, wr, R, D):
    u = R/2.0 * (wr + wl)
    w = R/D * (wr - wl)
    
    return u, w


def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    
    update_matrix(u, w, x_old, y_old, phi_old, delta_t)
    
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi

    delta_x = u * np.cos(phi) * delta_t
    delta_y = u * np.sin(phi) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y
    


    return x, y, phi





SIM_TIMESTEP = int(robot.getBasicTimeStep())



robotframe= np.array([[0],
            [0],
            [0]])


totalrobotframe=np.array([[0],
                 [0],
                [0]])

totalIframe=np.array([[0],
             [0],
             [0]])


tempIframe=np.array([[0],
             [0],
             [0]])


tmatrix=np.array([[0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]])



tempframe=np.array ([[0],
            [0],
            [0]])




invtmatrix=np.array([[0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]])

invrobotframe= np.array([[0],
            [0],
            [0]])

tempinvrobotframe= np.array([[0],
            [0],
            [0]])

#top is left,
#bottom is right.
invangleveloframe= np.array([[0],
            [0]])

theta=0

temptheta=0


def update_matrix(u, w, x_old, y_old, phi_old, delta_t):

    global totalIframe
    global tempIframe

    # --- 1. Compute new heading ---
    phi_new = phi_old + w * delta_t



    if phi_new >= np.pi:
        phi_new -= 2 * np.pi
    elif phi_new < -np.pi:
        phi_new += 2 * np.pi




    # --- 2. Build current world pose matrix T_k ---
    T_k = np.array([
        [np.cos(phi_old), -np.sin(phi_old), x_old],
        [np.sin(phi_old),  np.cos(phi_old), y_old],
        [0,                0,               1]
    ])



    # --- 3. Motion in robot frame (incremental motion) ---
    dT = np.array([
        [np.cos(w * delta_t), -np.sin(w * delta_t), u * delta_t],
        [np.sin(w * delta_t),  np.cos(w * delta_t), 0],
        [0,                    0,                   1]
    ])



    # --- 4. Compose transformations ---
    T_new = np.dot(T_k, dT)




    #this is another method of doing things, 
    #this basically just does the math outlined in lecture 3,
    #but multiplied by dt. 


    #print("x value: " + str(x_old + u*np.cos(phi_new)*delta_t) )
    #print("x value: " + str(x_old + u*np.cos(phi_old)*delta_t) )
    #print("y value: " + str(y_old + u*np.sin(phi_old)*delta_t) )
    #print("y value: " + str(y_old + u*np.sin(phi_new)*delta_t) )

    #print("theta val " + str(phi_old +  w * delta_t))
    


    #solve inverse kinematics for phi left and phi right. 
    rotvelosolver(u,w)


    #verify inverse kinematics.





    # --- 5. Extract new pose ---
    x_new = T_new[0, 2]
    y_new = T_new[1, 2]
    phi_new = np.arctan2(T_new[1, 0], T_new[0, 0])

    totalIframe = np.array([
        [x_new],
        [y_new],
        [phi_new]
    ])
    tempIframe=totalIframe

    IKrobotsolver()


def IKrobotsolver():

    #old globals
    global robotframe
    global totalrobotframe
    global totalIframe
    global tempframe
    global tmatrix
    global theta
    global tempIframe
    global pose_x
    global pose_y
    global pose_theta  

    #new globals
    global invtmatrix
    global invrobotframe
    global tempinvrobotframe
    global invangleveloframe


    invtmatrix=np.array([[math.cos(theta), math.sin(theta), 0],
                         [-math.sin(theta), math.cos(theta), 0],
                         [0, 0, 1]])
    
    tempinvrobotframe=np.dot(invtmatrix,totalIframe)
    IKanglevelosolver()





    
    #doesnt work that way
    #invrobotframe=np.add(tempinvrobotframe,invrobotframe)


    #there is some error when the robot turns 
    #when it comes to the inverse solving, 
    #it falsely solves for some y component of 
    #the robot frame being higher than zero 
    #which is impossile. 

    #this likely stems from the issues we had with the 
    #under-reporting of the angles


def IKanglevelosolver():

    #old globals
    global robotframe
    global totalrobotframe
    global totalIframe
    global tempframe
    global tmatrix
    global theta
    global tempIframe
    global pose_x
    global pose_y
    global pose_theta  

    #new globals
    global invtmatrix
    global invrobotframe
    global tempinvrobotframe
    global invangleveloframe
    global temptheta


    invtmatrix=np.array([[math.cos(temptheta), math.sin(temptheta), 0],
                         [-math.sin(temptheta), math.cos(temptheta), 0],
                         [0, 0, 1]])
    

    tempinvrobotframe=np.dot(invtmatrix,tempIframe)


    xrvelo=tempinvrobotframe[0][0]
    anglevelo=tempinvrobotframe[2][0]


def rotvelosolver(xrvelo,anglevelo):

    global invangleveloframe

    rotleft=((xrvelo-((anglevelo*D)/2)))/R
    rotright=((xrvelo+((anglevelo*D)/2)))/R
    print("rot left" + str(rotleft))
    print("rot right " +str(rotright))

    invangleveloframe=np.array([[rotleft],
            [rotright]])
    


















































# TODO Part 1: Setup Data structures for managing LiDAR data
#
# 1. Initialize an empty list to store LiDAR distance measurements.
# 2. Precompute lidar_ray_angles: the angles of each LiDAR ray (in radians):
#   The LiDAR has a total field of view of LIDAR_ANGLE_RANGE across
#   NUM_LIDAR_RAYS rays, evenly spaced with the middle ray at angle 0.
#   Hint: use np.linspace(start, stop, num=NUM_LIDAR_RAYS)





lidar_sensor_readings = []

# Precompute lidar ray angles (centered at 0)
lidar_sensor_readings = np.linspace(
    -LIDAR_ANGLE_RANGE/2,
    LIDAR_ANGLE_RANGE/2,
    NUM_LIDAR_RAYS
)

print(lidar_sensor_readings)

print(lidar_sensor_readings[10])







def report():
    print("CURRENT xPOSE " + str(pose_x) + " CURRENT yPOSE " +str(pose_y))





#### End of Part 1 #####

########################
# Part 2
########################
# TODO Part 2:Initialize the data structure for your map here

# Main Control Loop:
cnt = 0
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

    # TODO Part 4: Draw the obstacle, the robot's path, and free spaces on the map
    # Draw the occupied pixel in blue.
    # Draw the current robot’s visited pixel in red.
    # Draw a white line between the occupied pixel and the robot’s current visited pixel.
    # Please also refer to the instruction document for more details.
    # Obstacles

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
