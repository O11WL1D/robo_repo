"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
import numpy as np
from enum import Enum
from controller import Supervisor, Keyboard

pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257  # ePuck wheel speed in m/s (initial guess)
EPUCK_WHEEL_RADIUS = 0.025
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# -------------------------
# Keyboard control (press keys to switch modes)
# -------------------------
keyboard = robot.getKeyboard()
keyboard.enable(SIM_TIMESTEP)

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for _ in range(10):
    robot.step(SIM_TIMESTEP)

# Initialize gps and compass for odometry / ground-truth pose
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
waypoints = [
    [-0.164262, -0.353056],
    [0.313468,  -0.408795],
    [0.327901,  -0.233657],
    [0.120393,  -0.125894],
    [0.125596,   0.0856912],
    [0.352334,   0.281844],
    [0.066142,   0.422328],
    [-0.296528,  0.418701],
    [-0.15912,   0.236733],
    [-0.199771, -0.0211245],
    [-0.306611, -0.23925],
]

# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  ↓ OUR CODE ↓


class STATES(Enum):
    speed_measurement = 1
    line_follower = 2


class SUBSTATES(Enum):
    Drive_Forward = 1
    Start_Line_Detection = 2
    Stop = 3
    Calculate_Speed = 4
    # line follower state sub states
    Center_Sensor_detects_line = 5
    Left_Sensor_detects_line = 6
    Right_Sensor_detects_line = 7
    No_Sensors_Detect_Line = 8


robotstate = STATES.speed_measurement
robotsubstate = SUBSTATES.Drive_Forward

# init sensors.
left_wheel_sensor = robot.getDevice("left wheel sensor")
right_wheel_sensor = robot.getDevice("right wheel sensor")
left_wheel_sensor.enable(SIM_TIMESTEP)
right_wheel_sensor.enable(SIM_TIMESTEP)

angle_of_rotation_left_total = left_wheel_sensor.getValue()   # radians
angle_of_rotation_right_total = right_wheel_sensor.getValue() # radians

prevleft = 0.0
prevright = 0.0
prevtime = 0.0

diffleft = 0.0
diffright = 0.0

infvelofrotleft = 0.0
infvelofrotright = 0.0

ldetectioncnt = 0

# variables used by report()
leftsensordetection = False
centersensordetection = False
rightsensordetection = False
linedetected = False
currenttime = 0.0


# robot output function, please try to have all output go in here
# so that it can be customized.
def report(option, message):
    global pose_x, pose_y, pose_theta
    global gsr, currenttime
    global leftsensordetection, centersensordetection, rightsensordetection
    global angle_of_rotation_left_total, angle_of_rotation_right_total
    global diffleft, diffright, infvelofrotleft, infvelofrotright
    global linedetected, ldetectioncnt
    global totalrobotframe, totalIframe, tempinvrobotframe
    global theta, invangleveloframe
    global robotstate, robotsubstate

    if option == 0:
        print("CURRENT ROBOT STATE:  " + str(robotstate) + "  CURRENT ROBOT SUBSTATE:    " + str(robotsubstate))
        print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
        print("GROUND SENSOR VALUES: " + str(gsr))
        print("ELAPSED TIME: " + str(currenttime))
        print("Left detection? : " + str(leftsensordetection) +
              " center detection? " + str(centersensordetection) +
              " right detection? " + str(rightsensordetection))
        print(message)

        print("left_Wheel angle (rad):", angle_of_rotation_left_total)
        print("right_Wheel angle (rad):", angle_of_rotation_right_total)

        print("left_Wheel angle inf (rad):", diffleft)
        print("right_Wheel angle inf (rad):", diffright)

        print("left_Wheel angle velo inf (rad):", infvelofrotleft)
        print("right_Wheel angle velo inf (rad):", infvelofrotright)
        print("Line detected?  " + str(linedetected))
        print("line detected count " + str(ldetectioncnt))
        print("total Robot frame: \n", totalrobotframe)
        print("total I frame: \n", totalIframe)
        print("temp Inverse solved robot frame: \n", tempinvrobotframe)
        print("Theta " + str(theta))

    if option == 1:
        print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))

    if option == 2:
        print("CURRENT ROBOT STATE:  " + str(robotstate) + "  CURRENT ROBOT SUBSTATE:    " + str(robotsubstate))
        print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
        print("Temp angle velos" + str(invangleveloframe))


def loopclosure2():
    global pose_theta, pose_y, pose_x, linedetected
    if linedetected:
        print("LOOP CLOSURE!!! RESETTING POSE")
        pose_x, pose_y, pose_theta = 0, 0, 0
        resetmatricies()


def find_infi_left_angle_rot(totleft):
    global prevleft
    difference = totleft - prevleft
    prevleft = totleft
    return difference


def find_infi_right_angle_rot(totright):
    global prevright
    difference = totright - prevright
    prevright = totright
    return difference


def find_inf_time(now):
    global prevtime
    difference = now - prevtime
    prevtime = now
    return difference


def calc_velocity(distance, time):
    if time <= 0:
        return 0.0
    return distance / time


# Odometry
def update_odometry(vL, vR, delta_time):
    global pose_x, pose_y, pose_theta
    global EPUCK_MAX_WHEEL_SPEED, MAX_SPEED, EPUCK_AXLE_DIAMETER

    vL_mps = (vL / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
    vR_mps = (vR / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED

    dist_left = vL_mps * delta_time
    dist_right = vR_mps * delta_time

    dist_center = (dist_left + dist_right) / 2.0
    delta_theta = (dist_right - dist_left) / EPUCK_AXLE_DIAMETER

    pose_x += dist_center * math.cos(pose_theta)
    pose_y += dist_center * math.sin(pose_theta)
    pose_theta += delta_theta


robotframe = np.array([[0], [0], [0]])
totalrobotframe = np.array([[0], [0], [0]])
totalIframe = np.array([[0], [0], [0]])
tempIframe = np.array([[0], [0], [0]])
tmatrix = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
tempframe = np.array([[0], [0], [0]])

invtmatrix = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
invrobotframe = np.array([[0], [0], [0]])
tempinvrobotframe = np.array([[0], [0], [0]])
invangleveloframe = np.array([[0], [0]])

theta = 0.0
temptheta = 0.0


def resetmatricies():
    global robotframe, totalrobotframe, totalIframe, tempframe, tmatrix, theta, tempIframe
    robotframe = np.array([[0], [0], [0]])
    totalrobotframe = np.array([[0], [0], [0]])
    totalIframe = np.array([[0], [0], [0]])
    tempIframe = np.array([[0], [0], [0]])
    tmatrix = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    tempframe = np.array([[0], [0], [0]])
    theta = 0.0


def update_odometry2(infveloleft, infveloright):
    global totalrobotframe, totalIframe, tempframe, tmatrix, theta, tempIframe
    global pose_x, pose_y, pose_theta
    global temptheta

    correctionfactor = 1.37

    tempframe = np.array([[
        (((infveloleft * EPUCK_WHEEL_RADIUS)) + ((infveloright * EPUCK_WHEEL_RADIUS))) / 2.0
    ], [0], [
        correctionfactor * math.radians(((infveloright * EPUCK_WHEEL_RADIUS) - (infveloleft * EPUCK_WHEEL_RADIUS)) / (EPUCK_AXLE_DIAMETER))
    ]])

    totalrobotframe = np.add(tempframe, totalrobotframe)
    theta = float(totalrobotframe[2][0])

    tmatrix = np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta),  math.cos(theta), 0],
        [0,               0,               1]
    ])

    tempIframe = np.dot(tmatrix, tempframe)
    totalIframe = np.add(totalIframe, tempIframe)
    pose_x, pose_y, pose_theta = float(totalIframe[0][0]), float(totalIframe[1][0]), theta

    temptheta = float(tempframe[2][0])
    IKanglevelosolver()


def IKanglevelosolver():
    global invtmatrix, tempinvrobotframe, invangleveloframe, temptheta, tempIframe

    invtmatrix = np.array([
        [math.cos(temptheta),  math.sin(temptheta), 0],
        [-math.sin(temptheta), math.cos(temptheta), 0],
        [0,                   0,                   1]
    ])

    tempinvrobotframe = np.dot(invtmatrix, tempIframe)

    xrvelo = float(tempinvrobotframe[0][0])
    anglevelo = float(tempinvrobotframe[2][0])

    rotleft = ((xrvelo - ((anglevelo * EPUCK_AXLE_DIAMETER) / 2.0))) / EPUCK_WHEEL_RADIUS
    rotright = ((xrvelo + ((anglevelo * EPUCK_AXLE_DIAMETER) / 2.0))) / EPUCK_WHEEL_RADIUS

    invangleveloframe = np.array([[rotleft], [rotright]])


# Added for Lab 3 Part 2/3 controllers
class CONTROL_MODES(Enum):
    line_following = 1
    turn_drive_turn_control = 2
    proportional_controller = 3


control_mode = CONTROL_MODES.proportional_controller


def mode_name(m):
    if m == CONTROL_MODES.line_following:
        return "LINE_FOLLOWING"
    if m == CONTROL_MODES.proportional_controller:
        return "WAYPOINT_PROP"
    if m == CONTROL_MODES.turn_drive_turn_control:
        return "WAYPOINT_TDT"
    return str(m)


class TDT_STATES(Enum):
    turn_to_goal = 1
    drive_to_goal = 2
    turn_to_heading = 3


tdt_state = TDT_STATES.turn_to_goal


def wrap_to_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def read_ground_truth_pose():
    gx = gps.getValues()[0]
    gy = gps.getValues()[1]
    cv = compass.getValues()
    gtheta = np.atan2(cv[0], cv[1])
    return gx, gy, gtheta


def compute_errors(xr, yr, thetar, xg, yg, thetag):
    dx = xg - xr
    dy = yg - yr
    rho = np.sqrt(dx * dx + dy * dy)
    goal_angle = np.atan2(dy, dx)
    alpha = wrap_to_pi(goal_angle - thetar)
    eta = wrap_to_pi(thetag - thetar)
    return rho, alpha, eta, goal_angle


def ik_from_vw(v, w):
    wl = (v - w * (EPUCK_AXLE_DIAMETER / 2.0)) / EPUCK_WHEEL_RADIUS
    wr = (v + w * (EPUCK_AXLE_DIAMETER / 2.0)) / EPUCK_WHEEL_RADIUS
    wl = clamp(wl, -MAX_SPEED, MAX_SPEED)
    wr = clamp(wr, -MAX_SPEED, MAX_SPEED)
    return wl, wr


USE_GROUND_TRUTH_POSE = True


# WAYPOINT FSM (STATE)
class WP_STATE(Enum):
    go_to_waypoint = 1
    advance_waypoint = 2


wp_state = WP_STATE.go_to_waypoint
RHO_TOL = 0.05  # if still orbiting, raise to 0.06

groundthresh = 600
currenttime = 0.0

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
    delta_time = SIM_TIMESTEP / 1000.0
    currenttime = robot.getTime()

    # defaults so we never use uninitialized speeds
    leftSpeed = 0.0
    rightSpeed = 0.0

    # Key press: C = line following, V = waypoint controller
    key = keyboard.getKey()
    if key != -1:
        if key == ord('C') or key == ord('c'):
            control_mode = CONTROL_MODES.line_following
            print("Switched mode ->", mode_name(control_mode))
        elif key == ord('V') or key == ord('v'):
            control_mode = CONTROL_MODES.proportional_controller
            print("Switched mode ->", mode_name(control_mode))
        elif key == ord('B') or key == ord('b'):
            control_mode = CONTROL_MODES.turn_drive_turn_control
            print("Switched mode ->", mode_name(control_mode))

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    leftsensordetection = (gsr[0] < groundthresh)
    centersensordetection = (gsr[1] < groundthresh)
    rightsensordetection = (gsr[2] < groundthresh)

    linedetected = ((gsr[0] < groundthresh) and (gsr[2] < groundthresh) and (gsr[1] < groundthresh))
    offtrack = (not leftsensordetection and not centersensordetection and not rightsensordetection)

    if linedetected:
        ldetectioncnt += 1

    if USE_GROUND_TRUTH_POSE:
        gt_x, gt_y, gt_theta = read_ground_truth_pose()
    else:
        gt_x, gt_y, gt_theta = pose_x, pose_y, pose_theta

    # Waypoints
    if len(waypoints) > 0:
        x_goal = waypoints[index][0]
        y_goal = waypoints[index][1]

        # KEEP YOUR MARKER CALL EXACTLY THE SAME
        marker.setSFVec3f([x_goal, y_goal, 0.0199956])

        rho, alpha, eta, goal_angle = compute_errors(gt_x, gt_y, gt_theta, x_goal, y_goal, gt_theta)
        theta_goal = goal_angle
        rho, alpha, eta, goal_angle = compute_errors(gt_x, gt_y, gt_theta, x_goal, y_goal, theta_goal)
    else:
        x_goal, y_goal, theta_goal = gt_x, gt_y, gt_theta
        rho, alpha, eta, goal_angle = 0.0, 0.0, 0.0, gt_theta

    # WAYPOINT FSM LOGIC
    if len(waypoints) > 0:
        if wp_state == WP_STATE.go_to_waypoint:
            if rho < RHO_TOL:
                wp_state = WP_STATE.advance_waypoint
        elif wp_state == WP_STATE.advance_waypoint:
            index = (index + 1) % len(waypoints)
            wp_state = WP_STATE.go_to_waypoint

    # Debug
    if int(robot.getTime()) % 2 == 0:
        print("MODE:", control_mode, "WP_INDEX:", index, "WP_STATE:", wp_state, "GOAL:", x_goal, y_goal,
              "POSE:", gt_x, gt_y, "RHO:", rho, "ALPHA:", alpha, "ETA:", eta)

    # Controller selection
    if control_mode == CONTROL_MODES.proportional_controller and len(waypoints) > 0:
        K_RHO = 4.0
        K_ALPHA = 5.0
        K_ETA = 1.0

        v_cmd = clamp(K_RHO * rho, 0.0, 0.20)
        w_cmd = (K_ALPHA * alpha) + (K_ETA * eta)

        leftSpeed, rightSpeed = ik_from_vw(v_cmd, w_cmd)

    elif control_mode == CONTROL_MODES.turn_drive_turn_control and len(waypoints) > 0:
        ALPHA_TOL = 0.05
        RHO_TOL_TDT = 0.03
        ETA_TOL = 0.05

        K_TURN = 2.0
        K_DRIVE = 4.0

        if tdt_state == TDT_STATES.turn_to_goal:
            if abs(alpha) > ALPHA_TOL:
                v_cmd = 0.0
                w_cmd = K_TURN * alpha
            else:
                tdt_state = TDT_STATES.drive_to_goal
                v_cmd = 0.0
                w_cmd = 0.0

        if tdt_state == TDT_STATES.drive_to_goal:
            if rho > RHO_TOL_TDT:
                v_cmd = clamp(K_DRIVE * rho, 0.0, 0.20)
                w_cmd = 0.0
            else:
                tdt_state = TDT_STATES.turn_to_heading
                v_cmd = 0.0
                w_cmd = 0.0

        if tdt_state == TDT_STATES.turn_to_heading:
            if abs(eta) > ETA_TOL:
                v_cmd = 0.0
                w_cmd = K_TURN * eta
            else:
                index = (index + 1) % len(waypoints)
                tdt_state = TDT_STATES.turn_to_goal
                v_cmd = 0.0
                w_cmd = 0.0

        leftSpeed, rightSpeed = ik_from_vw(v_cmd, w_cmd)

    elif control_mode == CONTROL_MODES.line_following:
        # kept your original line-following behavior
        if centersensordetection:
            leftSpeed = MAX_SPEED * 0.85
            rightSpeed = MAX_SPEED * 0.85
        else:
            rotamt = 0.08
            if leftsensordetection:
                leftSpeed = -MAX_SPEED * rotamt
                rightSpeed = MAX_SPEED * rotamt
            elif rightsensordetection:
                leftSpeed = MAX_SPEED * rotamt
                rightSpeed = -MAX_SPEED * rotamt
            else:
                leftSpeed = -MAX_SPEED * rotamt
                rightSpeed = MAX_SPEED * rotamt

    # -------------------------
    # odometry calculations
    # -------------------------
    angle_of_rotation_left_total = left_wheel_sensor.getValue()
    angle_of_rotation_right_total = right_wheel_sensor.getValue()

    diffright = find_infi_right_angle_rot(angle_of_rotation_right_total)
    diffleft = find_infi_left_angle_rot(angle_of_rotation_left_total)

    infvelofrotleft = calc_velocity(diffleft, delta_time)
    infvelofrotright = calc_velocity(diffright, delta_time)

    if ldetectioncnt:
        update_odometry2(infvelofrotleft, infvelofrotright)

    if int(robot.getTime() * 5) % 5 == 0:
        report(2, currenttime)

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)


