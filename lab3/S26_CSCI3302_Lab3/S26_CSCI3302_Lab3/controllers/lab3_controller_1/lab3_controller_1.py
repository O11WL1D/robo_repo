"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np

import math
import numpy as np
from enum import Enum
from controller import Robot, Motor, DistanceSensor



pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28

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
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)

vL = 0
vR = 0


# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)


# TODO: Find waypoints to navigate around the arena while avoiding obstacles
waypoints = [
    [-0.164262, -0.353056],  # manually added waypoint
]

# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")


#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  ↓ OUR CODE ↓



class STATES(Enum):
    speed_measurement=1
    line_follower=2



class SUBSTATES(Enum):
    Drive_Forward=1
    Start_Line_Detection=2
    Stop=3
    Calculate_Speed=4
    #line follower state sub states
    Center_Sensor_detects_line=5
    Left_Sensor_detects_line=6
    Right_Sensor_detects_line=7
    No_Sensors_Detect_Line=8


robotstate=STATES.speed_measurement
robotsubstate=SUBSTATES.Drive_Forward




#init sensors.
left_wheel_sensor = robot.getDevice("left wheel sensor")
right_wheel_sensor = robot.getDevice("right wheel sensor")
left_wheel_sensor.enable(SIM_TIMESTEP)
right_wheel_sensor.enable(SIM_TIMESTEP)

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
# TODO: set the ePuck wheel speed in m/s after measuring the speed (Part 1)
EPUCK_MAX_WHEEL_SPEED = 0
EPUCK_WHEEL_RADIUS = 0.025
MAX_SPEED = 6.28




angle_of_rotation_left_total = left_wheel_sensor.getValue()  # radians
angle_of_rotation_right_total = right_wheel_sensor.getValue()  # radians

prevleft=0
prevright=0
prevtime=0

diffleft=0
diffright=0

infvelofrotleft=0
infvelofrotright=0

inf_time=0

ldetectioncnt=0



# robot output function, please try to have all output go in here
# so that it can be customized.
def report(option, message):

    if(option==0):
        print("CURRENT ROBOT STATE:  " + str(robotstate)+ "  CURRENT ROBOT SUBSTATE:    " + str(robotsubstate))
        print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
        print("GROUND SENSOR VALUES: " + str(gsr))
        print("ELAPSED TIME: " + str(currenttime))
        print("Left detection? : " + str(leftsensordetection) + " center detection? " + str(centersensordetection) + " right detection? " + str(rightsensordetection))
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
        #print("full Inverse solved robot frame: \n", invrobotframe)

        print("Theta " + str(theta))



        #print("inf_time :", inf_time)

    if(option==1):
         print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))


    #inv kinematics troubleshoot
    if(option==2):
        print("CURRENT ROBOT STATE:  " + str(robotstate)+ "  CURRENT ROBOT SUBSTATE:    " + str(robotsubstate))
        print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))



        print("left_Wheel angle (rad):", angle_of_rotation_left_total)
        print("right_Wheel angle (rad):", angle_of_rotation_right_total)

        print("left_Wheel angle inf (rad):", diffleft)
        print("right_Wheel angle inf (rad):", diffright)

        print("left_Wheel angle velo inf (rad):", infvelofrotleft)
        print("right_Wheel angle velo inf (rad):", infvelofrotright)

        print("total Robot frame: \n", totalrobotframe)
        print("total I frame: \n", totalIframe)

        print("temp Inverse solved robot frame: \n", tempinvrobotframe)
        #print("full Inverse solved robot frame: \n", invrobotframe)

        print("Theta " + str(theta))

        print("Temp angle velos" + str(invangleveloframe))






def loopclosure2():


        global leftsensordetection
        global centersensordetection
        global rightsensordetection
        global start_line_timer
        global start_line_time
        global pose_theta
        global pose_y
        global pose_x
        global linedetected

        if(linedetected):
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            pose_x, pose_y, pose_theta=0,0,0
            resetmatricies()





def find_infi_left_angle_rot(totleft):
     global prevleft
     difference=totleft-prevleft
     prevleft=totleft
     return difference



def find_infi_right_angle_rot(totright):
     global prevright
     difference=totright-prevright
     prevright=totright
     return difference

def find_inf_time(currenttime):
     global prevtime
     difference=currenttime-prevtime
     prevtime=currenttime
     return difference

def calc_velocity(distance,time):
     return (distance/time)



def calculate_inf_velo_matrix(rightinf):
     1==1





#Odometry
def update_odometry(vL, vR, delta_time):
    global pose_x, pose_y, pose_theta


    # normalize and scale with speeds
    vL_mps = (vL / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
    vR_mps = (vR / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED

    #find the distances
    dist_left = vL_mps * delta_time
    dist_right = vR_mps * delta_time

    # find the robot's linear and angular displacement
    dist_center = (dist_left + dist_right) / 2.0
    delta_theta = (dist_right - dist_left) / EPUCK_AXLE_DIAMETER

    # update pose
    pose_x += dist_center * math.cos(pose_theta)
    pose_y += dist_center * math.sin(pose_theta)
    pose_theta += delta_theta






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


def resetmatricies():
    global robotframe
    global totalrobotframe
    global totalIframe
    global tempframe
    global tmatrix
    global theta
    global tempIframe
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


    theta=0





def update_odometry2(infveloleft,infveloright):
     1==1
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

     #correction factor was calculated like this:
     # pre correction factor reported radians turned after 90 deg = 1.15
     # correct num radians = 1.57= 90 deg
     # 1.57 = x (1.15)
     # x= correctionfactor= 1.57/1.15

     correctionfactor=1.37



     tempframe=np.array([[(((infveloleft*EPUCK_WHEEL_RADIUS))  + ((infveloright*EPUCK_WHEEL_RADIUS) ))/(2)],
                                                                   [0],
                   [  correctionfactor*math.radians(((infveloright*EPUCK_WHEEL_RADIUS)  - (infveloleft*EPUCK_WHEEL_RADIUS) )/(EPUCK_AXLE_DIAMETER))  ]])


     totalrobotframe=np.add(tempframe,totalrobotframe)

     theta = totalrobotframe[2][0]

     tmatrix=np.array([[math.cos(theta), -math.sin(theta), 0],
                      [math.sin(theta), math.cos(theta), 0],
                      [0, 0, 1]])


     tempIframe=np.dot(tmatrix,tempframe)

     totalIframe=np.add(totalIframe,tempIframe)
     pose_x, pose_y, pose_theta=totalIframe[0][0] ,totalIframe[1][0] ,theta

     #now time to check if inverse kinematics works on temp frames.

     temptheta=tempframe[2][0]
     IKanglevelosolver()






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


    rotvelosolver(xrvelo,anglevelo)






def rotvelosolver(xrvelo,anglevelo):

    global invangleveloframe

    rotleft=((xrvelo-((anglevelo*EPUCK_AXLE_DIAMETER)/2)))/EPUCK_WHEEL_RADIUS
    rotright=((xrvelo+((anglevelo*EPUCK_AXLE_DIAMETER)/2)))/EPUCK_WHEEL_RADIUS

    invangleveloframe= np.array([[rotleft],
            [rotright]])



# Added for Lab 3 Part 2/3 controllers
class CONTROL_MODES(Enum):
    line_following=1
    turn_drive_turn_control=2
    proportional_controller=3

# Added for Lab 3 Part 2/3 controllers
control_mode=CONTROL_MODES.proportional_controller

class TDT_STATES(Enum):
    turn_to_goal=1
    drive_to_goal=2
    turn_to_heading=3

tdt_state=TDT_STATES.turn_to_goal

def wrap_to_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def read_ground_truth_pose():
    # GPS gives world x,z on the ground plane; y is height
    gx = gps.getValues()[0]
    gy = gps.getValues()[1]  # FIX: your world uses x,y as ground plane
    cv = compass.getValues()
    gtheta = np.arctan2(cv[0], cv[1])  # FIX: heading for x,y plane
    return gx, gy, gtheta

def compute_errors(xr, yr, thetar, xg, yg, thetag):
    dx = xg - xr
    dy = yg - yr
    rho = np.sqrt(dx*dx + dy*dy)
    goal_angle = np.arctan2(dy, dx)
    alpha = wrap_to_pi(goal_angle - thetar)
    eta = wrap_to_pi(thetag - thetar)
    return rho, alpha, eta, goal_angle

def ik_from_vw(v, w):
    # v in m/s, w in rad/s -> wheel angular speeds in rad/s
    wl = (v - w*(EPUCK_AXLE_DIAMETER/2.0)) / EPUCK_WHEEL_RADIUS
    wr = (v + w*(EPUCK_AXLE_DIAMETER/2.0)) / EPUCK_WHEEL_RADIUS
    wl = clamp(wl, -MAX_SPEED, MAX_SPEED)
    wr = clamp(wr, -MAX_SPEED, MAX_SPEED)
    return wl, wr

# Added for Lab 3 Part 2/3 controllers
USE_GROUND_TRUTH_POSE=True



groundthresh=600
groundcount=0
currenttime=0

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    delta_time = SIM_TIMESTEP / 1000.0


    currenttime = robot.getTime()

    # defaults so we never use uninitialized speeds
    leftSpeed  = 0.0
    rightSpeed = 0.0

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()


    leftsensordetection=(gsr[0]<groundthresh)
    centersensordetection=(gsr[1]<groundthresh)
    rightsensordetection=(gsr[2]<groundthresh)
    paststart=(not leftsensordetection and not centersensordetection and not rightsensordetection)

    rightcliff=(centersensordetection and not rightsensordetection and leftsensordetection)

    # NOTE: removed theta gating so "linedetected" means what it says (all three sensors see line)
    linedetected= ((gsr[0]<groundthresh) and (gsr[2]<groundthresh) and (gsr[1]<groundthresh))

    # offtrack recovery (copied behavior from your experimental controller)
    offtrack=(not leftsensordetection and not centersensordetection and not rightsensordetection)

    if(linedetected):
         ldetectioncnt+=1

    # Added for Lab 3 Part 2/3 controllers
    if USE_GROUND_TRUTH_POSE:
        gt_x, gt_y, gt_theta = read_ground_truth_pose()
    else:
        gt_x, gt_y, gt_theta = pose_x, pose_y, pose_theta

    # Added for Lab 3 Part 2/3 controllers
    if len(waypoints) > 0:
        x_goal = waypoints[index][0]
        y_goal = waypoints[index][1]
        marker.setSFVec3f([x_goal, y_goal, 0.0199956])
        rho, alpha, eta, goal_angle = compute_errors(gt_x, gt_y, gt_theta, x_goal, y_goal, gt_theta)
        theta_goal = goal_angle
        rho, alpha, eta, goal_angle = compute_errors(gt_x, gt_y, gt_theta, x_goal, y_goal, theta_goal)
    else:
        x_goal, y_goal, theta_goal = gt_x, gt_y, gt_theta
        rho, alpha, eta, goal_angle = 0.0, 0.0, 0.0, gt_theta

    # Added for debugging waypoint controller (prints every ~2 seconds)
    if int(robot.getTime()) % 2 == 0:
        print("MODE:", control_mode, "WP_INDEX:", index, "GOAL:", x_goal, y_goal, "POSE:", gt_x, gt_y, "RHO:", rho, "ALPHA:", alpha, "ETA:", eta)

    # Added for Lab 3 Part 2/3 controllers
    if control_mode==CONTROL_MODES.line_following:
        if(robotstate==STATES.speed_measurement):
                1==1


                if(robotsubstate==SUBSTATES.Drive_Forward):
                    leftSpeed  =  MAX_SPEED
                    rightSpeed = MAX_SPEED


                    if(linedetected):
                        robotsubstate=SUBSTATES.Stop



                if(robotsubstate==SUBSTATES.Stop):
                    leftSpeed  =  0
                    rightSpeed = 0
                    robotsubstate=SUBSTATES.Calculate_Speed



                if(robotsubstate==SUBSTATES.Calculate_Speed):
                    WHEEL_RADIUS = 0.025
                    distance_left = angle_of_rotation_left_total * WHEEL_RADIUS
                    print("DISTANCE LEFT " + str(distance_left))

                    EPUCK_MAX_WHEEL_SPEED = distance_left / currenttime
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                    print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")

                    #todo, calculate linear translation distance and store
                    #in the var EPUCK_MAX_WHEEL_SPEED
                    #This allows you to utilize speed in m/s for future calculations without measuring wheel diameter.

                    robotstate=STATES.line_follower
                    robotsubstate=SUBSTATES.Center_Sensor_detects_line



        if(robotstate==STATES.line_follower):


                #loopclosure()
                # NOTE: loopclosure can cause surprises mid-run; keep disabled unless you explicitly want pose reset
                #loopclosure2()

                # choose ONE substate (priority) so it doesn't overwrite itself
                if(centersensordetection):
                     robotsubstate=SUBSTATES.Center_Sensor_detects_line
                elif(leftsensordetection):
                     robotsubstate=SUBSTATES.Left_Sensor_detects_line
                elif(rightsensordetection):
                     robotsubstate=SUBSTATES.Right_Sensor_detects_line
                else:
                     robotsubstate=SUBSTATES.Left_Sensor_detects_line

                if(rightcliff):
                     #print("RIGHT CLIFF")
                     robotsubstate=SUBSTATES.Left_Sensor_detects_line

                if(offtrack):
                     robotsubstate=SUBSTATES.Left_Sensor_detects_line


                if(robotsubstate==SUBSTATES.Center_Sensor_detects_line):
                    # keep forward speed slightly below max so we don't outrun corners
                    leftSpeed  =  MAX_SPEED * 0.85  # small tuning knob
                    rightSpeed =  MAX_SPEED * 0.85

                else:

                    rotamt=0.08

                    if(robotsubstate==SUBSTATES.Left_Sensor_detects_line):
                        leftSpeed  = -MAX_SPEED*rotamt
                        rightSpeed = MAX_SPEED*rotamt


                    if(robotsubstate==SUBSTATES.Right_Sensor_detects_line):
                        leftSpeed  = MAX_SPEED*rotamt
                        rightSpeed = -MAX_SPEED*rotamt

        # Added for Lab 3 Part 2/3 controllers
        # print out the error terms as the robot does line following
        if len(waypoints) > 0 and int(robot.getTime()*2) % 2 == 0:
            print("RHO:", rho, "ALPHA:", alpha, "ETA:", eta)

    # Added for Lab 3 Part 2/3 controllers
    if control_mode==CONTROL_MODES.turn_drive_turn_control and len(waypoints) > 0:
        ALPHA_TOL=0.05
        RHO_TOL=0.03
        ETA_TOL=0.05

        K_TURN=2.0
        K_DRIVE=4.0

        # rotate in place until facing goal
        if tdt_state==TDT_STATES.turn_to_goal:
            if abs(alpha) > ALPHA_TOL:
                v_cmd=0.0
                w_cmd=K_TURN*alpha
            else:
                tdt_state=TDT_STATES.drive_to_goal
                v_cmd=0.0
                w_cmd=0.0

        # drive forward until close to goal
        if tdt_state==TDT_STATES.drive_to_goal:
            if rho > RHO_TOL:
                v_cmd=clamp(K_DRIVE*rho, 0.0, 0.20)
                w_cmd=0.0
            else:
                tdt_state=TDT_STATES.turn_to_heading
                v_cmd=0.0
                w_cmd=0.0

        # rotate to final heading
        if tdt_state==TDT_STATES.turn_to_heading:
            if abs(eta) > ETA_TOL:
                v_cmd=0.0
                w_cmd=K_TURN*eta
            else:
                # waypoint reached
                index=(index+1) % len(waypoints)
                tdt_state=TDT_STATES.turn_to_goal
                v_cmd=0.0
                w_cmd=0.0

        leftSpeed, rightSpeed = ik_from_vw(v_cmd, w_cmd)

    # Added for Lab 3 Part 2/3 controllers
    if control_mode==CONTROL_MODES.proportional_controller and len(waypoints) > 0:
        K_RHO=4.0
        K_ALPHA=3.0
        K_ETA=1.0

        v_cmd = clamp(K_RHO*rho, 0.0, 0.20)
        w_cmd = (K_ALPHA*alpha) + (K_ETA*eta)

        leftSpeed, rightSpeed = ik_from_vw(v_cmd, w_cmd)

        if rho < 0.03 and abs(alpha) < 0.08:
            index=(index+1) % len(waypoints)

    #odometry calculations.
    angle_of_rotation_left_total = left_wheel_sensor.getValue()  # radians
    angle_of_rotation_right_total = right_wheel_sensor.getValue()  # radians
    diffright=find_infi_right_angle_rot(angle_of_rotation_right_total) #radians per step.
    diffleft=find_infi_left_angle_rot(angle_of_rotation_left_total) #radians per step.

    inf_time=find_inf_time(currenttime)


    infvelofrotleft=calc_velocity(diffleft,delta_time)
    infvelofrotright=calc_velocity(diffright,delta_time)


    #update_odometry(leftSpeed, rightSpeed, delta_time)

    if(ldetectioncnt):
        update_odometry2(infvelofrotleft,infvelofrotright)
        #IKrobotsolver()


    # debug prints are expensive; print at a slower rate so control stays responsive
    if int(robot.getTime()*5) % 5 == 0:
        report(2,currenttime)

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

