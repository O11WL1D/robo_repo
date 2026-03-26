"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np

from controller import DistanceSensor
    



MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

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
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0
timepassed=0
timepassed1=0
timepassed2=0
check_wall=0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

# map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'autonomous' # Part 1.1: manual mode

#part 1
map = np.zeros(shape=[360,360], dtype=float)
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
    map = np.zeros(shape=[360,360], dtype=float)
    display.setColor(0x000000)
    display.fillRectangle(0, 0, MAP_SIZE, MAP_SIZE)
    print("Cleared map")

BASE_SPEED = 3.0




timethreshright=500
timethreshleft=500

angle=90



prevmax=0
currentmax=0




















def get_lidar_regions(ranges):
    n = len(ranges)

    front = ranges[n//2 - 20 : n//2 + 20]
    left  = ranges[int(0.75*n) : int(0.9*n)]
    right = ranges[int(0.1*n) : int(0.25*n)]

    def clean(vals):
        vals = [v for v in vals if np.isfinite(v)]
        return min(vals) if vals else LIDAR_SENSOR_MAX_RANGE

    return {
        "front": clean(front),
        "left": clean(left),
        "right": clean(right)
    }



#states
FORWARD=1
ROTATING_LEFT=2
A90_LEFT=5
A45_LEFT=6
A45_right=7
ROTATING_RIGHT=3


STABLIZE=9

STABLIZE1=1


robot_state=FORWARD
sub_robot_state=STABLIZE1


stabilize_timer_start=1










def reset_time_passed1():
    timepassed=0







































prev_time=0
current_time=0






while robot.step(timestep) != -1 and mode != 'planner':

    step_count += 1


    # Mapping

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        t = pose_theta + np.pi/2.
        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(t)*rx - math.sin(t)*ry + pose_x
        wy =  math.sin(t)*rx + math.cos(t)*ry + pose_y

        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
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
    display.drawPixel(360-abs(int(pose_x*30)), abs(int(pose_y*30)))

    if DEBUG_PRINT and step_count % 20 == 0:
        center_idx = len(lidar_sensor_readings) // 2
        left_idx = len(lidar_sensor_readings) // 4
        right_idx = (3 * len(lidar_sensor_readings)) // 4
        print("POSE", round(pose_x, 3), round(pose_y, 3), round(pose_theta, 3),
              "L", round(float(lidar_sensor_readings[left_idx]), 3),
              "C", round(float(lidar_sensor_readings[center_idx]), 3),
              "R", round(float(lidar_sensor_readings[right_idx]), 3))

 
    # Controllers 
    


























    if mode == 'manual':
        #Perform teleoperation and obtain a map of the entire environment and store the map as an npy file
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
        current_time=step_count*timestep
        #Autonomously explore the entire environment, generate a map, and store the map as an npy file
        vL = 0.0
        vR = 0.0

        regions = get_lidar_regions(lidar_sensor_readings)

        front = regions["front"]
        left  = regions["left"]
        right = regions["right"]

        
        WALL_DIST = 0.6
        FRONT_TH  = 0.8
        KP = 3.0

        vL = 0.0
        vR = 0.0


        #Condition detection:

        Left_thresh=1.2
        Front_thresh=1.2
        right_thresh=1.2

        front_active=0
        left_active=0
        right_active=0

        left_open=1.5
        front_open=1.5
        right_open=1.5
        front_free=0
        left_free=0
        right_free=0



        if(front<Front_thresh):
            front_active=1

        if(left<Left_thresh):
            left_active=1

       

        if(right<right_thresh):
            right_active=1




         
        if(left>left_open):
            left_free=1

 
        if(front>front_open):
            front_free=1

 
        if(right>right_open):
            right_free=1


        if(front_active and (not (robot_state==STABLIZE))):
            robot_state=ROTATING_LEFT
            angle=90


   


      

        #if(front_free and right_free):
        #    stabilize_timer_start=1
         #   angle=150
            

        
                 

        if(robot_state==STABLIZE):
            

            
            if(sub_robot_state==STABLIZE1):
                1==1





                timethreshright=500
                timethreshright=(500/90)*angle

                BASE_SPEED=2
                vL=BASE_SPEED
                vR=-BASE_SPEED

                
                timepassed=timepassed+(current_time-prev_time)

                print("TIME PASSED " +str(timepassed))

                if(timepassed>timethreshright):
                    sub_robot_state=2
                    timepassed=0
                    #timethreshright=100





            if(sub_robot_state==2):

                if(0):
                    1==1

                else:
                    robot_state=ROTATING_LEFT
                    stabilize_timer_start=1


                



                #currentmax=front
                
                #print("CURRENTMAX " + str(currentmax))
                #print("PREVMAX " + str(prevmax))
                
                #if(currentmax>prevmax):
                    
                 #   vL=(BASE_SPEED-1)
                 #   vR=(BASE_SPEED-1)

                #else:
                #    vL=0
                #    vR=0
        

                #prevmax=currentmax

        

        
        


        


 
        

        if(robot_state==ROTATING_RIGHT):
        
            timethreshright=500

            
            timethreshright=(500/90)*angle

            
            timepassed=timepassed+(current_time-prev_time)

            print(timepassed)

            if(timepassed>timethreshright):
                robot_state=FORWARD
                timepassed=0
                #timethreshright=100

        



        if(robot_state==ROTATING_LEFT):
            1==1

            timethreshleft=500
            timethreshright=(500/90)*angle

            
            timepassed=timepassed+(current_time-prev_time)

            print(timepassed)

            if(timepassed>timethreshleft):
                robot_state=FORWARD
                timepassed=0
            

   



        BASE_SPEED=2

        if(robot_state==FORWARD):
            vL=BASE_SPEED
            vR=BASE_SPEED

        
        if(robot_state==ROTATING_LEFT):
            vL=-BASE_SPEED
            vR=BASE_SPEED


        if(robot_state==ROTATING_RIGHT):
            vL=BASE_SPEED
            vR=-BASE_SPEED




        print("current front reading: "+ str(front)+ "current left reading: " + str(left)+ "current right: " + str(right))
        print("CURRENT state " + str(robot_state))
        

        vL = clamp(vL, -MAX_SPEED, MAX_SPEED)
        vR = clamp(vR, -MAX_SPEED, MAX_SPEED)







        
        
        timepassed1=timepassed1+(current_time-prev_time)
        

        #basic setup timer
        if(timepassed1<2000):
            vL=0
            vR=0
      





        if(stabilize_timer_start):
            timepassed2=timepassed2+(current_time-prev_time)
            print("stabilize Timer " +str(timepassed2))
    
            






        if(timepassed2>8000):
            timepassed2=0
            print("@@@@@@@@@@@@@@@@@ENTERING STABLIZE MODE!")
            robot_state=STABLIZE
            stabilize_timer_start=0





        prev_time=step_count*timestep


    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
while robot.step(timestep) != -1:
    # there is a bug where webots have to be restarted if the controller exits on Windows
    # this is to keep the controller running
    pass