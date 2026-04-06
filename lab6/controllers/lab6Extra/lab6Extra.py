import numpy as np
import sys

try:
    from controller import Supervisor
except ImportError:
    print("ERROR: Must run inside Webots as Supervisor."); sys.exit(1)


# ============================================================
# Robot Interface (DO NOT MODIFY)
# ============================================================
class UR5eInterface:
    MOTOR_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    SENSOR_NAMES = [n + "_sensor" for n in MOTOR_NAMES]
    HAND_MOTORS_1 = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
    HAND_MOTORS_2 = ["finger_1_joint_2", "finger_2_joint_2", "finger_middle_joint_2"]

    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.motors, self.sensors = [], []
        for i in range(6):
            m = robot.getDevice(self.MOTOR_NAMES[i])
            s = robot.getDevice(self.SENSOR_NAMES[i])
            s.enable(self.timestep); m.setVelocity(1.0)
            self.motors.append(m); self.sensors.append(s)
        self.hand_motors_1, self.hand_motors_2 = [], []
        for name in self.HAND_MOTORS_1:
            m = robot.getDevice(name)
            if m: self.hand_motors_1.append(m)
        for name in self.HAND_MOTORS_2:
            m = robot.getDevice(name)
            if m: self.hand_motors_2.append(m)
        for _ in range(10): robot.step(self.timestep)

    def get_joint_positions(self):
        return np.array([s.getValue() for s in self.sensors])

    def set_joint_positions(self, q):
        for i in range(6): self.motors[i].setPosition(float(q[i]))

    def set_speed(self, speed):
        for m in self.motors: m.setVelocity(speed)

    def open_gripper(self):
        for m in self.hand_motors_1: m.setPosition(0.05)
        for m in self.hand_motors_2: m.setPosition(0.0)

    def close_gripper(self):
        for m in self.hand_motors_1: m.setPosition(0.3)
        for m in self.hand_motors_2: m.setPosition(0.8)

    def wait(self, seconds):
        for _ in range(int(seconds * 1000 / self.timestep)):
            if self.robot.step(self.timestep) == -1: return

    def move_to(self, q, wait_time=3.0):
        self.set_joint_positions(q); self.wait(wait_time)
        return self.get_joint_positions()


# ============================================================
# Constants (DO NOT MODIFY)
# ============================================================
# UR5e Standard DH Parameters
UR5E_DH_A     = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
UR5E_DH_D     = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
UR5E_DH_ALPHA = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
GRIPPER_OFFSET = 0.18  # Robotiq 3F gripper length

# Webots UR5e PROTO has a 180-deg Z rotation between PROTO frame and DH frame
R_PROTO = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
ROBOT_POS = None  # Set at runtime from Supervisor

HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

TASK = "block_stack"
BLOCK_HEIGHT = 0.0635
APPROACH_HEIGHT = 0.14
GRASP_HEIGHT = 0.018
PLACE_HEIGHT = 0.012


# ============================================================
# PROVIDED: Coordinate Transforms (DO NOT MODIFY)
# ============================================================
def _dh(a, d, alpha, theta):
    """Standard DH transformation matrix (one joint)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]])



def world_to_base(pw):
    """Convert world position to DH base frame."""
    return R_PROTO.T @ (np.array(pw) - ROBOT_POS)


def base_to_world(pb):
    """Convert DH base frame position to world."""
    return R_PROTO @ np.array(pb) + ROBOT_POS





# ============================================================
# TASK 1: Forward Kinematics (15 pts)
# ============================================================
def forward_kinematics(q):



    """
    Compute the gripper tip position given joint angles.

    Parameters:
        q : array-like, 6 joint angles (radians)

    
    


    Returns:
        position : numpy array [x, y, z] in robot base frame
    """

    #dh1=_dh(q[0],q[1],q[2],q[3])
    #wrong ಠ_ಠ

    #np eye creates a 4x4 identity matrix
    T = np.eye(4)

    
    for i in range(6):
        a = UR5E_DH_A[i]
        d = UR5E_DH_D[i]
        alpha = UR5E_DH_ALPHA[i]
        theta = q[i]

        #this multiplies the current matrix T with the transform containing the new parameters a, d, alpa, theta.
        T = T @ _dh(a, d, alpha, theta)

    
    T_offset = np.eye(4)
    T_offset[2, 3] = GRIPPER_OFFSET

    T = T @ T_offset

    
    position = T[:3, 3]

    return position

    #raise NotImplementedError("TODO: Implement forward_kinematics()")







# ============================================================
# TASK 2: Numerical Jacobian (15 pts)
# ============================================================



def compute_jacobian(q, delta=1e-5):
    """
    Compute the 3x6 position Jacobian using central finite differences.

    Parameters:
        q     : numpy array (6,) current joint angles
        delta : perturbation size

    Returns:
        J : numpy array (3, 6)
    """
    # TODO: Implement (~8-10 lines)

    J = np.zeros((3, 6))
    for i in range(6):
        q_pos = np.copy(q)
        q_neg = np.copy(q)
        
        q_pos[i] += delta
        q_neg[i] -= delta

        f_pos = forward_kinematics(q_pos)
        f_neg = forward_kinematics(q_neg)
 
        J[:, i] = (f_pos - f_neg) / (2 * delta)
    return J

    raise NotImplementedError("TODO: Implement compute_jacobian()")





# ============================================================
# TASK 3: Define Waypoints (10 pts)
# ============================================================
def get_waypoints(block_world, bin_world):
    """
    Define the world-frame positions the gripper should visit.

    Parameters:
        block_world : [x, y, z] of block center in world frame
        bin_world   : [x, y, z] of bin center in world frame

    Returns:
        dict with keys: 'above_block', 'at_block', 'above_bin'
        Each value is a list [x, y, z] in world frame.
    """
    # 20cm approach height offset above objects
    z_offset = 0.20
    
    return {
        'above_block': [block_world[0], block_world[1], block_world[2] + z_offset],
        'at_block': [block_world[0], block_world[1], block_world[2] + 0.02], 
        'above_bin': [bin_world[0], bin_world[1], bin_world[2] + z_offset]
    }
    raise NotImplementedError("TODO: Implement get_waypoints()")


# ============================================================
# TASK 4: Gradient Descent IK (20 pts)
# ============================================================
def gradient_descent_ik(target_world, q_init, learning_rate=0.5, 
                         max_iterations=1000, tolerance=0.005):
    """
    Find joint angles that place the gripper at target_world.

    Parameters:
        target_world   : [x, y, z] desired position in world frame
        q_init         : starting joint angles (numpy array of 6)
        learning_rate  : step size (default 0.5)
        max_iterations : max steps (default 1000)
        tolerance      : converge when error < this (meters)

    Returns:
        q_solution : numpy array of 6 joint angles
        converged  : bool
        errors     : list of position error at each iteration
    """
    target_base = world_to_base(target_world)
    q = np.copy(q_init)
    errors = []

    for _ in range(max_iterations):
        current_pos = forward_kinematics(q)
        error = target_base - current_pos
        
        #Euclidean distance
        err_norm = np.linalg.norm(error)
        errors.append(err_norm)

        if err_norm < tolerance:
            return q, True, errors

        #apply the gradient step: q_new = q_old + alpha * J_T * error
        J = compute_jacobian(q)
        q = q + learning_rate * (J.T @ error)
        
        #clip joints to [-2pi, 2pi]
        q = np.clip(q, -2*np.pi, 2*np.pi)

    return q, False, errors
    raise NotImplementedError("TODO: Implement gradient_descent_ik()")


# ============================================================
# TASK 5: Pick and Place Sequence (10 pts)
# ============================================================
def pick_and_place(arm, block_world, bin_world):
    """
    Pick up the block and place it in the bin.

    """
    # TODO: Implement 
    waypoints = get_waypoints(block_world, bin_world)
    q_current = arm.get_joint_positions()

    sequence = [
        (waypoints['above_block'], 'open'),
        (waypoints['at_block'], 'open'),
        (waypoints['at_block'], 'close'),  
        (waypoints['above_block'], 'close'), 
        (waypoints['above_bin'], 'close'), 
        (waypoints['above_bin'], 'open')   
    ]

    for target_pos, gripper_state in sequence:
        q_sol, converged, _ = gradient_descent_ik(target_pos, q_current)
        
        if not converged:
            print(f"Warning: IK did not converge for target {target_pos}")
            
        # Move the arm
        arm.move_to(q_sol, wait_time=0.5)
        
        # Actuate gripper
        if gripper_state == 'close':
            arm.close_gripper()
        else:
            arm.open_gripper()
            
        arm.wait(1.0) 
        q_current = arm.get_joint_positions()

def find_node(robot, names):
    for name in names:
        node = robot.getFromDef(name)
        if node is not None:
            return node
    return None

def move_block(arm, source_world, target_world, q_current):
    above_source = [source_world[0], source_world[1], source_world[2] + APPROACH_HEIGHT]
    at_source = [source_world[0], source_world[1], source_world[2] + GRASP_HEIGHT]
    above_target = [target_world[0], target_world[1], target_world[2] + APPROACH_HEIGHT]
    at_target = [target_world[0], target_world[1], target_world[2] + PLACE_HEIGHT]

    sequence = [
        (above_source, 'open'),
        (at_source, 'open'),
        (at_source, 'close'),
        (above_source, 'close'),
        (above_target, 'close'),
        (at_target, 'close'),
        (at_target, 'open'),
        (above_target, 'open')
    ]

    for target_pos, gripper_state in sequence:
        q_sol, converged, _ = gradient_descent_ik(target_pos, q_current)
        if not converged:
            print(f"Warning: IK did not converge for target {target_pos}")
        arm.move_to(q_sol, wait_time=0.6)
        if gripper_state == 'close':
            arm.close_gripper()
        else:
            arm.open_gripper()
        arm.wait(1.2)
        q_current = arm.get_joint_positions()

    return q_current

def block_stack(arm, robot):
    gray_block_node = find_node(robot, ["GRAY_BLOCK", "DEF_GRAY_BLOCK", "GREY_BLOCK", "DEF_GREY_BLOCK"])
    green_node = find_node(robot, ["GREEN_BLOCK", "DEF_GREEN_BLOCK"])
    blue_node = find_node(robot, ["BLUE_BLOCK", "DEF_BLUE_BLOCK"])
    red_node = find_node(robot, ["RED_BLOCK", "DEF_RED_BLOCK"])
    pad_node = find_node(robot, ["STACK_ZONE", "DEF_STACK_ZONE", "STACK_PAD", "PAD", "TARGET", "BASE", "PLACE", "GRAY_AREA", "GRAY_PAD"])

    if gray_block_node is None or green_node is None or blue_node is None or red_node is None or pad_node is None:
        print("ERROR!!!")
        return

    gray_block_pos = list(gray_block_node.getPosition())
    green_pos = list(green_node.getPosition())
    blue_pos = list(blue_node.getPosition())
    red_pos = list(red_node.getPosition())
    pad_pos = list(pad_node.getPosition())

    stack_x = pad_pos[0]
    stack_y = pad_pos[1]
    stack_z = pad_pos[2]

    q_current = arm.get_joint_positions()

    gray_target = [stack_x, stack_y, stack_z]
    green_target = [stack_x, stack_y, stack_z + BLOCK_HEIGHT]
    blue_target = [stack_x, stack_y, stack_z + 2 * BLOCK_HEIGHT]
    red_target = [stack_x, stack_y, stack_z + 3 * BLOCK_HEIGHT]

    q_current = move_block(arm, gray_block_pos, gray_target, q_current)
    q_current = move_block(arm, green_pos, green_target, q_current)
    q_current = move_block(arm, blue_pos, blue_target, q_current)
    q_current = move_block(arm, red_pos, red_target, q_current)

    arm.move_to(HOME, wait_time=2.0)


# ============================================================
# MAIN — DO NOT MODIFY
# ============================================================
def main():
    global ROBOT_POS
    robot = Supervisor()
    arm = UR5eInterface(robot)

    # Fetch standard Webots nodes
    robot_node = robot.getFromDef("ur5e")
    block_node = robot.getFromDef("BLOCK") 
    bin_node = robot.getFromDef("BIN")
    
    if robot_node is not None:
        ROBOT_POS = robot_node.getPosition()
    else:
        ROBOT_POS = np.array([-0.4, 0, 0.74])

    # put the arm in home pos
    arm.move_to(HOME, wait_time=2.0)

    # get positions from supervisor
    if TASK == "pick_and_place":
        if block_node and bin_node:
            block_pos = block_node.getPosition()
            bin_pos = bin_node.getPosition()
            
            pick_and_place(arm, block_pos, bin_pos)
        else:
            print("ERROR!!!")
    elif TASK == "block_stack":
        block_stack(arm, robot)
    else:
        print("ERROR!!!")

if __name__ == "__main__":
    main()