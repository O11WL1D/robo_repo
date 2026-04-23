import json, math, os
import numpy as np
from controller import Robot


# ══════════════════════════════════════════════════════════════════════════════
# PR2 ARM KINEMATICS  (do not modify)
# ══════════════════════════════════════════════════════════════════════════════

# DH parameters for the PR2 right arm [a, d, alpha, theta_offset]
# Joints: shoulder_pan, shoulder_lift, upper_arm_roll, elbow_flex, wrist_roll
PR2_RIGHT_ARM_DH = [
    [0.0,   0.0, -math.pi/2,  0.0],   # r_shoulder_pan_joint
    [0.0,   0.0,  math.pi/2,  0.0],   # r_shoulder_lift_joint
    [0.4,   0.0, -math.pi/2,  0.0],   # r_upper_arm_roll_joint  (0.4 m upper arm)
    [0.0,   0.0,  math.pi/2,  0.0],   # r_elbow_flex_joint
    [0.321, 0.0,  0.0,        0.0],   # r_wrist_roll_joint      (0.321 m forearm)
]

# Physical joint limits [min_rad, max_rad]
# NOTE: r_elbow_flex is NEGATIVE when bent (0.0 = fully extended)
PR2_JOINT_LIMITS = [
    (-2.1353,  0.5646),   # shoulder_pan
    (-0.5236,  1.3963),   # shoulder_lift
    (-3.9000,  0.8000),   # upper_arm_roll
    (-2.1213,  0.0000),   # elbow_flex  ← negative means bent!
    (-6.2832,  6.2832),   # wrist_roll  (continuous)
]

GRIPPER_OFFSET = 0.18    # metres from wrist to gripper fingertip along Z
ARM_BASE_XY    = np.array([-0.05, -0.188])   # right shoulder in robot body frame (x,y)
ARM_BASE_Z     = 1.07    # shoulder height (metres) when torso = 0.33 m

# ══════════════════════════════════════════════════════════════════════════════
# NAVIGATION CONSTANTS  (do not modify)
# ══════════════════════════════════════════════════════════════════════════════

MAX_WHEEL_SPEED  = 6.0   # rad/s  — maximum wheel angular velocity
ROBOT_RADIUS     = 0.335 # m      — PR2 base half-width (for collision check)

# Potential field tuning (you may adjust these in your TODO section)
K_ATT    = 5.5   # attractive gain
K_REP    = 0.8   # repulsive gain
D0       = 1.2   # influence radius (m) — obstacles beyond this are ignored
STUCK_THRESHOLD = 0.03   # m — if robot moves less than this, it may be stuck


# ══════════════════════════════════════════════════════════════════════════════
# PROVIDED HELPER — DH transform  (do not modify)
# ══════════════════════════════════════════════════════════════════════════════

def _dh(a, d, alpha, theta):
    """
    Returns the 4×4 Denavit-Hartenberg homogeneous transform for one joint.

    Parameters
    ----------
    a     : link length  (metres)
    d     : link offset  (metres)
    alpha : twist angle  (radians)
    theta : joint angle  (radians) — this is what you solve for in IK
    """
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.,  sa,     ca,    d   ],
        [0.,  0.,     0.,    1.  ],
    ])


# ══════════════════════════════════════════════════════════════════════════════
# PROVIDED HELPER — coordinate frames  (do not modify)
# ══════════════════════════════════════════════════════════════════════════════

def world_to_base(world_pos, robot_x, robot_y, robot_yaw):
    """
    Convert a world-frame [x, y, z] position into the PR2 right-arm base frame.

    The arm base frame has:
      +X = forward (in the direction the robot faces)
      +Y = left
      +Z = up

    Parameters
    ----------
    world_pos  : array-like [x, y, z] in world coordinates
    robot_x    : robot base X in world frame
    robot_y    : robot base Y in world frame
    robot_yaw  : robot heading in radians (0 = facing +X, π/2 = facing +Y)

    Returns
    -------
    np.ndarray shape (3,) — target in arm base frame
    """
    cy, sy = math.cos(robot_yaw), math.sin(robot_yaw)
    # Shoulder position in world frame
    sx  = robot_x + cy * ARM_BASE_XY[0] - sy * ARM_BASE_XY[1]
    sy_ = robot_y + sy * ARM_BASE_XY[0] + cy * ARM_BASE_XY[1]
    dx, dy = world_pos[0] - sx, world_pos[1] - sy_
    dz     = world_pos[2] - ARM_BASE_Z
    # Rotate into body frame
    return np.array([cy*dx + sy*dy,  -sy*dx + cy*dy,  dz])


def base_to_world(base_pos, robot_x, robot_y, robot_yaw):
    """Inverse of world_to_base — converts arm base frame → world frame."""
    cy, sy = math.cos(robot_yaw), math.sin(robot_yaw)
    sx  = robot_x + cy * ARM_BASE_XY[0] - sy * ARM_BASE_XY[1]
    sy_ = robot_y + sy * ARM_BASE_XY[0] + cy * ARM_BASE_XY[1]
    bx, by, bz = base_pos
    wx = sx + cy*bx - sy*by
    wy = sy_ + sy*bx + cy*by
    wz = ARM_BASE_Z + bz
    return np.array([wx, wy, wz])

#TODO Implement1
def forward_kinematics(q):
    T = np.eye(4)

    for i in range(5):
        a, d, alpha, theta_offset = PR2_RIGHT_ARM_DH[i]
        T = T @ _dh(a, d, alpha, q[i] + theta_offset)

    T_offset = np.eye(4)
    T_offset[2, 3] = GRIPPER_OFFSET
    T = T @ T_offset

    return T[:3, 3]

#TODO Implement 2
def compute_jacobian(q):
    q = np.array(q, dtype=float)
    delta = 1e-5
    J = np.zeros((3, 5))

    for i in range(5):
        q_pos = np.copy(q)
        q_neg = np.copy(q)

        q_pos[i] += delta
        q_neg[i] -= delta

        f_pos = forward_kinematics(q_pos)
        f_neg = forward_kinematics(q_neg)

        J[:, i] = (f_pos - f_neg) / (2 * delta)

    return J

#TODO Implement 3
def gradient_descent_ik(target, q0=None, alpha=0.5, tol=0.008, max_iter=4000):
    target = np.array(target, dtype=float)

    if q0 is None:
        q = np.array([0.0, 1.35, 0.0, -2.0, 0.0], dtype=float)
    else:
        q = np.array(q0, dtype=float)

    for _ in range(max_iter):
        current = forward_kinematics(q)
        error = target - current

        if np.linalg.norm(error) < tol:
            return q, True

        J = compute_jacobian(q)
        q = q + alpha * (J.T @ error)

        for i in range(5):
            lo, hi = PR2_JOINT_LIMITS[i]
            q[i] = np.clip(q[i], lo, hi)

    return q, False

#TODO Implement 4

def compute_potential_field(robot_x, robot_y, goal_x, goal_y, lidar_ranges, lidar_fov):
    fx = goal_x - robot_x
    fy = goal_y - robot_y

    n = len(lidar_ranges)
    if n == 0:
        return np.array([fx, fy], dtype=float)

    for i, r in enumerate(lidar_ranges):
        if math.isnan(r) or math.isinf(r):
            continue
        if r <= 0.05 or r > D0:
            continue

        angle = -lidar_fov / 2.0 + i * lidar_fov / max(n - 1, 1)
        mag = K_REP * (1.0 / r - 1.0 / D0) / (r * r)

        fx += -mag * math.cos(angle)
        fy += -mag * math.sin(angle)

    return np.array([fx, fy], dtype=float)



#Gavin code

def compute_potential_field2(robot_x, robot_y, goal_x, goal_y, lidar_ranges, lidar_fov):

    
        def angle_diff(a, b):
            d = a - b
            return (d + math.pi) % (2 * math.pi) - math.pi


        n = len(lidar_ranges)
        max_radians=4.5378
        radians_per_measurement=(max_radians/n)
        c = n // 2

        front_vals = [r for r in lidar_ranges[max(0, c - n // 14):min(n, c + n // 14 + 1)]
                        if not (math.isnan(r) or math.isinf(r))]
        left_vals = [r for r in lidar_ranges[min(n - 1, c + n // 10):min(n, c + n // 4)]
                        if not (math.isnan(r) or math.isinf(r))]
        right_vals = [r for r in lidar_ranges[max(0, c - n // 4):max(0, c - n // 10)]
                        if not (math.isnan(r) or math.isinf(r))]


        if front_vals:
            front_min = min(front_vals)
        if left_vals:
            left_min = min(left_vals)
        if right_vals:
            right_min = min(right_vals)



        
        return np.array([left_min/100, right_min/100], dtype=float)


# Gavin Code other method
def compute_potential_field_reinit(robot_x, robot_y, goal_x, goal_y, lidar_ranges, lidar_fov):


        n = len(lidar_ranges)
        max_radians=4.5378
        radians_per_measurement=(max_radians/n)

        #goal_angle = math.atan2(dy, dx)
        #yaw_error = angle_diff(goal_angle, robot_yaw)

        reading_to_meters=1
        #this is the factor that converts a lidar reading to the distance at which 
        #it reads something. temp value to be adjusted in future. 


        #this is relative to the robot fov
        object_position_x=0
        object_position_y=0

        dx=0
        dy=0

        print("N VALUE" + str(n))

        for i in range(n):


            #print("do something")


            object_position_x= (reading_to_meters*lidar_ranges[i])*math.cos(radians_per_measurement*i)
            object_position_y= (reading_to_meters*lidar_ranges[i])*math.cos(radians_per_measurement*i)
            #print("OBJECT POSITION " + str(object_position_x) + "  " +str(object_position_y) )
            
            if(not((reading_to_meters*lidar_ranges[i])>D0)):

                #repulsive potential
                dx-=((1/2)*K_REP)* (1/(abs(robot_x-object_position_x)**2))
                dy-=((1/2)*K_REP)* (1/(abs(robot_y-object_position_y)**2))


  
        #attractive potential 
        dx+=((1/2)*K_ATT)* ((float(abs(robot_x-goal_x))**2))
        dy+=((1/2)*K_ATT)* ((float(abs(robot_y-goal_y)**2)))
        return np.array([dx, dy], dtype=float)






        


        





#TODO Implement 5 
def pick_object(pr2, obj_data):
    robot_x, robot_y, robot_yaw = pr2.get_pose()

    approach_world = np.array(obj_data["approach_position"], dtype=float)
    pick_world = np.array(obj_data["pick_position"], dtype=float)

    approach_base = world_to_base(approach_world, robot_x, robot_y, robot_yaw)
    pick_base = world_to_base(pick_world, robot_x, robot_y, robot_yaw)

    pr2.set_torso(0.33)
    pr2.open_gripper(right=True)

    q_start = pr2.get_right_arm_q()

    q_approach, ok1 = gradient_descent_ik(approach_base, q_start)
    if ok1:
        pr2.set_right_arm(q_approach)

    q_pick, ok2 = gradient_descent_ik(pick_base, pr2.get_right_arm_q())
    if ok2:
        pr2.set_right_arm(q_pick)

    pr2.close_gripper(right=True)

    q_lift, ok3 = gradient_descent_ik(approach_base, pr2.get_right_arm_q())
    if ok3:
        pr2.set_right_arm(q_lift)

    pr2.set_right_arm([0., 1.35, 0., -2.0, 0.])

#TODO Implement 6

def place_objects(pr2, place_zone):
    robot_x, robot_y, robot_yaw = pr2.get_pose()

    approach_world = np.array(place_zone["approach_position"], dtype=float)
    place_world = np.array(place_zone["place_position"], dtype=float)

    approach_base = world_to_base(approach_world, robot_x, robot_y, robot_yaw)
    place_base = world_to_base(place_world, robot_x, robot_y, robot_yaw)

    pr2.set_torso(0.33)

    q_start = pr2.get_right_arm_q()

    q_approach, ok1 = gradient_descent_ik(approach_base, q_start)
    if ok1:
        pr2.set_right_arm(q_approach)

    q_place, ok2 = gradient_descent_ik(place_base, pr2.get_right_arm_q())
    if ok2:
        pr2.set_right_arm(q_place)

    pr2.open_gripper(right=True)

    q_retreat, ok3 = gradient_descent_ik(approach_base, pr2.get_right_arm_q())
    if ok3:
        pr2.set_right_arm(q_retreat)

    pr2.set_right_arm([0., 1.35, 0., -2.0, 0.])

# ══════════════════════════════════════════════════════════════════════════════
# PR2 CONTROLLER CLASS  — provided, do not modify
# ══════════════════════════════════════════════════════════════════════════════

class PR2Controller:
    """
    Hardware abstraction layer for the PR2 in Webots.
    Exposes the devices you need — read the docstrings before using.
    """

    TIME_STEP = 32   # ms

    # ── Construction ──────────────────────────────────────────────────────────

    
    def __init__(self):
        
        self.robot = Robot()
        self.ts    = int(self.robot.getBasicTimeStep())

        # Dead-reckoning state (updated by supervisor pose file)
        self._x   = 0.0
        self._y   = 1.0
        self._yaw = -math.pi / 2   # PR2 starts facing south (-Y)
        self._collision_count = 0

        self._setup_devices()
        self._enable_devices()
        print("PR2 checkpoint 1/6")
        # Initial safe pose
        self.set_right_arm([0., 1.35, 0., -2.2, 0.])
        print("PR2 checkpoint 2/6")
        self.set_left_arm( [0., 1.35, 0., -2.2, 0.])
        print("PR2 checkpoint 3/6")
        self.open_gripper(right=True)
        print("PR2 checkpoint 4/6")
        self.open_gripper(right=False)
        print("PR2 checkpoint 5/6")
        self.set_torso(0.33)
        print("PR2 checkpoint 6/6")
        

    # ── Device setup (internal) ───────────────────────────────────────────────
    
    def _setup_devices(self):
        G = self.robot.getDevice
        self._wm = [G(n) for n in [
            "fl_caster_l_wheel_joint","fl_caster_r_wheel_joint",
            "fr_caster_l_wheel_joint","fr_caster_r_wheel_joint",
            "bl_caster_l_wheel_joint","bl_caster_r_wheel_joint",
            "br_caster_l_wheel_joint","br_caster_r_wheel_joint"]]
        self._ws  = [m.getPositionSensor() if m else None for m in self._wm]
        self._cm  = [G(n) for n in [
            "fl_caster_rotation_joint","fr_caster_rotation_joint",
            "bl_caster_rotation_joint","br_caster_rotation_joint"]]
        self._cs  = [m.getPositionSensor() if m else None for m in self._cm]
        self._ra  = [G(n) for n in [
            "r_shoulder_pan_joint","r_shoulder_lift_joint",
            "r_upper_arm_roll_joint","r_elbow_flex_joint","r_wrist_roll_joint"]]
        self._ras = [m.getPositionSensor() if m else None for m in self._ra]
        self._la  = [G(n) for n in [
            "l_shoulder_pan_joint","l_shoulder_lift_joint",
            "l_upper_arm_roll_joint","l_elbow_flex_joint","l_wrist_roll_joint"]]
        self._las = [m.getPositionSensor() if m else None for m in self._la]
        self._rf  = G("r_finger_gripper_motor::l_finger")
        self._lf  = G("l_finger_gripper_motor::l_finger")
        self._rfs = self._rf.getPositionSensor() if self._rf else None
        self._lfs = self._lf.getPositionSensor() if self._lf else None
        self._rcl = G("r_gripper_l_finger_tip_contact_sensor")
        self._rcr = G("r_gripper_r_finger_tip_contact_sensor")
        self._tor = G("torso_lift_joint")
        self._tors= G("torso_lift_joint_sensor")
        self._imu = G("imu_sensor")
        self._lid = G("base_laser")
        print("Device setup ran")

    
    def _enable_devices(self):
        ts = self.ts
        for s in self._ws:
            if s: s.enable(ts)
        for m in self._wm:
            if m: m.setPosition(float('inf')); m.setVelocity(0.)
        for s in self._cs + self._ras + self._las:
            if s: s.enable(ts)
        for s in [self._rfs, self._lfs, self._tors]:
            if s: s.enable(ts)
        if self._imu: self._imu.enable(ts)
        if self._lid: self._lid.enable(ts)
        if self._rcl: self._rcl.enable(ts)
        if self._rcr: self._rcr.enable(ts)
        print("Device enable ran")

    # ── Simulation step ───────────────────────────────────────────────────────
    
    def step(self):
        """Advance simulation by one time step. Returns False when sim ends."""
        alive = self.robot.step(self.ts) != -1
        
        if alive:
            self._check_collision()
      
        return alive
        

    # ── Pose (supervisor-corrected) ───────────────────────────────────────────
    
    def get_pose(self):
        """
        Return (x, y, yaw) in world frame.
        Reads robot_pose.json written by the supervisor for accuracy.
        Falls back to IMU + dead-reckoning if file not available.
        """
        for path in ["robot_pose.json",
                     "../lab7_supervisor/robot_pose.json",
                     "../../robot_pose.json"]:
            try:
                with open(path) as f:
                    d = json.load(f)
                    self._x, self._y, self._yaw = d["x"], d["y"], d["yaw"]
                    return self._x, self._y, self._yaw
            except Exception:
                pass
        # IMU fallback for yaw
        
        if self._imu:
            self._yaw = self._imu.getRollPitchYaw()[2]
   
        return self._x, self._y, self._yaw
        

    # ── Lidar ─────────────────────────────────────────────────────────────────
    
    def get_lidar(self):
        """
        Return (ranges, fov) from the base laser.
        ranges : list of floats (metres); may contain float('inf') or nan
        fov    : total field of view in radians
        """
        if not self._lid:
            return [], 0.0
      
        return self._lid.getRangeImage(), self._lid.getFov()

    # ── Collision counting ────────────────────────────────────────────────────

   
    def _check_collision(self):
        if not self._lid: return
        ranges = self._lid.getRangeImage()
        if not ranges: return
        n = len(ranges); fov = self._lid.getFov()
        c = n // 2; hw = int(0.20 / fov * n)
        for i in range(max(0, c-hw), min(n-1, c+hw)+1):
            r = ranges[i]
            if not (math.isnan(r) or math.isinf(r)) and 0.05 < r < 0.25:
                self._collision_count += 1
                break   # count once per step
        

    def get_collision_count(self):
        """Returns total collision count (used for grading penalty)."""
      
        return self._collision_count

    # ── Wheel control ─────────────────────────────────────────────────────────
 
    def set_wheel_speeds(self, left_speed, right_speed):
        """
        Set differential drive wheel velocities (rad/s).
        All four left wheels get left_speed; all four right wheels get right_speed.
        Casters must be in forward-facing configuration (use rotate_in_place for turns).

        Parameters
        ----------
        left_speed  : float in [-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED]
        right_speed : float in [-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED]
        """
        ls = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
        rs = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
        speeds = [ls, ls, rs, rs, ls, ls, rs, rs]
        for m, v in zip(self._wm, speeds):
            if m: m.setVelocity(v)

    def stop(self):
        """Stop all wheels."""
        for m in self._wm:
            if m: m.setVelocity(0.)

    def rotate_in_place(self, angle_rad):
        """
        Rotate the robot in place by angle_rad (positive = CCW).
        Blocks until rotation is complete (encoder-based).
        """
        self.stop()
        self._set_casters(3*math.pi/4, math.pi/4,
                         -3*math.pi/4,-math.pi/4, wait=True)
        spd = 2.0 if angle_rad > 0 else -2.0
        for m in self._wm:
            if m: m.setVelocity(spd)
        expected = abs(angle_rad) * 0.5 * (0.4492 + 0.098)
        s0 = self._ws[0]
        if s0:
            init = s0.getValue()
            while True:
                t = abs(0.08 * (s0.getValue() - init))
                if t >= expected: break
                if expected - t < 0.02:
                    for m in self._wm:
                        if m: m.setVelocity(0.15 * spd)
                self.step()
        self._set_casters(0., 0., 0., 0., wait=True)
        self.stop()
        if self._imu:
            self._yaw = self._imu.getRollPitchYaw()[2]

    # ── Torso ─────────────────────────────────────────────────────────────────

    def set_torso(self, height, wait=True):
        """
        Set torso height.

        Parameters
        ----------
        height : float in [0.0, 0.33] metres
        wait   : if True, blocks until torso reaches target
        """
        if self._tor: self._tor.setPosition(max(0., min(0.33, height)))
        if wait and self._tors:
            for _ in range(600):
                if not self.step(): return
                if abs(self._tors.getValue() - height) < 0.01: break

    # ── Arm control ───────────────────────────────────────────────────────────

    def set_right_arm(self, q, wait=True):
        """
        Command the right arm to joint angles q (length-5 list/array, radians).
        Clamps to PR2_JOINT_LIMITS automatically.
        If wait=True, blocks until all joints reach their targets.
        """
        self._set_arm(self._ra, self._ras, q, wait)

    def set_left_arm(self, q, wait=True):
        """Same as set_right_arm but for the left arm."""
        self._set_arm(self._la, self._las, q, wait)

    def get_right_arm_q(self):
        """Return current right arm joint angles as np.ndarray shape (5,)."""
        return np.array([s.getValue() if s else 0. for s in self._ras])

    # ── Gripper ───────────────────────────────────────────────────────────────

    def open_gripper(self, right=True, wait=True):
        """
        Open the specified gripper fully.
        right=True  → right gripper (default)
        right=False → left gripper
        """
        m = self._rf if right else self._lf
        s = self._rfs if right else self._lfs
        if m: m.setPosition(0.5)
        if wait and s:
            for _ in range(300):
                if not self.step(): return
                if abs(s.getValue() - 0.5) < 0.02: break

    def close_gripper(self, right=True, torque=20., wait=True):
        """
        Close the specified gripper until contact or fully closed.
        torque : gripping force in N·m (default 20 N·m)
        """
        m  = self._rf  if right else self._lf
        s  = self._rfs if right else self._lfs
        cl = self._rcl if right else None
        cr = self._rcr if right else None
        if not m: return
        m.setPosition(0.)
        if wait:
            for _ in range(500):
                if not self.step(): return
                contact = cl and cr and cl.getValue() > 0 and cr.getValue() > 0
                closed  = s and abs(s.getValue()) < 0.01
                if contact or closed:
                    if s:
                        m.setAvailableTorque(torque)
                        m.setPosition(max(0., s.getValue() * 0.95))
                    break

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _set_arm(self, motors, sensors, q, wait=True):
        for m, a in zip(motors, q):
            lo, hi = -6.28, 6.28
            if m: m.setPosition(max(lo, min(hi, float(a))))
        if wait:
            for _ in range(900):
                if not self.step(): return
                if all(s is None or abs(s.getValue() - float(a)) < 0.05
                       for s, a in zip(sensors, q)): break




    def _set_casters(self, fl, fr, bl, br, wait=True):
        T = [fl, fr, bl, br]
        if wait:
            self.stop()
            for m in self._wm:
                if m: m.setAvailableTorque(0.)
        for m, t in zip(self._cm, T):
            if m: m.setPosition(t)
        if wait:
            for _ in range(300):
                if all(s is None or abs(s.getValue()-t) < 0.05
                       for s, t in zip(self._cs, T)): break
                self.step()
            for m in self._wm:
                if m: m.setAvailableTorque(0.5)


#TODO 7

def load_environment_map():

    print("load environment map called")
    #why isnt this being called
    """Load the environment_map.json generated by the supervisor."""
    base_dir = os.path.dirname(os.path.abspath(""))
    project_dir = os.path.abspath(os.path.join(base_dir, "..", ".."))


    print("CURRENT BASE DIR: "+ str(base_dir) )
    print("CURRENT PROJECT DIR: "+ str(project_dir) )
    project_dir=project_dir+"\lab7"

    paths = [
        os.path.join(project_dir, "environment_map.json"),
        os.path.join(base_dir, "environment_map.json"),
        os.path.join(base_dir, "..", "lab7_supervisor", "environment_map.json"),
   
    ]
    for _ in range(200):
        for path in paths:
            print("Current path: "+str(path))
            if os.path.exists(path):
                with open(path, "r") as f:
                    return json.load(f)
        import time
        time.sleep(0.05)
    raise FileNotFoundError("environment_map.json not found")

























def report(pr2,dist, front, left, right,range1,range2,range3,range4,range5,range6,range7,goal_x,goal_y ):
    print("---------------------------------------------------NEW REPORT")
    print("Current x and y pose: " + str(pr2.get_pose()) )
    print("Goal x and y pose: " +str(goal_x) + str(goal_y) )
    print("Dist: " +str(dist))
    print("lidar front, left, right : " +str(front) + " " + str(left) + " " + str(right) )
    print("goal angle "+ str(range1))
    print("yaw error "+ str(range2))
    print("front min "+ str(range3))
    print("left min "+ str(range4))
    print("right min "+ str(range5))
    print("dx "+ str(range6))
    print("dy"+ str(range7))






#def calculate_fields_object_1(pr2):





#TODO 8
#juan code
def navigate_to_goal2(pr2, goal_x, goal_y, goal_yaw, env_map):
    pos_tol = 0.28
    yaw_tol = 0.18
    max_steps = 2000

    def angle_diff(a, b):
        d = a - b
        return (d + math.pi) % (2 * math.pi) - math.pi

    for _ in range(max_steps):
        robot_x, robot_y, robot_yaw = pr2.get_pose()

        

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist = math.hypot(dx, dy)

        

        if dist < pos_tol:
            break

        lidar_ranges, lidar_fov = pr2.get_lidar()

        front_min = float("inf")
        left_min = float("inf")
        right_min = float("inf")


        

        if lidar_ranges:
            n = len(lidar_ranges)
            c = n // 2

            front_vals = [r for r in lidar_ranges[max(0, c - n // 14):min(n, c + n // 14 + 1)]
                          if not (math.isnan(r) or math.isinf(r))]
            left_vals = [r for r in lidar_ranges[min(n - 1, c + n // 10):min(n, c + n // 4)]
                         if not (math.isnan(r) or math.isinf(r))]
            right_vals = [r for r in lidar_ranges[max(0, c - n // 4):max(0, c - n // 10)]
                          if not (math.isnan(r) or math.isinf(r))]


            if front_vals:
                front_min = min(front_vals)
            if left_vals:
                left_min = min(left_vals)
            if right_vals:
                right_min = min(right_vals)

        goal_angle = math.atan2(dy, dx)
        yaw_error = angle_diff(goal_angle, robot_yaw)


        report(pr2,dist,front_min,left_min,right_min, goal_angle,yaw_error, front_min,left_min,right_min)

        #looks like up until checkpoint1 the robot
        #rotates to face depot table. 
        if front_min < 0.75:
            if left_min > right_min:
                pr2.rotate_in_place(0.45)
            else:
                pr2.rotate_in_place(-0.45)
            continue
        

        #checkpoint1
        if(1):

            if abs(yaw_error) > 0.35:
                turn = max(-1.8, min(1.8, 2.0 * yaw_error))
                left = -turn
                right = turn
            else:
                forward = min(3.0, 1.2 + dist)
                turn = 1.2 * yaw_error
                left = forward - turn
                right = forward + turn

            left = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left))
            right = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right))

            pr2.set_wheel_speeds(left, right)

            if not pr2.step():
                return

        pr2.stop()

        for _ in range(200):
            _, _, robot_yaw = pr2.get_pose()
            yaw_error = angle_diff(goal_yaw, robot_yaw)

            if abs(yaw_error) < yaw_tol:
                break

            pr2.rotate_in_place(max(-0.35, min(0.35, yaw_error)))

        pr2.stop()



robot_state=0
counter=0


#TODO 8
#Gavin Code
def navigate_to_goal(pr2, goal_x, goal_y, goal_yaw, env_map):

    
    global robot_state
    global counter
    


    pos_tol = 0.28
    yaw_tol = 0.18
    max_steps = 20000

    def angle_diff(a, b):
        d = a - b
        return (d + math.pi) % (2 * math.pi) - math.pi



    for _ in range(max_steps):
        robot_x, robot_y, robot_yaw = pr2.get_pose()



        #potential field method 2, navigate to current pose+ dx and +dy until goal position actually reached.
        #result=compute_potential_field(pr2.get_pose()[0],pr2.get_pose()[1], goal_x, goal_y, lidar_ranges, lidar_fov)
        #dx=result[0]
        #dy=result[1]
        #goal_x=robot_x+dx
        #goal_y=robot_y+dy

        
            

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist = math.hypot(dx, dy)

        

        if dist < pos_tol:
            break

        lidar_ranges, lidar_fov = pr2.get_lidar()


        print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@NOW OUTPUTTING LIDAR FOV")
        print(lidar_fov)
        print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@NOW OUTPUTTING LIDAR ranges")
        #print(lidar_ranges)

        front_min = float("inf")
        left_min = float("inf")
        right_min = float("inf")


        

        if lidar_ranges:
            n = len(lidar_ranges)
            c = n // 2

            front_vals = [r for r in lidar_ranges[max(0, c - n // 14):min(n, c + n // 14 + 1)]
                          if not (math.isnan(r) or math.isinf(r))]
            left_vals = [r for r in lidar_ranges[min(n - 1, c + n // 10):min(n, c + n // 4)]
                         if not (math.isnan(r) or math.isinf(r))]
            right_vals = [r for r in lidar_ranges[max(0, c - n // 4):max(0, c - n // 10)]
                          if not (math.isnan(r) or math.isinf(r))]


            if front_vals:
                front_min = min(front_vals)
            if left_vals:
                left_min = min(left_vals)
            if right_vals:
                right_min = min(right_vals)

        goal_angle = math.atan2(dy, dx)
        
        yaw_error = angle_diff(goal_angle, robot_yaw)


        

  
        

        #checkpoint1
        if(1):

            
           
            robot_pose_x=pr2.get_pose()[0]
            robot_pose_y=pr2.get_pose()[1]


            result2=compute_potential_field(robot_pose_x,robot_pose_y, goal_x, goal_y, lidar_ranges, lidar_fov)
            result3=compute_potential_field_reinit(robot_pose_x,robot_pose_y, goal_x, goal_y, lidar_ranges, lidar_fov)
          
            print("potential field implementation 1 "+ str(result2))
            print("Potential field implementation 2 " + str(result3))



            #compute_potential_field2 is literially just adding the min left and min right values 
            #to the goal angle -> adds in a sort of potential field, things closer will repulse the 
            #robot angle away. 

            x2=result2[0]
            y2=result2[1]
            

            goal_x=robot_pose_x+x2
            goal_y=robot_pose_y+y2

            #goal_angle=goal_angle+x2+y2

            dx = goal_x - robot_pose_x
            dy = goal_y - robot_pose_y
            dist = math.hypot(dx, dy)
            goal_angle = math.atan2(dy, dx)
            yaw_error = angle_diff(goal_angle, robot_yaw)
            
            

            
            report(pr2,dist,front_min,left_min,right_min, goal_angle,yaw_error, front_min,left_min,right_min,dx,dy,goal_x,goal_y)
           


                #pr2.rotate_in_place(max(-0.35, min(0.35, yaw_error)))

            

            #pr2.set_wheel_speeds(MAX_WHEEL_SPEED ,MAX_WHEEL_SPEED)




            #
               # print("NOW DRIVING ")
            

            
         
            #if ((_%1000) > 500):
            if(yaw_error<0):
                pr2.rotate_in_place(max(-0.35, min(0.35, yaw_error)))
                print("@@@@@@@@@@@@@@@@@@@@@@@ROTATING")
            else:
                pr2.stop()
                pr2.set_wheel_speeds(MAX_WHEEL_SPEED ,MAX_WHEEL_SPEED)
                print("@@@@@@@@@@@@@@@@@@@@@@@DRIVING")
           

      





def navigate_to_goal_test(pr2, goal_x, goal_y, goal_yaw, env_map):

    pos_tol = 0.28
    yaw_tol = 0.18
    max_steps = 2000

    def angle_diff(a, b):
        d = a - b
        return (d + math.pi) % (2 * math.pi) - math.pi

    for _ in range(max_steps):
        robot_x, robot_y, robot_yaw = pr2.get_pose()

        

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist = math.hypot(dx, dy)

        


        #lidar_ranges, lidar_fov = pr2.get_lidar()

        front_min = float("inf")
        left_min = float("inf")
        right_min = float("inf")


        #report(pr2,dist,front_min,left_min,right_min, 1)

        #left = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left))
        #right = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right))
  

        pr2.set_wheel_speeds(MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)



















#Do not modify the main
def main():
    print("main started")
    pr2 = PR2Controller()

    #currently this checkpoint is never reached so there is something wrong with pr2
    print("main checkpoint 1")

    env = load_environment_map()
    #load_environment_map()
    #this 
    print("main checkpoint 1")

    objects    = env.get("pick_objects",    {})
    print("now printing objects")
    print(objects)
    nav_goals  = env.get("navigation_goals", {})
    place_zone = env.get("place_zone",      {})

    print("main checkpoint 2")
    # ── Pick OBJECT_1 ─────────────────────────────────────────────────────────
    if "OBJECT_1" in objects:
        print("OBJECT 1 DETECTED")
        g1 = nav_goals["OBJECT_1"]
        navigate_to_goal(pr2, g1["position"][0], g1["position"][1],
                         g1["yaw_radians"], env)
        pick_object(pr2, objects["OBJECT_1"])

    # ── Pick OBJECT_2 ─────────────────────────────────────────────────────────
    if "OBJECT_2" in objects:
        g2 = nav_goals["OBJECT_2"]
        navigate_to_goal(pr2, g2["position"][0], g2["position"][1],
                         g2["yaw_radians"], env)
        pick_object(pr2, objects["OBJECT_2"])

    # ── Place both objects ────────────────────────────────────────────────────
    if place_zone:
        pg = place_zone["nav_goal"]
        navigate_to_goal(pr2, pg["position"][0], pg["position"][1],
                         pg["yaw_radians"], env)
        place_objects(pr2, place_zone)

    print(f"\n[Lab7] Done!  Total collisions: {pr2.get_collision_count()}")
    print(f"[Lab7] Collision penalty: -{pr2.get_collision_count() * 5} pts")
    while pr2.step():
        pass


if __name__ == "__main__":
    main()