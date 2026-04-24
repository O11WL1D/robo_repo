import json
import math
import os
from controller import Supervisor

print("SUPERVISOR STARTED")
robot = Supervisor()
ts = int(robot.getBasicTimeStep())

base_dir = os.path.dirname(os.path.abspath(__file__))
controller_dir = os.path.abspath(os.path.join(base_dir, ".."))

env_path  = os.path.join(controller_dir, "lab7_pr2", "environment_map.json")
pose_path = os.path.join(controller_dir, "lab7_pr2", "robot_pose.json")

print(f"env_path:  {env_path}")
print(f"pose_path: {pose_path}")

pr2 = robot.getFromDef("PR2")
obj1 = robot.getFromDef("OBJECT_1")
obj2 = robot.getFromDef("OBJECT_2")
place_table = robot.getFromDef("place_table")

if pr2 is None:
    raise RuntimeError("PR2 DEF not found")
if obj1 is None:
    raise RuntimeError("obj1 DEF not found")

pr2_trans = pr2.getField("translation")
pr2_rot   = pr2.getField("rotation")


def get_yaw():
    r = pr2_rot.getSFRotation()
    axis_x, axis_y, axis_z, angle = r
    if axis_z < 0:
        angle = -angle
    return angle


def write_environment_map():
    data = {
        "pick_objects": {},
        "navigation_goals": {},
        "place_zone": {}
    }

    if obj1 is not None:
        p = obj1.getField("translation").getSFVec3f()
        data["pick_objects"]["OBJECT_1"] = {
            "approach_position": [p[0], p[1], p[2] + 0.22],
            "pick_position":     [p[0], p[1], p[2] + 0.10],
            "node_type": obj1.getTypeName()
        }

    if obj2 is not None:
        p = obj2.getField("translation").getSFVec3f()
        data["pick_objects"]["OBJECT_2"] = {
            "approach_position": [p[0], p[1], p[2] + 0.25],
            "pick_position":     [p[0], p[1], p[2] + 0.10],
            "node_type": obj2.getTypeName()
        }

    
    data["navigation_goals"]["OBJECT_1"] = {
        "position": [-2.743, -2.599],
        "yaw_radians": -math.pi / 2
    }

   
    data["navigation_goals"]["OBJECT_2"] = {
        "position": [2.05, -3.42],
        "yaw_radians": 0
    }

    data["place_zone"] = {
        "name": "place_table",
        "approach_position": [0.0, 5.0, 0.80],
        "place_position":    [0.0, 5.0, 0.66],
        "nav_goal": {
            "position": [0.0, 3.8],
            "yaw_radians": math.pi / 2
        }
    }

    os.makedirs(os.path.dirname(env_path), exist_ok=True)

    with open(env_path, "w") as f:
        json.dump(data, f)
        f.flush()

    print(f"environment_map written to {env_path}")


write_environment_map()

while robot.step(ts) != -1:
    p = pr2_trans.getSFVec3f()
    pose = {
        "x":   p[0],
        "y":   p[1],
        "yaw": get_yaw()
    }

    with open(pose_path, "w") as f:
        json.dump(pose, f)
        f.flush()