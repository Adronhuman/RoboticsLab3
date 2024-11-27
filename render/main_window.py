from collections import deque
import pybullet as p
import pybullet_data
import time
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple

from config import config

planeId = None
_link_name_to_index = {}

def initialize_environment():
    global planeId
    physicsClient = p.connect(p.GUI)  # or p.DIRECT
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    p.setGravity(0, 0, -9.81)

def rozkumar():
    for _ in range(1000):
        p.stepSimulation()
        time.sleep(0.001)

def create_spider():
    path_to_model = os.path.join(config.Config.PROJECT_ROOT_DIR, "models/SpiderBot_4Legs/urdf/SpiderBot_4Legs.urdf")
    body_id = p.loadURDF(path_to_model, [0,0,1])

    return body_id

def ketamine_dreams(spider_brat):

    # reminder: you can retrieve linkid from _link_name_to_index
    # leg3_shin -- leg2_shin
    # --                  --
    # leg4_shin -- leg1_shin
    front_left_leg_joint = 5
    front_right_leg_joint = 7
    back_left_leg_joint = 1
    back_right_leg_joint = 3
    back_left_link = _link_name_to_index["leg1_shin"]
    back_right_link = _link_name_to_index["leg2_shin"]
    def delay(x=100, while_in_contact=False):
        for _ in range(x):
            p.stepSimulation()
            time.sleep(0.0001)
        
        contact_left = contact_right = False
        while not contact_left and not contact_right:
            p.stepSimulation()
            contact_left = p.getContactPoints(bodyA=spider_brat, bodyB=planeId, linkIndexA=back_left_link)
            contact_left = p.getContactPoints(bodyA=spider_brat, bodyB=planeId, linkIndexA=back_right_link)

    while True:
        _30 = np.radians([30])[0]
        _45 = np.radians([45])[0]
        angle = _30
        force = 10
        p.setJointMotorControl2(spider_brat, back_left_leg_joint, p.POSITION_CONTROL, targetPosition=angle, force=10)
        p.setJointMotorControl2(spider_brat, back_right_leg_joint, p.POSITION_CONTROL, targetPosition=angle, force=10)
        # delay()
        # p.setJointMotorControl2(spider_brat, front_left_leg_joint, p.POSITION_CONTROL, targetPosition=angle, force=10)
        # p.setJointMotorControl2(spider_brat, front_right_leg_joint, p.POSITION_CONTROL, targetPosition=angle, force=10)
        delay()
        # time.sleep(111)

        p.setJointMotorControl2(spider_brat, front_left_leg_joint, p.POSITION_CONTROL, targetPosition=-angle, force=force)
        p.setJointMotorControl2(spider_brat, front_right_leg_joint, p.POSITION_CONTROL, targetPosition=-angle, force=force)
        
        delay(50)
        
        p.setJointMotorControl2(spider_brat, front_left_leg_joint, p.POSITION_CONTROL, targetPosition=0, force=force)
        p.setJointMotorControl2(spider_brat, front_right_leg_joint, p.POSITION_CONTROL, targetPosition=0, force=force)
        delay()

        p.setJointMotorControl2(spider_brat, back_left_leg_joint, p.POSITION_CONTROL, targetPosition=0, force=force)
        p.setJointMotorControl2(spider_brat, back_right_leg_joint, p.POSITION_CONTROL, targetPosition=0, force=force)
        delay(while_in_contact=True)

        


def jump_motherfucker(spider_brat):
    front_left_leg_joint = 5
    front_right_leg_joint = 7
    back_left_leg_joint = 1
    back_right_leg_joint = 3

    while True:
        _45 = np.pi/4
        p.setJointMotorControl2(spider_brat, front_left_leg_joint, p.POSITION_CONTROL, targetPosition=-_45, force=20)
        p.setJointMotorControl2(spider_brat, front_right_leg_joint, p.POSITION_CONTROL, targetPosition=-_45, force=20)
        p.setJointMotorControl2(spider_brat, back_left_leg_joint, p.POSITION_CONTROL, targetPosition=_45, force=20)
        p.setJointMotorControl2(spider_brat, back_right_leg_joint, p.POSITION_CONTROL, targetPosition=_45, force=20)

        for _ in range(100):
            p.stepSimulation()
            time.sleep(0.01) 

        p.setJointMotorControl2(spider_brat, front_left_leg_joint, p.POSITION_CONTROL, targetPosition=0.0, force=40)
        p.setJointMotorControl2(spider_brat, front_right_leg_joint, p.POSITION_CONTROL, targetPosition=0.0, force=40)
        p.setJointMotorControl2(spider_brat, back_left_leg_joint, p.POSITION_CONTROL, targetPosition=0.0, force=40)
        p.setJointMotorControl2(spider_brat, back_right_leg_joint, p.POSITION_CONTROL, targetPosition=0.0, force=40)

        for _ in range(1000):
            p.stepSimulation()
            time.sleep(0.01)

if __name__ == "__main__":
    initialize_environment()
    spider_brat = create_spider()
    num_joints = p.getNumJoints(spider_brat)
    for i in range(num_joints):
        joint_info = p.getJointInfo(spider_brat, i)
        _link_name_to_index[joint_info[12].decode('utf-8')] = i
        # print(f"Joint {i}: {joint_info[1].decode('utf-8')}, Type: {joint_info[2]}")
    # print(_link_name_to_index)
    rozkumar()

    # jump_motherfucker(spider_brat)
    time.sleep(10)
    ketamine_dreams(spider_brat)

    