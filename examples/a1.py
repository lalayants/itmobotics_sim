import os
import sys
from typing import final
import numpy as np
import copy
import pybullet as p 

from itmobotics_sim.utils.math import vec2SE3
from itmobotics_sim.utils.robot import EEState, JointState, Motion
from itmobotics_sim.pybullet_env.pybullet_world import PyBulletWorld, GUI_MODE
from itmobotics_sim.pybullet_env.pybullet_robot import PyBulletRobot

from spatialmath import SE3
from spatialmath import base as sb
from itmobotics_sim.utils.controllers import EEPositionToEEVelocityController, EEVelocityToJointVelocityController, JointPositionsController, JointTorquesController, JointVelocitiesController

final_time = 5.0

A1_DEFAULT_ABDUCTION_ANGLE = 0
A1_DEFAULT_HIP_ANGLE = 0.9
A1_DEFAULT_KNEE_ANGLE = -1.8
NUM_LEGS = 4
joint_pose = np.array([
    A1_DEFAULT_ABDUCTION_ANGLE,
    A1_DEFAULT_HIP_ANGLE,
    A1_DEFAULT_KNEE_ANGLE
] * NUM_LEGS)

controller_params = {'kp': 1*np.ones(12), 'kd': 1*np.array([1.0, 1.0, 1.0]*4)}

joint_state = JointState.from_position(joint_pose)

target_motion = Motion.from_joint_state(copy.deepcopy(joint_state))

def main():
    sim = PyBulletWorld(gui_mode = GUI_MODE.SIMPLE_GUI, time_step = 0.001, time_scale=1)
    sim.add_object('plane', 'plane.urdf', save=True)
    robot = sim.add_robot('a1/a1.urdf', SE3(0,0,0.4), 'a1')
    robot.joint_controller_params = controller_params

    print(robot.joint_limits)
    # print(robot.joint_state)
    # print(robot.joint_controller_params)
    # print(target_motion.joint_state)
    controller_joint_pose = JointPositionsController(robot)

    robot.reset_joint_state(JointState.from_position(joint_pose))
    # random_target_state = joint_pose + np.random.uniform(-np.pi/12, np.pi/12, joint_pose.shape)
    # target_motion.joint_state.joint_positions = random_target_state
    # log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'assets/a1.mp4')
    while sim.sim_time<final_time:
        controller_joint_pose.send_control_to_robot(target_motion)
        sim.sim_step()

    # p.stopStateLogging(log_id)


if __name__ == '__main__':
    main()
