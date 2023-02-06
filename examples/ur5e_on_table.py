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

controller_params = {'kp': np.array([12.0, 12.0, 12.0, 2.0, 2.0, 1.0]), 'kd': np.array([1.0, 5.0, 1.0, 0.05, 0.05, 0.05]) * 40}

joint_pose = np.array([0.0, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0])
tf = SE3(0.3, 0.0, 1.0) @ SE3.Rx(np.pi)

target_joint_state = JointState.from_position(joint_pose)

target_ee_state = EEState.from_tf(tf, 'ee_tool')
target_motion = Motion.from_states(copy.deepcopy(target_joint_state), copy.deepcopy(target_ee_state))

final_time = 5.0

def main():
    sim = PyBulletWorld(gui_mode = GUI_MODE.SIMPLE_GUI, time_step = 0.01, time_scale=1.0)
    sim.add_object('table', 'tests/urdf/table.urdf', save=True)
    robot = sim.add_robot('tests/urdf/ur5e_pybullet.urdf', SE3(0,0,0.625), 'robot1')
    robot.joint_controller_params = controller_params

    controller_joint_pose = JointPositionsController(robot)

    robot.reset_joint_state(JointState.from_position(joint_pose))
    random_target_state = joint_pose + np.random.uniform(-np.pi/5, np.pi/5, joint_pose.shape)
    target_motion.joint_state.joint_positions = random_target_state
    # log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'assets/ur5e.mp4')
    while sim.sim_time<final_time:
        controller_joint_pose.send_control_to_robot(target_motion)
        sim.sim_step()
    # p.stopStateLogging(log_id)

if __name__ == "__main__":
    main()
