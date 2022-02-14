from json import tool
import sys, os
import pybullet as p
import pybullet_data
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils import robot

from spatialmath import SE3,SO3

class PyBulletRobot(robot.Robot):
    def __init__(self, urdf_filename: str, base_transform: SE3 = SE3()):
        super().__init__(urdf_filename, base_transform)
        self.__base_pose = base_transform.t.tolist() # World position [x,y,z]
        self.__base_orient = R.from_matrix(base_transform.R).as_quat().tolist() # Quaternioun [x,y,z,w]
        
        self.__robot_id = p.loadURDF(
            self._urdf_filename,
            basePosition=self.__base_pose,
            baseOrientation=self.__base_orient,
            useFixedBase=True
        )
        self.__num_joints = p.getNumJoints(self.__robot_id)
        self.__actuators_id_list = []
        self.__link_id = {}
        for _id in range(self.__num_joints):
            # print(p.getJointInfo(self.__robot_id, _id))
            _name = p.getJointInfo(self.__robot_id, _id)[12].decode('UTF-8')
            if p.getJointInfo(self.__robot_id, _id)[4] != -1:
                self.__actuators_id_list.append(_id)
            self.__link_id[_name] = _id
        
        self.__num_actuators = len(self.__actuators_id_list)
        self._joint_state = robot.JointState(self.__num_actuators)
        p.setJointMotorControlArray(self.__robot_id, self.__actuators_id_list,
                                    p.VELOCITY_CONTROL, 
                                    forces=np.zeros(self.__num_actuators))
        self._send_jointcontrol_torque(np.zeros(self.__num_actuators))
        self._force_sensor_link = None
        self.__joint_controller_params = {
            'kp': np.ones(self.__num_actuators),
            'kd': np.ones(self.__num_actuators),
            'max_torque': 100*np.ones(self.__num_actuators)
        }

    def jacobian(self, joint_pose: np.ndarray, ee_link: str, ref_frame: str) -> np.ndarray:
        Jv = np.zeros((3, len(joint_pose)))
        Jw = np.zeros((3, len(joint_pose)))

        if not ee_link is 'world':
            jac_t, jac_r = p.calculateJacobian(
                self.__robot_id, self.__link_id[ee_link], [0,0,0],
                list(joint_pose), list(np.zeros(joint_pose.shape)),
                list(np.zeros(joint_pose.shape))
            )
            Jv = np.asarray(jac_t)
            Jw = np.asarray(jac_r)
                
        if not ref_frame is 'world':
            print("World")
            refFrameState = p.getLinkState(self.__robot_id, self.__link_id[ref_frame])
            _,_,_,_, ref_frame_pos, ref_frame_rot = refFrameState
            rot_matrix =  SO3(R.from_quat(ref_frame_rot).as_matrix(), check=False)
            Jv = rot_matrix.T @ np.asarray(Jv)
            Jw = rot_matrix.T @ np.asarray(Jw)

        J = np.concatenate((Jv,Jw), axis=0)
        return J
    
    def apply_force_sensor(self, link: str):
        self._force_sensor_link = link
        p.enableJointForceTorqueSensor(self.__robot_id, self.__link_id[link], 1)        
    
    def _send_cartcontrol_position(self, position: np.ndarray):
        raise RuntimeError('Robot does not support this type of control')

    def _send_cartcontrol_velocity(self, velocity: np.ndarray):
        raise RuntimeError('Robot does not support this type of control')

    def _send_jointcontrol_velocity(self, velocity: np.ndarray):
        p.setJointMotorControlArray(self.__robot_id,
            self.__actuators_id_list,
            p.VELOCITY_CONTROL,
            targetVelocities=velocity.tolist(),
            forces = self.__joint_controller_params['max_torque'].tolist()
        )
    
    def _send_jointcontrol_position(self, position: np.ndarray):
        p.setJointMotorControlArray(self.__robot_id,
            self.__actuators_id_list,
            p.POSITION_CONTROL,
            targetPositions=position.tolist(),
            targetVelocities=np.zeros(self.__num_actuators).tolist(),
            positionGains=self.__joint_controller_params['kp'].tolist(),
            velocityGains=self.__joint_controller_params['kd'].tolist()
        )
    
    def _send_jointcontrol_torque(self, torque: np.ndarray):
        p.setJointMotorControlArray(self.__robot_id, self.__actuators_id_list,
            p.VELOCITY_CONTROL, 
            forces=np.zeros(self.__num_actuators))
        p.setJointMotorControlArray(self.__robot_id, 
            self.__actuators_id_list,
            controlMode = p.TORQUE_CONTROL, 
            forces = torque.tolist()
        )
    
    def _update_cartesian_state(self, tool_state: robot.EEState):
        if not tool_state.ee_link is 'world':
            eeState = p.getLinkState(self.__robot_id, self.__link_id[tool_state.ee_link])
            _,_,_,_, link_frame_pos, link_frame_rot = eeState
        else:
            link_frame_pos = np.zeros(3)
            link_frame_rot = np.array([0,0,0,1])

        tool_state.tf = SE3(*link_frame_pos) @ SE3(SO3(R.from_quat(link_frame_rot).as_matrix(), check=False))

        if not tool_state.ref_frame is 'world':
            refFrameState = p.getLinkState(self.__robot_id, self.__link_id[tool_state.ref_frame])
            _,_,_,_, ref_frame_pos, ref_frame_rot = refFrameState
            tool_state.tf = (SE3(*ref_frame_pos) @ SE3(SO3(R.from_quat(ref_frame_rot).as_matrix(), check=False))).inv() @ tool_state.tf 

        
        tool_state.twist = self.jacobian(self.joint_state.joint_positions, tool_state.ee_link, tool_state.ref_frame).dot(
            self.joint_state.joint_velocities
        )
        
        if not self._force_sensor_link is None:
            pb_joint_state = p.getJointStates(self.__robot_id, [self.__link_id[self._force_sensor_link]])
            tool_state.force_torque = np.array(pb_joint_state[0][2])
    
    def _update_joint_state(self, joint_state: robot.JointState):
        pb_joint_state = p.getJointStates(self.__robot_id, self.__actuators_id_list)
        joint_state.joint_positions = np.array([state[0] for state in pb_joint_state])
        joint_state.joint_velocities = np.array([state[1] for state in pb_joint_state])
        joint_state.joint_torques = np.array([state[3] for state in pb_joint_state])
    
    @property
    def joint_controller_params(self) -> dict:
        return self.__joint_controller_params
    
    @joint_controller_params.setter
    def joint_controller_params(self, controller_params: dict):
        assert 'kp' in controller_params.keys() and 'kd' in controller_params.keys(), 'Dictionary does not contain kp or kd parameters'
        assert len(controller_params['kp']) == self.__num_actuators and len(controller_params['kd']) == self.__num_actuators, 'Shape of given parameters is not equal number of actuators'
        self.__joint_controller_params = controller_params
