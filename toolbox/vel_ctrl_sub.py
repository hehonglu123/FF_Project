from RobotRaconteur.Client import *
import time,traceback
import numpy as np

class VelocityControl(object):
    def __init__(self, robot, state_wire, vel_command_wire):
        self._seqno = 0
        self._state_wire = state_wire
        self._vel_command_wire = vel_command_wire
        self.RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)

    def enable_velocity_mode(self):
        return
    def disable_velocity_mode(self):
        return
        
    def joint_position(self):
        return self._state_wire.InValue.joint_position
    def robot_pose(self):
        return self._state_wire.InValue.kin_chain_tcp[0]

    def set_velocity_command(self, qdot):
        joint_cmd1 = self.RobotJointCommand()
        self._seqno += 1
        joint_cmd1.seqno = self._seqno
        joint_cmd1.state_seqno = self._state_wire.InValue.seqno
        joint_cmd1.command = qdot
        self._vel_command_wire.SetOutValueAll(joint_cmd1) 
