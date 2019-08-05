#///////////////////////////////////////////////////////////////////////////////
#// BSD 3-Clause License
#//
#// Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
#// Copyright note valid unless otherwise stated in individual files.
#// All rights reserved.
#///////////////////////////////////////////////////////////////////////////////

# brief Example for using the PinBulletWrapper for a quadruped robot.

from __future__ import print_function

import os
import rospkg
import numpy as np
import time

import robot_properties_solo
from robot_properties_solo.config import SoloConfig

import pybullet as p
import pinocchio as se3
from pinocchio.utils import zero

from py_pinocchio_bullet.wrapper import PinBulletWrapper

class QuadrupedRobot(PinBulletWrapper):
    def __init__(self, physicsClient=None):
        if physicsClient is None:
            self.physicsClient = p.connect(p.DIRECT)
            p.setGravity(0,0, -9.81)
            p.setPhysicsEngineParameter(fixedTimeStep=1.0/1000.0, numSubSteps=1)

        # Load the plain.
        plain_urdf = (rospkg.RosPack().get_path("robot_properties_solo") +
                      "/urdf/plane_with_restitution.urdf")
        self.planeId = p.loadURDF(plain_urdf)

        # Load the robot
        robotStartPos = [0.,0,0.40]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

        self.urdf_path = SoloConfig.urdf_path
        self.robotId = p.loadURDF(self.urdf_path, robotStartPos,
            robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=False)
        p.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) + '/urdf']
        self.pin_robot = SoloConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
            p.changeDynamics(self.robotId, ji, linearDamping=.04,
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        self.base_link_name = "base_link"
        self.joint_names = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE',
        'HL_KFE', 'HR_HFE', 'HR_KFE']
        controlled_joints = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE',
        'HL_KFE', 'HR_HFE', 'HR_KFE']

        # Creates the wrapper by calling the super.__init__.
        super(QuadrupedRobot,self).__init__(self.robotId, self.pin_robot,
            controlled_joints,
            ['FL_ANKLE', 'FR_ANKLE', 'HL_ANKLE', 'HR_ANKLE']
        )

if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)

    # Setup pybullet for the quadruped and a wrapper to pinocchio.
    quad = QuadrupedRobot()

    # Get the current state and modify the joints to have the legs
    # bend inwards.
    q, dq = quad.get_state()
    q[7] = q[9] = 0.8
    q[11] = q[13] = -0.8
    q[8] = q[10] = -1.6
    q[12] = q[14] = 1.6

    # Take the initial joint states as desired state.
    q_des = q[7:].copy()

    # Update the simulation state to the new initial configuration.
    quad.reset_state(q, dq)

    # Run the simulator for 2000 steps = 2 seconds.
    for i in range(2000):
        # Get the current state (position and velocity)
        q, dq = quad.get_state()
        active_contact_frames, contact_forces = quad.get_force()

        if i % 100 == 0:
            print('Forces:', active_contact_frames, contact_forces)

        # Compute the command torques at the joints. The torque
        # vector only takes the actuated joints (excluding the base)
        tau = 5. * (q_des - q[7:]) - 0.1 * dq[6:]

        # Send the commands to the robot.
        quad.send_joint_command(tau)

        # Step the simulator and sleep.
        p.stepSimulation()
        time.sleep(0.001)

    # Print the final active force frames and the forces
    force_frames, forces = quad.get_force()

    print("Active force_frames:", force_frames)
    print("Corresponding forces:", forces)

