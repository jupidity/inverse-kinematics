#!/usr/bin/env python

#
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Author: Sean Cassero

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses" % len(req.poses))
    if len(req.poses) < 1:
        print "No poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()


            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])


            # from the rpy received by this node, we can construct a rotation matrix
            # in order to derive the wrist center
            R_x = Matrix([[     1,         0 ,       0 ],
                         [      0,    cos(roll), -sin(roll)],
                         [      0,    sin(roll),  cos(roll)]])

            R_y = Matrix([[ cos(pitch),        0,  sin(pitch)],
                          [       0,        1,        0],
                          [-sin(pitch),        0,  cos(pitch)]])

            R_z = Matrix([[ cos(yaw), -sin(yaw),        0],
                          [ sin(yaw),  cos(yaw),        0],
                          [ 0,              0,        1]])


            R_rot = R_z * R_y * R_x


            # since the quaternion in relative to the urdf definition of the gripper frame, we apply two more rotations to
            # the above rotation matrix to get the correct orientation such that the gripper lies along the z axis
            R_corr = R_rot * Matrix([[0, 0, 1], [0, -1, 0], [1, 0,0]])

            l = .193 + .11# distance from the wrist center to the gripper.
            # Here the wrist center is assumed to be the location of joint 5, since it is the furthest fixed point
            # coincident with joint 3 and in the theta = q1 plane.
            d6 = 0

            # once we have the correct rotation matrix, we can solve for the position of the wrist center
            wc_x =   px - (d6+l) * R_corr[2]
            wc_y =   py - (d6+l) * R_corr[5]
            wc_z =   pz - (d6+l) * R_corr[8]

            # first, the q1 value can be obtained by projecting the wrist center onto the xy plane
            theta1 = float(atan2(wc_y,wc_x))

            d1 = .75 # length along z axis from base joint to J1
            L1 = 1.25   # Length along x2 axis from J2 to J3
            L2 = 1.5    # length along x3 axis from J3 to wc (joint 5)
            a1 = .35 # distance along x axis between J1 and J2
            r = sqrt((wc_x - a1*cos(theta1))**2 + (wc_y- a1*sin(theta1))**2 + (wc_z - d1)**2 ) #magnitude of vector from z - d1 axis to wrist center
            cosAlpha = (L2**2 - L1**2 -r**2) / (-2*L1*r) #cosine of alpha

            # In the equation below, sqrt(1+cosAlpha^2) works as well
            theta2 = float(pi/2 - atan2((wc_z - d1), sqrt((wc_x-a1*cos(theta1))**2 + (wc_y-a1*sin(theta1)) **2)) - atan2(sqrt(1-cosAlpha **2), cosAlpha))

            cosBeta = (r**2 - L1**2 -L2**2) / (-2*L1*L2)
            # here we also must subtract the offset from theta3 due to the fact that J4 is slightly lower on the z-axis
            theta3 =float( pi/2 - atan2(sqrt(1-cosBeta**2), cosBeta) - atan2(.056,1.5))

            #once the q1-q3 vales are known, we can derive the orientation

            # first, get the transformation matrix from the 3rd joint to the 6th joint to solve for q4-q6
            R0_3_inv = Matrix([
            [sin(theta2 + theta3)*cos(theta1), sin(theta1)*sin(theta2 + theta3),  cos(theta2 + theta3)],
            [cos(theta1)*cos(theta2 + theta3), sin(theta1)*cos(theta2 + theta3), -sin(theta2 + theta3)],
            [            -sin(theta1),              cos(theta1),             0]])



            R3_6 = R0_3_inv * R_corr

            # from here, we can just plug and chug to solve for the remaining angles
            theta6 = float(atan2((-R3_6[4]/sqrt(1-R3_6[5]**2)),(R3_6[3]/sqrt(1-R3_6[5]**2))))
            theta4 = float(atan2((R3_6[8]/sqrt(1-R3_6[5]**2)),(-R3_6[2]/sqrt(1-R3_6[5]**2))))
            theta5 = float(atan2((R3_6[3]/cos(theta6)),R3_6[5]))

        # Populate response for the IK request
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
