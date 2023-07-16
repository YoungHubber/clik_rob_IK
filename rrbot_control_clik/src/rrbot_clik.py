#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import PyKDL as kdl

class RRBotCLIK():
    def __init__(self):
        rospy.init_node('rrbot_clik')
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.pose_sub = rospy.Subscriber('/pose', Pose, self.pose_callback)
        self.joint_traj_pub = rospy.Publisher('/rrbot/joint_trajectory_controller/command', JointTrajectory, queue_size=1)

        # Assuming rrbot with 2 revolute joints
        self.kdl_chain = kdl.Chain()
        self.kdl_chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ)))
        self.kdl_chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ)))

        self.joint_states = JointState()
        self.target_pose = Pose()
        self.has_pose = False
        self.has_state = False

        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)
        self.ik_vel_solver = kdl.ChainIkSolverVel_pinv(self.kdl_chain)
        self.ik_pos_solver = kdl.ChainIkSolverPos_NR(self.kdl_chain, self.fk_solver, self.ik_vel_solver)

    def joint_states_callback(self, data):
        rospy.loginfo("Received joint state: %s", data)
        self.joint_states = data
        self.has_state = True

    def pose_callback(self, data):
        rospy.loginfo("Received pose: %s", data)
        self.target_pose = data
        self.has_pose = True

    def pose_to_kdl_frame(self, pose):
        return kdl.Frame(kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), 
                         kdl.Vector(pose.position.x, pose.position.y, pose.position.z))

    def joint_state_to_kdl(self, joint_state):
        kdl_jnt_array = kdl.JntArray(len(joint_state.position))
        for i, pos in enumerate(joint_state.position):
            kdl_jnt_array[i] = pos
        return kdl_jnt_array

    def compute_ik(self, target_pose, initial_joint_state):
        target_frame = self.pose_to_kdl_frame(target_pose)
        initial_joints = self.joint_state_to_kdl(initial_joint_state)
        final_joints = kdl.JntArray(2) # Assuming 2 joints

        status = self.ik_pos_solver.CartToJnt(initial_joints, target_frame, final_joints)
        rospy.loginfo("IK computation result: %s, %s", status, [final_joints[i] for i in range(final_joints.rows())])

        return status, final_joints

    def control_loop(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if not self.has_pose or not self.has_state:
                continue

            status, joint_solution = self.compute_ik(self.target_pose, self.joint_states)

            if status >= 0: # If a solution was found
                trajectory = JointTrajectory()
                trajectory.joint_names = self.joint_states.name
                point = JointTrajectoryPoint()
                point.positions = joint_solution
                point.time_from_start = rospy.Duration(1.0)
                trajectory.points.append(point)

                rospy.loginfo("Publishing trajectory: %s", trajectory)
                self.joint_traj_pub.publish(trajectory)

            rate.sleep()

if __name__ == '__main__':
    try:
        rrbot_clik = RRBotCLIK()
        rrbot_clik.control_loop()
    except rospy.ROSInterruptException:
        pass

