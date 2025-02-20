#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# 抓取
# python command_grasp.py name 
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def publish_trajectory_point():
    rospy.init_node('gripper_trajectory_publisher', anonymous=True)
    traj_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['gripper', 'gripper_sub']  # 根据你的实际关节名称修改
    traj_point = JointTrajectoryPoint()
    traj_point.positions = [0.1, 0.1]  # 设置夹爪关节的位置
    traj_point.velocities = [0.05, 0.05]  # 设置夹爪关节的速度
    traj_point.accelerations = [0.005, 0.005]  # 设置夹爪关节的加速度
    traj_point.effort = [10, 10]  # 设置夹爪关节的力矩
    traj_point.time_from_start = rospy.Duration(2.0)  # 设置到达目标位置的时间
    traj_msg.points.append(traj_point)

    while not rospy.is_shutdown():
        traj_pub.publish(traj_msg)
        rospy.loginfo("Published gripper trajectory point: positions={}, velocities={}, accelerations={}, effort={}".format(
            traj_point.positions, traj_point.velocities, traj_point.accelerations, traj_point.effort))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory_point()
    except rospy.ROSInterruptException:
        pass

