#!/usr/bin/env python3

import time
from math import atan2, ceil, copysign, cos, pi, radians, sin

import numpy
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion

pose = [0, 0, 0]
angle_to_goal = 0
steps_to_goal = 64
goal_pose = [None, None, None]
path_last_updated = 0

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def goal_callback(msg):
    global goal_pose
    x  = msg.pose.orientation.x
    y  = msg.pose.orientation.y
    z = msg.pose.orientation.z
    w = msg.pose.orientation.w
    goal_pose = [msg.pose.position.x, msg.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def path_callback(msg):
    global pose, angle_to_goal, steps_to_goal, path_last_updated
    
    rot_theta_about_z = radians(-90)
    rot_axis_matrix = numpy.array([[cos(rot_theta_about_z), sin(rot_theta_about_z)],[-sin(rot_theta_about_z), cos(rot_theta_about_z)]])
    
    goal_traj_1 = numpy.matmul(rot_axis_matrix, numpy.array([[msg.poses[0].pose.position.x], [msg.poses[0].pose.position.y]]))
    if len(msg.poses) > 5:
        goal_traj_2 = numpy.matmul(rot_axis_matrix, numpy.array([[msg.poses[5].pose.position.x], [msg.poses[5].pose.position.y]]))
    else:
        goal_traj_2 = numpy.matmul(rot_axis_matrix, numpy.array([[msg.poses[len(msg.poses)-1].pose.position.x], [msg.poses[len(msg.poses)-1].pose.position.y]]))

    angle = atan2 ( (goal_traj_2[1]-goal_traj_1[1]), (goal_traj_2[0]-goal_traj_1[0]))
    if -pi <= angle < -pi/2:
        angle_to_goal = pi - (abs(angle) - pi/2) 
    elif -pi/2 <= angle < 0:
        angle_to_goal = -1*(pi/2 + abs(angle))
    elif 0 <= angle < pi/2:
        angle_to_goal = -1*(pi/2 - angle)
    else:
        angle_to_goal = angle - pi/2

    if angle_to_goal <= -3.10:
        angle_to_goal = -3.10
    elif angle_to_goal >= 3.10:
        angle_to_goal = 3.10

    steps_to_goal = len(msg.poses)
    path_last_updated = time.time()


def get_sign_modifier(target, current):
    if target*current > 0:
        if target > current:
            sign_modifier = 1
        else:
            sign_modifier = -1
    
    else:
        if pi >= abs(target) >= pi/2:
            sign_modifier = -1 * copysign(1, target)
        elif -pi/2 >= abs(target) >= 0:
            sign_modifier = copysign(1, target)
        else:
            sign_modifier = 1
    
    return sign_modifier


def cmd_vel_publisher():
    global pose, goal_pose, angle_to_goal, steps_to_goal
    
    rospy.init_node('cmd_vel_publisher', anonymous=True)

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, path_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
    
    rate = rospy.Rate(25)
    
    print(f"[INFO] [{time.time()}]: Velocity publisher node started.")
    
    goal_pose_reached = True
    while not rospy.is_shutdown():
        if goal_pose != [None, None, None]:
            goal_pose_reached = False
        
        while not goal_pose_reached:
            # Calculate velocity to reach goal.
            target_vel_msg = Twist()

            if time.time()-path_last_updated > 0.5 and steps_to_goal < 20:  
                # Goal reached, reorient towards goal point.
                while abs(goal_pose[2]-pose[2]) >= 0.15:
                    target_vel_msg.linear.x = 0
                    target_vel_msg.linear.y = 0
                    target_vel_msg.linear.z = 0

                    target_vel_msg.angular.x = 0
                    target_vel_msg.angular.y = 0
                    target_vel_msg.angular.z = get_sign_modifier(goal_pose[2], pose[2]) * 0.25
                    pub_cmd_vel.publish(target_vel_msg)

                target_vel_msg.angular.z = 0
                pub_cmd_vel.publish(target_vel_msg)
                goal_pose_reached = True

            else:
                target_vel_msg.linear.x = min(ceil(steps_to_goal/16),4)
                target_vel_msg.linear.y = 0
                target_vel_msg.linear.z = 0


                target_vel_msg.angular.x = 0
                target_vel_msg.angular.y = 0
                target_vel_msg.angular.z = get_sign_modifier(angle_to_goal, pose[2]) * (abs(angle_to_goal-pose[2]))
                pub_cmd_vel.publish(target_vel_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass
