#!/usr/bin/env python3
from math import ceil, pi
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


reqd_vel = [0, 0, 0]


def vel_callback(msg):
    global reqd_vel
    reqd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]


def generate_angle_lists(gripped_step_size, air_step_size):
    grip_start_angle = -60
    
    # Make sure that the end angle is not negative, otherwise the calculation steps will break
    grip_end_angle = 20

    # Also make sure that the mid angle does not end up being a float, otherwise it will get truncated
    grip_mid_angle = (grip_start_angle + grip_end_angle)//2

    start_to_mid_angle_list = [i+360 if i<0 else i for i in range(grip_start_angle, grip_mid_angle, gripped_step_size)]
    mid_to_end_angle_list = [i+360 if i<0 else i for i in range(grip_mid_angle, grip_end_angle, gripped_step_size)]

    if len(start_to_mid_angle_list) != len(mid_to_end_angle_list):
        print ("The gripper angle list lengths are not equal! Terminating...")
        raise SystemExit

    if grip_start_angle < 0:
        grip_start_angle = grip_start_angle + 360

    end_to_start_angle_list = [i+360 if i<0 else i for i in range(grip_end_angle, grip_start_angle, air_step_size)]

    if grip_mid_angle < 0:
        grip_mid_angle = grip_mid_angle + 360

    # Arbitrarily starting the movement with the right side tripod
    left_tripod_angles = [grip_mid_angle]*len(end_to_start_angle_list) +\
                            mid_to_end_angle_list +\
                            end_to_start_angle_list +\
                            start_to_mid_angle_list
    left_tripod_angles = left_tripod_angles + left_tripod_angles[0:1]

    right_tripod_angles = end_to_start_angle_list +\
                            start_to_mid_angle_list +\
                            [grip_mid_angle]*len(end_to_start_angle_list) +\
                            mid_to_end_angle_list
    right_tripod_angles = right_tripod_angles + right_tripod_angles[0:1]

    # print("Here's what the lists look like:\nLeft tripod:", left_tripod_angles,"\n\nRight tripod:", right_tripod_angles, end="\n\n")

    if len(left_tripod_angles) != len(right_tripod_angles):
        print ("The angle list lengths are not equal! Terminating...")
        raise SystemExit
    
    return (left_tripod_angles, right_tripod_angles)


def control():
    global reqd_vel
    rospy.init_node('cmd_vel_control')
    rospy.Subscriber("/cmd_vel", Twist, vel_callback)
    print(f"[INFO] [{time.time()}]: Automatic control node started.")

    # R = Right, L = Left
    # H = Hind, M = Mid, F = Front
    # LJ = Leg Joint

    RHLJ_TOPIC = "/hexapod/right_hind_leg_joint_position_controller/command"
    RMLJ_TOPIC = "/hexapod/right_mid_leg_joint_position_controller/command"
    RFLJ_TOPIC = "/hexapod/right_front_leg_joint_position_controller/command"
    LHLJ_TOPIC = "/hexapod/left_hind_leg_joint_position_controller/command"
    LMLJ_TOPIC = "/hexapod/left_mid_leg_joint_position_controller/command"
    LFLJ_TOPIC = "/hexapod/left_front_leg_joint_position_controller/command"

    pub_rhlj = rospy.Publisher(RHLJ_TOPIC, Float64, queue_size=10)
    pub_rmlj = rospy.Publisher(RMLJ_TOPIC, Float64, queue_size=10)
    pub_rflj = rospy.Publisher(RFLJ_TOPIC, Float64, queue_size=10)
    pub_lhlj = rospy.Publisher(LHLJ_TOPIC, Float64, queue_size=10)
    pub_lmlj = rospy.Publisher(LMLJ_TOPIC, Float64, queue_size=10)
    pub_lflj = rospy.Publisher(LFLJ_TOPIC, Float64, queue_size=10)

    rate = rospy.Rate(50)
    
    # Speeds are in degrees/second. It is not recommended to go above 10,20
    grip_speeds = [2, 2, 5, 5, 5, 10]
    air_speeds = [5, 5, 10, 10, 10, 20]

    successPrinted = True

    while not rospy.is_shutdown():
        while reqd_vel != [0.0, 0.0, 0.0]:
            successPrinted = False
            while abs(reqd_vel[2]) >= 0.075:
                speedSelector = min(ceil(abs(reqd_vel[2])), 4)
                # speedSelector = 1
                
                left_tripod_angles, right_tripod_angles = generate_angle_lists(grip_speeds[speedSelector], air_speeds[speedSelector])
                
                # Rotate right
                if reqd_vel[2] < 0:
                    sign_modifiers = [-1,1,-1,1,-1,1]
                
                # Rotate left
                else:
                    sign_modifiers = [1,-1,1,-1,1,-1]
                
                for i in range(len(left_tripod_angles)):
                    # Left side tripod
                    pub_lhlj.publish(sign_modifiers[0]*left_tripod_angles[i]*pi/180)
                    pub_rmlj.publish(sign_modifiers[1]*left_tripod_angles[i]*pi/180)
                    pub_lflj.publish(sign_modifiers[2]*left_tripod_angles[i]*pi/180)
                    
                    # Right side tripod
                    pub_rhlj.publish(sign_modifiers[3]*right_tripod_angles[i]*pi/180)
                    pub_lmlj.publish(sign_modifiers[4]*right_tripod_angles[i]*pi/180)
                    pub_rflj.publish(sign_modifiers[5]*right_tripod_angles[i]*pi/180)

                    rate.sleep()
            
            speedSelector = min(ceil(abs(reqd_vel[0])), 5)
            # speedSelector = 1

            left_tripod_angles, right_tripod_angles = generate_angle_lists(grip_speeds[speedSelector], air_speeds[speedSelector]) 
            
            for i in range(len(left_tripod_angles)):
                # Left side tripod
                pub_lhlj.publish(-left_tripod_angles[i]*pi/180)
                pub_rmlj.publish(-left_tripod_angles[i]*pi/180)
                pub_lflj.publish(-left_tripod_angles[i]*pi/180)
                
                # Right side tripod
                pub_rhlj.publish(-right_tripod_angles[i]*pi/180)
                pub_lmlj.publish(-right_tripod_angles[i]*pi/180)
                pub_rflj.publish(-right_tripod_angles[i]*pi/180)

                rate.sleep()

        if not successPrinted:
            print(f"[INFO] [{time.time()}]: Automatic control: Goal reached!")
            successPrinted = True
        
        for i in range(25):
            # Left side tripod        
            pub_lhlj.publish(0*pi/180)
            pub_rmlj.publish(0*pi/180)
            pub_lflj.publish(0*pi/180)

            # Right side tripod
            pub_rhlj.publish(0*pi/180)
            pub_lmlj.publish(0*pi/180)
            pub_rflj.publish(0*pi/180)

            rate.sleep()
    

if __name__ == '__main__':
    control()
