#!/usr/bin/env python3

from math import pi

import rospy
from nav_msgs.msg import Odometry
from pynput.keyboard import Listener
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


pose = [0, 0, 0] # X, Y, Theta


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


last_key = 'q'


def update_key_buffer(key):
    global last_key
    allowed_key_list = ('w','s','a','d','q')

    try:
        key_pressed = key.char
    except AttributeError:
        # Special key pressed, which is to be ignored.
        key_pressed = None
    except KeyboardInterrupt:
        return False
    
    if key_pressed != last_key and key_pressed in allowed_key_list:
        last_key = key_pressed


def key_controller():
    global last_key, pose
    
    rospy.init_node('bot_controller')
    rospy.Subscriber('/odom', Odometry, odom_callback)

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

    grip_start_angle = -60
    
    # Make sure that the end angle is not negative, otherwise the calculation steps will break
    grip_end_angle = 20

    # Also make sure that the mid angle does not end up being a float, otherwise it will get truncated
    grip_mid_angle = (grip_start_angle + grip_end_angle)//2

    gripped_step_size = 5
    start_to_mid_angle_list = [i+360 if i<0 else i for i in range(grip_start_angle, grip_mid_angle, gripped_step_size)]
    mid_to_end_angle_list = [i+360 if i<0 else i for i in range(grip_mid_angle, grip_end_angle, gripped_step_size)]

    if len(start_to_mid_angle_list) != len(mid_to_end_angle_list):
        print ("The gripper angle list lengths are not equal! Terminating...")
        raise SystemExit

    if grip_start_angle < 0:
        grip_start_angle = grip_start_angle + 360

    end_to_start_angle_list = [i+360 if i<0 else i for i in range(grip_end_angle, grip_start_angle, 5)]

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
    
    while not rospy.is_shutdown():
        # print("Current pose is:", pose)
        
        if last_key == 'q':
            for i in range(100):
                # Left side tripod        
                pub_lhlj.publish(0*pi/180)
                pub_rmlj.publish(0*pi/180)
                pub_lflj.publish(0*pi/180)

                # Right side tripod
                pub_rhlj.publish(0*pi/180)
                pub_lmlj.publish(0*pi/180)
                pub_rflj.publish(0*pi/180)
            
            continue

        # Forward
        elif last_key == 'w':
            sign_modifiers = [-1]*6
        
        # Backward
        elif last_key == 's':
            sign_modifiers = [1]*6
        
        # Left
        elif last_key == 'a':
            sign_modifiers = [1,-1,1,-1,1,-1]
        
        # Right
        elif last_key == 'd':
            sign_modifiers = [-1,1,-1,1,-1,1]
        
        else:
            continue
        
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


if __name__ == '__main__':
    try:
        print("Control scheme:\n\tw\n\na\tq\td\n\n\ts\n")
        print("Press or hold the keys to control the robot. 'q' brings the joints back to zero degrees. Use 'Ctrl+C' to exit.\n")
        print("Focus on the terminal window is not required and not even recommended. Make sure to exit the program before you start typing somewhere else.")
        
        listener = Listener(on_press = update_key_buffer)
        listener.start()
        
        key_controller()
        
        print("Program terminated.")
    
    except rospy.ROSInterruptException:
        pass
