#!/usr/bin/env python3

from math import cos, sin
import time

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_about_axis


poseChange = [0, 0, 0] # Delta X, Y, Theta

previous_time = [0, 0]
delta_t = [0.02, 0.02] # Time since last update of IMU, GPS

imuOrientation = 0

gps_data = [0, 0]
origin_lat_long=[49.9,8.9]
radius_earth=6378100


def imu_callback(msg):
    global imuOrientation, poseChange, delta_t, previous_time
    x  = msg.orientation.x
    y  = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w
    
    temp = euler_from_quaternion([x,y,z,w])[2]
    poseChange[2] = temp - imuOrientation
    imuOrientation = temp

    temp2 = rospy.Time.now()
    delta_t[0] = (temp2 - previous_time[0]).to_sec()


def gps_callback(msg):
    global gps_data
    # print("Latitiude and Longitude Data:",msg.latitude,msg.longitude)
    # Multiply by constants to convert GPS Lat-Long to x-y
    temp = [radius_earth*(msg.latitude-origin_lat_long[0])*0.017438916,radius_earth*(msg.longitude-origin_lat_long[1])*-0.011264224]
    poseChange[0] = temp[0] - gps_data[0]
    poseChange[1] = temp[1] - gps_data[1]
    
    gps_data = temp[:]
    # print(gps_data)

    temp2 = rospy.Time.now()
    delta_t[1] = (temp2 - previous_time[1]).to_sec()


def odom_publisher():
    global imuOrientation, gps_data, previous_time, delta_t
    
    rospy.init_node('odom_publisher', anonymous=True)

    previous_time = [rospy.Time.now() for _ in range(2)]
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.Subscriber('/fix', NavSatFix, gps_callback)

    pub_odom = rospy.Publisher("/odom", Odometry, queue_size=50)
    broadcaster_odom = TransformBroadcaster()
    rate = rospy.Rate(25)
    
    print(f"[INFO] [{time.time()}]: Odometry publisher node started.")
    while not rospy.is_shutdown():
        odom_quaternion = quaternion_about_axis(imuOrientation, (0, 0, 1))
        
        # Updates happen so fast for the IMU that the stored value becomes zero. This is done to prevent division errors.
        delta_t_copy = [delta_t[i] if delta_t[i] != 0 else 0.00001 for i in range(2)]

        # The updates for IMU and GPS do not happen at 50 Hz even though we have set it to be so in the gazebo file.
        # For this reason, separate time delta variables need to be maintained for both sensors so that
        # odometry calculations can be done without any issues.
        vx = poseChange[0]/delta_t_copy[1]*cos(imuOrientation) + poseChange[1]/delta_t_copy[1]*sin(imuOrientation)
        vy = poseChange[1]/delta_t_copy[1]*cos(imuOrientation) - poseChange[0]/delta_t_copy[1]*sin(imuOrientation)
        vth = imuOrientation/delta_t_copy[0]

        current_time = rospy.Time.now()

        # Broadcast transform to tf.
        broadcaster_odom.sendTransform((gps_data[0], gps_data[1], 0.), odom_quaternion, current_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(gps_data[0], gps_data[1], 0.), Quaternion(*odom_quaternion))
        
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        pub_odom.publish(odom)
        rate.sleep()


if __name__ == '__main__':
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass
