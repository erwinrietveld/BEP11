#!/usr/bin/env python
import rospy
from numpy import sin, cos, pi

from geometry_msgs.msg import Pose
from barc.msg import laser_sensor
import tf



def laser_data_acquisition(data):
    global delta_x_L, delta_y_L,delta_x_R,delta_y_R
    data.XL =delta_x_L
    data.YL =delta_y_L
    data.XR = delta_x_R
    data.YR = delta_y_R

def position():
    global L_axis
    #init node
    rospy.init_node('position', anonymous=True)
    quaternion=[0,0,0,0]
    #define subscribtion and publishing of messages
    rospy.Subscriber('laser_sensor', laser_sensor, laser_data_acquisition)
    pose_pub = rospy.Publisher('pose', Pose, queue_size = 50)

    #init parameters
    L_axis = rospy.get_param("L_axis")
    x_center = rospy.get_param("x_center")
    y_center = rospy.get_param("y_center")

    smp_rate        = 50
	rate            = rospy.Rate(smp_rate)
    x = 0.0
    y = 0.0
    psi = 0.0
    yaw = 0.0
    current_time = rospy.get_time()
    last_time = rospy.get_time()

    while not rospy.is_shutdown():
        psi = (delta_y_L - delta_y_R)/L_axis
        yaw += psi
        yaw = yaw %(2*pi)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

        delta_x_car = delta_x_L
        delta_y_car = min(abs(delta_y_L),abs(delta_y_R))+abs(delta_y_R-delta_y_L)/2

        delta_x_base = delta_x_car*cos(yaw)+delta_y_car*sin(yaw)
        delta_y_base = delta_x_car*sin(yaw)+delta_y_car*cos(yaw)

        x+=delta_x_base
        y+=delta_y_base

        br = tf.TransformBroadcaster()
        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        pose_pub.publish(pose)
        rate.sleep()


if __name__=='__main__':
    try:
        position()
    except rospy.ROSInterruptException:
        pass
