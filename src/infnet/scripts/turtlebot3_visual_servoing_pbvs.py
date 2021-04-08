#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy, time, sys, math, control_lib, tf
import numpy as np
from geometry_msgs.msg import Pose2D, Twist, PointStamped
from turtlesim.msg import Pose
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import image_geometry
import tf2_ros as tf2
import tf2_geometry_msgs


def callback_camera_info(data):
    global model
    global camera_matrix

    model.fromCameraInfo(data)

    K = np.array(data.K).reshape([3, 3])
    f = K[0][0]
    u0 = K[0][2]
    v0 = K[1][2]

    camera_matrix[0] = f
    camera_matrix[1] = u0
    camera_matrix[2] = v0


def callback_img_point(data):
    
    global camera_height
    global image_point
    global mask_is_true

    # recovering point
    u = data.x
    v = data.y
    base_points = [u, v]
    mask_is_true = data.theta
    distance = 0

    try:
        # finding distance to the point
        pixel_rectified = model.rectifyPoint(base_points)
        line = model.projectPixelTo3dRay(pixel_rectified)
        th = math.atan2(line[2], line[1])
        distance = math.tan(th) * camera_height

        image_point.x = u
        image_point.y = v
        image_point.theta = distance

    except:
        pass


def callback_odom(data):
    global robot_pose

    robot_pose.x = round(data.pose.pose.position.x, 3)
    robot_pose.y = round(data.pose.pose.position.y, 3)

    q = [data.pose.pose.orientation.x,
         data.pose.pose.orientation.y,
         data.pose.pose.orientation.z,
         data.pose.pose.orientation.w]

    euler = tf.transformations.euler_from_quaternion(q)

    theta = round(euler[2], 3)

    robot_pose.theta = theta

    # print('/*/*/**/*/\n' + str(robot_pose))


def callback_control_type(data):
    
    global ctrl_type

    ctrl_type = data.data


def control_robot():

    # Global variables
    global img_goal
    global image_point
    global robot_pose
    global gains_cart
    global ctrl_type
    global max_lin
    global max_ang
    global goal
    global camera_matrix
    global mask_is_true

    # Initializing ros node
    rospy.init_node('turtle_control', anonymous=True)  # node name

    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)

    # Subscribers
    rospy.Subscriber('img_point', Pose2D, callback_img_point)  # receives the goal coordinates
    rospy.Subscriber('odom', Odometry, callback_odom)  # receives thr robot odometry
    rospy.Subscriber('control_type', Int32, callback_control_type)  # receives c.t.
    rospy.Subscriber('camera_info', CameraInfo, callback_camera_info)  # receives the goal coordinates

    # Publishers
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # send control signals

    # control rate
    rate = rospy.Rate(30)  # run the node at 15H
    time.sleep(5)
    # main loop
    while not rospy.is_shutdown():

        control_signal = Twist()
        img_goal = control_lib.get_img_point(image_point, camera_matrix)

        goal_stamped = tfBuffer.transform(img_goal, "odom")
        try:
            if mask_is_true:
                control_signal = control_lib.cartesian_control(robot_pose,
                                                               goal_stamped.point,
                                                               0.25, 0.25,
                                                               threshold=0.35)
            else:
                #control_signal = Twist()
                control_signal.linear.x = 0.0
                control_signal.angular.z = 0.5
                rate.sleep()
        except:
            pass

        # print control_signal
        cmd_vel.publish(control_signal)

        #print('\rDistance to the target:', image_point.theta, end='\r')

        #


############ MAIN CODE #######################
# initializing Global variables
# Readin from launch
K_eu = float(sys.argv[1])  # Control gain for linear velocity
K_ev = float(sys.argv[2])  # Control gain for angular velocity
X_goal = float(sys.argv[3])
Y_goal = float(sys.argv[4])
max_lin = float(sys.argv[5])
max_ang = float(sys.argv[6])
ctrl_type = float(sys.argv[7])
camera_height = float(sys.argv[8])

# Inner values
robot_pose = Pose2D()
image_point = Pose2D()
gains_cart = [K_eu, K_ev]
img_goal = Pose2D()
img_goal.x = X_goal
img_goal.y = Y_goal
camera_matrix = np.zeros((3, 1))
vel_lim = [max_lin, max_ang]
mask_is_true = False

# creating a camera model
model = image_geometry.PinholeCameraModel()


if __name__ == '__main__':
    control_robot()
    '''
    try:
        control_robot()
    except:
        print('Node ended.')
    '''
