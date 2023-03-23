#! /usr/bin/python3.8

import numpy as np
import casadi as ca
import time

# ROS
import rospy
import tf

from px4_bridge.msg import ThrustRates
from nav_msgs.msg import Odometry

from quadrotor import QuadrotorModel, QuadrotorSim

# 
import os, sys
BASEPATH = os.path.abspath(__file__).split('script', 1)[0]
# sys.path += [BASEPATH]

#######################################################################################################
rospy.init_node("q_sim")
rospy.loginfo("ROS: Hello")

thrust_rates_msg =ThrustRates()
def thrust_rates_cb(msg:ThrustRates):
    global thrust_rates_msg
    thrust_rates_msg = msg
    pass

thrust_rates_sub = rospy.Subscriber("~thrust_rates", ThrustRates, callback=thrust_rates_cb, queue_size=1, tcp_nodelay=True)

quad = QuadrotorModel(BASEPATH+"config/quad_px4.yaml")
q_sim = QuadrotorSim(quad)

tf_br = tf.TransformBroadcaster(queue_size=1)
odom_pub = rospy.Publisher("~odom", Odometry, tcp_nodelay=True, queue_size=1)

cnt = 0
def sim_run(e):
    # rospy.loginfo("run")
    global thrust_rates_msg, cnt
    cnt += 1

    if thrust_rates_msg != None:

        u = np.zeros(4)
        u[0] = thrust_rates_msg.thrust*quad._T_max  
        u[1] = thrust_rates_msg.wx
        u[2] = thrust_rates_msg.wy
        u[3] = thrust_rates_msg.wz
        q_sim.step10ms(u)

        q_state = q_sim._X
        odom_msg = Odometry()
        odom_msg.header.frame_id="world"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = q_state[0]
        odom_msg.pose.pose.position.y = q_state[1]
        odom_msg.pose.pose.position.z = q_state[2]
        odom_msg.twist.twist.linear.x = q_state[3]
        odom_msg.twist.twist.linear.y = q_state[4]
        odom_msg.twist.twist.linear.z = q_state[5]
        odom_msg.pose.pose.orientation.w = q_state[6]
        odom_msg.pose.pose.orientation.x = q_state[7]
        odom_msg.pose.pose.orientation.y = q_state[8]
        odom_msg.pose.pose.orientation.z = q_state[9]
        odom_msg.twist.twist.angular.x = q_state[10]
        odom_msg.twist.twist.angular.y = q_state[11]
        odom_msg.twist.twist.angular.z = q_state[12]
        odom_pub.publish(odom_msg)

        if cnt%3 == 0:
            tf_br.sendTransform((q_state[1],q_state[0],-q_state[2]), (q_state[8],q_state[7],-q_state[9],q_state[6]), rospy.Time.now(), "quad_body", "world")
    pass
    

timer = rospy.Timer(rospy.Duration(0.01), sim_run, oneshot=False, reset=False)

rospy.spin()
rospy.loginfo("ROS: Goodby")
