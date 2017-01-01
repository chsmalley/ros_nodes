#!/usr/bin/python
from collections import namedtuple
import sys
import time
try:
    import ConfigParser
except ImportError:
    import configparser as ConfigParser
import numpy as np
import random
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf
import models

import utils

class RosPolicyNode(object):  # {{{1

    """
    Create a ROS node that implements a policy
    """
    ParameterVector = namedtuple("DiffDriveRosPid",
                                 ["k_p",
                                  "k_i",
                                  "k_d"])

    def __init__(self, subscribe_topics, publish_topics, model, paramdescr=None):  # {{{2
        print "descr: ", descr
        self.set_param(descr["policyparam"])
        # Set up rate for loop
        rate = rospy.Rate(10)  # TODO add to config
        # Set up publishing
        for topic in publish_topics:
            # topic (name, message type)
            self.pub = rospy.Publisher(topic[0], topic[1])
        # Set up subscribing
        for topic in subscribe_topics:
            if topic == "base_scan":
                rospy.Subscriber("base_scan", LaserScan, self.laserScanCallback)
            elif topic == "odom":
                rospy.Subscriber("odom", Odometry, self.odomCallback)
        # self.policy = policy
        # Set model used in policy
        # self.q_start = (0.0, 0.0, 0.0)
        # while self.q is None:
        #     print "waiting to subscribe to odom"
        #     time.sleep(0.2)
        self.q = eval("models." + model + "((0.0, 0.0, 0.0), (0.25,))")
        # self.q = model
        while not rospy.is_shutdown():
            twist = pid(self.q, self.param)
            self.pub.publish(twist)
            rate.sleep()

    def set_param(self, paramdescr):  # {{{2
        try:
            self.param = self.ParameterVector(**paramdescr)
        except TypeError:
            self.param = None
        except AttributeError:
            self.param = None

    # Callback functions {{{2
    def odomCallback(self, data):  # {{{3
        # Get quaternion orientation from pose data
        quaternion = (data.pose.pose.orientation.x,
                      data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z,
                      data.pose.pose.orientation.w)
        # Convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # Yaw is the only Euler angle currently used
        self.q = (data.pose.pose.position.x,
                  data.pose.pose.position.y,
                  euler[2])
        print "q: ", self.q

    def laserScanCallback(self, data):  # {{{3
        angleMin = data.angle_min
        inc = data.angle_increment
        phi = angleMin
        self.laserscan = []
        for i, d in enumerate(data.ranges):
            if d == data.range_max:
                continue
            phi = angleMin + inc * i
            rho = d
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            scan = models.Point2D()
            scan.set_q((x, y))
            self.laserscan.append(scan)
        print "laserscan: ", self.laserscan[0]

    def step(self, t, model):  # {{{2
        pass

# Main function.
if __name__ == '__main__':  # {{{1
    # READ ARGS FOR HOW TO CONFIGURE POLICY
    configfilename = sys.argv[1]
    print "\n\nconfigfilename: ", configfilename
    config = ConfigParser.ConfigParser()
    config.read(configfilename)
    descr = eval(config.get("Policy", "descr"))
    model = descr["model"]
    policyname = descr["policyname"]
    publish_topics = descr["publish_topics"]
    print "publish_topics: ", publish_topics
    subscribe_topics = descr["subscribe_topics"]
    print "subscribe_topics: ", subscribe_topics
    # Initialize the node and name it.
    rospy.init_node(policyname)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = RosPolicyNode(subscribe_topics, publish_topics, model, descr)
    except rospy.ROSInterruptException:
        pass
