#!/usr/bin/python
# from collections import namedtuple
import sys
# import time
import numpy as np
# import random
import rospy
from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# import tf
import models

# from robot-motion-planning import utils

def particle_filter(particles):  # {{{1
    # Input: particles
    # Output: updated particles
    # https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb

    # Predict next state of partilces
    # Update the weighting of the particles
    # Resample partilces
    # Compute Estimate
    pass

def kalman_filter():  # {{{1
    pass


class RosEstimationNode(object):  # {{{1

    """
    Create a ROS node that implements an estimation algorithms
    inputs: All inputs should be from from sensors? (camera, lidar..)
    outputs: Outputs should be an estimate of robot's pose (x, y, theta)
    """
#    ParameterVector = namedtuple("DiffDriveRosPid",
#                                 ["k_p",
#                                  "k_i",
#                                  "k_d"])

    def __init__(self, descr):  # {{{2
        # SET UP RATE FOR LOOP
        rate = rospy.Rate(10)  # TODO add to config
        # SET UP PUBLISHING
        self.pub = rospy.Publisher(descr["publishtopics"]["name"],
                                   descr["publishtopics"]["type"])
        # SET UP SUBSCRIBING
        # rospy.Subscriber("base_scan", LaserScan, self.laserScanCallback)
        for topic in descr["subscribetopics"]:
            rospy.Subscriber(topic["name"],
                             topic["type"],
                             eval("self." + topic["callback"]))
        # SET UP ALGORITHM FOR NODE
        descr["function"]
        while not rospy.is_shutdown():
            print("running")
            # if all(dataAvailable):
            #     data = eval()
            # TODO May need to repackage data
            # for pubobj in self.pubs:
            #     self.pub.publish(data)
            rate.sleep()

    # Callback functions {{{2
    def cameraCallback(self, data):  # {{{3
        pass

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
        print("laserscan: ", self.laserscan[0])

    # Publish functions {{{2
    def publishPose(self, pub, data):  # {{{3
        print("name: ", pub.name)
        print("data_class: ", pub.data_class)
        pub.publish(data)
