#!/usr/bin/python
from collections import namedtuple
import sys
# import time
try:
    import ConfigParser
except ImportError:
    import configparser as ConfigParser
import numpy as np
# import random
import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# import tf
import models

# from robot-motion-planning import utils


class RosMappingNode(object):  # {{{1

    """
    Create a ROS node that implements a object detection algorithms
    inputs: All inputs should be from from sensors? (camera, lidar..)
    outputs: Outputs should be obstacles (type TBD)
    """
    Topic = namedtuple("Topic",
                       ["name",
                        "type",
                        "callback"])
#    ParameterVector = namedtuple("DiffDriveRosPid",
#                                 ["k_p",
#                                  "k_i",
#                                  "k_d"])

    def __init__(self, subscribe_topics, publish_topics, model, paramdescr=None):  # {{{2
        self.set_param(descr["policyparam"])
        # SET UP RATE FOR LOOP
        rate = rospy.Rate(10)  # TODO add to config
        # SET UP PUBLISHING
        self.pub = rospy.Publisher(publish_topic.name,
                                   publish_topic.type)
        # SET UP SUBSCRIBING
        # rospy.Subscriber("base_scan", LaserScan, self.laserScanCallback)
        for topic in subscribe_topics:
            rospy.Subscriber(topic.name,
                             topic.type,
                             eval("self." + topic.callback))
        # SET UP ALGORITHM FOR NODE
        # Set model used in policy
        # self.q = eval("models." + model + "((0.0, 0.0, 0.0), (0.25,))")
        while not rospy.is_shutdown():
            data = self.policy.step()
            # TODO May need to repackage data
            self.pub.publish(data)
            rate.sleep()

    # Callback functions {{{2
    def cameraCallback(self, data):  # {{{3
        pass

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

    def ObjectCallback(self, data):  # {{{3
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
    def publishMap(self, mapPub, data):  # {{{3
        print("name: ", pub.name)
        print("data_class: ", pub.data_class)
        pub.publish(data)

    def publishPose(self, posePub, data):  # {{{3
        pass

# Main function.
if __name__ == '__main__':  # {{{1
    """
    Mapping node:
        inputs:
            current map
            objects
            estimated orientation
        output:
            updated map
            updated orientation
    """
    # READ ARGS FOR HOW TO CONFIGURE POLICY
    configfilename = sys.argv[1]
    print("\n\nconfigfilename: ", configfilename)
    config = ConfigParser.ConfigParser()
    config.read(configfilename)
    descr = eval(config.get("Policy", "descr"))
    model = descr["model"]
    policyname = descr["policyname"]
    publish_topic = descr["publish_topic"]
    print("publish_topic: ", publish_topic)
    subscribe_topics = descr["subscribe_topics"]
    print("subscribe_topics: ", subscribe_topics)
    # Initialize the node and name it.
    rospy.init_node(policyname)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = RosDetectionNode(subscribe_topics, publish_topic, model, descr)
    except rospy.ROSInterruptException:
        pass

