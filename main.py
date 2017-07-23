import utils
import sys
import json
import time
import subprocess
import rospy
from detection_node import RosDetectionNode
# from estimation_node import RosEstimationNode
# import estimators


def launch_stage(descr):  # {{{1
    utils.dictToLaunch(descr)  # Creates the launch file
    launchstr = "roslaunch " + descr["world"]["package"]
    launchstr += " " + descr["world"]["launchfilename"]
    launcher = subprocess.Popen(launchstr.split())
    return launcher


if __name__ == '__main__':  # {{{1
    # LOAD CONFIG FILE
    configfilename = sys.argv[1]
    with open(configfilename) as f:
        datadict = json.load(f)
    # GET WORLD FROM CONFIG FILE
    worldmap = utils.dictToWorld(datadict["descr"])

    # LAUNCH STAGE SIMULATOR
    launcher = launch_stage(datadict["descr"])
    time.sleep(2)  # Let stage start up

    # CREATE AGENTS
    for agent in datadict["descr"]["agents"]:
        print(agent)
        # CREATE DETECTION OBJECTS
        ros_nodes = []
        for detector in agent['detectors']:
            detectorname = detector["name"]
            # Initialize the node and name it.
            rospy.init_node(detectorname)
            ros_nodes.append(RosDetectionNode(detector))
        # # CREATE ESTMATOR OBJECTS
        # estimators = []
        # for estimator in agent['estimators']:
        #     estimatorname = estimator["name"]
        #     print("estimatorname: ", estimatorname)
        #     # Initialize the node and name it.
        #     rospy.init_node(estimatorname)
        #     ros_nodes.append(RosEstimationNode(estimator))
        # START ROS OBJECTS
        for node in ros_nodes:
            node.daemon = True
            node.start()
    while not rospy.is_shutdown():
        time.sleep(1)
        print("running main")
