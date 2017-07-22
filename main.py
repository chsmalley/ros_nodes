import utils
import sys
import os
import signal
import json
import time
import subprocess
import rospy
from detection_node import RosDetectionNode
# from estimation_node import RosEstimationNode
from multiprocessing import Process
# import estimators
# import os
# import signal


def launch_stage(descr):  # {{{1
    utils.dictToLaunch(datadict["descr"])  # Creates the launch file
    launchstr = "roslaunch " + datadict["descr"]["world"]["package"]
    launchstr += " " + datadict["descr"]["world"]["launchfilename"]
    launcher = subprocess.Popen(launchstr.split())
    return launcher

if __name__ == '__main__':
    # LOAD CONFIG FILE
    configfilename = sys.argv[1]
    with open(configfilename) as f:
        datadict = json.load(f)
    # GET WORLD FROM CONFIG FILE
    worldmap = utils.dictToWorld(datadict["descr"])

    # LAUNCH STAGE SIMULATOR
    launcher = Process(target=launch_stage, args=(datadict["descr"],))
    launcher.start()
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
            node.start()
    try:
        while True:
            time.sleep(1)
            print("running")
    except KeyboardInterrupt:
        # CLEAN SOME THINGS UP
        print("KeyboardInterrupt")
        # os.kill(os.getpgid(launcher.pid), signal.SIGTERM)
        launcher.terminate()
    launcher.terminate()
