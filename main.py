import utils
import sys
import json
import time
import subprocess
import rospy
from detection_node import RosDetectionNode
# import estimators
# import os
# import signal


if __name__ == '__main__':
    # LOAD CONFIG FILE
    configfilename = sys.argv[1]
    with open(configfilename) as f:
        datadict = json.load(f)
    # GET WORLD FROM CONFIG FILE
    worldstr, worldmap = utils.dictToWorld(datadict["descr"])
    f = open(datadict["descr"]["world"]["worldfilename"], 'w')
    f.write(worldstr)
    f.close()

    # LAUNCH STAGE SIMULATOR
    launchfilestr = utils.dictToLaunch(datadict["descr"])
    f = open(datadict["descr"]["world"]["launchfilename"], 'w')
    f.write(launchfilestr)
    f.close()
    launchstr = "roslaunch " + datadict["descr"]["world"]["package"]
    launchstr += " " + datadict["descr"]["world"]["launchfilename"]
    launcher = subprocess.Popen(launchstr.split())
    time.sleep(2)  # Let stage start up

    # CREATE AGENTS
    for agent in datadict["descr"]["agents"]:
        print(agent)
        # CREATE ESTMATOR OBJECTS
        for estimator in agent['estimators']:
            estimatorname = estimator["name"]
            print("estimatorname: ", estimatorname)
            # Initialize the node and name it.
            rospy.init_node(estimatorname)
            try:
                ne = RosDetectionNode(estimator["subscribetopics"],
                                      estimator["publishtopic"])
            except rospy.ROSInterruptException:
                pass
        # CREATE OTHER ROS OBJECTS
    try:
        while True:
            time.sleep(1)
            print("running")
    except KeyboardInterrupt:
        # CLEAN SOME THINGS UP
        launcher.terminate()
    launcher.terminate()
