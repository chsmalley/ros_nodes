import utils
import os
import sys
import json
import rospy
import estimators


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
    launchstr = utils.dictToLaunch(datadict["descr"])
    f = open(datadict["descr"]["world"]["launchfilename"], 'w')
    f.write(launchstr)
    f.close()
    os.system("roslaunch "
              + datadict["descr"]["world"]["package"] + " "
              + datadict["descr"]["world"]["launchfilename"])

    # CREATE AGENTS
    for agent in datadict["descr"]["agents"]:
        print(agent)
        # CREATE ESTMATOR OBJECTS
        # for estimator in agent['estimators']:
        #     estimatorname = estimator["name"]
        #     # Initialize the node and name it.
        #     rospy.init_node(estimatorname)
        #     try:
        #         ne = RosEstimationNode(estimator)
        #     except rospy.ROSInterruptException:
        #         pass
        # CREATE OTHER ROS OBJECTS
