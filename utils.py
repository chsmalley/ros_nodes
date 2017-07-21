import numpy as np
from scipy.misc import imsave
import json
# import sys
import Image
from shapely.geometry import LineString, Point, Polygon
from shapely.affinity import rotate


# MapToPolarScan  {{{1
def polygonsToPolarScan(polygons,  # List(List(float))
                        pose,  # List(float)
                        angle_min,  # float
                        angle_max,  # float
                        angle_increment,  # float
                        range_max):  # float
    polarScan = []
    for i in range(angle_min, angle_max, angle_increment):
        # CREATE A LINE from the laser scanner to its max range
        x = range_max * np.cos(i)
        y = range_max * np.sin(i)
        end_point = Point(x, y)
        start_point = Point(pose['x'], pose['y'])
        line = LineString([start_point, end_point])
        # CHECK IF THAT LINE INTERSECTS WITH ANY OBJECTS
        intersections = []
        # OBJECTS IS A LIST OF SHAPELY POLYGONS
        for obj in polygons:
            tmp = line.intersects(obj)
            intersections.append(tmp)
        # SAVE THE DISTANCE TO THE CLOSEST INTERSECTING POINT
        if intersections:
            distances = [start_point.distance(p) for p in intersections]
            polarScan.append(min(distances))
        else:
            polarScan.append(range_max)
    return polarScan


def createEmptyWorldImage2(n, m, name="world.png"):  # {{{1
    print("n, m: ", n, m)
    # image = np.zeros([n, m, 3], dtype=np.uint8)
    m = m * 10
    n = n * 10
    size = (n, m)
    image = Image.new("RGB", size)
    pixels = image.load()
    for i in range(1, n - 1):
        for j in range(1, m - 1):
            pixels[i, j] = (255, 255, 255)
    image.save(name)
    # image = np.zeros([m, n, 3], dtype=np.uint8)
    # image[:, :, :] = 255
    # image[0, 0:n - 1, :] = 0
    # image[0:m - 1, 1, 0] = 0
    # image[0:m - 1, n - 1, 0] = 0
    # image[m - 1, 0:n - 1, 0] = 0
    # imsave(name, image)


def createEmptyWorldImage(n, m, name="world.png"):  # {{{1
    print("n, m: ", n, m)
    # m = m * 10
    # n = n * 10
    # image = np.zeros([n, m, 3], dtype=np.uint8)
    image = np.zeros([m, n, 3], dtype=np.uint8)
    image[:, :, :] = 255
    image[0, 0:n - 1, :] = 0
    image[0:m - 1, 0, :] = 0
    image[0:m - 1, n - 1, :] = 0
    image[m - 1, 0:n - 1, :] = 0
    imsave(name, image)


def createEmptyWorldImageNoBoarder(n, m, name="world.png"):  # {{{1
    image = np.zeros([m, n, 3], dtype=np.uint8)
    image[:, :, :] = 255
    imsave(name, image)


# def createWorldLayout  {{{1
def createWorldLayout(layoutName="floorplan",
                      color="gray30",
                      boundary="1",
                      gui_nose="0",
                      gui_grid="0",
                      gui_outline="0",
                      gripper_return="0",
                      fiducial_return="0",
                      laser_return="1"):
    """
    EXAMPLE
    define floorplan model
    (
      color "gray30"
      # most maps will need a bounding box
      boundary 1
      gui_nose 0
      gui_grid 0
      gui_outline 0
      gripper_return 0
      fiducial_return 0
      laser_return 1
    )
    """
    layout = '\ndefine {} model\n'.format(layoutName)
    layout += '(\n'
    layout += '\tcolor \"{}\"\n'.format(color)
    layout += '\tboundary {}\n'.format(boundary)
    layout += '\tgui_nose {}\n'.format(gui_nose)
    layout += '\tgui_grid {}\n'.format(gui_grid)
    layout += '\tgui_outline {}\n'.format(gui_outline)
    layout += '\tgripper_return {}\n'.format(gripper_return)
    layout += '\tfiducial_return {}\n'.format(fiducial_return)
    layout += '\tlaser_return {}\n'.format(laser_return)
    layout += ')\n'
    layout += '\nresolution 0.02\n'
    layout += 'interval_sim 100\n'  # simulation timestep in milliseconds
    return layout


def placeWorldLayout(layoutName, bitmap, size, worldOrientationQuat):  # {{{1
    """
    floorplan
    (
      name "empty"
      bitmap "empty.png"
      size [ 10.0 10.0 2.0 ]
      pose [ 10.0 5.0 0.0 0.0 ]
    )
    """
    # layout = '\n{}\n'.format(layoutName)
    layout = '\nfloorplan\n'
    layout += '(\n'
    layout += '\tname \"{}\"\n'.format(layoutName)
    layout += '\tbitmap \"{}\"\n'.format(bitmap)
    layout += '\tsize [{} {} {}]\n'.format(size[0], size[1], size[2])
    layout += '\tpose [{} {} {} {}]\n'.format(worldOrientationQuat[0],
                                              worldOrientationQuat[1],
                                              worldOrientationQuat[2],
                                              worldOrientationQuat[3])
    layout += ')\n'
    return layout


# def createSensor {{{1
def createSensor(sensorName,
                 sensorType,
                 rangeMin=0,
                 rangeMax=50,
                 fov=360, samples=360,
                 sizeX=1,
                 sizeY=1,
                 sizeZ=1,
                 color="black"):
    # EXAMPLE:
    # define topurg ranger
    # (
    #   sensor(
    #     range [ 0.0  50.0 ]
    #     fov 360
    #     samples 360
    #   )
    #   # generic model properties
    #   color "black"
    #   size [ 0.05 0.05 0.1 ]
    # )
    sensor = '\ndefine {} {}\n'.format(sensorName, sensorType)
    sensor += '(\n'
    sensor += '\tsensor(\n'
    sensor += '\t\trange [{} {}]\n'.format(rangeMin, rangeMax)
    sensor += '\t\tfov {}\n'.format(fov)
    sensor += '\t\tsamples {}\n'.format(samples)
    sensor += '\t)\n'
    sensor += '\tsize [{} {} {}]\n'.format(sizeX, sizeY, sizeZ)
    sensor += '\tcolor \"{}\"\n'.format(color)
    sensor += ')\n'
    return sensor


def createObject(objectName, objSize):  # {{{1
    # define block model
    # (
    #   size [0.5 0.5 0.5]
    #   gui_nose 0
    #   gripper_return 0
    #   fiducial_return 0
    #   laser_return 1
    # )
    model = '\ndefine {} model\n'.format(objectName)
    model += '(\n'
    model += '\tsize [{} {} {}]\n'.format(objSize, objSize, 2)
    model += '\tgui_nose 0\n'
    model += '\tgripper_return 0\n'
    model += '\tfiducial_return 0\n'
    model += '\tlaser_return 1\n'
    model += ')\n'
    return model


def createModel(modelName, size, origin, drive, sensors):  # {{{1
    """
    define bot position
    (
      #size [0.415 0.392 0.25]
      size [0.35 0.35 0.25]
      origin [-0.05 0 0 0]
      gui_nose 1
      drive "diff"
      topurg(pose [ 0.050 0.000 0 0.000 ])
    )
    """
    model = '\ndefine {} position\n'.format(modelName)
    model += '(\n'
    model += '\tsize [{} {} {}]\n'.format(size, size, size)
    model += '\torigin [{} {} 0 {}]\n'.format(origin[0],
                                              origin[1],
                                              origin[2])
    model += '\tgui_nose 1\n'
    model += '\tdrive \"{}\"\n'.format(drive)
    for sensor in sensors:
        model += placeSensor(sensor)
    model += ')\n'
    return model


def placeSensor(sensor):  # {{{1
    # topurg(pose [ 0.050 0.000 0 0.000 ])
    sensor = '\t{} ( pose [ {} {} {} {} ])\n'.format(sensor['name'],
                                                     sensor['x'],
                                                     sensor['y'],
                                                     sensor['z'],
                                                     sensor['heading'])
    return sensor


def placeModel(objectName, pose, name, color="black"):  # {{{1
    # modelName(pose [x, y, heading]
    # block(pose [ 0.0 0.0 0.000 ] name "obj1" color "black")
    model = '\n{} ( pose [ {} {} 0 {} ] '.format(objectName,
                                                 pose[0],
                                                 pose[1],
                                                 pose[2])
    model += 'name \"{}\" color \"{}\")\n'.format(name, color)
    return model


def dictToWorld(datadict):  # {{{1
    """
    inputs: dictionary with description file
    output: string in stage world format
            and a dictionary of the map objects
    """
    # LOAD CONFIG FILE
    # print("datadict: ", datadict)
    worldmap = {}
    # CREATE WORLD FILE FROM CONFIG
    worldstr = ""
    verts = [q for q in datadict["world"]["vertices"]]
    worldmap['vertices'] = verts
    x_min = min([p[0] for p in verts])
    x_max = max([p[0] for p in verts])
    y_min = min([p[1] for p in verts])
    y_max = max([p[1] for p in verts])
    # CREATE SENSORS
    worldstr += createSensor("scanner", "ranger")
    # CREATE ROBOT MODELS
    sensors = [{'name': 'scanner',
                'x': 0.05,
                'y': 0.0,
                'z': 0.0,
                'heading': 0.0}]
    for agent in datadict["agents"]:
        size = agent["modelparam"][0]
        worldstr += createModel(agent["class"],
                                size,
                                agent["q_start"],
                                agent["model"],
                                sensors)
        # PLACE MODELS
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        worldstr += placeModel(agent["class"],
                               agent["q_start"],
                               agent["name"],
                               color="blue")
    # CREATE WORLD LAYOUT
    worldstr += createWorldLayout()
    # worldstr += createWorldLayout(layoutName=descr["world"]["class"],
    #                               color="gray30",
    #                               boundary="1",
    #                               gui_nose="0",
    #                               gui_grid="0",
    #                               gui_outline="0",
    #                               gripper_return="0",
    #                               fiducial_return="0",
    #                               laser_return="1")
    # PLACE LAYOUT
    n = x_max - x_min
    m = y_max - y_min
    imageName = datadict["world"]["class"] + '.png'
    createEmptyWorldImageNoBoarder(n, m, name=imageName)
    # createEmptyWorld(n, m, name=imageName)
    worldOrientationQuat = (0, 0, 0, 0)
    worldSize = (n, m, 2)  # x = n, y = m, z = 2
    worldstr += placeWorldLayout(datadict["world"]["class"],
                                 imageName,
                                 worldSize,
                                 worldOrientationQuat)
    # CREATE OBJECTS
    obsDescr = datadict["world"]["obstacles"]
    worldmap['obstacles'] = {}
    # PLACE OBJECTS
    for objectName, n in zip(obsDescr["Random"]["class"],
                             obsDescr["Random"]["n"]):
        worldmap['obstacles'][objectName] = []
        objSize = obsDescr["Random"]["size"]
        worldstr += createObject(objectName, objSize)
        for i in range(n):
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            heading = 0
            halfsize = size / 2
            polygon = Polygon([(x + halfsize, y + halfsize),
                               (x - halfsize, y + halfsize),
                               (x - halfsize, y - halfsize),
                               (x + halfsize, y - halfsize)])
            polygon = rotate(polygon, heading)
            pose = (x, y, heading)
            # obj = Polygon()
            worldmap['obstacles'][objectName].append(polygon)
            worldstr += placeModel(objectName, pose, i, color="black")

    # resolution 0.02
    # interval_sim 100  # simulation timestep in milliseconds
    return worldstr, worldmap


def dictToLaunch(datadict):  # {{{1
    """
    <?xml version="1.0" encoding="UTF-8"?>
    <launch>
      <param name="/use_sim_time" value="true"/>
      <node pkg="stage_ros" type="stageros" name="stageros" args="$(find ros_nodes)/stage/test.world" respawn="false">
        <param name="base_watchdog_timeout" value="0.2"/>
      </node>
    </launch>
    """
    packagename = datadict['world']['package']
    worldfilename = datadict['world']['worldfilename']
    launchstr = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    launchstr += "<launch>\n"
    launchstr += "  <param name=\"/use_sim_time\" value=\"true\"/>\n"
    launchstr += "  <node pkg=\"stage_ros\" type=\"stageros\" name=\"stageros\""
    launchstr += " args=\"$(find {})/{}\" respawn=\"false\">".format(packagename, worldfilename)
    launchstr += "  <param name=\"base_watchdog_timeout\" value=\"0.2\"/>\n"
    launchstr += "  </node>\n"
    launchstr += "</launch>\n"
    return launchstr


def testJsonToWorld(configfilename):  # {{{1
    # LOAD CONFIG FILE
    with open(configfilename) as f:
        descr = json.load(f)['descr']
    # CREATE WORLD FILE FROM CONFIG
    worldstr = ""
    verts = [q for q in descr["world"]["vertices"]]
    x_min = min([p[0] for p in verts])
    x_max = max([p[0] for p in verts])
    y_min = min([p[1] for p in verts])
    y_max = max([p[1] for p in verts])
    # CREATE SENSORS
    worldstr += createSensor("scanner", "ranger")
    # CREATE ROBOT MODELS
    agentDescr = descr["agents"]
    sensors = [{'name': 'scanner',
                'x': 0.05,
                'y': 0.0,
                'z': 0.0,
                'heading': 0.0}]
    size = agentDescr["Agent0"]["modelparam"][0]
    worldstr += createModel(agentDescr["Agent0"]["class"],
                            size,
                            agentDescr["Agent0"]["q_start"],
                            'diff',
                            sensors)
    # CREATE WORLD LAYOUT
    worldstr += createWorldLayout()
    # worldstr += createWorldLayout(layoutName=descr["world"]["class"],
    #                               color="gray30",
    #                               boundary="1",
    #                               gui_nose="0",
    #                               gui_grid="0",
    #                               gui_outline="0",
    #                               gripper_return="0",
    #                               fiducial_return="0",
    #                               laser_return="1")
    # PLACE LAYOUT
    n = x_max - x_min
    m = y_max - y_min
    imageName = descr["world"]["class"] + '.png'
    createEmptyWorldImage(n, m, name=imageName)
    worldOrientationQuat = (0, 0, 0, 0)
    worldSize = (n, m, 2)  # x = n, y = m, z = 2
    worldstr += placeWorldLayout(descr["world"]["class"],
                                 imageName,
                                 worldSize,
                                 worldOrientationQuat)
    # CREATE OBJECTS
    obsDescr = descr["world"]["obstacles"]
    # PLACE OBJECTS
    for objectName, n in zip(obsDescr["Random"]["class"],
                             obsDescr["Random"]["n"]):
        objSize = obsDescr["Random"]["paramDistribution"][0]
        worldstr += createObject(objectName, objSize)
        for i in range(n):
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            heading = 0
            pose = (x, y, heading)
            worldstr += placeModel(objectName, pose, i, color="black")
    # PLACE MODELS
    agentDescr["Agent0"]
    x = np.random.uniform(x_min, x_max)
    y = np.random.uniform(y_min, y_max)
    initPose = agentDescr["Agent0"]["q_start"]
    worldstr += placeModel(agentDescr["Agent0"]["class"], initPose, "Agent0", color="blue")

    # resolution 0.02
    # interval_sim 100  # simulation timestep in milliseconds
    return worldstr


def testDictToWorld(configfilename):  # {{{1
    # LOAD CONFIG FILE
    try:
        import ConfigParser
    except ImportError:
        import configparser as ConfigParser
    config = ConfigParser.ConfigParser()
    config.read(configfilename)
    descr = eval(config.get("Experiment", "descr"))
    # CREATE WORLD FILE FROM CONFIG
    worldstr = ""
    verts = [q for q in descr["world"]["vertices"]]
    x_min = min([p[0] for p in verts])
    x_max = max([p[0] for p in verts])
    y_min = min([p[1] for p in verts])
    y_max = max([p[1] for p in verts])
    # CREATE SENSORS
    worldstr += createSensor("scanner", "ranger")
    # CREATE ROBOT MODELS
    agentDescr = descr["world"]["agents"]
    sensors = [{'name': 'scanner',
                'x': 0.05,
                'y': 0.0,
                'z': 0.0,
                'heading': 0.0}]
    size = agentDescr["Agent0"]["modelparam"][0]
    worldstr += createModel(agentDescr["Agent0"]["class"],
                            size,
                            agentDescr["Agent0"]["q_start"],
                            'diff',
                            sensors)
    # CREATE WORLD LAYOUT
    worldstr += createWorldLayout()
    # worldstr += createWorldLayout(layoutName=descr["world"]["class"],
    #                               color="gray30",
    #                               boundary="1",
    #                               gui_nose="0",
    #                               gui_grid="0",
    #                               gui_outline="0",
    #                               gripper_return="0",
    #                               fiducial_return="0",
    #                               laser_return="1")
    # PLACE LAYOUT
    n = x_max - x_min
    m = y_max - y_min
    imageName = descr["world"]["class"] + '.png'
    createEmptyWorldImage(n, m, name=imageName)
    worldOrientationQuat = (0, 0, 0, 0)
    worldSize = (n, m, 2)  # x = n, y = m, z = 2
    worldstr += placeWorldLayout(descr["world"]["class"],
                                 imageName,
                                 worldSize,
                                 worldOrientationQuat)
    # CREATE OBJECTS
    obsDescr = descr["world"]["obstacles"]
    # PLACE OBJECTS
    for objectName, n in zip(obsDescr["Random"]["class"],
                             obsDescr["Random"]["n"]):
        objSize = obsDescr["Random"]["paramDistribution"][0]
        worldstr += createObject(objectName, objSize)
        for i in range(n):
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            heading = 0
            pose = (x, y, heading)
            worldstr += placeModel(objectName, pose, i, color="black")
    # PLACE MODELS
    agentDescr["Agent0"]
    x = np.random.uniform(x_min, x_max)
    y = np.random.uniform(y_min, y_max)
    initPose = agentDescr["Agent0"]["q_start"]
    worldstr += placeModel(agentDescr["Agent0"]["class"], initPose, "Agent0", color="black")

    # resolution 0.02
    # interval_sim 100  # simulation timestep in milliseconds
    return worldstr
