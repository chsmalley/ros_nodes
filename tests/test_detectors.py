import utils
import unittest
from unittest import TestCase
import os
import numpy as np
from detection_node import Ransac
from ros_nodes.msg import line, Point2D
import json
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point


class TestDetectors(TestCase):  # {{{1

    def setUp(self):  # {{{2
        # LOAD DATA FILE
        dir_path = os.path.dirname(os.path.realpath(__file__))
        datafilename = os.path.join(dir_path, 'data/ranges.json')
        with open(datafilename) as f:
            ranges = json.load(f)["ranges"]

        angle_min = -3.14159274101
        phi = angle_min
        # angle_max = 3.14159274101
        angle_increment = 0.0175019092858
        range_max = 50.0
        self.laserscan = []
        self.points = []
        for i, d in enumerate(ranges):
            if d == range_max:
                continue
            phi = angle_min + angle_increment * i
            rho = d
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            # scan = Point2D(x=x, y=y)
            # scan = Point2D
            # scan.x = x
            # scan.y = y
            self.points.append((x, y))
            p = Point((x, y))
            self.laserscan.append(p)

    def tearDown(self):  # {{{2
        pass

    def test_Ransac(self):  # {{{2
        result = Ransac(self.laserscan, iterations=50, threshold=3, ratio=0.6)
        print("result: ", list(result.coords))
        x, y = zip(*self.points)
        plt.plot(x, y, color='b')
        linex, liney = zip(*list(result.coords))
        plt.plot(linex, liney, color='r')
        plt.show()


if __name__ == '__main__':  # {{{1
    unittest.main()
