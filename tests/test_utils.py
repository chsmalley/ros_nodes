import utils
import unittest
from unittest import TestCase
import os


class TestUtils(TestCase):  # {{{1

    def test_testJsonToWorld(self):  # {{{2
        dir_path = os.path.dirname(os.path.realpath(__file__))
        configfilename = os.path.join(dir_path, 'configs/test.json')
        worldstr = utils.testJsonToWorld(configfilename)
        f = open('test.world', 'w')
        f.write(worldstr)
        f.close()

    def test_testDictToWorld(self):  # {{{2
        dir_path = os.path.dirname(os.path.realpath(__file__))
        configfilename = os.path.join(dir_path, 'configs/test.cfg')
        worldstr = utils.testDictToWorld(configfilename)
        f = open('test.world', 'w')
        f.write(worldstr)
        f.close()

    def test_polygonsToPolarScan(self):  # {{{2
        pass

if __name__ == '__main__':
    unittest.main()
