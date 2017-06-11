import utils
import unittest
from unittest import TestCase
import os


class TestUtils(TestCase):

    def test_testJsonToWorld(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        configfilename = os.path.join(dir_path, 'configs/test.json')
        worldstr = utils.testJsonToWorld(configfilename)
        f = open('test.world', 'w')
        f.write(worldstr)
        f.close()

    def test_testDictToWorld(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        configfilename = os.path.join(dir_path, 'configs/test.cfg')
        worldstr = utils.testDictToWorld(configfilename)
        f = open('test.world', 'w')
        f.write(worldstr)
        f.close()


if __name__ == '__main__':
    unittest.main()
