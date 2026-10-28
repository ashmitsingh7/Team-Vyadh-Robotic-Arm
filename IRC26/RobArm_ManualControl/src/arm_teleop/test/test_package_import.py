"""Skeleton smoke test: the package imports without ROS being sourced."""
import importlib
import unittest


class TestPackageImport(unittest.TestCase):
    def test_import(self):
        self.assertIsNotNone(importlib.import_module('arm_teleop'))


if __name__ == '__main__':
    unittest.main()
