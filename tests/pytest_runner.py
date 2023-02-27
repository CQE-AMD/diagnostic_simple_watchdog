#!/usr/bin/env python3

import pathlib
import sys

import pytest
import rospy


def get_output_file():
    for arg in sys.argv:
        if arg.startswith("--gtest_output"):
            return arg.split("=xml:")[1]

    raise RuntimeError("No output file has been passed")


if __name__ == "__main__":
    output_file = get_output_file()
    test_module = pathlib.Path(rospy.get_param("test_module"))
    runner_path = pathlib.Path(__file__).parent
    module_path = runner_path.joinpath(test_module)

    sys.exit(pytest.main([str(module_path), f"--junitxml={output_file}"]))
