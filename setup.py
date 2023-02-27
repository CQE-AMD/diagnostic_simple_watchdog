# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

import pathlib
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=["scripts/topic_rate_watchdog"],
    packages=[pathlib.Path(__file__).parent.name],
    package_dir={"": "src"},
)

setup(**setup_args)
