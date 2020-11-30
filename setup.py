"""Setup the ezrassor_topic_switch module.

This file and the package.xml file are required in ROS 2. Some metadata listed
below are duplicated in the package.xml. This metadata must be identical in
both files.

references:
  index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package
"""
import setuptools
import glob


setuptools.setup(
    name="ezrassor_topic_switch",
    version="2.0.0",
    description="Route topic messages on the EZRASSOR.",
    maintainer="EZRASSOR Team",
    maintainer_email="ez.rassor@gmail.com",
    license="MIT",
    keywords=["EZRASSOR", "ROS", "ISRU", "NASA", "Rover", "UCF", "Robotics"],
    classifiers=[
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Astronomy",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    packages=["ezrassor_topic_switch"],
    package_dir={"": "source"},
    install_requires=["setuptools"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_topic_switch"],
        ),
        (
            "share/ezrassor_topic_switch",
            ["package.xml"] + glob.glob("launch/*"),
        ),
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "topic_switch = ezrassor_topic_switch.__main__:main",
        ],
    },
)
