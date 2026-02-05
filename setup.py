import glob
from setuptools import find_packages, setup

package_name = "yolo26_ros2"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ("share/" + package_name + "/models", glob.glob("models/*")),
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Hart (Mike Likes Robots)",
    maintainer_email="mikelikesrobots@outlook.com",
    description="ROS 2 package for real-time object detection using YOLO26 models with USB camera integration",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["yolo26_ros2 = yolo26_ros2.yolo26_ros2:main"],
    },
)
