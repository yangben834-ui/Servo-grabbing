from setuptools import setup

package_name = "h3001_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/h3001_bridge.yaml"]),
        ("share/" + package_name + "/launch", ["launch/h3001_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="qin",
    maintainer_email="2745146351@qq.com",
    description="H3001 机械臂 CAN ↔ ROS 2 桥接节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "h3001_bridge = h3001_bridge.h3001_bridge:main",
        ],
    },
)
