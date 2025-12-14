from setuptools import find_packages, setup


package_name = "amr_cobot_teleop"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/teleop_tcp.launch.py", "launch/demo.launch.py"]),
        ("share/" + package_name + "/config", ["config/teleop_tcp.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="Joystick teleop: /joy -> TwistStamped in tool0 for MoveIt Servo.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_tcp_node = amr_cobot_teleop.teleop_tcp_node:main",
            "ros2_control_description_node = amr_cobot_teleop.ros2_control_description_node:main",
            "joint_state_mux_node = amr_cobot_teleop.joint_state_mux_node:main",
            "joint_state_extras_node = amr_cobot_teleop.joint_state_extras_node:main",
            "servo_start_client_node = amr_cobot_teleop.servo_start_client_node:main",
        ],
    },
)


