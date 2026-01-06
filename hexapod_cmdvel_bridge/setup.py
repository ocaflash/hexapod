from setuptools import setup


package_name = "hexapod_cmdvel_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="christian",
    maintainer_email="stein.robotics@gmail.com",
    description="Bridge /cmd_vel and /joy to hexapod movement requests",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmdvel_bridge = hexapod_cmdvel_bridge.cmdvel_bridge_node:main",
        ],
    },
)
