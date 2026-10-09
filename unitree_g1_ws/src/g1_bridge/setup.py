from setuptools import setup

package_name = "g1_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "g1_merger = g1_bridge.merger:main",
            "arm_ctrl_json = g1_bridge.arm_ctrl_json:main",
        ],
    },
)
