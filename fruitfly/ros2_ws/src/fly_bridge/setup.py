from glob import glob

from setuptools import setup

package_name = "fly_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/data", glob("data/*.csv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fly Bridge maintainers",
    maintainer_email="todo@example.com",
    description="Couple the flyscreen fruit-fly brain/body to the EUFS car.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fly_bridge = fly_bridge.node:main",
        ],
    },
)
