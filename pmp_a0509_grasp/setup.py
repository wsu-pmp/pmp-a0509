from glob import glob

from setuptools import find_packages, setup

package_name = "pmp_a0509_grasp"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zed_object_bridge = pmp_a0509_grasp.zed_object_bridge_node:main",
            "object_selection_manager = "
            "pmp_a0509_grasp.object_selection_manager_node:main",
        ],
    },
)
