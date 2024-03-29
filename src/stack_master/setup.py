from setuptools import setup

package_name = "stack_master"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/stack_launch.py"]),
    ],
    install_requires=["setuptools", "launch"],
    zip_safe=True,
    maintainer="alko",
    maintainer_email="koldy@seas.upenn.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
