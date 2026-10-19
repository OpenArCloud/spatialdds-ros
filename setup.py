from setuptools import find_packages, setup

PACKAGE = "spatialdds_ros"

setup(
    name=PACKAGE,
    version="0.1.0",
    # `spatialdds_idl18` ships alongside: the generated bindings annotate
    # nested members with their own fully-qualified names, so the package has
    # to be importable as `spatialdds_idl18` and cannot be nested inside this
    # one. See spatialdds_idl18/PROVENANCE.md.
    packages=find_packages(include=[PACKAGE, f"{PACKAGE}.*",
                                    "spatialdds_idl18", "spatialdds_idl18.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    # cyclonedds is deliberately not listed. On a ROS install it comes from the
    # middleware's own packaging, and pinning it here would fight that; for the
    # pip path it is in requirements.txt. rclpy is never a pip dependency.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="James Jackson",
    maintainer_email="fiberhog@gmail.com",
    description="A ROS 2 robot as a SpatialDDS participant",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"robot_node = {PACKAGE}.robot_node:main",
        ],
    },
)
