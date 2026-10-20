"""
Packaging for both worlds, which is the whole point of the dependency split.

`pip install spatialdds-ros` gets a library that puts a robot on a SpatialDDS
bus and needs no ROS at all; `rosdep`/`colcon` get the ROS node, and take
their dependencies from `package.xml` instead. The two declarations do not
overlap and neither is a subset of the other:

    install_requires (here)  cyclonedds              -- the bus
    package.xml              rclpy, tf2_ros, ...     -- the ROS node

**rclpy is deliberately absent from `install_requires`**, and that is not an
omission to be tidied up later. It is the packaging statement of what
`test/test_no_ros_needed.py` enforces in the source: no module reachable from
`import spatialdds_ros` imports rclpy, so the library is importable, usable
and testable on a machine that has never had ROS installed. Only
`robot_node.py` needs a ROS 2 environment, and it imports rclpy inside its
entry point rather than at module scope. Listing rclpy here would make every
pip user install a ROS client library to publish a pose, and it is not on PyPI
in a form that would work anyway.

`cyclonedds` goes the other way. It was left out until 0.3.0 on the reasoning
that a ROS install gets it from the middleware's own packaging -- true, and
irrelevant to the pip path, where leaving it out means `pip install
spatialdds-ros` yields a package that cannot import. The ROS path does not
read this file's `install_requires`, so declaring it here costs that path
nothing and fixes the other one.
"""
from pathlib import Path

from setuptools import find_packages, setup

PACKAGE = "spatialdds_ros"
HERE = Path(__file__).parent
REPO_URL = "https://github.com/OpenArCloud/spatialdds-ros"

# The long description is the README, verbatim. `test/test_packaging.py`
# asserts that, and asserts the README carries no relative link and no empty
# table header row -- both of which render as damage on PyPI, which has no
# repository around it to resolve them against.
LONG_DESCRIPTION = (HERE / "README.md").read_text(encoding="utf-8")

setup(
    name="spatialdds-ros",
    version="0.3.0",
    # `spatialdds_idl18` ships alongside: the generated bindings annotate
    # nested members with their own fully-qualified names, so the package has
    # to be importable as `spatialdds_idl18` and cannot be nested inside this
    # one. See spatialdds_idl18/PROVENANCE.md.
    packages=find_packages(include=[PACKAGE, f"{PACKAGE}.*",
                                    "spatialdds_idl18", "spatialdds_idl18.*"]),
    # PROVENANCE.md travels with the bindings it describes. The README's table
    # points at it, and a reference that resolves only in the repository is
    # exactly the kind of prose that breaks once it ships.
    package_data={"spatialdds_idl18": ["PROVENANCE.md"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    install_requires=["cyclonedds>=0.10"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    author="Open AR Cloud",
    maintainer="James Jackson",
    maintainer_email="fiberhog@gmail.com",
    description="A ROS 2 robot as a SpatialDDS participant",
    long_description=LONG_DESCRIPTION,
    long_description_content_type="text/markdown",
    # setuptools 84 warns that `License ::` classifiers are deprecated in
    # favour of a PEP 639 SPDX expression. It is kept anyway, and the reason
    # is not inertia: `License-Expression` comes from a `[project]` table in
    # pyproject.toml, and this package's metadata has to stay in setup.py
    # because `ament_python` builds read it from there. A pyproject.toml
    # carrying only `[build-system]` would not produce the field, and one
    # carrying `[project]` would fight setup.py for ownership of every other
    # value here. So: one metadata source, the classifier kept because PyPI's
    # own license filter reads it, and a build warning that is understood
    # rather than ignored.
    license="MIT",
    license_files=["LICENSE"],
    url=REPO_URL,
    project_urls={
        "Source": REPO_URL,
        "Issues": f"{REPO_URL}/issues",
        "Changelog": f"{REPO_URL}/releases",
    },
    keywords=["spatialdds", "ros2", "dds", "robotics", "nav2",
              "digital-twin", "spatial-computing"],
    # The named ROS targets are Humble (Python 3.10) and Jazzy (3.12), and the
    # floor says so rather than claiming a reach nothing here is tested on.
    python_requires=">=3.10",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering",
        "Topic :: System :: Networking",
    ],
    entry_points={
        "console_scripts": [
            f"robot_node = {PACKAGE}.robot_node:main",
            f"kinematic_node = {PACKAGE}.standalone:main",
        ],
    },
)
