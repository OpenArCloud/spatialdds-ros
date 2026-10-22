# The two-domain acceptance image: ROS 2 Humble, nav2's planner and costmap,
# and this library's DDS binding beside ROS's own.
#
# Test tooling, not a deployment. The library itself is mounted at run time
# (`-v $PWD:/work`; not `/lib`, which is the system library directory), so the image is the environment and the working tree is
# what is tested; a stale image cannot be testing stale code.
#
# Built from the same recipe as SpatialDDS-demo's robot tier, line for line up
# to the binding, which is the proven arrangement: ROS Humble's middleware is
# CycloneDDS 0.10.5 while the Python binding is 11.0.1, built from source
# because there is no aarch64 wheel, and the two load separate libddsc builds
# and share no symbols.
#
#     docker build -t spatialdds-ros-nav2 -f test/ros/Dockerfile .
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-nav2-bringup \
      ros-humble-nav2-costmap-2d \
      ros-humble-nav2-navfn-planner \
      ros-humble-nav2-controller \
      ros-humble-nav2-lifecycle-manager \
      ros-humble-nav2-map-server \
      ros-humble-nav2-msgs \
      ros-humble-tf2-ros \
      ros-humble-rmw-cyclonedds-cpp \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends       build-essential cmake git libssl-dev pkg-config     && rm -rf /var/lib/apt/lists/*
WORKDIR /opt
RUN git clone --depth 1 --branch 11.0.1 https://github.com/eclipse-cyclonedds/cyclonedds.git
WORKDIR /opt/cyclonedds
RUN mkdir build && cd build &&     cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON              -DENABLE_SECURITY=OFF -DBUILD_DDSPERF=OFF -DBUILD_IDLC=ON              -DBUILD_TOOLS=ON &&     make -j$(nproc) && make install
RUN ldconfig
ENV CYCLONEDDS_HOME=/usr/local
ENV PATH=/usr/local/bin:$PATH
# certifi before anything else is fetched: GHSA-6q48-6c7w-9gw2.
RUN python3 -m pip install --no-cache-dir -U pip certifi
RUN python3 -m pip install --no-cache-dir cyclonedds==11.0.1

# What the recorder needs, from this library's own requirements.
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

ENV PYTHONPATH=/work
WORKDIR /work
