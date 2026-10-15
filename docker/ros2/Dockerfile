# ROS 2 Humble tools for AWSIM for WHILL:
# ros2 CLI, rosbag2, CycloneDDS, autoware_vehicle_msgs (gear command),
# the ROS 2 examples and the vehicle parameter optimizer.
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-autoware-vehicle-msgs \
        ros-humble-cv-bridge \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        ros-humble-rosbag2-storage-default-plugins \
        python3-pip \
        python3-opencv \
    && rm -rf /var/lib/apt/lists/*

COPY optimizer/requirements.txt /tmp/requirements-optimizer.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements-optimizer.txt

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1
WORKDIR /workspace
CMD ["bash"]
