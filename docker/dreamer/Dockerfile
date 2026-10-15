# DreamerV3 (JAX) training environment for AWSIM for WHILL.
# Built on top of the ROS 2 image because the gym environment talks to AWSIM over ROS 2.
ARG BASE_IMAGE=awsim-for-whill/ros2:latest
FROM ${BASE_IMAGE}

COPY rl/requirements.txt /tmp/requirements-rl.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements-rl.txt

ENV XLA_PYTHON_CLIENT_PREALLOCATE=false
WORKDIR /workspace
CMD ["bash"]
