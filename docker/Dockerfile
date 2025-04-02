# Use an official ROS 2 base image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install Python 3.10.12
RUN  apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y --no-install-recommends \
    python3.10 \
    python3.10-venv \
    python3.10-dev \
    python3.10-distutils \
    && rm -rf /var/lib/apt/lists/*

# Update alternatives to set Python 3.10 as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 && \
    update-alternatives --config python3

# Verify Python version
RUN python3 --version



# Set environment variables for Python
ENV PYTHON_VERSION=3.10.12

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*



# Bootstrap rosdep
# Initialize and update rosdep (only if not already initialized)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update --rosdistro humble


# Add colcon mixin and metadata only if they are not already added
RUN if ! colcon mixin list | grep -q 'default'; then \
        colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml; \
    fi && \
    colcon mixin update && \
    if ! colcon metadata list | grep -q 'default'; then \
        colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml; \
    fi && \
    colcon metadata update


# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /ros2_ws

# Ensure the src directory exists before cloning
RUN git clone https://github.com/tui-abdul/3d_lidar_camera_calib.git

RUN apt-get update && apt-get install python3-pip -y 
RUN pip3 install numpy==1.21.5 pandas==1.3.3 open3d matplotlib pyyaml opencv-python scipy scikit-spatial
