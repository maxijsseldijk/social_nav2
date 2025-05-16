ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS ros-base

SHELL [ "/bin/bash" , "-c" ]

# Install essential packages
RUN apt-get update && apt-get install -y \
    gnupg2 \
    curl \
    wget \
    git \
    python3 \
    python3-pip \
    python3-torch \
    python3-transforms3d \
    xterm \
    ros-dev-tools \
    lsb-release \
    sudo \
    #clang-format \
    && rm -rf /var/lib/apt/lists/* 

FROM ros-base AS nvidia-setup

# Install NVIDIA Container Toolkit
RUN curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add - \
    && distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
    && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list \
    && apt-get update && apt-get install -y nvidia-container-toolkit \
    && rm -rf /var/lib/apt/lists/*


# Python dependencies
FROM nvidia-setup AS python-deps
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ROS/Ignition dependencies
FROM python-deps AS ros-deps
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Install Ignition Fortress
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y ignition-fortress \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


FROM ros-deps AS deps-install

WORKDIR /docker_ws/src

# RUN git config --global --add safe.directory '*'

COPY packages.repos /tmp/packages.repos

RUN groupadd -g 1000 developer \
    && useradd -m -u 1000 -g developer developer \
    && usermod -aG sudo developer \
    && echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && chown -R developer:developer /docker_ws

USER developer
# Create temporary workspace and install dependencies
RUN vcs import --skip-existing  < /tmp/packages.repos && \
    cd imitation && \
    pip install . && \
    cd ../lightsfm && \
    rm -rf /tmp/ws && \
    rm -rf /var/lib/apt/lists/*

FROM deps-install AS final



RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \  
    && echo "source /docker_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash " >> ~/.bashrc
