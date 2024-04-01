ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ARG STARS_MESSAGES_REPO=https://github.com/valentinrusche/stars-ros-messages.git
ARG ENABLED_FLAG_DEV_MODE=0
ARG UID
ARG GID

WORKDIR /app

SHELL ["/bin/bash", "-c"]

RUN apt-get update -y && apt-get -y dist-upgrade && \
    apt-get install --no-install-recommends --fix-missing -y \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*
RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && python3 -m pip install --upgrade pip
COPY requirements.txt /app/requirements.txt
RUN python3 -m pip install --ignore-installed -r requirements.txt

# create a non root user group with GID 1000
RUN groupadd -g ${GID} developer

# create a non root user with UID 1000
RUN useradd -u ${UID} -g developer --create-home --groups sudo --shell /bin/bash developer && \
    mkdir -p /etc/sudoers.d && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer
ENV HOME /home/developer

COPY entrypoint.sh /

# allow access for all users to source the nessecary files. u+x won't do it..
RUN chmod a+x /entrypoint.sh

RUN chown -R developer: /app/

# build  the stars message types
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && git clone ${STARS_MESSAGES_REPO} \
    && colcon build --merge-install --packages-select stars_msgs

# Copy stars_ros_exporter to the container
COPY StarsRosExporter/ros2_ws /app/ros2_ws

# Clone OpenDrive Parser
RUN git clone https://github.com/valentinrusche/python-opendrive-parser && \
    mkdir /app/ros2_ws/src/stars_ros_exporter/xodr && \
    mv python-opendrive-parser/xodr/* /app/ros2_ws/src/stars_ros_exporter/xodr

# build the bridge ros2 program.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --merge-install --packages-select stars_ros_exporter

RUN if [ "$ENABLED_FLAG_DEV_MODE" != "1" ]; then \
      rm -rf /app/ros2_ws && rm -rf /app/build; \
    fi

# Return to the non-root developer user
USER developer

ENTRYPOINT [ "/entrypoint.sh"]
