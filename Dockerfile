# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full-focal

ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install necessary packages
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-vcstool \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init && rosdep update; \
    else \
        rosdep update; \
    fi

# Create a workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Clone the necessary repositories
RUN git clone https://github.com/MiguelSolisSegura/tortoisebot_waypoints.git \
    && git clone https://github.com/rigbetellabs/tortoisebot.git

# Install dependencies and build the workspace
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  catkin_make"

# Source the setup.bash file and run the entrypoint
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Use the entrypoint to run your commands
ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && (roslaunch tortoisebot_waypoints tortoisebot_playground_headless.launch &) && sleep 15 && rostest tortoisebot_waypoints waypoints_test.test -r"]
