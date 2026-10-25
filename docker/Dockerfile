FROM airobotbook/ros2-desktop-ai-robot-book-humble

# The base image ships an expired ROS apt key (EXPKEYSIG F42ED6FBAB17C654); fetch the current one.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg

# Faster bag writer than sqlite3 for raw camera images (ros2 bag record -s mcap).
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-rosbag2-storage-mcap \
 && rm -rf /var/lib/apt/lists/*
