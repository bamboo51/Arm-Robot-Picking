FROM airobotbook/ros2-desktop-ai-robot-book-humble

# The base image ships an expired ROS apt key (EXPKEYSIG F42ED6FBAB17C654); fetch the current one.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg

# Faster bag writer than sqlite3 for raw camera images (ros2 bag record -s mcap).
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-rosbag2-storage-mcap \
 && rm -rf /var/lib/apt/lists/*

# lerobot 0.6.1 needs Python >= 3.12; ROS Humble is 3.10. uv installs a separate Python
# and the exact locked environment, used only by learning/policy_server.py.
COPY --from=ghcr.io/astral-sh/uv:0.8.15 /uv /uvx /usr/local/bin/
ENV UV_PROJECT_ENVIRONMENT=/opt/lerobot-venv \
    UV_PYTHON_INSTALL_DIR=/opt/uv-python \
    UV_LINK_MODE=copy
COPY learning/pyproject.toml learning/uv.lock /opt/learning/
RUN cd /opt/learning \
 && uv sync --frozen --no-install-project \
 && rm -rf /root/.cache/uv \
 && chmod -R a+rX /opt/lerobot-venv /opt/uv-python

# ZMQ client for scripts/infer_node.py (system Python 3.10, next to rclpy).
RUN pip3 install --no-cache-dir pyzmq