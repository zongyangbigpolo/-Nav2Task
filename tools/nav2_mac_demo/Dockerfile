FROM ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

RUN apt-get update -qq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-minimal-tb3-sim \
      ros-jazzy-rviz2 \
      xvfb x11vnc novnc websockify openbox wmctrl && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY start_demo.bash /opt/start_demo.bash
COPY smoke_test.py /opt/smoke_test.py

CMD ["bash", "/opt/start_demo.bash"]
