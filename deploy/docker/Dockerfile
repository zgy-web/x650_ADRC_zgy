FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG VIRTUALGL_VERSION=3.1.4

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    AMSIM_WORKSPACE=/opt/amsim \
    AMSIM_PX4_DIR=/opt/amsim/PX4-Autopilot \
    AMSIM_ROS_WS=/opt/amsim/ros2_px4_ws \
    AMSIM_VENV=/opt/amsim/amsim-venv \
    AMSIM_RESULTS_DIR=/opt/amsim/project/runs \
    AMSIM_DISPLAY=:1 \
    GZ_PARTITION=amsim_default

WORKDIR /opt/amsim/project
COPY . .

RUN ./deploy/host/install_ubuntu22.sh

RUN curl -fL --retry 3 \
      "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_amd64.deb" \
      -o /tmp/virtualgl.deb \
    && (dpkg -i /tmp/virtualgl.deb || apt-get -f install -y) \
    && rm -f /tmp/virtualgl.deb

RUN ./scripts/amsim setup
RUN ./scripts/amsim setup --check-only

ENTRYPOINT ["/opt/amsim/project/deploy/docker/entrypoint.sh"]
CMD ["./scripts/amsim", "setup", "--check-only"]
