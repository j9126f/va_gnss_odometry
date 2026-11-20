# Grupo 07: GNSS/INS NovAtel SPAN-IGM-S1 + CAN bus do carro + perceção/planeamento local (ROS 2 Jazzy)
# Quando a imagem base dos docentes estiver publicada:
#   docker compose build --build-arg BASE_IMAGE=<imagem-base-do-repositorio-comum>
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-novatel-oem7-driver \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-ros2-socketcan \
    ros-jazzy-can-msgs \
    ros-jazzy-ackermann-msgs \
    iproute2 \
    can-utils \
    python3-serial \
    ros-jazzy-rviz2 \
    python3-numpy \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Workspace ROS 2: o pacote comum (va_planeamento) e o pacote do grupo (va_g07)
WORKDIR /ws
COPY src/ src/
RUN source /opt/ros/jazzy/setup.bash && colcon build

# Scripts do projeto (fim de linha LF garantido mesmo se vierem do Windows)
COPY docker/ /usr/local/bin/
RUN sed -i 's/\r$//' /usr/local/bin/*.sh /usr/local/bin/*.py \
    && chmod +x /usr/local/bin/*.sh /usr/local/bin/*.py \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
