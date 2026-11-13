# Aquisição GNSS/INS NovAtel SPAN-IGM-S1 (ROS 2 Jazzy)
# Quando a imagem base dos docentes estiver publicada:
#   docker compose build --build-arg BASE_IMAGE=<imagem-base-do-repositorio-comum>
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-novatel-oem7-driver \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-bridge \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Workspace para nós do grupo (só compila se existir algum pacote em src/)
WORKDIR /ws
COPY src/ src/
RUN source /opt/ros/jazzy/setup.bash \
    && if [ -n "$(find src -name package.xml)" ]; then colcon build; fi

# Scripts do projeto (fim de linha LF garantido mesmo se vierem do Windows)
COPY docker/ /usr/local/bin/
RUN sed -i 's/\r$//' /usr/local/bin/*.sh /usr/local/bin/*.py \
    && chmod +x /usr/local/bin/*.sh /usr/local/bin/*.py \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
