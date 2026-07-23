# syntax=docker/dockerfile:1.7
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# Tag oficial 2.5.0, ancestral da branch kilted usada para ROS 2 Jazzy.
ARG UR_SIMULATION_GZ_COMMIT=048c80cd1faf87a2c74e14baadb65bd22b564d8f

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_PYTHON_VERSION=3 \
    VIRTUAL_ENV=/opt/ur_cbf_venv \
    PATH=/opt/ur_cbf_venv/bin:${PATH} \
    MPLCONFIGDIR=/tmp/matplotlib \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      git \
      gnupg \
      locales \
      lsb-release \
      software-properties-common \
      sudo \
    && locale-gen en_US en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# Repositorio oficial do ROS 2.
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
      > /etc/apt/sources.list.d/ros2.list

# Repositorio oficial do Gazebo Harmonic.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      gz-harmonic \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      python3-venv \
      python3-vcstool \
      ros-${ROS_DISTRO}-desktop \
      ros-${ROS_DISTRO}-forward-command-controller \
      ros-${ROS_DISTRO}-gz-ros2-control \
      ros-${ROS_DISTRO}-joint-state-broadcaster \
      ros-${ROS_DISTRO}-joint-trajectory-controller \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros2controlcli \
      ros-${ROS_DISTRO}-ur-robot-driver \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true \
    && rosdep update --rosdistro ${ROS_DISTRO}

# A simulacao oficial e compilada sem o launch opcional do MoveIt.
RUN mkdir -p /opt/ur_vendor_ws/src \
    && git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
      /opt/ur_vendor_ws/src/Universal_Robots_ROS2_GZ_Simulation \
    && cd /opt/ur_vendor_ws/src/Universal_Robots_ROS2_GZ_Simulation \
    && git checkout ${UR_SIMULATION_GZ_COMMIT} \
    && sed -i '/ur_moveit_config/d' ur_simulation_gz/package.xml \
    && rm -f ur_simulation_gz/launch/ur_sim_moveit.launch.py \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd /opt/ur_vendor_ws \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf build log /var/lib/apt/lists/*

COPY requirements/requirements.txt /tmp/requirements.txt
RUN python3 -m venv --system-site-packages ${VIRTUAL_ENV} \
    && python -m pip install --no-cache-dir --upgrade pip \
    && python -m pip install --no-cache-dir -r /tmp/requirements.txt \
    && python -m pip freeze > /opt/ur_cbf_python_versions.txt \
    && rm /tmp/requirements.txt

# A imagem Ubuntu 24.04 pode trazer o usuario/grupo "ubuntu" com UID/GID 1000.
# Reutiliza essas identidades quando existirem para manter as permissoes do host.
RUN existing_group="$(getent group "${USER_GID}" | cut -d: -f1 || true)" \
    && if [[ -n "${existing_group}" && "${existing_group}" != "${USERNAME}" ]]; then \
         groupmod --new-name "${USERNAME}" "${existing_group}"; \
       elif [[ -z "${existing_group}" ]]; then \
         groupadd --gid "${USER_GID}" "${USERNAME}"; \
       fi \
    && existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)" \
    && if [[ -n "${existing_user}" && "${existing_user}" != "${USERNAME}" ]]; then \
         usermod --login "${USERNAME}" --home "/home/${USERNAME}" \
           --move-home "${existing_user}"; \
       elif [[ -z "${existing_user}" ]]; then \
         useradd --uid "${USER_UID}" --gid "${USER_GID}" \
           -m -s /bin/bash "${USERNAME}"; \
       fi \
    && usermod --gid "${USER_GID}" --shell /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /workspace /tmp/matplotlib \
    && chown -R ${USERNAME}:${USERNAME} /workspace /tmp/matplotlib

COPY --chown=${USERNAME}:${USERNAME} . /workspace
COPY docker/entrypoint.sh /usr/local/bin/ur-cbf-entrypoint
RUN chmod +x /usr/local/bin/ur-cbf-entrypoint

USER ${USERNAME}
WORKDIR /workspace

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /opt/ur_vendor_ws/install/setup.bash \
    && cd /workspace/ur_cbf_ws \
    && colcon build --symlink-install

ENTRYPOINT ["/usr/local/bin/ur-cbf-entrypoint"]
CMD ["bash"]
