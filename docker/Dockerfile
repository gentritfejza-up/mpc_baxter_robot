FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=robot
ARG UID=1000
ARG GID=1000

RUN apt-get update && apt-get install -y \
    locales fontconfig \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    bash-completion \
    curl \
    git \
    gnupg2 \
    lsb-release \
    pkg-config \
    python3-pip \
    wget \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    coinor-libipopt-dev \
    gfortran \
    swig \
    ros-noetic-effort-controllers \
    ros-noetic-kdl-parser \
    python3-yaml \
    python2 \
    python-is-python2 \
    nlohmann-json3-dev \
    libglvnd0 libglvnd-dev \
    libgl1 libglx0 libegl1 libgles2 \
    libxcb-xinerama0 libxkbcommon-x11-0 \
    libxcb-xkb1 libxcb-cursor0 \
    libqt5x11extras5 \
    libgl1-mesa-glx libgl1-mesa-dri \
    mesa-utils \
    zstd \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${GID} ${USERNAME} && \
    useradd --uid ${UID} --gid ${GID} --create-home --shell /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

ENV WORKSPACE=/workspace
RUN mkdir -p ${WORKSPACE} && chown -R ${UID}:${GID} ${WORKSPACE}

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR ${WORKSPACE}