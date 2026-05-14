ARG BASE_IMAGE=ghcr.io/usc-robosub/barracuda-camera-image:latest
FROM ${BASE_IMAGE}

ARG PKG_SEL
ENV PKG_SEL=$PKG_SEL

RUN apt-get update && apt-get install -y \
    vim \
    git \
    git-lfs \
    python3-pip \
    python3-dev \
    python3-pylsp \
    python3-colcon-common-extensions \
    python3-vcstool \
    clangd \
    curl \
    gnupg \
    wget \
    software-properties-common \
    && wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - \
    && ISAAC_APT_SOURCE="deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(. /etc/os-release && echo ${UBUNTU_CODENAME}) release-3.0" \
    && grep -qxF "${ISAAC_APT_SOURCE}" /etc/apt/sources.list || echo "${ISAAC_APT_SOURCE}" >> /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y \
        ros-humble-isaac-ros-gxf \
        ros-humble-isaac-ros-managed-nitros \
        ros-humble-isaac-ros-nitros-image-type \
        ros-humble-isaac-ros-launch-utils \
        ros-humble-isaac-ros-nvblox \
        ros-humble-foxglove-bridge \
        ros-humble-joint-state-publisher \
        python3-smbus \
        python3-gpiozero \
    && python3 -m pip install --no-cache-dir 'numpy<2' \
    && cd && curl -L -O https://github.com/helix-editor/helix/releases/download/25.07.1/helix-25.07.1-$(uname -m)-linux.tar.xz \
    && tar xf helix-25.07.1-$(uname -m)-linux.tar.xz && mv helix-25.07.1-$(uname -m)-linux/hx /usr/local/bin \
    && mkdir -p ~/.config/helix && mv helix-25.07.1-$(uname -m)-linux/runtime ~/.config/helix \
    && rm -rf helix-25.07.1-$(uname -m)-linux \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/barracuda_ws/install/setup.bash ] && source ~/barracuda_ws/install/setup.bash" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/barracuda_ws/

COPY entrypoint.sh /root/entrypoint.sh
CMD [ "/bin/bash", "/root/entrypoint.sh" ]
