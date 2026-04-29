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
    && cd && curl -L -O https://github.com/helix-editor/helix/releases/download/25.07.1/helix-25.07.1-$(uname -m)-linux.tar.xz \
    && tar xf helix-25.07.1-$(uname -m)-linux.tar.xz && mv helix-25.07.1-$(uname -m)-linux/hx /usr/local/bin \
    && mkdir -p ~/.config/helix && mv helix-25.07.1-$(uname -m)-linux/runtime ~/.config/helix \
    && rm -rf helix-25.07.1-$(uname -m)-linux

COPY src /root/barracuda_ws/src

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/barracuda_ws/install/setup.bash ] && source ~/barracuda_ws/install/setup.bash" >> ~/.bashrc \
    && . /opt/ros/humble/setup.sh \
    && cd /root/barracuda_ws/src \
    && PKG_PATHS="" \
    && PKG_SKIP="zed_debug isaac_ros_nvblox nvblox_examples_bringup nvblox_image_padding nvblox_ros nvblox_msgs nvblox_ros_common nvblox_ros_python_utils nvblox_rviz_plugin nvblox_nav2 realsense_splitter semantic_label_conversion" \
    && INCLUDE_ZED_WRAPPER=0 \
    && if [ -n "$PKG_SEL" ]; then \
        for pkg in $PKG_SEL; do \
            if [ "$pkg" != "barracuda_onboard" ] && [ -f "/root/barracuda_ws/src/$pkg/package.xml" ]; then \
                PKG_PATHS="$PKG_PATHS $pkg"; \
                if [ "$pkg" = "barracuda_camera" ]; then \
                    INCLUDE_ZED_WRAPPER=1; \
                fi; \
            fi; \
        done; \
        PKG_PATHS="barracuda_onboard${PKG_PATHS:+$PKG_PATHS}"; \
        if [ "$INCLUDE_ZED_WRAPPER" = "1" ] && [ -f "/root/barracuda_ws/src/zed-ros2-wrapper/zed_wrapper/package.xml" ]; then \
            PKG_PATHS="$PKG_PATHS zed_wrapper"; \
        fi; \
    fi \
    && rosdep install --from-paths . -y --ignore-src --skip-keys="ament_python isaac_ros_dnn_image_encoder isaac_ros_gxf isaac_ros_peoplesemseg_models_install isaac_ros_test isaac_ros_triton isaac_ros_unet isaac_ros_visual_slam nova_carter_navigation" \
    && cd /root/barracuda_ws \
    && colcon build --symlink-install --packages-skip $PKG_SKIP ${PKG_PATHS:+--packages-up-to $PKG_PATHS} \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/barracuda_ws/

COPY entrypoint.sh /root/entrypoint.sh
CMD [ "/bin/bash", "/root/entrypoint.sh" ]
