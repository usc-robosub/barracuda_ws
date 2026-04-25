FROM ghcr.io/usc-robosub/barracuda-camera-image:latest

RUN apt-get update && apt-get install -y \
    vim \
    git \
    python3-pip \
    python3-pylsp \
    clangd \
    curl \
    software-properties-common \
    && cd && curl -L -O https://github.com/helix-editor/helix/releases/download/25.07.1/helix-25.07.1-$(uname -m)-linux.tar.xz \
    && tar xf helix-25.07.1-$(uname -m)-linux.tar.xz && mv helix-25.07.1-$(uname -m)-linux/hx /usr/local/bin \
    && mkdir -p ~/.config/helix && mv helix-25.07.1-$(uname -m)-linux/runtime ~/.config/helix \
    && rm -rf helix-25.07.1-$(uname -m)-linux

COPY src /root/barracuda_ws/src

# will not build if zed_debug isn't skipped
ARG PKG_SKIP="zed_debug"
ARG PKG_SEL
ENV PKG_SEL=$PKG_SEL

# on line 30:
# will deps in dir nested in src get installed (e.g. zed package dependencies if PKG_PATHS="barracuda_camera")? 
# test after getting rid of the part of the base image that builds a separate barracuda_camera in extraneous /ros2_ws dir it creates
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/barracuda_ws/install/setup.bash ] && source ~/barracuda_ws/install/setup.bash" >> ~/.bashrc \
    && . /opt/ros/humble/setup.sh \
    && cd /root/barracuda_ws/src \
    && export PKG_PATHS=${PKG_SEL:+barracuda_onboard $PKG_SEL} \
    && rosdep install --from-paths ${PKG_PATHS:-"."} -r -y --ignore-src \
    && cd /root/barracuda_ws \
    && colcon build --symlink-install --packages-skip ${PKG_SKIP} ${PKG_PATHS:+--packages-up-to $PKG_PATHS} \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/barracuda_ws/

COPY entrypoint.sh /root/entrypoint.sh
CMD [ "/bin/bash", "/root/entrypoint.sh" ]
