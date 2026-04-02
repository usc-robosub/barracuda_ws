FROM ros:humble-ros-base-jammy

# Build-time UID/GID passed from host for non-root user mapping.
ARG HOST_UID=1000
ARG HOST_GID=1000


# install dev packages
RUN apt-get update && apt-get install -y \
    vim \
    git \
    python3-pip \
    python3-pylsp \
    clangd \
    software-properties-common \
    && cd && curl -L -O https://github.com/helix-editor/helix/releases/download/25.07.1/helix-25.07.1-$(uname -m)-linux.tar.xz \
    && tar xf helix-25.07.1-$(uname -m)-linux.tar.xz && mv helix-25.07.1-$(uname -m)-linux/hx /usr/local/bin \
    && mkdir -p ~/.config/helix && mv helix-25.07.1-$(uname -m)-linux/runtime ~/.config/helix \
    && rm -rf helix-25.07.1-$(uname -m)-linux

# Create a non-root user that can match the host UID/GID for Fast DDS SHM access.
RUN groupadd -g ${HOST_GID} ros \
    && useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ros

# src is mounted at runtime; build happens in entrypoint.sh
RUN mkdir -p /home/ros/barracuda_ws/src /home/ros/barracuda_ws/install && \
    chown -R ros:ros /home/ros

# Workspace selection and shell environment defaults.
ARG PKG_SEL
ENV PKG_SEL=$PKG_SEL
ENV HOME=/home/ros
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc \
    && echo "[ -f ~/barracuda_ws/install/setup.bash ] && source ~/barracuda_ws/install/setup.bash" >> /home/ros/.bashrc

# Default working directory for runtime commands.
WORKDIR /home/ros/barracuda_ws/

# Runtime ROS packages used by the description stack.
RUN apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro

# Entrypoint script setup.
COPY entrypoint.sh /home/ros/entrypoint.sh
RUN chmod +x /home/ros/entrypoint.sh && chown ros:ros /home/ros/entrypoint.sh

# Default runtime user should be non-root for Fast DDS shared-memory
# interoperability with host-side ROS tools.
USER ros

# Start via entrypoint script.
CMD [ "/bin/bash", "/home/ros/entrypoint.sh" ]
