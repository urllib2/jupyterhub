# Final, STABLE Dockerfile - v3
ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV USER=jovyan
ENV HOME=/home/${USER}
ENV VENV_PATH=/opt/venv

# 1. Basic system setup and ROS/Gazebo repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release wget git sudo ca-certificates openssl vim nano \
    python3-pip python3-venv ubuntu-keyring && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    wget -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# 2. Install ROS2 packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop python3-colcon-common-extensions python3-rosdep && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. Install Gazebo.
RUN apt-get update && apt-get install -y --no-install-recommends gz-harmonic && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 4. Install ROS-Gazebo bridge.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-ros-gz-sim && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 5. Install GUI essentials.
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11vnc websockify x11-utils xterm openbox obconf menu x11-xserver-utils && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 6. Install graphics and UI libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils libgl1-mesa-dri libglu1-mesa libglx-mesa0 \
    libqt5widgets5 libqt5gui5 libqt5core5a && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 7. Install noVNC.
RUN git clone --depth 1 https://github.com/novnc/noVNC.git /opt/novnc && \
    git clone --depth 1 https://github.com/novnc/websockify /opt/novnc/utils/websockify && \
    chmod +x /opt/novnc/utils/novnc_proxy && \
    ln -s /opt/novnc/vnc.html /opt/novnc/index.html

# 8. Create the jovyan user (robust version).
RUN export OLD_USER=$(id -un 1000 2>/dev/null || true) && \
    if [ -n "$OLD_USER" ]; then userdel -r "$OLD_USER"; fi && \
    groupadd -g 1000 ${USER} && \
    useradd -m -s /bin/bash -u 1000 -g 1000 ${USER} && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 9. Install Node.js and JupyterHub proxy.
RUN apt-get update && apt-get install -y nodejs npm && \
    npm install -g configurable-http-proxy && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 10. Create Virtual Environment and Install Jupyter packages.
RUN python3 -m venv ${VENV_PATH} && \
    ${VENV_PATH}/bin/pip install --upgrade pip && \
    ${VENV_PATH}/bin/pip install --no-cache-dir \
        jupyterhub jupyterlab notebook ipywidgets matplotlib numpy \
        xeus-python

# 11. Create user directories and .Xresources theme.
RUN mkdir -p /srv/jupyterhub ${HOME}/shared_workspace ${HOME}/notebooks ${HOME}/ros2_ws/src && \
    cat > ${HOME}/.Xresources << 'EOF'
! XTerm settings
xterm*faceName: DejaVu Sans Mono
xterm*faceSize: 14
xterm*background: #282a36
xterm*foreground: #f8f8f2
xterm*scrollBar: true
xterm*rightScrollBar: true
xterm*saveLines: 2000
EOF

# 12. Set final permissions.
RUN chown -R ${USER}:${USER} ${HOME} && \
    chmod 700 /srv/jupyterhub

# 13. Initialize rosdep.
RUN rosdep init || true && \
    sudo -u ${USER} rosdep update --include-eol-distros || true

# 14. Final container setup.
USER ${USER}
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
USER root
WORKDIR /home/${USER}
EXPOSE 8000 6080
CMD ["/usr/local/bin/start-services.sh"]
