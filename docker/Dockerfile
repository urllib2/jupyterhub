# Multi-User JupyterHub Dockerfile - v10 (CLEANED - removed opencv, nav2, jupyter-git)
ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV USER=jovyan
ENV HOME=/home/${USER}
ENV VENV_PATH=/opt/venv

# ---- refresh Ubuntu archive key (base image keys are stale) ----
RUN apt-get update || true && \
    apt-get install -y --no-install-recommends ca-certificates wget curl && \
    wget -qO- 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x871920D1991BC93C' | tee /etc/apt/trusted.gpg.d/ubuntu-archive.asc > /dev/null

# 1. Basic system setup and ROS/Gazebo repositories
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release wget git sudo ca-certificates openssl vim nano \
    python3-pip python3-venv ubuntu-keyring && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu   $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    wget -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg   && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable   $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# 2. Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop python3-colcon-common-extensions python3-rosdep && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Add this after ROS2 installation to fix library loading
RUN echo "/opt/ros/jazzy/lib" >> /etc/ld.so.conf.d/ros2-jazzy.conf && \
    echo "/opt/ros/jazzy/lib/x86_64-linux-gnu" >> /etc/ld.so.conf.d/ros2-jazzy.conf && \
    ldconfig

# Set library path environment variable
ENV LD_LIBRARY_PATH="/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="/opt/ros/jazzy/local/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH}"

# 3. Install Gazebo
RUN apt-get update && apt-get install -y --no-install-recommends gz-harmonic && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ---- re-refresh Ubuntu key before ROS/Gazebo block (keys expire again) ----
RUN wget -qO- 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x871920D1991BC93C' | tee /etc/apt/trusted.gpg.d/ubuntu-archive.asc > /dev/null

# 4. Install ROS-Gazebo bridge and essential ROS2 development tools (CLEANED - removed nav2)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-rqt-* ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-turtlesim ros-${ROS_DISTRO}-demo-nodes-cpp ros-${ROS_DISTRO}-demo-nodes-py && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 5. Install GUI essentials and development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11vnc websockify x11-utils xterm openbox obconf menu x11-xserver-utils \
    tree htop tmux screen net-tools dbus-x11 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 5-bis. Terminal emulator for the desktop
RUN apt-get update && apt-get install -y --no-install-recommends terminator && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 5-c. Keyboard utilities (fix stuck Caps Lock / Shift in noVNC)
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-xkb-utils xdotool && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '#!/bin/bash\nxdotool key Caps_Lock' > /usr/local/bin/fixcaps && \
    chmod +x /usr/local/bin/fixcaps

# 6. Install graphics and UI libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils libgl1-mesa-dri libglu1-mesa libglx-mesa0 \
    libqt5widgets5 libqt5gui5 libqt5core5a && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 🔥 Force software rendering for clean turtlesim on first launch
ENV LIBGL_ALWAYS_SOFTWARE=1

# 7. Install noVNC
RUN git clone --depth 1 https://github.com/novnc/noVNC.git   /opt/novnc && \
    git clone --depth 1 https://github.com/novnc/websockify   /opt/novnc/utils/websockify && \
    chmod +x /opt/novnc/utils/novnc_proxy && \
    ln -s /opt/novnc/vnc.html /opt/novnc/index.html

# 8. Create the jovyan user (robust version)
RUN export OLD_USER=$(id -un 1000 2>/dev/null || true) && \
    if [ -n "$OLD_USER" ]; then userdel -r "$OLD_USER"; fi && \
    groupadd -g 1000 ${USER} && \
    useradd -m -s /bin/bash -u 1000 -g 1000 ${USER} && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 9. Install Node.js and JupyterHub proxy
RUN apt-get update && apt-get install -y nodejs npm && \
    npm install -g configurable-http-proxy && \
    ln -s /usr/local/bin/configurable-http-proxy /usr/bin/configurable-http-proxy && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 10. REMOVED: OpenCV dependencies (not needed for basic ROS2 functionality)
# Removed to reduce image size - can be added back if computer vision features needed

# 11. CRITICAL: Install DockerSpawner dependencies in the container
RUN apt-get update && apt-get install -y --no-install-recommends \
    docker.io && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 12. Create Virtual Environment and Install Python packages (CLEANED - removed opencv and jupyter-git)
RUN python3 -m venv ${VENV_PATH} && \
    ${VENV_PATH}/bin/pip install --upgrade pip && \
    ${VENV_PATH}/bin/pip install --no-cache-dir \
        jupyterhub==4.0.* jupyterlab notebook ipywidgets matplotlib numpy scipy \
        dockerspawner==12.1.* oauthenticator==16.0.* \
        jupyterhub-idle-culler \
        jupyter-server-proxy \
        xeus-python ruff \
        plotly dash seaborn pandas \
        pillow \
        jupyterlab-code-formatter jupyterlab-lsp python-lsp-server

# 13. Create user directories, config files, and JupyterHub multi-user configuration
RUN mkdir -p /srv/jupyterhub ${HOME}/shared_workspace ${HOME}/notebooks ${HOME}/ros2_ws/src ${HOME}/.config/openbox && \
    echo '! XTerm settings' > ${HOME}/.Xresources && \
    echo 'xterm*faceName: DejaVu Sans Mono' >> ${HOME}/.Xresources && \
    echo 'xterm*faceSize: 14' >> ${HOME}/.Xresources && \
    echo 'xterm*background: #282a36' >> ${HOME}/.Xresources && \
    echo 'xterm*foreground: #f8f8f2' >> ${HOME}/.Xresources && \
    echo 'xterm*scrollBar: true' >> ${HOME}/.Xresources && \
    echo 'xterm*rightScrollBar: true' >> ${HOME}/.Xresources && \
    echo 'xterm*saveLines: 2000' >> ${HOME}/.Xresources && \
    echo '<?xml version="1.0" encoding="UTF-8"?>' > ${HOME}/.config/openbox/menu.xml && \
    echo '<openbox_menu xmlns="http://openbox.org/3.4/menu  ">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '  <menu id="root-menu" label="Desktop">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    <item label="Terminal">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '      <action name="Execute">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '        <command>xterm</command>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '      </action>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    </item>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    <item label="Terminator">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '      <action name="Execute">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '        <command>terminator</command>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '      </action>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    </item>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    <separator/>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    <item label="Reconfigure">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '      <action name="Reconfigure"/>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    </item>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    <item label="Exit">' >> ${HOME}/.config/openbox/menu.xml && \
    echo '      <action name="Exit"/>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '    </item>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '  </menu>' >> ${HOME}/.config/openbox/menu.xml && \
    echo '</openbox_menu>' >> ${HOME}/.config/openbox/menu.xml

# 14. Create Ruff configuration for ROS2 development
RUN cat > ${HOME}/pyproject.toml << 'EOF'
[tool.ruff]
line-length = 100
select = ["E", "W", "F", "I", "N", "UP", "B", "A", "C4", "PIE", "SIM", "RET", "ARG"]
ignore = ["E501", "N999", "N806", "ARG001", "ARG002", "B008"]
exclude = [
    ".bzr", ".direnv", ".eggs", ".git", ".git-rewrite", ".hg", ".mypy_cache", 
    ".nox", ".pants.d", ".pytype", ".ruff_cache", ".svn", ".tox", ".venv",
    "__pypackages__", "_build", "buck-out", "build", "dist", "node_modules", 
    "venv", "install", "log", "build"
]
target-version = "py312"

[tool.ruff.format]
quote-style = 'single'
indent-style = 'space'

[tool.ruff.lint.isort]
known-first-party = ["rclpy", "rclcpp"]
known-third-party = ["numpy", "matplotlib", "tf2_ros", "geometry_msgs", "sensor_msgs", "std_msgs"]

[tool.ruff.lint.pep8-naming]
classmethod-decorators = ["classmethod"]
staticmethod-decorators = ["staticmethod"]
EOF

# 15. FIXED: Fix permissions and create required directories
RUN chown -R ${USER}:${USER} ${HOME} && \
    chmod 755 ${HOME} && \
    chmod 700 /srv/jupyterhub && \
    mkdir -p /tmp && chmod 1777 /tmp && \
    mkdir -p /run/user/1000 && chown ${USER}:${USER} /run/user/1000 && chmod 700 /run/user/1000 && \
    mkdir -p /home/jovyan/srv/jupyterhub && chown ${USER}:${USER} /home/jovyan/srv/jupyterhub

# 16. CRITICAL: Add SEPARATE scripts for hub and single-user - use COPY instead of inline heredoc
COPY docker/start-singleuser.sh /usr/local/bin/start-singleuser.sh
COPY docker/start-hub.sh /usr/local/bin/start-hub.sh
COPY student_management.py /srv/jupyterhub/student_management.py
RUN chmod +x /usr/local/bin/start-singleuser.sh && \
    chmod +x /usr/local/bin/start-hub.sh && \
    chmod +r /srv/jupyterhub/student_management.py

# 17. Initialize rosdep
RUN rosdep init || true && \
    sudo -u ${USER} rosdep update --include-eol-distros || true

# 18. CRITICAL: Add user to docker group for DockerSpawner
RUN usermod -aG docker ${USER} || true

# 19. Final container setup
USER ${USER}
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'export PATH=${VENV_PATH}/bin:$PATH' >> ~/.bashrc && \
    # 🔥 permanent fix: remove any leftover ROS_DOMAIN_ID exports
    sed -i '/ROS_DOMAIN_ID/d' ~/.bashrc && \
    mkdir -p ~/.jupyter && \
    cat > ~/.jupyter/jupyter_lab_config.py << 'EOF'
c.ServerApp.ip = '0.0.0.0'
c.ServerApp.allow_origin = '*'
c.ServerApp.allow_remote_access = True
c.ServerApp.open_browser = False
c.LabApp.default_url = '/lab'
c.ServerApp.port = 8888
c.ServerApp.port_retries = 0
EOF

# 21. Switch back to root and finalize
USER root
WORKDIR /home/${USER}
EXPOSE 8000 8888 6080

# CRITICAL: Default command for spawned containers (single-user mode)
CMD ["/usr/local/bin/start-singleuser.sh"]
