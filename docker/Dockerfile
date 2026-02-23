# Multi-User JupyterHub Dockerfile - OPTIMIZED (No Gazebo + URDF Support)
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

# 1. COMBINED: System setup, ROS/Gazebo repositories, and ROS2 packages installation
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release wget git sudo ca-certificates openssl vim nano \
    python3-pip python3-venv ubuntu-keyring && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-urdf-tutorial \
    python3-colcon-common-extensions \
    python3-rosdep && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure library paths for ROS2
RUN echo "/opt/ros/jazzy/lib" >> /etc/ld.so.conf.d/ros2-jazzy.conf && \
    echo "/opt/ros/jazzy/lib/x86_64-linux-gnu" >> /etc/ld.so.conf.d/ros2-jazzy.conf && \
    ldconfig

ENV LD_LIBRARY_PATH="/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="/opt/ros/jazzy/local/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH}"

# ---- re-refresh Ubuntu key before additional packages ----
RUN wget -qO- 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x871920D1991BC93C' | tee /etc/apt/trusted.gpg.d/ubuntu-archive.asc > /dev/null

# 2. Essential ROS2 development tools (NO GAZEBO) - OPTIMIZED
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-rqt-graph && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. GUI essentials + terminal - OPTIMIZED
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11vnc websockify x11-utils xterm openbox x11-xserver-utils \
    tree htop net-tools dbus-x11 \
    terminator \
    x11-xkb-utils xdotool && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '#!/bin/bash\nxdotool key Caps_Lock' > /usr/local/bin/fixcaps && \
    chmod +x /usr/local/bin/fixcaps

# 4. Graphics libraries + Node.js (NO DOCKER) - OPTIMIZED
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils libgl1-mesa-dri libglu1-mesa libglx-mesa0 \
    libqt5widgets5 libqt5gui5 libqt5core5a \
    nodejs npm && \
    npm install -g configurable-http-proxy && \
    ln -s /usr/local/bin/configurable-http-proxy /usr/bin/configurable-http-proxy && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Force software rendering
ENV LIBGL_ALWAYS_SOFTWARE=1

# 5. Install noVNC
RUN git clone --depth 1 https://github.com/novnc/noVNC.git /opt/novnc && \
    git clone --depth 1 https://github.com/novnc/websockify /opt/novnc/utils/websockify && \
    chmod +x /opt/novnc/utils/novnc_proxy && \
    ln -s /opt/novnc/vnc.html /opt/novnc/index.html

# 6. Create jovyan user
RUN export OLD_USER=$(id -un 1000 2>/dev/null || true) && \
    if [ -n "$OLD_USER" ]; then userdel -r "$OLD_USER"; fi && \
    groupadd -g 1000 ${USER} && \
    useradd -m -s /bin/bash -u 1000 -g 1000 ${USER} && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 7. Create Virtual Environment and Install Python packages - OPTIMIZED
RUN python3 -m venv ${VENV_PATH} && \
    ${VENV_PATH}/bin/pip install --upgrade pip && \
    ${VENV_PATH}/bin/pip install --no-cache-dir \
        jupyterhub==4.0.* jupyterlab notebook ipywidgets \
        matplotlib numpy scipy pandas \
        dockerspawner==12.1.* oauthenticator==16.0.* \
        jupyterhub-multiauthenticator \
        jupyterhub-idle-culler \
        jupyter-server-proxy \
        ruff \
        pillow \
        jupyterlab-lsp python-lsp-server

# 8. Create user directories, config files - UPDATED: work -> temporary_ws
RUN mkdir -p /srv/jupyterhub ${HOME}/temporary_ws ${HOME}/.config/openbox && \
    echo '! XTerm settings' > ${HOME}/.Xresources && \
    echo 'xterm*faceName: DejaVu Sans Mono' >> ${HOME}/.Xresources && \
    echo 'xterm*faceSize: 14' >> ${HOME}/.Xresources && \
    echo 'xterm*background: #282a36' >> ${HOME}/.Xresources && \
    echo 'xterm*foreground: #f8f8f2' >> ${HOME}/.Xresources && \
    echo 'xterm*scrollBar: true' >> ${HOME}/.Xresources && \
    echo 'xterm*rightScrollBar: true' >> ${HOME}/.Xresources && \
    echo 'xterm*saveLines: 2000' >> ${HOME}/.Xresources && \
    echo '<?xml version="1.0" encoding="UTF-8"?>' > ${HOME}/.config/openbox/menu.xml && \
    echo '<openbox_menu xmlns="http://openbox.org/3.4/menu">' >> ${HOME}/.config/openbox/menu.xml && \
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

# 9. Create Ruff configuration for ROS2 development
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
EOF

# 10. Fix permissions and create required directories
RUN chown -R ${USER}:${USER} ${HOME} && \
    chmod 755 ${HOME} && \
    chmod 700 /srv/jupyterhub && \
    mkdir -p /tmp && chmod 1777 /tmp && \
    mkdir -p /run/user/1000 && chown ${USER}:${USER} /run/user/1000 && chmod 700 /run/user/1000 && \
    mkdir -p /home/jovyan/srv/jupyterhub && chown ${USER}:${USER} /home/jovyan/srv/jupyterhub

# 11. Copy startup scripts
COPY docker/start-singleuser.sh /usr/local/bin/start-singleuser.sh
COPY docker/start-hub.sh /usr/local/bin/start-hub.sh
COPY student_management.py /srv/jupyterhub/student_management.py
COPY jupyterhub_config.py /srv/jupyterhub/jupyterhub_config.py
COPY templates/ /srv/jupyterhub/templates/
RUN chmod +x /usr/local/bin/start-singleuser.sh && \
    chmod +x /usr/local/bin/start-hub.sh && \
    chmod +r /srv/jupyterhub/student_management.py

# 12. Initialize rosdep
RUN rosdep init || true && \
    sudo -u ${USER} rosdep update --include-eol-distros || true && \
    usermod -aG docker ${USER} || true

# 13. Final user setup with JupyterLab configuration
USER ${USER}
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'export PATH=${VENV_PATH}/bin:$PATH' >> ~/.bashrc && \
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

# Add JupyterLab terminal default settings (FONT SIZE 18)
RUN mkdir -p ~/.jupyter/lab/user-settings/@jupyterlab/terminal-extension && \
    cat > ~/.jupyter/lab/user-settings/@jupyterlab/terminal-extension/plugin.jupyterlab-settings << 'EOF'
{
    "fontFamily": "monospace",
    "fontSize": 18,
    "lineHeight": 1,
    "scrollback": 1000,
    "shutdownOnClose": false,
    "closeOnExit": true,
    "theme": "inherit"
}
EOF

# 14. Switch back to root and finalize
USER root
WORKDIR /home/${USER}
EXPOSE 8000 8888 6080

# Default command for spawned containers
CMD ["/usr/local/bin/start-singleuser.sh"]
