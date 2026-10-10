# Multi-User JupyterHub Dockerfile - ROSFORGE Edition (Meshcat, No noVNC)
ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV USER=jovyan
ENV HOME=/home/${USER}
ENV VENV_PATH=/opt/venv

# ---- refresh Ubuntu archive key ----
RUN apt-get update || true && \
    apt-get install -y --no-install-recommends ca-certificates wget curl && \
    wget -qO- 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x871920D1991BC93C' | tee /etc/apt/trusted.gpg.d/ubuntu-archive.asc > /dev/null

# 1. System setup + ROS2 repository + ROS2 packages
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

ENV LD_LIBRARY_PATH="/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu"
ENV PYTHONPATH="/opt/ros/jazzy/local/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages"

# ---- re-refresh Ubuntu key ----
RUN wget -qO- 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x871920D1991BC93C' | tee /etc/apt/trusted.gpg.d/ubuntu-archive.asc > /dev/null

# 2. ROS2 tools (NO GUI, NO RViz, NO rqt)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-demo-nodes-py && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. Node.js for configurable-http-proxy
RUN apt-get update && apt-get install -y --no-install-recommends \
    nodejs npm && \
    npm install -g configurable-http-proxy && \
    ln -s /usr/local/bin/configurable-http-proxy /usr/bin/configurable-http-proxy && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 4. Create jovyan user
RUN export OLD_USER=$(id -un 1000 2>/dev/null || true) && \
    if [ -n "$OLD_USER" ]; then userdel -r "$OLD_USER"; fi && \
    groupadd -g 1000 ${USER} && \
    useradd -m -s /bin/bash -u 1000 -g 1000 ${USER} && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 5. Python venv + all packages including Meshcat + nbgrader
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
        jupyterlab-lsp python-lsp-server \
        meshcat \
        yourdfpy \
        ipympl \
        sympy \
        filterpy \
        networkx \
        transforms3d \
        pytransform3d \
        nbgrader \
        nbconvert \
        nbformat \
        sqlalchemy \
        flask

# 6. Create user directories
RUN mkdir -p /srv/jupyterhub ${HOME}/.jupyter && \
    chown -R ${USER}:${USER} ${HOME}

# 7. Ruff configuration
RUN cat > ${HOME}/pyproject.toml << 'RUFF'
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
RUFF

# 8. Fix permissions
RUN chown -R ${USER}:${USER} ${HOME} && \
    chmod 755 ${HOME} && \
    chmod 700 /srv/jupyterhub && \
    mkdir -p /tmp && chmod 1777 /tmp

# 9. Copy startup scripts and config
COPY docker/start-singleuser.sh /usr/local/bin/start-singleuser.sh
COPY docker/start-hub.sh /usr/local/bin/start-hub.sh
COPY student_management.py /srv/jupyterhub/student_management.py
COPY jupyterhub_config.py /srv/jupyterhub/jupyterhub_config.py
COPY templates/ /srv/jupyterhub/templates/
RUN chmod +x /usr/local/bin/start-singleuser.sh && \
    chmod +x /usr/local/bin/start-hub.sh && \
    chmod +r /srv/jupyterhub/student_management.py

# 10. Initialize rosdep
RUN rosdep init || true && \
    sudo -u ${USER} rosdep update --include-eol-distros || true

# 11. Final user setup
USER ${USER}
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'export PATH=${VENV_PATH}/bin:$PATH' >> ~/.bashrc && \
    mkdir -p ~/.jupyter && \
    cat > ~/.jupyter/jupyter_lab_config.py << 'JLAB'
c.ServerApp.ip = '0.0.0.0'
c.ServerApp.allow_origin = '*'
c.ServerApp.allow_remote_access = True
c.ServerApp.open_browser = False
c.LabApp.default_url = '/lab?reset'
c.ServerApp.port = 8888
c.ServerApp.port_retries = 0
c.LabApp.check_for_updates_class = 'jupyterlab.NeverCheckForUpdate'
c.ServerApp.disable_check_xsrf = True
JLAB

RUN mkdir -p ~/.jupyter/lab/user-settings/@jupyterlab/terminal-extension && \
    cat > ~/.jupyter/lab/user-settings/@jupyterlab/terminal-extension/plugin.jupyterlab-settings << 'TERM'
{
    "fontFamily": "monospace",
    "fontSize": 18,
    "lineHeight": 1,
    "scrollback": 1000,
    "shutdownOnClose": false,
    "closeOnExit": true,
    "theme": "inherit"
}
TERM

RUN mkdir -p ~/.jupyter/lab/user-settings/@jupyterlab/docmanager-extension && \
    cat > ~/.jupyter/lab/user-settings/@jupyterlab/docmanager-extension/plugin.jupyterlab-settings << 'DOC'
{
    "defaultViewers": {},
    "storeName": ""
}
DOC

# 12. Back to root
USER root
WORKDIR /home/${USER}
EXPOSE 8000 8888

CMD ["/usr/local/bin/start-singleuser.sh"]
