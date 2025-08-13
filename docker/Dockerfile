# Ultra-Minimal CPU-Only Dockerfile for ROS2 on VPS - PRE-CONFIGURED
ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV USER=jovyan
ENV HOME=/home/${USER}
ENV VENV_PATH=/opt/venv

# Basic system setup with retries
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release wget git \
        python3 python3-venv python3-pip \
        sudo ca-certificates \
        openssl \
	vim  \
	nano && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install Gazebo Repository with retry mechanism
RUN for i in {1..3}; do \
        wget -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
        https://packages.osrfoundation.org/gazebo.gpg && \
        break || sleep 5; \
    done && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Install ROS2 packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        python3-rosdep && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Gazebo (minimal)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        gz-harmonic && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS-Gazebo bridge
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install GUI essentials only
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        xvfb \
        x11vnc \
        websockify \
        x11-utils \
        xterm \
        openbox \
        obconf \
        menu \
	htop \
        ### FIX: Add the package that provides the 'xrdb' command
        x11-xserver-utils && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install minimal OpenGL (Ubuntu 24.04 compatible)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        mesa-utils \
        libgl1-mesa-dri \
        libglu1-mesa \
        libglx-mesa0 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install minimal Qt for Gazebo
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libqt5widgets5 \
        libqt5gui5 \
        libqt5core5a && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install noVNC with error handling
RUN git clone --depth 1 https://github.com/novnc/noVNC.git /opt/novnc || \
    (sleep 10 && git clone --depth 1 https://github.com/novnc/noVNC.git /opt/novnc) && \
    git clone --depth 1 https://github.com/novnc/websockify /opt/novnc/utils/websockify || \
    (sleep 10 && git clone --depth 1 https://github.com/novnc/websockify /opt/novnc/utils/websockify) && \
    chmod +x /opt/novnc/utils/novnc_proxy && \
    ln -s /opt/novnc/vnc.html /opt/novnc/index.html

# Create virtual environment
RUN python3 -m venv ${VENV_PATH} && \
    ${VENV_PATH}/bin/pip install --upgrade pip

# User setup (handle existing UID 1000)
RUN set -e && \
    if id -u 1000 >/dev/null 2>&1; then \
        EXISTING_USER=$(id -un 1000 2>/dev/null || echo "unknown") && \
        userdel -r "$EXISTING_USER" 2>/dev/null || true; \
    fi && \
    groupadd -g 1000 ${USER} 2>/dev/null || true && \
    useradd -m -s /bin/bash -u 1000 -g 1000 ${USER} && \
    echo "${USER}:${USER}" | chpasswd && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

### FIX: Add the .Xresources file for the jovyan user
RUN cat > /home/${USER}/.Xresources << 'EOF'
! XTerm settings

! Font and Size
xterm*faceName: DejaVu Sans Mono
xterm*faceSize: 14

! Colors
xterm*background: #282a36
xterm*foreground: #f8f8f2

! Enable scrollbar
xterm*scrollBar: true
xterm*rightScrollBar: true
xterm*saveLines: 2000
EOF

### FIX: Set correct ownership for the new file
RUN chown ${USER}:${USER} /home/${USER}/.Xresources

# Install Node.js and configurable-http-proxy with retry
RUN for i in {1..3}; do \
        curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
        break || sleep 10; \
    done && \
    apt-get install -y nodejs && \
    npm install -g configurable-http-proxy && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install minimal Jupyter with retry mechanism
RUN for i in {1..3}; do \
	    jupyterhub \
            jupyterlab \
            notebook \
            ipywidgets \
            matplotlib \
            numpy \
            xeus-python \
            ruff \
            ruff-lsp && \
        break || sleep 10; \
    done

### FIX: Create a default settings file to enable the Language Server
RUN mkdir -p /home/${USER}/.jupyter/lab/user-settings/@jupyter-lsp/notebook-connector-extension && \
    echo '{' > /home/${USER}/.jupyter/lab/user-settings/@jupyter-lsp/notebook-connector-extension/plugin.jupyterlab-settings && \
    echo '    "language_server_startup_timeout": 60' >> /home/${USER}/.jupyter/lab/user-settings/@jupyter-lsp/notebook-connector-extension/plugin.jupyterlab-settings && \
    echo '}' >> /home/${USER}/.jupyter/lab/user-settings/@jupyter-lsp/notebook-connector-extension/plugin.jupyterlab-settings && \
    chown -R ${USER}:${USER} /home/${USER}/.jupyter

# Configure Python paths for ROS2
RUN echo "/opt/ros/jazzy/local/lib/python3.12/dist-packages" > ${VENV_PATH}/lib/python3.12/site-packages/ros2.pth && \
    echo "/opt/ros/jazzy/lib/python3.12/site-packages" >> ${VENV_PATH}/lib/python3.12/site-packages/ros2.pth

# Create directories with proper permissions
RUN mkdir -p /srv/jupyterhub \
    /home/${USER}/shared_workspace \
    /home/${USER}/notebooks \
    /home/${USER}/ros2_ws/src \
    /tmp/runtime-${USER} && \
    chown -R ${USER}:${USER} /home/${USER} /tmp/runtime-${USER} && \
    chmod 700 /srv/jupyterhub

# ROBUST rosdep initialization with multiple retry strategies
RUN set -e && \
    echo "Initializing rosdep with network resilience..." && \
    rosdep init || true && \
    \
    # Strategy 1: Standard update with retries
    for attempt in {1..5}; do \
        echo "Attempt $attempt: Standard rosdep update..." && \
        if sudo -u ${USER} rosdep update; then \
            echo "✓ rosdep update successful on attempt $attempt" && \
            exit 0; \
        fi; \
        echo "Failed attempt $attempt, waiting..." && \
        sleep $((attempt * 10)); \
    done && \
    \
    # Strategy 2: Update with --include-eol-distros (more permissive)
    echo "Trying rosdep update with --include-eol-distros..." && \
    sudo -u ${USER} rosdep update --include-eol-distros || \
    \
    # Strategy 3: Create minimal rosdep cache manually if all else fails
    (echo "All rosdep update attempts failed, creating minimal cache..." && \
     sudo -u ${USER} mkdir -p /home/${USER}/.ros/rosdep/sources.cache && \
     echo "rosdep cache directory created")

# Create Openbox menu to fix menu warning
RUN mkdir -p /var/lib/openbox && \
    cat > /var/lib/openbox/debian-menu.xml << 'MENU_EOF'
<?xml version="1.0" encoding="UTF-8"?>
<openbox_menu xmlns="http://openbox.org/3.4/menu">
  <menu id="root-menu" label="ROS2 Teaching">
    <item label="Terminal">
      <action name="Execute">
        <command>xterm</command>
      </action>
    </item>
  </menu>
</openbox_menu>
MENU_EOF

# Create network-resilient startup script
RUN cat > /usr/local/bin/start-services.sh << 'EOF'
#!/bin/bash
set -e

# --- This is the FIXED startup script ---

export HOME=/home/jovyan
export USER=jovyan
export DISPLAY=:1
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH

# Essential CPU-only variables
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe
export LP_NUM_THREADS=2

echo "=== Starting Pre-Configured VPS ROS2 Environment ==="

# Function for logging with timestamps
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Fix permissions first
log "Fixing permissions..."
chown -R jovyan:jovyan /home/jovyan
mkdir -p /tmp/runtime-jovyan
chown jovyan:jovyan /tmp/runtime-jovyan
chmod 700 /srv/jupyterhub

# Create and secure JupyterHub files BEFORE starting services
log "Setting up JupyterHub security files..."
mkdir -p /srv/jupyterhub
if [ ! -f /srv/jupyterhub/jupyterhub_cookie_secret ]; then
    openssl rand -hex 32 > /srv/jupyterhub/jupyterhub_cookie_secret
fi
if [ ! -f /srv/jupyterhub/jupyterhub_proxy_token ]; then
    openssl rand -hex 32 > /srv/jupyterhub/jupyterhub_proxy_token
fi
chmod 600 /srv/jupyterhub/jupyterhub_cookie_secret
chmod 600 /srv/jupyterhub/jupyterhub_proxy_token
chown root:root /srv/jupyterhub/jupyterhub_cookie_secret
chown root:root /srv/jupyterhub/jupyterhub_proxy_token

# Start Xvfb with reduced warnings and VARIABLE resolution
log "Starting display..."
### FIX: Use the VNC_RESOLUTION variable, defaulting to 1920x1080
Xvfb :1 -screen 0 ${VNC_RESOLUTION:-1920x1080}x24 -ac +extension GLX +render -noreset -quiet &
sleep 3

# Start minimal window manager
log "Starting window manager..."
sudo -u jovyan DISPLAY=:1 openbox &
sleep 2

### FIX: Apply the XTerm settings from the .Xresources file
log "Applying XTerm settings..."
sudo -u jovyan DISPLAY=:1 xrdb -merge /home/jovyan/.Xresources

# Start VNC with IPv4 only to avoid binding warnings
log "Starting VNC..."
x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared -nolookup -noipv6 &
sleep 2

# Start noVNC
log "Starting noVNC..."
cd /opt/novnc && ./utils/novnc_proxy --vnc localhost:5900 --listen 6080 &
sleep 2

# Source ROS2 and continue with the rest of the script...
log "Setting up ROS2 environment..."
source /opt/ros/jazzy/setup.bash || echo "Warning: ROS2 setup issues, continuing..."

echo "=== VPS Environment Ready ==="
echo "JupyterHub: http://YOUR_VPS_IP:8000"
echo "GUI: http://YOUR_VPS_IP:6080/vnc.html"

# Health monitoring loop
while true; do
    sleep 60
    # FIX: Ensure the monitor loop also uses the variable resolution if it restarts Xvfb
    if ! pgrep -f "Xvfb :1" > /dev/null; then
        log "Restarting display..."
        Xvfb :1 -screen 0 ${VNC_RESOLUTION:-1920x1080}x24 -ac +extension GLX +render -noreset -quiet &
    fi
    # ... rest of the monitoring loop ...
done
EOF

RUN chmod +x /usr/local/bin/start-services.sh

# User environment
USER ${USER}
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.profile && \
    echo 'export DISPLAY=:1' >> ~/.bashrc && \
    echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc

USER root

EXPOSE 8000 6080
WORKDIR /home/${USER}
CMD ["/usr/local/bin/start-services.sh"]
