# Ultra-Minimal CPU-Only Dockerfile for ROS2 on VPS - NETWORK RESILIENT
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
        openssl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install Gazebo Repository with retry mechanism
RUN for i in {1..3}; do \
        wget -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
        https://packages.osrfoundation.org/gazebo.gpg && \
        break || sleep 5; \
    done && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Install minimal ROS2 packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-rviz2 \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-geometry-msgs && \
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
        menu && \
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

# Install Node.js and configurable-http-proxy with retry
RUN for i in {1..3}; do \
        curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
        break || sleep 10; \
    done && \
    apt-get install -y nodejs && \
    npm install -g configurable-http-proxy && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install minimal Jupyter with retry mechanism
RUN for i in {1..3}; do \
        ${VENV_PATH}/bin/pip install --no-cache-dir \
            jupyterhub==4.1.6 \
            jupyterlab==4.2.5 \
            notebook==7.2.2 \
            ipywidgets==8.1.5 \
            matplotlib==3.9.2 \
            numpy==2.1.1 && \
        break || sleep 10; \
    done

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

echo "=== Starting Network-Resilient VPS ROS2 Environment ==="

# Fix permissions first
chown -R jovyan:jovyan /home/jovyan
mkdir -p /tmp/runtime-jovyan
chown jovyan:jovyan /tmp/runtime-jovyan
chmod 700 /srv/jupyterhub

# Create and secure JupyterHub files BEFORE starting services
echo "Setting up JupyterHub security files..."
mkdir -p /srv/jupyterhub
if [ ! -f /srv/jupyterhub/jupyterhub_cookie_secret ]; then
    openssl rand -hex 32 > /srv/jupyterhub/jupyterhub_cookie_secret
fi
if [ ! -f /srv/jupyterhub/jupyterhub_proxy_token ]; then
    openssl rand -hex 32 > /srv/jupyterhub/jupyterhub_proxy_token
fi

# Fix file permissions (CRITICAL)
chmod 600 /srv/jupyterhub/jupyterhub_cookie_secret
chmod 600 /srv/jupyterhub/jupyterhub_proxy_token
chown root:root /srv/jupyterhub/jupyterhub_cookie_secret
chown root:root /srv/jupyterhub/jupyterhub_proxy_token

# Start Xvfb with reduced warnings
echo "Starting display..."
Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset -quiet &
sleep 3

# Start minimal window manager
echo "Starting window manager..."
DISPLAY=:1 openbox &
sleep 2

# Start VNC with IPv4 only to avoid binding warnings
echo "Starting VNC..."
x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared -nolookup -noipv6 &
sleep 2

# Start noVNC
echo "Starting noVNC..."
cd /opt/novnc && ./utils/novnc_proxy --vnc localhost:5900 --listen 6080 &
sleep 2

# Source ROS2 (with fallback if rosdep issues)
echo "Setting up ROS2 environment..."
source /opt/ros/jazzy/setup.bash || echo "Warning: ROS2 setup issues, continuing..."

# Attempt rosdep update if it failed during build
if [ ! -d "/home/jovyan/.ros/rosdep/sources.cache" ] || [ -z "$(ls -A /home/jovyan/.ros/rosdep/sources.cache 2>/dev/null)" ]; then
    echo "Attempting rosdep update at runtime..."
    sudo -u jovyan rosdep update --include-eol-distros 2>/dev/null || echo "rosdep update failed, continuing without it"
fi

# Create minimal test world
mkdir -p ${HOME}/gazebo_worlds
cat > ${HOME}/gazebo_worlds/minimal.sdf << 'WORLD_EOF'
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="minimal_world">
    <physics name="fast_physics" type="ode">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>0 0 -1</direction>
    </light>
    
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
WORLD_EOF

chown -R jovyan:jovyan ${HOME}/gazebo_worlds

# Start JupyterHub with proper security
echo "Starting JupyterHub..."
cd /srv/jupyterhub
${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py --log-level=INFO &

# Wait for services with better error handling
echo "Waiting for services..."
for i in {1..30}; do
    if curl -s -I http://localhost:8000/hub/health | grep -q "200\|302"; then
        echo "✓ JupyterHub Ready!"
        break
    elif [ $i -eq 30 ]; then
        echo "❌ JupyterHub failed to start, checking logs..."
        echo "Recent JupyterHub logs:"
        tail -20 /srv/jupyterhub/*.log 2>/dev/null || echo "No log files found"
    fi
    sleep 5
done

echo "=== VPS Environment Ready ==="
echo "JupyterHub: http://YOUR_VPS_IP:8000"
echo "GUI: http://YOUR_VPS_IP:6080/vnc.html"

# Enhanced health monitoring with error recovery
while true; do
    sleep 60
    
    # Check and restart JupyterHub if needed
    if ! pgrep -f jupyterhub > /dev/null; then
        echo "Restarting JupyterHub..."
        cd /srv/jupyterhub && ${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py --log-level=INFO &
    fi
    
    # Check and restart Xvfb if needed
    if ! pgrep -f "Xvfb :1" > /dev/null; then
        echo "Restarting display..."
        Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset -quiet &
    fi
    
    # Check and restart VNC if needed
    if ! pgrep -f x11vnc > /dev/null; then
        echo "Restarting VNC..."
        x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared -nolookup -noipv6 &
    fi
done
EOF

RUN chmod +x /usr/local/bin/start-services.sh

# User environment
USER ${USER}
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'export DISPLAY=:1' >> ~/.bashrc && \
    echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc

USER root

EXPOSE 8000 6080
WORKDIR /home/${USER}
CMD ["/usr/local/bin/start-services.sh"]