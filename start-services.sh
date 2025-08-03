#!/bin/bash
set -e

export HOME=/home/jovyan
export USER=jovyan
export DISPLAY=:1
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH

# CRITICAL: Fix XDG_RUNTIME_DIR before anything else
export XDG_RUNTIME_DIR=/tmp/runtime-jovyan

# Essential CPU-only variables
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe
export LP_NUM_THREADS=2

echo "=== Starting FIXED Runtime Directory VPS ROS2 Environment ==="

# Function for logging with timestamps
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Function to check if a port is listening (alternative to netstat)
check_port() {
    local port=$1
    local max_attempts=${2:-15}
    local sleep_time=${3:-2}
    
    for i in $(seq 1 $max_attempts); do
        if [ -f /proc/net/tcp ]; then
            local hex_port=$(printf "%04X" $port)
            if grep -q ":${hex_port} 00000000:0000 0A" /proc/net/tcp 2>/dev/null; then
                return 0
            fi
        fi
        
        if timeout 1 bash -c "</dev/tcp/localhost/$port" 2>/dev/null; then
            return 0
        fi
        
        if [ $i -eq $max_attempts ]; then
            return 1
        fi
        sleep $sleep_time
    done
}

# CRITICAL FIX: Create and fix ALL directory permissions first
log "Fixing all directory permissions and runtime setup..."
chown -R jovyan:jovyan /home/jovyan

# Fix runtime directory with proper permissions
mkdir -p /tmp/runtime-jovyan
chown jovyan:jovyan /tmp/runtime-jovyan
chmod 700 /tmp/runtime-jovyan

# Additional runtime directories that might be needed
mkdir -p /tmp/runtime-jovyan/dbus-1
mkdir -p /tmp/runtime-jovyan/pulse
chown -R jovyan:jovyan /tmp/runtime-jovyan
chmod -R 700 /tmp/runtime-jovyan

# Create additional directories for GUI applications
mkdir -p /home/jovyan/.cache
mkdir -p /home/jovyan/.config
mkdir -p /home/jovyan/.local/share
chown -R jovyan:jovyan /home/jovyan/.cache /home/jovyan/.config /home/jovyan/.local

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

# Fix file permissions (CRITICAL)
chmod 600 /srv/jupyterhub/jupyterhub_cookie_secret
chmod 600 /srv/jupyterhub/jupyterhub_proxy_token
chown root:root /srv/jupyterhub/jupyterhub_cookie_secret
chown root:root /srv/jupyterhub/jupyterhub_proxy_token

# Kill any existing processes to avoid conflicts
log "Cleaning up any existing processes..."
pkill -f "Xvfb :1" || true
pkill -f "x11vnc" || true
pkill -f "jupyterhub" || true
pkill -f "configurable-http-proxy" || true
pkill -f "novnc_proxy" || true
sleep 3

# Start Xvfb WITHOUT -quiet flag (not supported in this version)
log "Starting display server..."
Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!

# Wait for Xvfb to be ready with better error handling
log "Waiting for display server..."
for i in {1..15}; do
    if DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then
        log "✓ Display server ready"
        break
    fi
    if [ $i -eq 15 ]; then
        log "❌ Display server failed to start"
        exit 1
    fi
    sleep 2
done

# Start minimal window manager
log "Starting window manager..."
DISPLAY=:1 openbox &
sleep 2

# Start VNC server (ensure display is available first)
log "Starting VNC server..."
x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared -nolookup -noipv6 &
VNC_PID=$!

# Wait for VNC to be ready
log "Waiting for VNC server..."
if check_port 5900 15 2; then
    log "✓ VNC server ready on port 5900"
else
    log "❌ VNC server failed to start on port 5900"
    if ! kill -0 $VNC_PID 2>/dev/null; then
        log "VNC process died"
    fi
    exit 1
fi

# Start noVNC web interface
log "Starting noVNC web interface..."
cd /opt/novnc && ./utils/novnc_proxy --vnc localhost:5900 --listen 6080 &
NOVNC_PID=$!

# Wait for noVNC to be ready
log "Waiting for noVNC..."
if check_port 6080 15 2; then
    log "✓ noVNC ready on port 6080"
else
    log "❌ noVNC failed to start on port 6080"
    if ! kill -0 $NOVNC_PID 2>/dev/null; then
        log "noVNC process died"
    fi
    exit 1
fi

# Source ROS2 environment
log "Setting up ROS2 environment..."
source /opt/ros/jazzy/setup.bash || log "Warning: ROS2 setup issues, continuing..."

# Attempt rosdep update if needed
if [ ! -d "/home/jovyan/.ros/rosdep/sources.cache" ] || [ -z "$(ls -A /home/jovyan/.ros/rosdep/sources.cache 2>/dev/null)" ]; then
    log "Attempting rosdep update at runtime..."
    sudo -u jovyan rosdep update --include-eol-distros 2>/dev/null || log "rosdep update failed, continuing without it"
fi

# Create improved test world for Gazebo with better performance
log "Creating optimized Gazebo test world..."
mkdir -p ${HOME}/gazebo_worlds
cat > ${HOME}/gazebo_worlds/minimal.sdf << 'WORLD_EOF'
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="minimal_world">
    <physics name="fast_physics" type="ode">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>50</real_time_update_rate>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    
    <gui fullscreen="0">
      <plugin filename="GzScene3D" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>6 0 6 0 0.5 3.14</camera_pose>
      </plugin>
    </gui>
    
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
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
WORLD_EOF

chown -R jovyan:jovyan ${HOME}/gazebo_worlds

# CRITICAL: Create a wrapper script for running Gazebo with proper environment
log "Creating Gazebo wrapper script..."
cat > /home/jovyan/run_gazebo.sh << 'GAZEBO_SCRIPT'
#!/bin/bash
# Gazebo wrapper script with proper environment setup

export DISPLAY=:1
export XDG_RUNTIME_DIR=/tmp/runtime-jovyan
export HOME=/home/jovyan
export USER=jovyan

# CPU-only rendering
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe
export LP_NUM_THREADS=2

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Ensure runtime directory exists and has correct permissions
mkdir -p /tmp/runtime-jovyan
sudo chown jovyan:jovyan /tmp/runtime-jovyan
sudo chmod 700 /tmp/runtime-jovyan

echo "Starting Gazebo with CPU-only rendering..."
echo "Runtime directory: $XDG_RUNTIME_DIR"
echo "Display: $DISPLAY"

# Run Gazebo
if [ "$1" ]; then
    gz sim "$1"
else
    gz sim /home/jovyan/gazebo_worlds/minimal.sdf
fi
GAZEBO_SCRIPT

chmod +x /home/jovyan/run_gazebo.sh
chown jovyan:jovyan /home/jovyan/run_gazebo.sh

# CRITICAL FIX: Wait a bit before starting JupyterHub to ensure all services are stable
log "Waiting for system stabilization..."
sleep 5

# Start JupyterHub with FIXED configuration
log "Starting JupyterHub with FIXED configuration..."
cd /srv/jupyterhub

# CRITICAL: Set the correct environment for JupyterHub
export JUPYTERHUB_API_URL="http://127.0.0.1:8081/hub/api"
export JUPYTERHUB_BASE_URL="/hub/"

# Start JupyterHub with improved error handling
log "Launching JupyterHub..."
${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py --log-level=INFO > /srv/jupyterhub/jupyterhub.log 2>&1 &
JUPYTER_PID=$!

# Wait for JupyterHub to be ready with better timeout handling
log "Waiting for JupyterHub (this may take a few minutes)..."
for i in {1..60}; do
    if ! kill -0 $JUPYTER_PID 2>/dev/null; then
        log "❌ JupyterHub process died! Checking logs..."
        if [ -f /srv/jupyterhub/jupyterhub.log ]; then
            log "Recent JupyterHub logs:"
            tail -20 /srv/jupyterhub/jupyterhub.log
        fi
        exit 1
    fi
    
    if check_port 8000 1 1; then
        if curl -s -I http://localhost:8000/hub/health | grep -q "200\|302" 2>/dev/null; then
            log "✓ JupyterHub ready (health check passed)!"
            break
        elif curl -s -I http://localhost:8000/ | grep -q "200\|302" 2>/dev/null; then
            log "✓ JupyterHub ready (root check passed)!"
            break
        fi
    fi
    
    if [ $i -eq 60 ]; then
        log "❌ JupyterHub failed to start within timeout"
        log "JupyterHub process status: $(ps aux | grep jupyterhub | grep -v grep || echo 'No process found')"
        log "Port 8000 status: $(check_port 8000 1 1 && echo 'Open' || echo 'Closed')"
        if [ -f /srv/jupyterhub/jupyterhub.log ]; then
            log "Recent JupyterHub logs:"
            tail -50 /srv/jupyterhub/jupyterhub.log
        fi
        break
    elif [ $((i % 10)) -eq 0 ]; then
        log "Still waiting for JupyterHub... (attempt $i/60)"
        if [ -f /srv/jupyterhub/jupyterhub.log ]; then
            log "Latest log entries:"
            tail -3 /srv/jupyterhub/jupyterhub.log
        fi
    fi
    sleep 5
done

# Get server IP for display
SERVER_IP=$(curl -s ifconfig.me 2>/dev/null || curl -s ipinfo.io/ip 2>/dev/null || echo "23.88.34.151")

log "=== FIXED Runtime Directory VPS Environment Ready ==="
echo "🌐 Access URLs:"
echo "   JupyterHub: http://${SERVER_IP}:8000 (login: jovyan/password)"
echo "   GUI Desktop: http://${SERVER_IP}:6080/vnc.html"
echo ""
echo "🧪 Test Commands (in GUI terminal):"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   rviz2"
echo "   /home/jovyan/run_gazebo.sh"
echo "   gz sim /home/jovyan/gazebo_worlds/minimal.sdf"

# Enhanced monitoring with automatic recovery and better error handling
log "Starting enhanced service monitoring..."
while true; do
    sleep 60
    
    # Check and fix runtime directory permissions periodically
    if [ ! -d "/tmp/runtime-jovyan" ] || [ "$(stat -c %U /tmp/runtime-jovyan)" != "jovyan" ]; then
        log "Fixing runtime directory permissions..."
        mkdir -p /tmp/runtime-jovyan
        chown jovyan:jovyan /tmp/runtime-jovyan
        chmod 700 /tmp/runtime-jovyan
    fi
    
    # Check Xvfb
    if ! pgrep -f "Xvfb :1" > /dev/null; then
        log "Restarting Xvfb..."
        pkill -f "Xvfb :1" || true
        sleep 2
        Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
    fi
    
    # Check x11vnc
    if ! pgrep -f x11vnc > /dev/null; then
        log "Restarting VNC..."
        pkill -f x11vnc || true
        sleep 2
        if DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then
            x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared -nolookup -noipv6 &
        fi
    fi
    
    # Check noVNC
    if ! pgrep -f "novnc_proxy" > /dev/null; then
        log "Restarting noVNC..."
        pkill -f novnc_proxy || true
        sleep 2
        cd /opt/novnc && ./utils/novnc_proxy --vnc localhost:5900 --listen 6080 &
    fi
    
    # Check JupyterHub with better health check
    if ! pgrep -f jupyterhub > /dev/null; then
        log "JupyterHub process missing, restarting..."
        pkill -f jupyterhub || true
        pkill -f configurable-http-proxy || true
        sleep 5
        cd /srv/jupyterhub && ${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py --log-level=INFO > /srv/jupyterhub/jupyterhub.log 2>&1 &
    elif ! curl -s http://localhost:8000/hub/health > /dev/null 2>&1; then
        log "JupyterHub health check failed, restarting..."
        pkill -f jupyterhub || true
        pkill -f configurable-http-proxy || true
        sleep 5
        cd /srv/jupyterhub && ${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py --log-level=INFO > /srv/jupyterhub/jupyterhub.log 2>&1 &
    fi
    
    # Health status (only log every 10 minutes to reduce log spam)
    if [ $(($(date +%s) % 600)) -lt 60 ]; then
        log "Services status:"
        echo "  Xvfb: $(pgrep -f 'Xvfb :1' > /dev/null && echo '✓' || echo '❌')"
        echo "  VNC: $(pgrep -f x11vnc > /dev/null && echo '✓' || echo '❌')"
        echo "  noVNC: $(pgrep -f novnc_proxy > /dev/null && echo '✓' || echo '❌')"
        echo "  JupyterHub: $(pgrep -f jupyterhub > /dev/null && echo '✓' || echo '❌')"
        echo "  Hub Health: $(curl -s http://localhost:8000/hub/health > /dev/null 2>&1 && echo '✓' || echo '❌')"
        echo "  Port 8000: $(check_port 8000 1 1 && echo '✓' || echo '❌')"
        echo "  Runtime Dir: $([ -d '/tmp/runtime-jovyan' ] && [ "$(stat -c %U /tmp/runtime-jovyan)" = "jovyan" ] && echo '✓' || echo '❌')"
    fi
done