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

# Start Xvfb
log "Starting display server..."
### FIX #1: Use the VNC_RESOLUTION variable, defaulting to 1920x1080
Xvfb :1 -screen 0 ${VNC_RESOLUTION:-1920x1080}x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!

# Wait for Xvfb to be ready
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
log "Starting window manager as user 'jovyan'..."
### FIX #2: Run the entire graphical session as the correct 'jovyan' user
sudo -u jovyan DISPLAY=:1 openbox &
sleep 2

### FIX #3: Apply the XTerm settings from the .Xresources file for the 'jovyan' user
log "Applying XTerm settings..."
sudo -u jovyan DISPLAY=:1 xrdb -merge /home/jovyan/.Xresources

# Start VNC server
log "Starting VNC server..."
x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared -nolookup -noipv6 &
VNC_PID=$!

# Wait for VNC to be ready
log "Waiting for VNC server..."
if check_port 5900 15 2; then
    log "✓ VNC server ready on port 5900"
else
    log "❌ VNC server failed to start on port 5900"
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
    exit 1
fi

# Source ROS2 environment
log "Setting up ROS2 environment..."
source /opt/ros/jazzy/setup.bash || log "Warning: ROS2 setup issues, continuing..."

# Start JupyterHub
log "Starting JupyterHub..."
cd /srv/jupyterhub
${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py --log-level=INFO > /srv/jupyterhub/jupyterhub.log 2>&1 &
JUPYTER_PID=$!

# Wait for JupyterHub to be ready
log "Waiting for JupyterHub (this may take a few minutes)..."
if ! check_port 8000 60 5; then
    log "❌ JupyterHub failed to start within timeout. Checking logs..."
    tail -50 /srv/jupyterhub/jupyterhub.log
    exit 1
fi
log "✓ JupyterHub ready!"

log "=== All services are running. Monitoring for failures. ==="
# Enhanced monitoring with automatic recovery and better error handling
while true; do
    sleep 60
    
    # Check Xvfb
    if ! pgrep -f "Xvfb :1" > /dev/null; then
        log "Restarting Xvfb..."
        pkill -f "Xvfb :1" || true
        sleep 2
        ### FIX #1 (Repeated): Use the VNC_RESOLUTION variable in the monitoring loop too
	    Xvfb :1 -screen 0 ${VNC_RESOLUTION:-1920x1080}x24 -ac +extension GLX +render -noreset &
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
    
    # Check JupyterHub
    if ! pgrep -f jupyterhub > /dev/null; then
        log "JupyterHub process missing, restarting..."
        pkill -f jupyterhub || true
        pkill -f configurable-http-proxy || true
        sleep 5
        cd /srv/jupyterhub && ${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py --log-level=INFO > /srv/jupyterhub/jupyterhub.log 2>&1 &
    fi
done
