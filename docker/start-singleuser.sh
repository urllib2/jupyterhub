#!/bin/bash
set -e

# Single User Container - Fixed Desktop Setup
export HOME=/home/jovyan
export USER=jovyan
export DISPLAY=:1
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH
export XDG_RUNTIME_DIR=/run/user/1000
export LIBGL_ALWAYS_SOFTWARE=1

# Use environment variable for desktop port (default 6080)
DESKTOP_PORT=${DESKTOP_PORT:-6080}

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] DESKTOP: $1"
}

log "=== Starting Single User Container with Desktop ==="
log "Container: $(hostname)"
log "User: $USER"
log "Desktop Port: $DESKTOP_PORT"

# --- 0. Setup User Workspace (NEW) ---
log "Setting up user workspace..."
sudo mkdir -p /home/jovyan/work
sudo chown jovyan:jovyan /home/jovyan/work
sudo chmod 755 /home/jovyan/work
log "✅ User workspace created at /home/jovyan/work"

# --- NEW: Clean up unwanted folders ---
log "Cleaning up unwanted folders..."
sudo rm -rf /home/jovyan/notebooks /home/jovyan/ros2_ws /home/jovyan/shared_workspace /home/jovyan/work 2>/dev/null || true
log "✅ Unwanted folders removed"

# --- 1. Setup Runtime Environment ---
log "Setting up runtime directories..."
sudo mkdir -p /run/user/1000 /tmp/.X11-unix
sudo chown jovyan:jovyan /run/user/1000
sudo chmod 700 /run/user/1000
sudo chmod 1777 /tmp/.X11-unix

# --- 2. Cleanup Existing Services ---
log "Cleaning up existing services..."
sudo pkill -f "Xvfb" || true
sudo pkill -f "x11vnc" || true  
sudo pkill -f "novnc" || true
sudo pkill -f "openbox" || true
sudo rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
sleep 2

# --- 3. Start Display Server ---
log "Starting Xvfb display server on :1..."
sudo -u jovyan Xvfb :1 -screen 0 1920x1080x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!
sleep 5

# Verify display is working
for i in {1..10}; do
    if sudo -u jovyan DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then
        log "✅ Display server is ready"
        break
    fi
    log "Waiting for display server... ($i/10)"
    sleep 2
    if [ $i -eq 10 ]; then
        log "❌ Display server failed to start"
        exit 1
    fi
done

# --- 4. Start Window Manager ---
log "Starting OpenBox window manager..."
sudo -u jovyan DISPLAY=:1 openbox-session &
sleep 3

# Load X resources if available
if [ -f "/home/jovyan/.Xresources" ]; then
    log "Loading X resources..."
    sudo -u jovyan DISPLAY=:1 xrdb -merge /home/jovyan/.Xresources || true
fi

# --- 5. Start VNC Server ---
log "Starting VNC server on port 5900..."
sudo -u jovyan x11vnc -display :1 -nopw -listen 0.0.0.0 -xkb -rfbport 5900 -forever -shared -bg
sleep 3

# Verify VNC server
for i in {1..5}; do
    if netstat -ln | grep -q ":5900"; then
        log "✅ VNC server is ready"
        break
    fi
    log "Waiting for VNC server... ($i/5)"
    sleep 2
    if [ $i -eq 5 ]; then
        log "❌ VNC server failed to start"
        exit 1
    fi
done

# --- 6. Start noVNC ---
log "Starting noVNC web interface on port $DESKTOP_PORT..."
cd /opt/novnc
sudo -u jovyan ./utils/novnc_proxy --vnc localhost:5900 --listen $DESKTOP_PORT --web /opt/novnc &
NOVNC_PID=$!
sleep 5

# Verify noVNC
for i in {1..5}; do
    if netstat -ln | grep -q ":$DESKTOP_PORT"; then
        log "✅ noVNC web interface is ready on port $DESKTOP_PORT"
        break
    fi
    log "Waiting for noVNC... ($i/5)"
    sleep 2
    if [ $i -eq 5 ]; then
        log "❌ noVNC failed to start on port $DESKTOP_PORT"
    fi
done

# --- 7. ROS2 Environment Setup ---
log "Setting up ROS2 environment..."
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

# --- 8. Cleanup Handler ---
cleanup() {
    log "Shutting down desktop services..."
    kill $NOVNC_PID 2>/dev/null || true
    kill $XVFB_PID 2>/dev/null || true
    sudo pkill -f x11vnc || true
    sudo pkill -f openbox || true
    exit 0
}
trap cleanup SIGTERM SIGINT

# --- 9. Final Status Check ---
log "=== Service Status ==="
log "Display Server: $(if sudo -u jovyan DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then echo 'Running'; else echo 'Failed'; fi)"
log "VNC Server: $(if netstat -ln | grep -q ':5900'; then echo 'Running'; else echo 'Failed'; fi)"
log "noVNC Web: $(if netstat -ln | grep -q ":$DESKTOP_PORT"; then echo 'Running'; else echo 'Failed'; fi)"

# --- 10. Start JupyterLab ---
log "Starting JupyterLab..."
cd /home/jovyan

# Add ROS2 setup to bashrc if not already there
if ! grep -q "source /opt/ros/jazzy/setup.bash" /home/jovyan/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> /home/jovyan/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> /home/jovyan/.bashrc
fi

exec ${VENV_PATH}/bin/jupyterhub-singleuser \
    --ip=0.0.0.0 \
    --port=8888 \
    --allow-root \
    --notebook-dir=/home/jovyan \
    --debug
