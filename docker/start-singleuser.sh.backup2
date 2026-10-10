#!/bin/bash
set -e

# ULTRA-OPTIMIZED Single User Container - Background GUI startup
export HOME=/home/jovyan
export USER=jovyan
export DISPLAY=:1
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH
export XDG_RUNTIME_DIR=/run/user/1000
export LIBGL_ALWAYS_SOFTWARE=1

DESKTOP_PORT=${DESKTOP_PORT:-6080}

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] USER: $1"
}

log "=== Fast Single User Container Startup ==="

# Quick environment setup (< 1 second)
log "Setting up environment..."
sudo mkdir -p /home/jovyan/temporary_ws /run/user/1000 /tmp/.X11-unix
sudo chown jovyan:jovyan /home/jovyan/temporary_ws /run/user/1000
sudo chmod 755 /home/jovyan/temporary_ws
sudo chmod 700 /run/user/1000
sudo chmod 1777 /tmp/.X11-unix

# Remove any legacy directories (including old 'work' folder)
sudo rm -rf /home/jovyan/notebooks /home/jovyan/work /home/jovyan/shared_workspace 2>/dev/null || true

# Compute unique ROS_DOMAIN_ID based on username
BASE_DOMAIN=${ROS_DOMAIN_BASE:-100}
RANGE=${ROS_DOMAIN_RANGE:-200}

if [ -n "${JUPYTERHUB_USER:-}" ]; then
    checksum=$(printf '%s' "$JUPYTERHUB_USER" | cksum | awk '{print $1}')
else
    checksum=$(id -u)
fi

ROS_DOMAIN_ID=$(( BASE_DOMAIN + (checksum % RANGE) ))
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_DOMAIN_ID RMW_IMPLEMENTATION

log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, RMW=${RMW_IMPLEMENTATION}"

# Aggressive cleanup
sudo pkill -f "Xvfb|x11vnc|novnc|openbox" 2>/dev/null || true
sudo rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# KEY OPTIMIZATION: Start all GUI services in background (don't wait)
log "Starting GUI services in background..."
(
    # Start Xvfb
    sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
        DISPLAY=${DISPLAY} \
        bash -c "Xvfb ${DISPLAY} -screen 0 1920x1080x24 -ac +extension GLX +render -noreset" &
    
    sleep 1
    
    # Start remaining GUI services in parallel
    sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
        DISPLAY=${DISPLAY} \
        bash -c "openbox-session" &
    sleep 2

    sudo -u jovyan env DISPLAY=${DISPLAY} \
        bash -c "tint2" &
    
    if [ -f "/home/jovyan/.Xresources" ]; then
        sudo -u jovyan env DISPLAY=${DISPLAY} \
            bash -c "xrdb -merge /home/jovyan/.Xresources" &
    fi
    
    sudo -u jovyan env DISPLAY=${DISPLAY} \
        bash -c "x11vnc -display ${DISPLAY} -nopw -listen 0.0.0.0 -xkb -rfbport 5900 -forever -shared" &
    
    sleep 0.5
    
    cd /opt/novnc
    sudo -u jovyan bash -c \
        "./utils/novnc_proxy --vnc localhost:5900 --listen ${DESKTOP_PORT} --web /opt/novnc" &
    
    log "GUI services started in background (will be ready in ~5-10 seconds)"
) &

# Configure ROS environment in .bashrc (parallel with GUI startup)
(
    if ! sudo -u jovyan grep -q "source /opt/ros/jazzy/setup.bash" /home/jovyan/.bashrc 2>/dev/null; then
        sudo -u jovyan bash -c "echo 'source /opt/ros/jazzy/setup.bash' >> /home/jovyan/.bashrc"
    fi
    
    sudo -u jovyan bash -c "sed -i '/export ROS_DOMAIN_ID=/d; /export RMW_IMPLEMENTATION=/d' /home/jovyan/.bashrc"
    
    sudo -u jovyan bash -c "cat >> /home/jovyan/.bashrc << EOF
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
export LIBGL_ALWAYS_SOFTWARE=1
EOF"
) &

# CRITICAL: Start JupyterLab IMMEDIATELY (don't wait for GUI)
log "Starting JupyterLab (GUI will be ready shortly)..."
cd /home/jovyan

exec ${VENV_PATH}/bin/jupyterhub-singleuser \
    --ip=0.0.0.0 \
    --port=8888 \
    --allow-root \
    --notebook-dir=/home/jovyan \
    --debug
