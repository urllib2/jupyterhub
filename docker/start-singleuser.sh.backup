# REAL BOTTLENECK: The X11/VNC startup sequence in start-singleuser.sh
# Current: 24+ seconds total (mostly from GUI startup)
# Target: 8-12 seconds

# Replace the entire start-singleuser.sh with this optimized version:

#!/bin/bash
set -e

# DRAMATICALLY OPTIMIZED Single User Container - BACKGROUND GUI startup
export HOME=/home/jovyan
export USER=jovyan
export DISPLAY=:1
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH
export XDG_RUNTIME_DIR=/run/user/1000
export LIBGL_ALWAYS_SOFTWARE=1

DESKTOP_PORT=${DESKTOP_PORT:-6080}

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] DESKTOP: $1"
}

log "=== ULTRA-FAST Single User Container Startup ==="

# IMMEDIATE setup (1 second total)
log "Quick environment setup..."
sudo mkdir -p /home/jovyan/work /run/user/1000 /tmp/.X11-unix
sudo chown jovyan:jovyan /home/jovyan/work /run/user/1000
sudo chmod 755 /home/jovyan/work
sudo chmod 700 /run/user/1000
sudo chmod 1777 /tmp/.X11-unix
sudo rm -rf /home/jovyan/notebooks /home/jovyan/ros2_ws /home/jovyan/shared_workspace 2>/dev/null || true

# ROS domain computation
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

log "✅ Environment ready (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"

# AGGRESSIVE cleanup
sudo pkill -f "Xvfb|x11vnc|novnc|openbox" 2>/dev/null || true
sudo rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# THE KEY OPTIMIZATION: Start GUI services in BACKGROUND and don't wait
log "Starting GUI services in background..."
{
    # Start Xvfb
    sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
        DISPLAY=${DISPLAY} bash -c "Xvfb ${DISPLAY} -screen 0 1920x1080x24 -ac +extension GLX +render -noreset" &
    
    sleep 1  # Minimal wait for display
    
    # Start all other GUI services in parallel (no waiting)
    sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
        DISPLAY=${DISPLAY} bash -c "openbox-session" &
    
    [ -f "/home/jovyan/.Xresources" ] && \
    sudo -u jovyan env DISPLAY=${DISPLAY} bash -c "xrdb -merge /home/jovyan/.Xresources" &
    
    sudo -u jovyan env DISPLAY=${DISPLAY} bash -c \
        "x11vnc -display ${DISPLAY} -nopw -listen 0.0.0.0 -xkb -rfbport 5900 -forever -shared" &
    
    sleep 0.5  # Minimal VNC startup delay
    cd /opt/novnc
    sudo -u jovyan bash -c \
        "./utils/novnc_proxy --vnc localhost:5900 --listen ${DESKTOP_PORT} --web /opt/novnc" &
    
    log "✅ GUI services started in background"
} &

# ROS environment setup (parallel with GUI)
{
    if ! sudo -u jovyan grep -q "source /opt/ros/jazzy/setup.bash" /home/jovyan/.bashrc 2>/dev/null; then
        sudo -u jovyan bash -c "echo 'source /opt/ros/jazzy/setup.bash' >> /home/jovyan/.bashrc"
    fi
    sudo -u jovyan bash -c "sed -i '/export ROS_DOMAIN_ID=/d; /export RMW_IMPLEMENTATION=/d' /home/jovyan/.bashrc"
    sudo -u jovyan bash -c "echo -e 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}\nexport RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}\nexport LIBGL_ALWAYS_SOFTWARE=1' >> /home/jovyan/.bashrc"
} &

# NO WAITING - Start JupyterLab immediately while GUI initializes in background
log "Starting JupyterLab immediately (GUI will be ready in ~10-15 seconds)..."
cd /home/jovyan

exec ${VENV_PATH}/bin/jupyterhub-singleuser \
    --ip=0.0.0.0 \
    --port=8888 \
    --allow-root \
    --notebook-dir=/home/jovyan \
    --debug
