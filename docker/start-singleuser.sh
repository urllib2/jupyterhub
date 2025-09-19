#!/bin/bash
set -e

# Single User Container - Fixed Desktop Setup (updated for ROS env propagation)
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
sudo rm -rf /home/jovyan/notebooks /home/jovyan/ros2_ws /home/jovyan/shared_workspace 2>/dev/null || true
log "✅ Unwanted folders removed"

# --- Compute unique ROS_DOMAIN_ID early (BEFORE launching GUI processes) ---
# Use UID-based deterministic mapping (adjust BASE and RANGE to your needs)
BASE_DOMAIN=${ROS_DOMAIN_BASE:-100}
RANGE=${ROS_DOMAIN_RANGE:-200}

# if JUPYTERHUB_USER present, prefer it; otherwise use uid
if [ -n "${JUPYTERHUB_USER:-}" ]; then
  checksum=$(printf '%s' "$JUPYTERHUB_USER" | cksum | awk '{print $1}')
else
  checksum=$(id -u)
fi

ROS_DOMAIN_ID=$(( BASE_DOMAIN + (checksum % RANGE) ))
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

log "✅ Computed ROS_DOMAIN_ID=$ROS_DOMAIN_ID (base=${BASE_DOMAIN}, range=${RANGE})"
log "✅ Using RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

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

# --- 3. Start Display Server (as jovyan, with ROS env passed) ---
log "Starting Xvfb display server on :1 (as ${USER})..."
sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    DISPLAY=${DISPLAY} bash -lc "Xvfb ${DISPLAY} -screen 0 1920x1080x24 -ac +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 & echo \$! > /tmp/xvfb.pid"

# Give a moment for it to start
sleep 4

# Verify display is working (run check also as jovyan with the env)
for i in {1..10}; do
    if sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" DISPLAY=${DISPLAY} \
          bash -lc "xdpyinfo" >/dev/null 2>&1; then
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

# --- 4. Start Window Manager (Openbox) as jovyan with env ---
log "Starting OpenBox window manager..."
sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    DISPLAY=${DISPLAY} bash -lc "openbox-session >/tmp/openbox.log 2>&1 & echo \$! > /tmp/openbox.pid"
sleep 3

# Load X resources if available (run as jovyan)
if [ -f "/home/jovyan/.Xresources" ]; then
    log "Loading X resources..."
    sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
        DISPLAY=${DISPLAY} bash -lc "xrdb -merge /home/jovyan/.Xresources" || true
fi

# --- 5. Start VNC Server (x11vnc) as jovyan with env ---
log "Starting VNC server on port 5900..."
sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    DISPLAY=${DISPLAY} bash -lc "x11vnc -display ${DISPLAY} -nopw -listen 0.0.0.0 -xkb -rfbport 5900 -forever -shared >/tmp/x11vnc.log 2>&1 & echo \$! > /tmp/x11vnc.pid"
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

# --- 6. Start noVNC (as jovyan, with env) ---
log "Starting noVNC web interface on port $DESKTOP_PORT..."
cd /opt/novnc
sudo -u jovyan env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    bash -lc "./utils/novnc_proxy --vnc localhost:5900 --listen ${DESKTOP_PORT} --web /opt/novnc >/tmp/novnc.log 2>&1 & echo \$! > /tmp/novnc.pid"
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

# --- 6-bis. invisible Qt/X11 warm-up to avoid first-launch distortion ---
log "Warming up X11/Qt pipeline ..."
sudo -u jovyan env LIBGL_ALWAYS_SOFTWARE=1 DISPLAY=:1 bash -lc \
  'ros2 run turtlesim turtlesim_node __node:=warmup & WP=$!; sleep 1; kill $WP 2>/dev/null; wait $WP 2>/dev/null' || true
log "✅ Warm-up done"

# ensure CPU-only rendering for every GUI process
sudo -u jovyan bash -lc "echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> /home/jovyan/.bashrc"

# --- 7. ROS2 Environment Setup for interactive shells (and duplicated here) ---
log "Setting up ROS2 environment for interactive shells..."
# Put exports into user's .bashrc so terminals pick them up
if ! sudo -u jovyan grep -q "source /opt/ros/jazzy/setup.bash" /home/jovyan/.bashrc 2>/dev/null; then
    sudo -u jovyan bash -lc "echo 'source /opt/ros/jazzy/setup.bash' >> /home/jovyan/.bashrc"
fi

# Ensure the unique domain and RMW are set in .bashrc too
sudo -u jovyan bash -lc "sed -i '/export ROS_DOMAIN_ID=/d' /home/jovyan/.bashrc || true"
sudo -u jovyan bash -lc "sed -i '/export RMW_IMPLEMENTATION=/d' /home/jovyan/.bashrc || true"
sudo -u jovyan bash -lc "echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}' >> /home/jovyan/.bashrc"
sudo -u jovyan bash -lc "echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}' >> /home/jovyan/.bashrc"

log "✅ ROS2 setup added to /home/jovyan/.bashrc"

# --- 8. Cleanup Handler ---
cleanup() {
    log "Shutting down desktop services..."
    # kill by pid files if exist
    for pidfile in /tmp/novnc.pid /tmp/xvfb.pid /tmp/x11vnc.pid /tmp/openbox.pid; do
        if [ -f "$pidfile" ]; then
            pid=$(cat "$pidfile" 2>/dev/null)
            kill "$pid" 2>/dev/null || true
            rm -f "$pidfile"
        fi
    done
    sudo pkill -f x11vnc || true
    sudo pkill -f openbox || true
    exit 0
}
trap cleanup SIGTERM SIGINT

# --- 9. Final Status Check ---
log "=== Service Status ==="
log "Display Server: $(if sudo -u jovyan env ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID}\" RMW_IMPLEMENTATION=\"${RMW_IMPLEMENTATION}\" DISPLAY=${DISPLAY} bash -lc 'xdpyinfo' >/dev/null 2>&1; then echo 'Running'; else echo 'Failed'; fi)"
log "VNC Server: $(if netstat -ln | grep -q ':5900'; then echo 'Running'; else echo 'Failed'; fi)"
log "noVNC Web: $(if netstat -ln | grep -q \":$DESKTOP_PORT\"; then echo 'Running'; else echo 'Failed'; fi)"

# --- 10. Start JupyterLab (still as the current user; your environment already prepared) ---
log "Starting JupyterLab..."
cd /home/jovyan

exec ${VENV_PATH}/bin/jupyterhub-singleuser \
    --ip=0.0.0.0 \
    --port=8888 \
    --allow-root \
    --notebook-dir=/home/jovyan \
    --debug

