#!/bin/bash
set -e
export HOME=/home/jovyan
export USER=jovyan
export DISPLAY=:1
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH
export XDG_RUNTIME_DIR=/run/user/1000
export LIBGL_ALWAYS_SOFTWARE=1

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "=== Initializing JupyterHub Hub ==="

# Cleanup
log "Cleaning up stale lock files..."
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# CRITICAL FIX: Set up permissions for JupyterHub data directory
log "Setting up permissions..."
mkdir -p /srv/jupyterhub/data /run/user/1000

# Fix ownership and permissions for JupyterHub data directory
chown -R root:root /srv/jupyterhub/data
chmod 700 /srv/jupyterhub/data

# If cookie secret file exists, fix its permissions
if [ -f "/srv/jupyterhub/data/jupyterhub_cookie_secret" ]; then
    log "Fixing cookie secret file permissions..."
    chmod 600 /srv/jupyterhub/data/jupyterhub_cookie_secret
    chown root:root /srv/jupyterhub/data/jupyterhub_cookie_secret
fi

# Set up jovyan user directories
chown -R jovyan:jovyan /home/jovyan /run/user/1000
chmod 700 /run/user/1000

# Start background GUI services
log "Starting display server (Xvfb)..."
Xvfb :1 -screen 0 1920x1080x24 -ac +extension GLX +render -noreset &
sleep 3

# Wait for display to be ready
log "Waiting for display server to be ready..."
for i in {1..30}; do
    if DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then
        log "Display server ready"
        break
    fi
    sleep 1
done

log "Starting window manager (Openbox)..."
sudo -u jovyan DISPLAY=:1 openbox &
sleep 2

log "Applying XTerm settings..."
if [ -f "/home/jovyan/.Xresources" ]; then
    sudo -u jovyan DISPLAY=:1 xrdb -merge /home/jovyan/.Xresources
fi

log "Starting VNC and noVNC proxies..."
x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared &
/opt/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &

# Wait a moment for services to stabilize
sleep 3

# Start JupyterHub as the main process
log "Starting JupyterHub..."
exec ${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py
