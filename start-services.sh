#!/bin/bash
set -e

# --- Final, STABLE Startup Script v4 ---
export HOME=/home/jovyan
export USER=jovyan
export DISPLAY=:1
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH
export XDG_RUNTIME_DIR=/tmp/runtime-jovyan
export LIBGL_ALWAYS_SOFTWARE=1

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "=== Initializing Services ==="

# --- 1. Cleanup ---
log "Cleaning up stale lock files..."
rm -f /tmp/.X1-lock

# --- 2. Permissions ---
log "Setting up permissions..."
mkdir -p /srv/jupyterhub/data
chown -R jovyan:jovyan /home/jovyan
chmod 700 /srv/jupyterhub/data

# --- 3. Start Background GUI Services ---
log "Starting display server (Xvfb)..."
Xvfb :1 -screen 0 1920x1080x24 -ac +extension GLX +render -noreset &
sleep 2

log "Starting window manager (Openbox)..."
sudo -u jovyan DISPLAY=:1 openbox &
sleep 1

log "Applying XTerm settings..."
if [ -f "/home/jovyan/.Xresources" ]; then
    sudo -u jovyan DISPLAY=:1 xrdb -merge /home/jovyan/.Xresources
fi

log "Starting VNC and noVNC proxies..."
x11vnc -display :1 -nopw -listen localhost -xkb -rfbport 5900 -forever -shared &
/opt/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &

# --- 4. Start JupyterHub as the Main Process ---
log "Starting JupyterHub..."
exec env JUPYTERHUB_CRYPT_KEY=$(openssl rand -hex 32) \
    /opt/venv/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py
