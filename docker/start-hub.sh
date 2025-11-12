#!/bin/bash
set -e

# OPTIMIZED Hub Startup - NO GUI SERVICES NEEDED
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH
# CRITICAL FIX: Set HOME to root to avoid path confusion
export HOME=/root

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] HUB: $1"
}

log "=== Starting JupyterHub Hub (Control Plane) ==="

# CRITICAL FIX: Ensure /srv/jupyterhub/data exists and has correct permissions
log "Setting up data directory..."
mkdir -p /srv/jupyterhub/data

# JupyterHub needs to run as root to spawn Docker containers
chown -R root:root /srv/jupyterhub/data
chmod 700 /srv/jupyterhub/data

# Fix cookie secret file if it exists
if [ -f "/srv/jupyterhub/data/jupyterhub_cookie_secret" ]; then
    chmod 600 /srv/jupyterhub/data/jupyterhub_cookie_secret
    chown root:root /srv/jupyterhub/data/jupyterhub_cookie_secret
fi

# CRITICAL: Remove the conflicting directory that confuses JupyterHub
if [ -d "/home/jovyan/srv/jupyterhub" ]; then
    log "Removing conflicting directory /home/jovyan/srv/jupyterhub"
    rm -rf /home/jovyan/srv/jupyterhub
fi

# CRITICAL: Change to /srv/jupyterhub directory before starting
cd /srv/jupyterhub

# Start JupyterHub as root (required for Docker spawning)
log "Starting JupyterHub control plane from $(pwd)"
exec ${VENV_PATH}/bin/jupyterhub -f /srv/jupyterhub/jupyterhub_config.py
