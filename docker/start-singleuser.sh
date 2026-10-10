#!/bin/bash
set -e

export HOME=/home/jovyan
export USER=jovyan
export VENV_PATH=/opt/venv
export PATH=${VENV_PATH}/bin:/usr/local/bin:$PATH

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "=== ROSFORGE JupyterHub Single User Startup ==="

# Setup directories
sudo mkdir -p /home/jovyan/temporary_ws /tmp
sudo chown jovyan:jovyan /home/jovyan/temporary_ws
sudo chmod 755 /home/jovyan/temporary_ws
sudo chmod 1777 /tmp

# Clean old artifacts
sudo rm -rf /home/jovyan/notebooks /home/jovyan/work /home/jovyan/shared_workspace 2>/dev/null || true

# Compute unique ROS_DOMAIN_ID per student
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

# Configure ROS environment
if ! sudo -u jovyan grep -q "source /opt/ros/jazzy/setup.bash" /home/jovyan/.bashrc 2>/dev/null; then
    sudo -u jovyan bash -c "echo 'source /opt/ros/jazzy/setup.bash' >> /home/jovyan/.bashrc"
fi
sudo -u jovyan bash -c "sed -i '/export ROS_DOMAIN_ID=/d; /export RMW_IMPLEMENTATION=/d' /home/jovyan/.bashrc"
sudo -u jovyan bash -c "cat >> /home/jovyan/.bashrc << BASHRC
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
BASHRC"

log "Starting JupyterLab..."
cd /home/jovyan
exec ${VENV_PATH}/bin/jupyterhub-singleuser \
    --ip=0.0.0.0 \
    --port=8888 \
    --allow-root \
    --notebook-dir=/home/jovyan \
    --debug
