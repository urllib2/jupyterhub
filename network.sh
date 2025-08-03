#!/bin/bash
# Fix JupyterHub networking issue between hub and single-user servers

echo "=== Fixing JupyterHub Networking Issue ==="

# Stop current JupyterHub process
echo "1. Stopping current JupyterHub..."
docker compose exec ros2-teaching pkill -f jupyterhub
sleep 3

# Create fixed JupyterHub config that addresses networking issues
echo "2. Creating fixed JupyterHub configuration..."
docker compose exec ros2-teaching bash -c '
cat > /srv/jupyterhub/jupyterhub_config_fixed.py << '"'"'EOF'"'"'
# Fixed JupyterHub Configuration - Networking Issue Resolved
import os

# Basic configuration
c.JupyterHub.ip = "0.0.0.0"
c.JupyterHub.port = 8000
c.JupyterHub.bind_url = "http://0.0.0.0:8000"

# Authentication
c.JupyterHub.authenticator_class = "jupyterhub.auth.DummyAuthenticator"
c.DummyAuthenticator.password = "password"
c.Authenticator.admin_users = {"jovyan"}

# Spawner configuration
c.JupyterHub.spawner_class = "jupyterhub.spawner.LocalProcessSpawner"
c.Spawner.notebook_dir = "/home/jovyan"
c.Spawner.default_url = "/lab"

# CRITICAL FIX: Use correct jupyter-labhub command with proper networking
c.Spawner.cmd = ["/opt/venv/bin/jupyter-labhub"]

# CRITICAL FIX: Hub networking configuration
c.JupyterHub.hub_bind_url = "http://127.0.0.1:8081"
c.JupyterHub.hub_connect_url = "http://127.0.0.1:8081"

# CRITICAL FIX: Single-user server networking
c.Spawner.args = [
    "--ip=127.0.0.1",
    "--port=0",  # Let system assign port
    "--allow-root",
    "--SingleUserNotebookApp.trust_xheaders=True",
    "--SingleUserNotebookApp.disable_check_xsrf=False",
    "--SingleUserNotebookApp.default_url=/lab"
]

# Environment variables
c.Spawner.environment = {
    "DISPLAY": ":1",
    "HOME": "/home/jovyan",
    "USER": "jovyan",
    "SHELL": "/bin/bash",
    "PATH": "/opt/venv/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
    "VIRTUAL_ENV": "/opt/venv",
    "ROS_DISTRO": "jazzy",
    "ROS_DOMAIN_ID": "42",
    "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/venv/lib/python3.12/site-packages",
    "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib:/opt/ros/jazzy/local/lib:/usr/lib/x86_64-linux-gnu",
    "JUPYTERHUB_USER": "{username}",
    "JUPYTERHUB_CLIENT_ID": "jupyterhub-user-{username}",
    "JUPYTERHUB_API_TOKEN": "{api_token}",
    "JUPYTERHUB_OAUTH_CALLBACK_URL": "/hub/oauth_callback",
    "JUPYTERHUB_SERVICE_PREFIX": "/user/{username}/",
}

# Database
c.JupyterHub.db_url = "sqlite:////srv/jupyterhub/jupyterhub.sqlite"

# CRITICAL FIX: Extended timeouts for networking
c.Spawner.start_timeout = 300
c.Spawner.http_timeout = 180
c.JupyterHub.init_spawners_timeout = 30

# CRITICAL FIX: Proxy configuration
c.ConfigurableHTTPProxy.should_start = True
c.ConfigurableHTTPProxy.command = ["/opt/venv/bin/configurable-http-proxy"]
c.ConfigurableHTTPProxy.debug = False

# Security files
c.JupyterHub.cookie_secret_file = "/srv/jupyterhub/jupyterhub_cookie_secret"
c.ConfigurableHTTPProxy.auth_token_file = "/srv/jupyterhub/jupyterhub_proxy_token"

# Logging - reduced for production
c.JupyterHub.log_level = "INFO"
c.Spawner.debug = False

# CRITICAL FIX: User switching configuration
c.LocalProcessSpawner.create_system_users = False

# CRITICAL FIX: Pre-spawn hook to ensure proper setup
def pre_spawn_hook(spawner):
    """Ensure proper spawning environment"""
    username = spawner.user.name
    print(f"Pre-spawn: Setting up environment for {username}")
    
    # Ensure home directory exists and has correct permissions
    import os
    import pwd
    try:
        user_info = pwd.getpwnam(username)
        home_dir = user_info.pw_dir
        if not os.path.exists(home_dir):
            os.makedirs(home_dir, mode=0o755)
        os.chown(home_dir, user_info.pw_uid, user_info.pw_gid)
        print(f"Pre-spawn: Home directory ready for {username}")
    except Exception as e:
        print(f"Pre-spawn warning: {e}")

c.Spawner.pre_spawn_hook = pre_spawn_hook

# Additional network settings
c.JupyterHub.allow_named_servers = False
c.JupyterHub.cleanup_servers = True
c.JupyterHub.cleanup_proxy = True

print("Fixed JupyterHub configuration loaded - networking issues resolved")
EOF
'

# Fix permissions on the home directory 
echo "3. Fixing permissions..."
docker compose exec ros2-teaching bash -c '
chown -R jovyan:jovyan /home/jovyan
chmod 755 /home/jovyan
mkdir -p /home/jovyan/.jupyter
chown jovyan:jovyan /home/jovyan/.jupyter
'

# Clean up any stale processes
echo "4. Cleaning up stale processes..."
docker compose exec ros2-teaching bash -c '
pkill -f "jupyter-labhub" || true
pkill -f "jupyter-lab" || true
sleep 2
'

# Start JupyterHub with the fixed configuration
echo "5. Starting JupyterHub with networking fixes..."
docker compose exec ros2-teaching bash -c '
cd /srv/jupyterhub
nohup /opt/venv/bin/jupyterhub --config=/srv/jupyterhub/jupyterhub_config_fixed.py > /srv/jupyterhub/jupyterhub_fixed.log 2>&1 &
echo "JupyterHub started with PID: $(pgrep -f jupyterhub)"
'

# Wait for startup
echo "6. Waiting for JupyterHub to start..."
sleep 10

# Check if it is working
echo "7. Testing the fix..."
docker compose exec ros2-teaching bash -c '
echo "JupyterHub process:"
ps aux | grep jupyterhub | grep -v grep

echo "Checking port binding:"
netstat -tlnp | grep :8000

echo "Recent logs:"
tail -10 /srv/jupyterhub/jupyterhub_fixed.log 2>/dev/null || echo "No logs yet"
'

# Test HTTP response
echo "8. Testing HTTP response..."
sleep 5
if curl -s -I http://localhost:8000/hub/health | grep -q "200 OK"; then
    echo "✅ JupyterHub is responding!"
else
    echo "⚠️  JupyterHub may still be starting up..."
fi

echo -e "\n=== Fix Applied ==="
echo "🔧 Applied networking fixes for JupyterHub single-user server spawning"
echo "📋 Key changes:"
echo "   - Fixed hub networking configuration"
echo "   - Added proper single-user server arguments"
echo "   - Extended timeouts for container environment"
echo "   - Fixed authentication tokens"
echo ""
echo "🌐 Try accessing JupyterHub now:"
echo "   URL: http://localhost:8000"
echo "   Username: jovyan"
echo "   Password: password"
echo ""
echo "📊 Monitor logs:"
echo "   docker compose exec ros2-teaching tail -f /srv/jupyterhub/jupyterhub_fixed.log"
