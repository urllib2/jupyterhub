# Multi-User JupyterHub Configuration for Docker Compose v2 - WITH DESKTOP ACCESS (FIXED)
from dockerspawner import DockerSpawner
from oauthenticator import GitHubOAuthenticator
import os

# --- Basic server configuration ---
# Bind the public-facing hub on all interfaces inside the hub container
c.JupyterHub.bind_url = 'http://0.0.0.0:8000'
c.JupyterHub.base_url = '/'
# Hub internal API bind address (what hub process listens on)
c.JupyterHub.hub_bind_url = 'http://0.0.0.0:8081'

# --- GitHub OAuth Configuration ---
c.JupyterHub.authenticator_class = GitHubOAuthenticator
c.GitHubOAuthenticator.oauth_callback_url = 'https://rosforge.com/hub/oauth_callback'
c.GitHubOAuthenticator.client_id = os.environ.get('GITHUB_CLIENT_ID')
c.GitHubOAuthenticator.client_secret = os.environ.get('GITHUB_CLIENT_SECRET')
c.GitHubOAuthenticator.enable_auth_state = True

# Allow authentication from anyone with a GitHub account
c.Authenticator.admin_users = {"urllib2"}

# --- Docker Spawner Configuration for MULTI-USER JupyterHub ---
c.JupyterHub.spawner_class = DockerSpawner

# Image and network for spawned user containers (match actual network name)
c.DockerSpawner.image = 'ros2-teaching-multiuser:latest'
c.DockerSpawner.network_name = 'ros2-teaching_ros2-network'

# Hub connection settings: how spawned containers should reach the Hub
# Use full URL form (supported) instead of hub_connect_port (not recognized).
c.JupyterHub.hub_connect_url = 'http://ros2-teaching-hub:8081'
c.DockerSpawner.hub_ip_connect = 'ros2-teaching-hub'
c.DockerSpawner.hub_connect_url = 'http://ros2-teaching-hub:8081'

# Ensure DockerSpawner uses internal docker networking
c.DockerSpawner.use_internal_ip = True

# Each user container networking - IMPORTANT: Expose desktop port
c.DockerSpawner.host_ip = '0.0.0.0'
c.DockerSpawner.port = 8888

# CRITICAL: Expose desktop port for each user container
c.DockerSpawner.extra_host_config = {
    'port_bindings': {
        6080: None,  # Let Docker assign a random host port for noVNC
        8888: None   # Jupyter
    },
    'auto_remove': True,
    'restart_policy': {'Name': 'no'},
    'shm_size': '512m',
    'init': True,
    'cap_add': ['SYS_NICE'],
}

# Command for EACH spawned user container
c.DockerSpawner.cmd = ['/usr/local/bin/start-singleuser.sh']

# Environment for EACH user container
c.DockerSpawner.environment = {
    'JUPYTER_ENABLE_LAB': '1',
    'GRANT_SUDO': 'yes',
    'CHOWN_HOME': 'yes',
    'USER': 'jovyan',
    'HOME': '/home/jovyan',
    'SHELL': '/bin/bash',
    # ROS2 Environment
    'ROS_DOMAIN_ID': '42',
    'DISPLAY': ':1',
    'LIBGL_ALWAYS_SOFTWARE': '1',
    # Python environment
    'VENV_PATH': '/opt/venv',
    'PATH': '/opt/venv/bin:/usr/local/bin:/usr/bin:/bin',
    'PYTHONPATH': '/opt/venv/lib/python3.12/site-packages',
}

# --- Volume Mounts for proper student workspace isolation ---
c.DockerSpawner.volumes = {
    # Individual user workspace (writable, persistent)
    'jupyterhub-user-{username}': '/home/jovyan/work',
    # Shared read-only course materials
    '/srv/shared_workspace': { "bind": '/home/jovyan/shared_materials', "mode": 'ro' },
}

# Debugging / lifecycle settings
c.DockerSpawner.remove = True

# Use local image, don't pull (you build it locally)
c.DockerSpawner.pull_policy = 'never'

# Resource limits per user
c.DockerSpawner.mem_limit = '2G'  # 2GB RAM per user
c.DockerSpawner.cpu_limit = 1.0   # 1 CPU core per user

# Container user configuration
c.DockerSpawner.extra_create_kwargs = {
    'user': 'jovyan',
    'working_dir': '/home/jovyan',
}

# --- Hub Data Persistence ---
c.JupyterHub.cookie_secret_file = "/srv/jupyterhub/data/jupyterhub_cookie_secret"
c.JupyterHub.db_url = "sqlite:////srv/jupyterhub/data/jupyterhub.sqlite"

# --- Proxy configuration ---
# Trusted downstream IPs (adjust if you add other proxies)
c.JupyterHub.trusted_downstream_ips = ['127.0.0.1', '10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16']

# ConfigurableHTTPProxy auth token (read from env); fallback to a random token if not set.
c.ConfigurableHTTPProxy.auth_token = os.environ.get('JUPYTERHUB_CRYPT_KEY', os.urandom(32).hex())

# --- FIXED Desktop Proxy Service ---
c.JupyterHub.services = [
    {
        'name': 'desktop-proxy',
        'url': 'http://127.0.0.1:9999',
        'command': [
            'python3', '-c', '''
import tornado.web
import tornado.ioloop
import docker
import json
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DesktopHandler(tornado.web.RequestHandler):
    async def get(self, username):
        """Proxy desktop access to user container"""
        logger.info(f"Desktop access requested for user: {username}")
        
        try:
            client = docker.from_env()
            all_containers = client.containers.list()
            logger.info(f"All running containers: {[c.name for c in all_containers]}")
            
            # Try the correct naming patterns based on your config
            container_patterns = [
                f"jupyter-{username}-",      # With trailing dash (your original)
                f"jupyter-{username}",       # Without trailing dash
                f"jupyter-{username}-default", # With default servername
            ]
            
            container = None
            for pattern in container_patterns:
                # Try exact match first
                try:
                    container = client.containers.get(pattern)
                    logger.info(f"Found exact match: {container.name}")
                    break
                except docker.errors.NotFound:
                    pass
                
                # Try containers that start with the pattern
                for c in all_containers:
                    if c.name.startswith(pattern.rstrip("-")):
                        container = c
                        logger.info(f"Found pattern match: {container.name}")
                        break
                
                if container:
                    break
            
            if not container:
                container_list = [c.name for c in all_containers]
                msg = f"No container found for {username}. Available: {container_list}"
                logger.error(msg)
                self.write(f"<h1>Container Not Found</h1><p>{msg}</p>")
                return
            
            # Check container status
            if container.status != "running":
                logger.warning(f"Container {container.name} status: {container.status}")
                self.write(f"<h1>Container Not Running</h1><p>Container {container.name} is not running (status: {container.status})</p>")
                return
            
            # Get container's internal IP from the Docker network
            container.reload()
            network_name = "ros2-teaching_ros2-network"
            
            if network_name in container.attrs["NetworkSettings"]["Networks"]:
                container_ip = container.attrs["NetworkSettings"]["Networks"][network_name]["IPAddress"]
                logger.info(f"Redirecting to noVNC at container IP: {container_ip}")
                # FIXED: Redirect using container's internal IP
                self.redirect(f"https://rosforge.com/vnc/{container_ip}/")
            else:
                logger.error(f"Container {container.name} not found in network {network_name}")
                self.write(f"<h1>Network Error</h1><p>Container not found in expected network</p>")
                
        except Exception as e:
            logger.error(f"Error for user {username}: {e}")
            self.write(f"<h1>Error</h1><p>Error accessing desktop for {username}: {str(e)}</p>")

class HealthHandler(tornado.web.RequestHandler):
    def get(self):
        """Health check endpoint for JupyterHub"""
        self.write({"status": "healthy", "service": "desktop-proxy"})

# FIXED: Add routes for the service path structure
app = tornado.web.Application([
    (r"/", HealthHandler),                           # Health check at service root
    (r"/desktop/([^/]+)", DesktopHandler),          # Desktop access
    (r"/services/desktop-proxy/", HealthHandler),    # JupyterHub service health check
    (r"/services/desktop-proxy/desktop/([^/]+)", DesktopHandler), # Full service path
])

if __name__ == "__main__":
    logger.info("Starting desktop-proxy on port 9999")
    app.listen(9999)
    tornado.ioloop.IOLoop.current().start()
'''
        ],
        'oauth_no_confirm': True,
    }
]

# --- Reverse proxy / headers ---
c.JupyterHub.tornado_settings = {
    'headers': {
        'Content-Security-Policy': "frame-ancestors 'self' https://rosforge.com"
    }
}

c.JupyterHub.trust_user_provided_tokens = True
c.Authenticator.auto_login = True

# Timeouts (increased for debugging / slow container starts)
c.Spawner.start_timeout = 600    # time to wait for spawn to start
c.Spawner.http_timeout = 600     # wait for the single-user server to appear
c.DockerSpawner.start_timeout = 600

# Hub logging level
c.JupyterHub.log_level = 'INFO'

# --- Debug settings ---
c.DockerSpawner.debug = True

# Concurrency limits
c.JupyterHub.concurrent_spawn_limit = 10
c.JupyterHub.active_server_limit = 20

# Container naming
c.DockerSpawner.name_template = 'jupyter-{username}-{servername}'

# --- Hooks for user workspace management ---
def pre_spawn_hook(spawner):
    """Hook to run before spawning user containers"""
    spawner.log.info(f"Pre-spawn hook: Starting container for user {spawner.user.name}")

    # Create individual student workspace directory if needed
    username = spawner.user.name
    user_workspace_dir = f'/srv/student_workspaces/{username}'
    
    try:
        import os
        os.makedirs(user_workspace_dir, exist_ok=True)
        os.chmod(user_workspace_dir, 0o755)
        spawner.log.info(f"Created/verified workspace directory for user {username}")
    except Exception as e:
        spawner.log.warning(f"Could not create workspace directory: {e}")

    # Ensure the image exists and the network exists
    try:
        import docker
        client = docker.from_env()

        # Check if image exists
        try:
            image = client.images.get(spawner.image)
            spawner.log.info(f"Using image: {image.tags}")
        except docker.errors.ImageNotFound:
            spawner.log.error(f"Image {spawner.image} not found!")
            raise Exception(f"Docker image {spawner.image} not found")

        # Check network exists
        try:
            network = client.networks.get(spawner.network_name)
            spawner.log.info(f"Using network: {network.name}")
        except docker.errors.NotFound:
            spawner.log.error(f"Network {spawner.network_name} not found!")
            raise Exception(f"Docker network {spawner.network_name} not found")

    except Exception as e:
        spawner.log.error(f"Pre-spawn hook error: {e}")
        raise

c.Spawner.pre_spawn_hook = pre_spawn_hook

def post_stop_hook(spawner):
    """Hook to run after stopping user containers"""
    spawner.log.info(f"Post-stop hook: Cleaning up for user {spawner.user.name}")

c.Spawner.post_stop_hook = post_stop_hook
