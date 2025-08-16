# Fixed Multi-User JupyterHub Configuration
import os
import stat
from dockerspawner import DockerSpawner

# Basic server configuration
c.JupyterHub.ip = "0.0.0.0"
c.JupyterHub.port = 8000
c.JupyterHub.bind_url = "http://0.0.0.0:8000"

# CRITICAL: Configure for Cloudflare + nginx setup
c.JupyterHub.base_url = '/'
c.JupyterHub.trust_downstream_proxy = True

# =============================================================================
# AUTHENTICATION CONFIGURATION - CHOOSE ONE
# =============================================================================

# FOR TESTING: Use DummyAuthenticator (REMOVE IN PRODUCTION)
c.JupyterHub.authenticator_class = "jupyterhub.auth.DummyAuthenticator"
c.DummyAuthenticator.password = "password"

# UNCOMMENT FOR GOOGLE OAUTH (and comment out DummyAuthenticator above):
# from oauthenticator import GoogleOAuthenticator
# c.JupyterHub.authenticator_class = GoogleOAuthenticator
# c.GoogleOAuthenticator.client_id = os.environ.get('GOOGLE_OAUTH_CLIENT_ID', 'your-google-client-id')
# c.GoogleOAuthenticator.client_secret = os.environ.get('GOOGLE_OAUTH_CLIENT_SECRET', 'your-google-client-secret')
# c.GoogleOAuthenticator.oauth_callback_url = 'https://rosforge.com/hub/oauth-callback'
# c.GoogleOAuthenticator.whitelist = {'sami.turki84@gmail.com', 'bekri.atef@gmail.com'}

# UNCOMMENT FOR GITHUB OAUTH (and comment out DummyAuthenticator above):
# from oauthenticator import GitHubOAuthenticator  
# c.JupyterHub.authenticator_class = GitHubOAuthenticator
# c.GitHubOAuthenticator.client_id = os.environ.get('GITHUB_OAUTH_CLIENT_ID', 'your-github-client-id')
# c.GitHubOAuthenticator.client_secret = os.environ.get('GITHUB_OAUTH_CLIENT_SECRET', 'your-github-client-secret')
# c.GitHubOAuthenticator.oauth_callback_url = 'https://rosforge.com/hub/oauth-callback'
# c.GitHubOAuthenticator.whitelist = {'urllib2'}

# Admin users
c.Authenticator.admin_users = {"admin", "jovyan"}
c.Authenticator.auto_login = False
c.Authenticator.create_system_users = False

# =============================================================================
# DOCKER SPAWNER CONFIGURATION
# =============================================================================

c.JupyterHub.spawner_class = DockerSpawner

# Use the same image as your main container
c.DockerSpawner.image = 'ros2-teaching-multiuser:latest'

# Remove containers when they stop
c.DockerSpawner.remove = True

# Network configuration
c.DockerSpawner.network_name = 'ros2-teaching_ros2-network'

# Hub connectivity - FIXED
c.JupyterHub.hub_bind_url = "http://0.0.0.0:8081"
c.JupyterHub.hub_connect_url = "http://ros2-teaching-hub:8081"  # Use container name

# Container configuration
c.DockerSpawner.notebook_dir = '/home/{username}'
c.DockerSpawner.default_url = '/lab'

# Volume mounts for user isolation
c.DockerSpawner.volumes = {
    # Shared workspace accessible to all users
    '/home/jovyan/shared_workspace': '/home/{username}/shared_workspace',
    # Student notebooks accessible to all users  
    '/home/jovyan/student_notebooks': '/home/{username}/notebooks',
    # Individual user workspace (persistent)
    'jupyterhub-user-{username}': '/home/{username}/workspace',
}

# Environment variables for spawned containers
c.DockerSpawner.environment = {
    "DISPLAY": ":1",
    "HOME": "/home/{username}",
    "USER": "{username}",
    "SHELL": "/bin/bash",
    "PATH": "/opt/venv/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
    "VIRTUAL_ENV": "/opt/venv",
    "ROS_DISTRO": "jazzy",
    "ROS_DOMAIN_ID": "42",
    "GZ_VERSION": "harmonic",
    "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/venv/lib/python3.12/site-packages",
    "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib:/opt/ros/jazzy/local/lib:/usr/lib/x86_64-linux-gnu",
    # CPU-only rendering
    "LIBGL_ALWAYS_SOFTWARE": "1",
    "MESA_GL_VERSION_OVERRIDE": "3.3",
    "GALLIUM_DRIVER": "llvmpipe",
    "LP_NUM_THREADS": "2",
    # GUI settings
    "QT_X11_NO_MITSHM": "1",
    "QT_GRAPHICSSYSTEM": "native",
    "XDG_RUNTIME_DIR": "/tmp/runtime-{username}",
    "XDG_SESSION_TYPE": "x11",
    # JupyterLab specific
    "JUPYTER_ENABLE_LAB": "yes",
    "JUPYTER_LAB_ENABLE": "1",
}

# Resource limits per user
c.DockerSpawner.mem_limit = '1G'
c.DockerSpawner.cpu_limit = 1.0

# Timeout settings
c.DockerSpawner.start_timeout = 300
c.DockerSpawner.http_timeout = 180

# =============================================================================
# USER MANAGEMENT HOOKS
# =============================================================================

def pre_spawn_hook(spawner):
    """Setup user environment before spawning"""
    username = spawner.user.name
    print(f"PRE-SPAWN: Setting up environment for user {username}")
    
    # Create user in container if it doesn't exist
    spawner.extra_create_kwargs = {
        'user': '0',  # Start as root to create user
        'working_dir': f'/home/{username}',
    }
    
    spawner.extra_host_config = {
        'init': True,
    }
    
    # Pre-spawn command to create user and setup directories
    spawner.pre_spawn_cmd = f"""
    # Create user if doesn't exist
    if ! id "{username}" &>/dev/null; then
        useradd -m -s /bin/bash -u $(( $(id -u jovyan) + $(echo {username} | md5sum | cut -d' ' -f1 | tr -d 'a-f' | cut -c1-4 | sed 's/^0*//' || echo 1001) % 1000 + 1001 )) "{username}" || true
        usermod -aG sudo "{username}" || true
        echo "{username} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers || true
    fi
    
    # Setup directories
    mkdir -p /home/{username}/.jupyter
    mkdir -p /home/{username}/.local/share/jupyter
    mkdir -p /home/{username}/.config
    mkdir -p /home/{username}/.cache
    mkdir -p /home/{username}/workspace
    mkdir -p /tmp/runtime-{username}
    
    # Fix permissions
    chown -R {username}:{username} /home/{username}
    chmod 700 /tmp/runtime-{username}
    chown {username}:{username} /tmp/runtime-{username}
    
    # Setup ROS2 environment for user
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/{username}/.bashrc
    echo 'export PATH=/opt/venv/bin:$PATH' >> /home/{username}/.bashrc
    echo 'export ROS_DOMAIN_ID=42' >> /home/{username}/.bashrc
    
    # Create minimal Jupyter config
    cat > /home/{username}/.jupyter/jupyter_lab_config.py << 'EOF'
c.ServerApp.ip = '0.0.0.0'
c.ServerApp.allow_origin = '*'
c.ServerApp.allow_remote_access = True
c.ServerApp.open_browser = False
c.ServerApp.token = ''
c.ServerApp.password = ''
c.LabApp.default_url = '/lab'
EOF
    
    chown {username}:{username} /home/{username}/.jupyter/jupyter_lab_config.py
    """

c.DockerSpawner.pre_spawn_hook = pre_spawn_hook

# =============================================================================
# DATABASE AND SECURITY
# =============================================================================

# Database configuration
c.JupyterHub.db_url = "sqlite:////srv/jupyterhub/jupyterhub.sqlite"

# Security files handling
def ensure_security_file(file_path, required_mode=0o600):
    """Ensure security file exists and has correct permissions"""
    try:
        if not os.path.exists(file_path):
            import secrets
            with open(file_path, 'w') as f:
                f.write(secrets.token_hex(32))
        
        os.chmod(file_path, required_mode)
        os.chown(file_path, 0, 0)  # root:root
        return True
    except Exception as e:
        print(f"Warning: Could not setup {file_path}: {e}")
        return False

# Setup security files
cookie_secret_file = "/srv/jupyterhub/jupyterhub_cookie_secret"
proxy_token_file = "/srv/jupyterhub/jupyterhub_proxy_token"

ensure_security_file(cookie_secret_file)
ensure_security_file(proxy_token_file)

c.JupyterHub.cookie_secret_file = cookie_secret_file
c.ConfigurableHTTPProxy.auth_token_file = proxy_token_file

# =============================================================================
# PROXY CONFIGURATION
# =============================================================================

c.ConfigurableHTTPProxy.should_start = True
c.ConfigurableHTTPProxy.command = ["/usr/bin/configurable-http-proxy"]
c.ConfigurableHTTPProxy.debug = False
c.ConfigurableHTTPProxy.api_url = "http://127.0.0.1:8001"

# =============================================================================
# LOGGING AND MONITORING
# =============================================================================

c.JupyterHub.log_level = "INFO"
c.DockerSpawner.debug = False

# Cleanup settings
c.JupyterHub.cleanup_servers = True
c.JupyterHub.cleanup_proxy = True
c.JupyterHub.allow_named_servers = False
c.JupyterHub.shutdown_on_logout = False

# Resource monitoring
c.DockerSpawner.poll_interval = 30
c.DockerSpawner.consecutive_failure_limit = 5

print("Fixed Multi-User JupyterHub configuration loaded")
