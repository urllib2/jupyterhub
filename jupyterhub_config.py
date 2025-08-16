# Multi-User JupyterHub Configuration with OAuth Authentication
import os
import stat
from dockerspawner import DockerSpawner

# Basic server configuration for your existing nginx setup
c.JupyterHub.ip = "0.0.0.0"
c.JupyterHub.port = 8000
c.JupyterHub.bind_url = "http://0.0.0.0:8000"

# IMPORTANT: Configure for Cloudflare + your host nginx setup
c.JupyterHub.base_url = '/'
# Since Cloudflare handles SSL, JupyterHub sees HTTP from nginx
c.JupyterHub.trust_downstream_proxy = True

# =============================================================================
# OAUTH AUTHENTICATION CONFIGURATION
# =============================================================================

# Choose your OAuth provider by uncommenting the appropriate section:

# --- GOOGLE OAUTH ---
# Uncomment this section to use Google OAuth

from oauthenticator import GoogleOAuthenticator
c.JupyterHub.authenticator_class = GoogleOAuthenticator

# Set these environment variables or replace with your actual values
c.GoogleOAuthenticator.client_id = os.environ.get('GOOGLE_OAUTH_CLIENT_ID', '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')
c.GoogleOAuthenticator.client_secret = os.environ.get('GOOGLE_OAUTH_CLIENT_SECRET', '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')

# OAuth callback URL - for your domain
c.GoogleOAuthenticator.oauth_callback_url = os.environ.get('OAUTH_CALLBACK_URL', 'https://rosforge.com/hub/oauth-callback')

# Optional: Restrict to specific Google domains
# c.GoogleOAuthenticator.hosted_domain = ['yourdomain.com']

# Optional: Whitelist specific users
c.GoogleOAuthenticator.whitelist = {'sami.turki84@gmail.com', 'bekri.atef@gmail.com'}


# --- GITHUB OAUTH ---
# Uncomment this section to use GitHub OAuth

from oauthenticator import GitHubOAuthenticator
c.JupyterHub.authenticator_class = GitHubOAuthenticator

# Set these environment variables or replace with your actual values
c.GitHubOAuthenticator.client_id = os.environ.get('GITHUB_OAUTH_CLIENT_ID', 'Ov23li4bnxSh2SubSeHa')
c.GitHubOAuthenticator.client_secret = os.environ.get('GITHUB_OAUTH_CLIENT_SECRET', 'c3058dcec82c4bc85137cc42178f659c59a704b0')

# OAuth callback URL - for your domain
c.GitHubOAuthenticator.oauth_callback_url = os.environ.get('OAUTH_CALLBACK_URL', 'https://rosforge.com/hub/oauth-callback')

# Optional: Restrict to organization members
# c.GitHubOAuthenticator.github_organization_whitelist = {'your-organization'}

# Optional: Whitelist specific users
c.GitHubOAuthenticator.whitelist = {'urllib2'}


# TEMPORARY: Keep dummy auth for testing - REMOVE IN PRODUCTION
c.JupyterHub.authenticator_class = "jupyterhub.auth.DummyAuthenticator"
c.DummyAuthenticator.password = "password"

# Admin users - update with actual usernames from OAuth
c.Authenticator.admin_users = {"admin", "jovyan"}

# Enable user creation for new OAuth users
c.Authenticator.auto_login = False
c.Authenticator.create_system_users = False

# =============================================================================
# DOCKER SPAWNER CONFIGURATION FOR MULTI-USER
# =============================================================================

c.JupyterHub.spawner_class = DockerSpawner

# Use the same image as your main container
c.DockerSpawner.image = 'ros2-teaching-vps:latest'

# Remove containers when they stop
c.DockerSpawner.remove = True

# Network configuration - use the same network as JupyterHub
c.DockerSpawner.network_name = 'ros2-teaching_ros2-network'

# Hub connectivity
c.JupyterHub.hub_bind_url = "http://0.0.0.0:8081"
c.JupyterHub.hub_connect_url = "http://ros2-teaching:8081"

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
    
    # Add initialization command to create user and setup environment
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
    
    print(f"PRE-SPAWN: Configured spawn command for {username}")

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

print("Multi-User JupyterHub configuration with OAuth support loaded")
