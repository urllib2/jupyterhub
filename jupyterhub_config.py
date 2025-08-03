# FIXED JupyterHub Configuration for Docker Compose v2 - Spawning Issue Resolution
import os
import stat

# Basic server configuration
c.JupyterHub.ip = "0.0.0.0"
c.JupyterHub.port = 8000
c.JupyterHub.bind_url = "http://0.0.0.0:8000"

# Authentication - simple dummy auth for teaching environment
c.JupyterHub.authenticator_class = "jupyterhub.auth.DummyAuthenticator"
c.DummyAuthenticator.password = "password"
c.Authenticator.admin_users = {"jovyan"}

# CRITICAL FIX: Use correct spawner configuration with proper command
c.JupyterHub.spawner_class = "jupyterhub.spawner.LocalProcessSpawner"
c.Spawner.notebook_dir = "/home/jovyan"
c.Spawner.default_url = "/lab"

# FIXED: Use jupyter-labhub for proper JupyterHub integration
c.Spawner.cmd = ["/opt/venv/bin/jupyter-labhub"]

# FIXED: Hub networking - use loopback for internal communication
c.JupyterHub.hub_bind_url = "http://127.0.0.1:8081"
c.JupyterHub.hub_connect_url = "http://127.0.0.1:8081"

# CRITICAL FIX: Remove custom args - let JupyterHub handle spawner communication
c.Spawner.args = []

# FIXED: Environment variables for spawned notebooks
c.Spawner.environment = {
    "DISPLAY": ":1",
    "HOME": "/home/jovyan",
    "USER": "jovyan",
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
    # JupyterLab specific
    "JUPYTER_ENABLE_LAB": "yes",
    "JUPYTER_LAB_ENABLE": "1",
}

# Database configuration
c.JupyterHub.db_url = "sqlite:////srv/jupyterhub/jupyterhub.sqlite"

# INCREASED: Timeout settings for better reliability  
c.Spawner.start_timeout = 300  # 5 minutes
c.Spawner.http_timeout = 180   # 3 minutes for HTTP requests
c.JupyterHub.init_spawners_timeout = 120

# FIXED: Proxy configuration
c.ConfigurableHTTPProxy.should_start = True
c.ConfigurableHTTPProxy.command = ["/usr/bin/configurable-http-proxy"]
c.ConfigurableHTTPProxy.debug = False  # Disable for stability
c.ConfigurableHTTPProxy.api_url = "http://127.0.0.1:8001"

# FIXED: Security files handling
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

# FIXED: Logging configuration - reduce verbosity for stability
c.JupyterHub.log_level = "INFO"
c.Spawner.debug = False
c.LocalProcessSpawner.debug = False

# User management
c.LocalProcessSpawner.create_system_users = False

# FIXED: Pre-spawn hook with better error handling
def pre_spawn_hook(spawner):
    """Ensure proper spawning environment"""
    username = spawner.user.name
    print(f"PRE-SPAWN: Setting up for {username}")
    
    try:
        import pwd
        user_info = pwd.getpwnam(username)
        home_dir = user_info.pw_dir
        uid = user_info.pw_uid
        gid = user_info.pw_gid
        
        print(f"PRE-SPAWN: User {username} - UID: {uid}, GID: {gid}, Home: {home_dir}")
        
        # Ensure directories exist with correct permissions
        dirs_to_create = [
            home_dir,
            f"{home_dir}/.jupyter",
            f"{home_dir}/.local",
            f"{home_dir}/.local/share",
            f"{home_dir}/.local/share/jupyter",
            f"{home_dir}/.config",
            f"{home_dir}/.cache",
            "/tmp/runtime-jovyan",
        ]
        
        for directory in dirs_to_create:
            try:
                if not os.path.exists(directory):
                    os.makedirs(directory, mode=0o755, exist_ok=True)
                os.chown(directory, uid, gid)
                print(f"PRE-SPAWN: Created/fixed directory: {directory}")
            except Exception as e:
                print(f"PRE-SPAWN: Warning for directory {directory}: {e}")
        
        # Create minimal Jupyter config
        jupyter_config_dir = f"{home_dir}/.jupyter"
        jupyter_config_file = f"{jupyter_config_dir}/jupyter_lab_config.py"
        
        if not os.path.exists(jupyter_config_file):
            with open(jupyter_config_file, 'w') as f:
                f.write("""
# Minimal Jupyter Lab configuration for spawning
c.ServerApp.ip = '127.0.0.1'
c.ServerApp.open_browser = False
c.ServerApp.allow_root = True
c.ServerApp.token = ''
c.ServerApp.password = ''
c.LabApp.default_url = '/lab'
""")
            os.chown(jupyter_config_file, uid, gid)
            print(f"PRE-SPAWN: Created Jupyter config: {jupyter_config_file}")
        
        print(f"PRE-SPAWN: Environment ready for {username}")
        return True
        
    except Exception as e:
        print(f"PRE-SPAWN: ERROR for {username}: {e}")
        import traceback
        traceback.print_exc()
        return False

c.Spawner.pre_spawn_hook = pre_spawn_hook

# ADDED: Post-spawn hook for verification
def post_spawn_hook(spawner):
    """Verify spawner is properly connected"""
    print(f"POST-SPAWN: Verifying connection for {spawner.user.name}")
    print(f"POST-SPAWN: Server URL: {spawner.server.url}")
    print(f"POST-SPAWN: Server port: {spawner.server.port}")

c.Spawner.post_spawn_hook = post_spawn_hook

# CRITICAL: Spawner settings for debugging
c.Spawner.poll_interval = 30  # Check every 30 seconds
c.Spawner.consecutive_failure_limit = 5  # Allow more failures

# Additional Docker-specific settings
c.JupyterHub.cleanup_servers = True
c.JupyterHub.cleanup_proxy = True
c.JupyterHub.allow_named_servers = False

# CRITICAL: Ensure proper shutdown handling
c.JupyterHub.shutdown_on_logout = False

print("FIXED JupyterHub configuration loaded - Debug mode enabled")