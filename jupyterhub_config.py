#!/usr/bin/env python3
# Multi-User JupyterHub Configuration – OPTIMIZED FOR 4GB VPS
from dockerspawner import DockerSpawner
from oauthenticator import GitHubOAuthenticator
import os
import sys

# =============================================================================
# 1. BASIC SERVER CONFIGURATION
# =============================================================================
c.JupyterHub.bind_url = 'http://0.0.0.0:8000'
c.JupyterHub.base_url = '/'
c.JupyterHub.hub_bind_url = 'http://0.0.0.0:8081'
c.JupyterHub.port = 8000
c.JupyterHub.hub_port = 8081

# =============================================================================
# 2. GITHUB OAUTH AUTHENTICATION
# =============================================================================
c.JupyterHub.authenticator_class = GitHubOAuthenticator
c.GitHubOAuthenticator.oauth_callback_url = 'https://app.rosforge.com/hub/oauth_callback'
c.GitHubOAuthenticator.client_id = os.environ.get('GITHUB_CLIENT_ID')
c.GitHubOAuthenticator.client_secret = os.environ.get('GITHUB_CLIENT_SECRET')
c.GitHubOAuthenticator.enable_auth_state = True
c.GitHubOAuthenticator.scope = ['read:user', 'user:email']
c.GitHubOAuthenticator.username_claim = 'login'

# Admin users configuration
c.Authenticator.admin_users = {"urllib2"}
c.Authenticator.allow_all = True  # Allow any GitHub user
c.Authenticator.auto_login = False

# =============================================================================
# 3. DOCKER SPAWNER CONFIGURATION
# =============================================================================
c.JupyterHub.spawner_class = DockerSpawner
c.DockerSpawner.image = 'ros2-teaching-multiuser:latest'
c.DockerSpawner.network_name = 'ros2-teaching_ros2-network'

# Hub connection configuration
c.JupyterHub.hub_connect_url = 'http://ros2-teaching-hub:8081'
c.DockerSpawner.hub_ip_connect = 'ros2-teaching-hub'
c.DockerSpawner.hub_connect_url = 'http://ros2-teaching-hub:8081'
c.DockerSpawner.use_internal_ip = True
c.DockerSpawner.host_ip = '0.0.0.0'
c.DockerSpawner.port = 8888

# Container naming and removal
c.DockerSpawner.name_template = 'jupyter-{username}'
c.DockerSpawner.remove = True
c.DockerSpawner.pull_policy = 'ifnotpresent'

# =============================================================================
# 4. OPTIMIZED RESOURCE LIMITS FOR 4GB VPS
# =============================================================================
# FIXED: Proper Docker resource limits
c.DockerSpawner.mem_limit = '1G'        # Memory limit per container
c.DockerSpawner.cpu_limit = 1.0         # CPU limit (1 core)
c.DockerSpawner.cpu_guarantee = 0.1     # Minimum CPU guarantee

# FIXED: Docker host configuration
c.DockerSpawner.extra_host_config = {
    'port_bindings': {
        6080: None,    # noVNC port
        8888: None,    # Jupyter port
    },
    'auto_remove': True,
    'restart_policy': {'Name': 'no'},
    'shm_size': '256m',           # Reduced shared memory for GUI apps
    'init': True,
    'cap_add': ['SYS_NICE'],
    # 'memory_swappiness': 10,      # Minimize swap usage
    'oom_kill_disable': False,    # Allow OOM killer
    # CRITICAL: Proper Docker API memory format
    'mem_limit': '1g',            # 1GB memory limit
    'memswap_limit': '1g',        # No additional swap
    'cpu_period': 100000,         # CPU period (microseconds)
    'cpu_quota': 100000,          # CPU quota (1 CPU core)
}

# =============================================================================
# 5. CONTAINER STARTUP CONFIGURATION
# =============================================================================
c.DockerSpawner.cmd = ['/usr/local/bin/start-singleuser.sh']
c.DockerSpawner.start_timeout = 300     # 5 minutes for container start
c.DockerSpawner.http_timeout = 120      # HTTP timeout

# =============================================================================
# 6. ENVIRONMENT VARIABLES
# =============================================================================
c.DockerSpawner.environment = {
    # Jupyter configuration
    'JUPYTER_ENABLE_LAB': '1',
    'JUPYTER_CONFIG_DIR': '/tmp/.jupyter',
    'JUPYTER_RUNTIME_DIR': '/tmp/.local/share/jupyter/runtime',
    'JUPYTER_DATA_DIR': '/home/jovyan/.local/share/jupyter',
    
    # User and permissions
    'GRANT_SUDO': 'yes',
    'CHOWN_HOME': 'yes',
    'USER': 'jovyan',
    'HOME': '/home/jovyan',
    'SHELL': '/bin/bash',
    
    # ROS2 environment
    'ROS_DISTRO': 'jazzy',
    'ROS_DOMAIN_ID': '42',
    'ROS_LOCALHOST_ONLY': '0',
    'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
    'COLCON_PREFIX_PATH': '/opt/ros/jazzy',
    'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu',
    
    # Display and graphics
    'DISPLAY': ':1',
    'LIBGL_ALWAYS_SOFTWARE': '1',
    'QT_X11_NO_MITSHM': '1',
    'XAUTHORITY': '/tmp/.Xauth',
    
    # Python environment
    'VENV_PATH': '/opt/venv',
    'PATH': '/opt/venv/bin:/opt/ros/jazzy/bin:/usr/local/bin:/usr/bin:/bin',
    'PYTHONPATH': '/opt/venv/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages',
    
    # Memory optimizations
    'MALLOC_TRIM_THRESHOLD_': '100000',
    'MALLOC_MMAP_THRESHOLD_': '131072',
    'PYTHONHASHSEED': '0',
}

# =============================================================================
# 7. VOLUME MOUNTS AND PERSISTENCE
# =============================================================================
c.DockerSpawner.volumes = {
    # shared course material (read-only)
    '/home/sami/ros2-teaching/shared': {
        'bind': '/home/jovyan/course_materials',
        'mode': 'ro'
    },
    # personal workspace (read-write)
    '/home/sami/ros2-teaching/users/{username}': {
        'bind': '/home/jovyan/my_workspace',
        'mode': 'rw'
    },
}

# Create user directories on spawn
c.DockerSpawner.pre_spawn_hook = lambda spawner: os.makedirs(
    f'/home/sami/ros2-teaching/users/{spawner.user.name}', 
    exist_ok=True
)

# =============================================================================
# 8. PROXY CONFIGURATION (FIXED)
# =============================================================================
c.ConfigurableHTTPProxy.api_url = 'http://0.0.0.0:8001'
c.ConfigurableHTTPProxy.should_start = True
c.ConfigurableHTTPProxy.auth_token = os.environ.get('JUPYTERHUB_CRYPT_KEY', '')
# REMOVED: No memory options for proxy (this was causing the error)

# =============================================================================
# 9. JUPYTERHUB OPTIMIZATIONS FOR 4GB VPS
# =============================================================================
c.JupyterHub.active_server_limit = 3        # Max 3 concurrent users
c.JupyterHub.concurrent_spawn_limit = 1     # Spawn containers sequentially
c.Spawner.start_timeout = 300               # 5 minutes spawn timeout
c.JupyterHub.init_spawners_timeout = 10     # Quick spawner init

# =============================================================================
# 10. IDLE CULLING SERVICE (Resource Management)
# =============================================================================
c.JupyterHub.load_roles = [
    {
        "name": "idle-culler",
        "services": ["idle-culler"],
    }
]

c.JupyterHub.services = [
    {
        "name": "idle-culler",
        "command": [
            sys.executable,
            "-m", "jupyterhub_idle_culler",
            "--timeout=3600",      # Cull after 1 hour of inactivity
            "--cull-every=300",    # Check every 5 minutes
            "--concurrency=1",     # Cull one container at a time
            "--max-age=28800",     # Maximum age: 8 hours
        ],
        "admin": True,
    }
]

# =============================================================================
# 11. DATABASE AND PERSISTENCE
# =============================================================================
c.JupyterHub.db_url = 'sqlite:///srv/jupyterhub/jupyterhub.sqlite'
c.JupyterHub.cookie_secret_file = '/srv/jupyterhub/cookie_secret'
c.JupyterHub.cleanup_servers = True

# =============================================================================
# 12. SECURITY CONFIGURATION
# =============================================================================
c.JupyterHub.cookie_max_age_days = 7
c.JupyterHub.reset_db = False
c.Authenticator.refresh_pre_spawn = True
c.Authenticator.auth_refresh_age = 3600     # Refresh auth every hour

# =============================================================================
# 13. LOGGING AND DEBUGGING
# =============================================================================
c.JupyterHub.log_level = 'INFO'
c.DockerSpawner.debug = False               # Disable to save resources
c.Application.log_level = 'INFO'
c.ConfigurableHTTPProxy.debug = False

# =============================================================================
# 14. CUSTOM SPAWNER OPTIONS (Advanced)
# =============================================================================
c.DockerSpawner.extra_create_kwargs = {
    'working_dir': '/home/jovyan',
    'user': '1000:1000',
}

# =============================================================================
# 15. SHUTDOWN AND CLEANUP
# =============================================================================
c.JupyterHub.shutdown_on_logout = False
c.JupyterHub.redirect_to_server = False
c.Spawner.disable_user_config = True        # Prevent user config override

# =============================================================================
# 16. CUSTOM ERROR PAGES AND TEMPLATES
# =============================================================================
c.JupyterHub.template_paths = ['/srv/jupyterhub/templates']
c.JupyterHub.extra_log_file = '/srv/jupyterhub/jupyterhub.log'

# =============================================================================
# 17. PERFORMANCE MONITORING
# =============================================================================
c.JupyterHub.statsd_host = ''  # Disable statsd for now
c.JupyterHub.statsd_port = 8125
c.JupyterHub.statsd_prefix = 'jupyterhub'

# =============================================================================
# 18. CUSTOM STUDENT MANAGEMENT (if using custom authenticator)
# =============================================================================
# Uncomment if you want to use custom student management
# c.JupyterHub.authenticator_class = 'student_management.CustomAuthenticator'
# c.CustomAuthenticator.student_list_file = '/srv/jupyterhub/students.txt'
