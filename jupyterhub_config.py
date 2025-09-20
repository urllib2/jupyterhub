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

# VNC PROXY SUPPORT - SIMPLE ADDITION
c.DockerSpawner.cmd = [
    '/usr/local/bin/start-singleuser.sh',
    '--ServerApp.jpserver_extensions=jupyter_server_proxy=True',
    '--ServerApp.allow_origin=*',
    '--ServerApp.disable_check_xsrf=True'
]

# Set default user landing page
c.Spawner.default_url = '/lab'

# =============================================================================
# 4. OPTIMIZED RESOURCE LIMITS FOR 2 STUDENTS
# =============================================================================
c.DockerSpawner.mem_limit = '1.8G'        # 1.8G per student (3.6G total)
c.DockerSpawner.cpu_limit = 1.0           # 1 core per student
c.DockerSpawner.cpu_guarantee = 0.3       # Higher minimum guarantee

# OPTIMIZED: Docker host configuration for faster startup
c.DockerSpawner.extra_host_config = {
    'port_bindings': {
        6080: None,    # noVNC port
        8888: None,    # Jupyter port
    },
    'auto_remove': True,
    'restart_policy': {'Name': 'no'},
    'shm_size': '256m',           # REDUCED from 512m for faster startup
    'init': True,
    'cap_add': ['SYS_NICE'],
    'oom_kill_disable': False,    # Allow OOM killer
    # CRITICAL: Proper Docker API memory format
    'mem_limit': '1.8g',          # 1.8GB memory limit
    'memswap_limit': '1.8g',      # No additional swap
    'cpu_period': 100000,         # CPU period (microseconds)
    'cpu_quota': 100000,          # CPU quota (1 CPU core)
    # ADDED: Performance optimizations for faster startup
    'ulimits': [{'name': 'nofile', 'soft': 65536, 'hard': 65536}],
    'tmpfs': {'/tmp': 'size=256m,noexec'},  # Faster temp filesystem
}

# =============================================================================
# 5. OPTIMIZED STARTUP TIMEOUTS - BIG WINNER
# =============================================================================
c.DockerSpawner.start_timeout = 180     # REDUCED from 300 to 180 seconds
c.DockerSpawner.http_timeout = 30       # REDUCED from 60 to 30 seconds

# CRITICAL: Container reuse for massive performance gain
c.DockerSpawner.remove = False  # Keep containers for restart instead of recreating
c.DockerSpawner.use_internal_hostname = True

# =============================================================================
# 6. STREAMLINED ENVIRONMENT VARIABLES - BIG WINNER
# =============================================================================
c.DockerSpawner.environment = {
    # Essential Jupyter configuration
    'JUPYTER_ENABLE_LAB': '1',
    'GRANT_SUDO': 'yes',
    'USER': 'jovyan',
    'HOME': '/home/jovyan',
    'SHELL': '/bin/bash',
    
    # Essential ROS2 environment
    'ROS_DISTRO': 'jazzy',
    'ROS_LOCALHOST_ONLY': '0',
    'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu',
    'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
    'COLCON_PREFIX_PATH': '/opt/ros/jazzy',
    
    # Essential Display configuration
    'DISPLAY': ':1',
    'QT_X11_NO_MITSHM': '1',
    'LIBGL_ALWAYS_SOFTWARE': '1',
    'QT_QPA_PLATFORM': 'xcb',
    
    # Essential Python environment
    'VENV_PATH': '/opt/venv',
    'PATH': '/opt/venv/bin:/opt/ros/jazzy/bin:/usr/local/bin:/usr/bin:/bin',
    'PYTHONPATH': '/opt/venv/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages',
    
    # VNC proxy support
    'JUPYTER_SERVER_PROXY_VNC_COMMAND': '/bin/true',
    'JUPYTER_SERVER_PROXY_VNC_PORT': '6080',
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
c.ConfigurableHTTPProxy.debug = False

# =============================================================================
# 9. OPTIMIZED JUPYTERHUB FOR FASTER SPAWNING - BIG WINNER
# =============================================================================
c.JupyterHub.active_server_limit = 2        # 2 concurrent students
c.JupyterHub.concurrent_spawn_limit = 2     # INCREASED from 1 for parallel spawning
c.Spawner.start_timeout = 180               # REDUCED from 300 seconds
c.JupyterHub.init_spawners_timeout = 5      # REDUCED from 10 seconds

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
            "--timeout=1800",      # Cull after 30 minutes of inactivity (reduced)
            "--cull-every=120",    # Check every 2 minutes (more frequent)
            "--concurrency=1",     # Cull one container at a time
            "--max-age=14400",     # Maximum age: 4 hours (reduced)
        ],
        "admin": True,
    }
]

# =============================================================================
# 11. OPTIMIZED DATABASE AND PERSISTENCE - BIG WINNER
# =============================================================================
c.JupyterHub.db_url = 'sqlite:///srv/jupyterhub/jupyterhub.sqlite?check_same_thread=false'
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
# 13. OPTIMIZED LOGGING - LESS VERBOSE FOR PERFORMANCE
# =============================================================================
c.JupyterHub.log_level = 'INFO'
c.DockerSpawner.debug = False               # Disable to save resources
c.Application.log_level = 'INFO'
c.ConfigurableHTTPProxy.debug = False

# =============================================================================
# 14. OPTIMIZED SPAWNER OPTIONS
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
