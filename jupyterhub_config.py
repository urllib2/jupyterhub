#!/usr/bin/env python3
# OPTIMIZED FOR 4-5 STUDENTS (No Gazebo) - 4GB VPS
# Auth: GitHub OAuth + Google OAuth (Multi-Authenticator)
from dockerspawner import DockerSpawner
from oauthenticator.github import GitHubOAuthenticator
from oauthenticator.google import GoogleOAuthenticator
from multiauthenticator import MultiAuthenticator
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

# Custom templates for RosForge branding
c.JupyterHub.template_paths = ['/srv/jupyterhub/templates']

# =============================================================================
# 2. MULTI-AUTHENTICATOR: GITHUB + GOOGLE
# =============================================================================
c.JupyterHub.authenticator_class = MultiAuthenticator

# Multi-Auth
c.MultiAuthenticator.authenticators = [
    {
        "authenticator_class": "google",
        "url_prefix": "/google",
        "config": {
            "client_id": os.environ.get("GOOGLE_CLIENT_ID"),
            "client_secret": os.environ.get("GOOGLE_CLIENT_SECRET"),
            "oauth_callback_url": "https://app.rosforge.com/hub/google/oauth_callback",
        },
    },
    {
        "authenticator_class": "github",
        "url_prefix": "/github",
        "config": {
            "client_id": os.environ.get("GITHUB_CLIENT_ID"),
            "client_secret": os.environ.get("GITHUB_CLIENT_SECRET"),
            "oauth_callback_url": "https://app.rosforge.com/hub/github/oauth_callback",
        },
    },
]

# Shared access control
c.Authenticator.admin_users       = {"github:urllib2"}
c.Authenticator.allow_all         = True
c.Authenticator.auto_login        = False
c.Authenticator.refresh_pre_spawn = True
c.Authenticator.auth_refresh_age  = 3600

import sqlite3

async def block_unauthorized(authenticator, handler, authentication):
    username = authentication.get("name", "")

    if username == "github:urllib2":
        return authentication

    try:
        conn = sqlite3.connect("/srv/jupyterhub/data/students.db")
        cursor = conn.cursor()

        if username.startswith("github:"):
            github_user = username.replace("github:", "")
            cursor.execute("SELECT id FROM students WHERE github_username=? AND status=?", (github_user, "active"))
        elif username.startswith("google:"):
            email = username.replace("google:", "")
            cursor.execute("SELECT id FROM students WHERE email=? AND status=?", (email, "active"))
        else:
            conn.close()
            return None

        result = cursor.fetchone()
        conn.close()

        if result:
            return authentication
        authenticator.log.warning(f"Blocking unauthorized user: {username}")
        return None
    except Exception as e:
        authenticator.log.error(f"Auth DB error: {e}")
        return None

c.Authenticator.post_auth_hook = block_unauthorized

# =============================================================================
# 3. DOCKER SPAWNER CONFIGURATION
# =============================================================================
c.JupyterHub.spawner_class = DockerSpawner
c.DockerSpawner.image = 'ros2-teaching-multiuser:latest'
c.DockerSpawner.network_name = 'ros2-teaching_ros2-network'

c.JupyterHub.hub_connect_url      = 'http://ros2-teaching-hub:8081'
c.DockerSpawner.hub_ip_connect    = 'ros2-teaching-hub'
c.DockerSpawner.hub_connect_url   = 'http://ros2-teaching-hub:8081'
c.DockerSpawner.use_internal_ip   = True
c.DockerSpawner.use_internal_hostname = True
c.DockerSpawner.host_ip = '0.0.0.0'
c.DockerSpawner.port = 8888

c.DockerSpawner.name_template = 'jupyter-{username}'
c.DockerSpawner.remove      = False
c.DockerSpawner.pull_policy = 'ifnotpresent'

c.DockerSpawner.cmd = [
    '/usr/local/bin/start-singleuser.sh',
    '--ServerApp.jpserver_extensions=jupyter_server_proxy=True',
    '--ServerApp.allow_origin=*',
    '--ServerApp.disable_check_xsrf=True'
]

c.Spawner.default_url = '/lab/tree/course_materials/00_Welcome_Start_Here.ipynb'

# =============================================================================
# 4. RESOURCE LIMITS
# =============================================================================
c.DockerSpawner.mem_limit     = '600M'
c.DockerSpawner.mem_guarantee = '150M'
c.DockerSpawner.cpu_limit     = 0.6
c.DockerSpawner.cpu_guarantee = 0.15

# =============================================================================
# 5. DOCKER HOST CONFIGURATION
# =============================================================================
c.DockerSpawner.extra_host_config = {
    'port_bindings': {6080: None, 8888: None},
    'auto_remove': False,
    'restart_policy': {'Name': 'no'},
    'shm_size': '64m',
    'init': True,
    'cap_add': ['SYS_NICE'],
    'oom_kill_disable': False,
    'ulimits': [
        {'name': 'nofile', 'soft': 16384, 'hard': 16384},
        {'name': 'nproc',  'soft': 1024,  'hard': 1024}
    ],
    'tmpfs': {'/tmp': 'size=64m,noexec'},
}

# =============================================================================
# 6. TIMEOUTS
# =============================================================================
c.DockerSpawner.start_timeout = 90
c.DockerSpawner.http_timeout  = 15
c.Spawner.start_timeout       = 90

# =============================================================================
# 7. ENVIRONMENT VARIABLES
# =============================================================================
c.DockerSpawner.environment = {
    'JUPYTER_ENABLE_LAB': '1',
    'GRANT_SUDO': 'yes',
    'USER': 'jovyan',
    'HOME': '/home/jovyan',
    'SHELL': '/bin/bash',
    'ROS_DISTRO': 'jazzy',
    'ROS_LOCALHOST_ONLY': '0',
    'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu',
    'AMENT_PREFIX_PATH':  '/opt/ros/jazzy',
    'COLCON_PREFIX_PATH': '/opt/ros/jazzy',
    'DISPLAY':           ':1',
    'QT_X11_NO_MITSHM':  '1',
    'LIBGL_ALWAYS_SOFTWARE': '1',
    'QT_QPA_PLATFORM':   'xcb',
    'VENV_PATH':  '/opt/venv',
    'PATH':       '/opt/venv/bin:/opt/ros/jazzy/bin:/usr/local/bin:/usr/bin:/bin',
    'PYTHONPATH': '/opt/venv/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages',
    'DESKTOP_PORT': '6080',
}

# =============================================================================
# 8. VOLUME MOUNTS
# =============================================================================
c.DockerSpawner.volumes = {
    '/home/sami/ros2-teaching/shared': {
        'bind': '/home/jovyan/course_materials',
        'mode': 'rw'
    },
    '/home/sami/ros2-teaching/users/{username}': {
        'bind': '/home/jovyan/ros2_ws',
        'mode': 'rw'
    },
}

c.DockerSpawner.pre_spawn_hook = lambda spawner: os.makedirs(
    f'/home/sami/ros2-teaching/users/{spawner.user.name}',
    exist_ok=True
)

# =============================================================================
# 9. PROXY
# =============================================================================
c.ConfigurableHTTPProxy.api_url      = 'http://0.0.0.0:8001'
c.ConfigurableHTTPProxy.should_start = True
c.ConfigurableHTTPProxy.auth_token   = os.environ.get('JUPYTERHUB_CRYPT_KEY', '')
c.ConfigurableHTTPProxy.debug        = False

# =============================================================================
# 10. CONCURRENCY
# =============================================================================
c.JupyterHub.active_server_limit    = 6
c.JupyterHub.concurrent_spawn_limit = 6
c.JupyterHub.init_spawners_timeout  = 1

# =============================================================================
# 11. IDLE CULLING
# =============================================================================
c.JupyterHub.load_roles = [{"name": "idle-culler", "services": ["idle-culler"]}]
c.JupyterHub.services = [
    {
        "name": "idle-culler",
        "command": [
            sys.executable, "-m", "jupyterhub_idle_culler",
            "--timeout=7200", "--cull-every=180",
            "--concurrency=1", "--max-age=5400",
        ],
        "admin": True,
    }
]

# =============================================================================
# 12. DATABASE & PERSISTENCE
# =============================================================================
c.JupyterHub.db_url             = 'sqlite:////srv/jupyterhub/data/jupyterhub.sqlite'
c.JupyterHub.cookie_secret_file = '/srv/jupyterhub/data/jupyterhub_cookie_secret'
c.JupyterHub.cleanup_servers    = True

# =============================================================================
# 13. SECURITY & CLOUDFLARE
# =============================================================================
c.JupyterHub.cookie_max_age_days    = 7
c.JupyterHub.reset_db               = False
c.JupyterHub.trust_downstream_proxy = True
c.JupyterHub.cookie_options         = {'secure': True, 'samesite': 'Lax'}
c.JupyterHub.tornado_settings       = {
    'xsrf_cookie_kwargs': {'secure': True, 'samesite': 'lax'}
}

# =============================================================================
# 14. LOGGING
# =============================================================================
c.JupyterHub.log_level        = 'INFO'
c.DockerSpawner.debug         = False
c.Application.log_level       = 'INFO'
c.ConfigurableHTTPProxy.debug = False

# =============================================================================
# 15. MISC
# =============================================================================
c.DockerSpawner.extra_create_kwargs = {'working_dir': '/home/jovyan', 'user': '1000:1000'}
c.JupyterHub.shutdown_on_logout = False
c.JupyterHub.redirect_to_server = False
c.Spawner.disable_user_config   = True
