#!/bin/bash

echo "=== JupyterHub Spawning Debug ==="

# 1. Check JupyterHub logs for spawning errors
echo "1. Checking JupyterHub logs..."
docker compose exec ros2-teaching tail -50 /srv/jupyterhub/jupyterhub.log

echo -e "\n=== Separator ===\n"

# 2. Check if jupyter-labhub command exists and works
echo "2. Testing jupyter-labhub command..."
docker compose exec ros2-teaching which jupyter-labhub
docker compose exec ros2-teaching /opt/venv/bin/jupyter-labhub --version

echo -e "\n=== Separator ===\n"

# 3. Check Python environment and paths
echo "3. Checking Python environment..."
docker compose exec ros2-teaching /opt/venv/bin/python -c "import sys; print('Python paths:'); [print(p) for p in sys.path]"

echo -e "\n=== Separator ===\n"

# 4. Check JupyterHub database and user status
echo "4. Checking JupyterHub database..."
docker compose exec ros2-teaching ls -la /srv/jupyterhub/
docker compose exec ros2-teaching /opt/venv/bin/python -c "
import sqlite3
try:
    conn = sqlite3.connect('/srv/jupyterhub/jupyterhub.sqlite')
    cursor = conn.cursor()
    cursor.execute('SELECT name FROM sqlite_master WHERE type=\"table\";')
    tables = cursor.fetchall()
    print('Database tables:', tables)
    if ('users',) in tables:
        cursor.execute('SELECT * FROM users;')
        users = cursor.fetchall()
        print('Users:', users)
    conn.close()
except Exception as e:
    print('Database error:', e)
"

echo -e "\n=== Separator ===\n"

# 5. Test manual notebook startup
echo "5. Testing manual notebook startup..."
docker compose exec -u jovyan ros2-teaching bash -c "
cd /home/jovyan && 
source /opt/venv/bin/activate && 
/opt/venv/bin/jupyter lab --version &&
echo 'Testing notebook startup...' &&
timeout 10 /opt/venv/bin/jupyter lab --ip=127.0.0.1 --port=8888 --no-browser --allow-root --debug 2>&1 | head -20
"
