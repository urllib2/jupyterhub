# ROS2 Teaching Environment

This Docker environment provides a complete ROS2 Jazzy setup with GUI support for teaching purposes.

## Features

- ROS2 Jazzy Desktop
- JupyterLab interface
- noVNC web-based GUI access
- Fluxbox window manager
- Pre-configured Python environment with ROS2 packages

## Access URLs

- JupyterLab: http://localhost:8888
- GUI Desktop: http://localhost:6080/vnc.html

## Usage

1. Start the environment:
   ```bash
   docker compose up -d
   ```

2. Monitor startup:
   ```bash
   docker logs -f ros2-teaching-env
   ```

3. Access JupyterLab at http://localhost:8888
4. Access GUI desktop at http://localhost:6080/vnc.html

## Troubleshooting

- If services don't start properly, check logs with `docker logs ros2-teaching-env`
- The health check will restart failed services automatically
- For GUI issues, ensure the VNC server is running on port 5900

## Development

The shared_workspace directory is mounted to /home/jovyan/shared_workspace in the container.
Student notebooks are in the student_notebooks directory.

## Stopping

```bash
docker compose down
```

## Rebuilding

```bash
docker compose down
docker compose build --no-cache
docker compose up -d
```
