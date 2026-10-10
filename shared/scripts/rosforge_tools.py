"""ROSFORGE helper tools for the course notebooks.

rqt_graph()   notebook version of rqt_graph: draws the live ROS 2 graph
show_graph()  draw the graph once, without updating
TurtleView    a turtlesim viewer and keypad that run inside the notebook
rviz2()       notebook version of RViz: a Meshcat 3D view of the robot, frames, scan and map
demo_robot()  a stand-in robot that publishes the topics a real robot would
start_robot() run robot_state_publisher on a URDF file; joint_sliders() move its joints
view_frames() draw the TF tree; frames_demo() play with frames and a point
"""
import math
import threading
import time
from collections import defaultdict

try:
    import rclpy
    from rclpy.node import Node
except ImportError:  # lets the module be imported (and tested) without ROS
    rclpy, Node = None, object

SYSTEM_TOPICS = {"/rosout", "/parameter_events"}


def _hidden(topic, show_system):
    """System topics and action internals are hidden unless show_system=True."""
    return not show_system and (topic in SYSTEM_TOPICS or "/_action/" in topic)
_state = {"node": None, "watcher": None, "turtle": None}


# ---------------------------------------------------------------- ROS access
def _ensure_ros():
    if not rclpy.ok():
        rclpy.init()
        _state["node"] = None
    if _state["node"] is None:
        _state["node"] = rclpy.create_node("rosforge_graph_viewer")
        time.sleep(1.0)  # give discovery a moment
    return _state["node"]


def read_graph(show_system=False):
    """Return (nodes, publishers, subscribers) for the running ROS 2 graph."""
    ros = _ensure_ros()
    nodes, pubs, subs = [], defaultdict(list), defaultdict(list)
    for name, ns in ros.get_node_names_and_namespaces():
        if name.startswith("_") or name == "rosforge_graph_viewer":
            continue
        full = ns.rstrip("/") + "/" + name
        nodes.append(full)
        try:
            for topic, _ in ros.get_publisher_names_and_types_by_node(name, ns):
                if not _hidden(topic, show_system):
                    pubs[topic].append(full)
            for topic, _ in ros.get_subscriber_names_and_types_by_node(name, ns):
                if not _hidden(topic, show_system):
                    subs[topic].append(full)
        except Exception:  # the node disappeared while we were asking
            continue
    return sorted(set(nodes)), dict(pubs), dict(subs)


# ------------------------------------------------------------------- drawing
def _width(label):
    return 7.6 * len(label) + 30


def render_graph(nodes, pubs, subs):
    """Build an SVG string: ellipses are nodes, rectangles are topics."""
    topics = sorted(set(pubs) | set(subs))
    if not nodes:
        return ('<svg width="520" height="70"><text x="10" y="40" font-family="sans-serif" '
                'font-size="15" fill="#6b7280">No ROS 2 programs are running yet.</text></svg>')

    items = [("n", n) for n in nodes] + [("t", t) for t in topics]
    adj = defaultdict(list)
    for t in topics:
        for p in pubs.get(t, []):
            adj[("n", p)].append(("t", t))
        for s in subs.get(t, []):
            adj[("t", t)].append(("n", s))

    # find back edges (cycles) so the layout stays left-to-right
    indeg = defaultdict(int)
    for u in adj:
        for v in adj[u]:
            indeg[v] += 1
    state, back = {}, set()

    def dfs(u):
        state[u] = 1
        for v in adj[u]:
            if state.get(v) == 1:
                back.add((u, v))
            elif v not in state:
                dfs(v)
        state[u] = 2

    for it in sorted(items, key=lambda i: (indeg[i] > 0, i)):
        if it not in state:
            dfs(it)

    col = {it: 0 for it in items}
    for _ in range(len(items)):
        for u in adj:
            for v in adj[u]:
                if (u, v) not in back and col[v] < col[u] + 1:
                    col[v] = col[u] + 1
    ncols = max(col.values()) + 1
    columns = [sorted(i for i in items if col[i] == c) for c in range(ncols)]

    colw = [max((_width(i[1]) for i in c), default=60) for c in columns]
    gap, row_h = 90, 58
    xs, x = [], 20
    for w in colw:
        xs.append(x + w / 2)
        x += w + gap
    rows = max(len(c) for c in columns)
    height = rows * row_h + 110
    width = x - gap + 20

    pos = {}
    for c, members in enumerate(columns):
        top = 20 + (rows - len(members)) * row_h / 2
        for r, it in enumerate(members):
            pos[it] = (xs[c], top + r * row_h + row_h / 2, _width(it[1]))

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}">',
           '<defs><marker id="rfarrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" '
           'markerHeight="8" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#6b7280"/></marker></defs>']
    for u in adj:
        for v in adj[u]:
            x1, y1, w1 = pos[u]
            x2, y2, w2 = pos[v]
            if (u, v) in back:
                off = lambda it: 20 if it[0] == "n" else 16
                sx, ex, low = x1, x2, max(y1, y2) + 75
                d = f"M{sx:.0f},{y1 + off(u):.0f} C{sx:.0f},{low:.0f} {ex:.0f},{low:.0f} {ex:.0f},{y2 + off(v) + 2:.0f}"
            else:
                sx, ex = x1 + w1 / 2, x2 - w2 / 2 - 2
                dx = max((ex - sx) / 2, 20)
                d = f"M{sx:.0f},{y1:.0f} C{sx + dx:.0f},{y1:.0f} {ex - dx:.0f},{y2:.0f} {ex:.0f},{y2:.0f}"
            out.append(f'<path d="{d}" fill="none" stroke="#6b7280" stroke-width="1.6" marker-end="url(#rfarrow)"/>')
    for (kind, label), (cx, cy, w) in pos.items():
        if kind == "n":
            out.append(f'<ellipse cx="{cx:.0f}" cy="{cy:.0f}" rx="{w / 2:.0f}" ry="20" fill="#dbeafe" stroke="#2563eb" stroke-width="2"/>')
        else:
            out.append(f'<rect x="{cx - w / 2:.0f}" y="{cy - 16:.0f}" width="{w:.0f}" height="32" rx="6" fill="#fef3c7" stroke="#d97706" stroke-width="2"/>')
        out.append(f'<text x="{cx:.0f}" y="{cy + 5:.0f}" text-anchor="middle" font-family="sans-serif" font-size="13" fill="#111827">{label}</text>')
    ly = height - 24
    out.append(f'<ellipse cx="40" cy="{ly}" rx="24" ry="11" fill="#dbeafe" stroke="#2563eb" stroke-width="2"/>'
               f'<text x="72" y="{ly + 5}" font-family="sans-serif" font-size="13" fill="#6b7280">node (a program)</text>'
               f'<rect x="200" y="{ly - 11}" width="40" height="22" rx="5" fill="#fef3c7" stroke="#d97706" stroke-width="2"/>'
               f'<text x="250" y="{ly + 5}" font-family="sans-serif" font-size="13" fill="#6b7280">topic (a channel)</text>'
               f'<text x="380" y="{ly + 5}" font-family="sans-serif" font-size="13" fill="#6b7280">arrow: the direction messages flow</text>')
    out.append("</svg>")
    return "".join(out)


# ------------------------------------------------------------------- display
def show_graph(show_system=False):
    """Draw the ROS 2 graph once."""
    from IPython.display import SVG, display
    display(SVG(render_graph(*read_graph(show_system))))


def rqt_graph(interval=1.0, show_system=False):
    """Notebook version of rqt_graph. Draws the ROS 2 graph and keeps it updated
    until you press Stop."""
    import ipywidgets as w
    from IPython.display import display
    if _state["watcher"]:
        _state["watcher"].set()
    stop = threading.Event()
    _state["watcher"] = stop
    view = w.HTML()
    button = w.Button(description="Stop updating", icon="stop")
    button.on_click(lambda _: (stop.set(), setattr(button, "disabled", True)))

    def loop():
        last = None
        while not stop.is_set():
            try:
                graph = read_graph(show_system)
                if repr(graph) != last:
                    last = repr(graph)
                    view.value = render_graph(*graph)
            except Exception as err:
                view.value = f"<pre>{err}</pre>"
            stop.wait(interval)

    threading.Thread(target=loop, daemon=True).start()
    display(w.VBox([button, view]))


# --------------------------------------------------------------- turtle view
class TurtleView(Node):
    """Shows turtlesim inside the notebook and gives you a keypad."""
    S = 40  # pixels per turtlesim unit (11 x 11 world)

    def __init__(self):
        from geometry_msgs.msg import Twist
        from turtlesim.msg import Pose
        import ipywidgets as w
        if not rclpy.ok():
            rclpy.init()
        old = _state["turtle"]
        if old is not None:
            old.close()
        super().__init__("jupyter_turtle_view")
        self._Twist = Twist
        self.screen, self.label = w.HTML(), w.Label()
        self.trail, self._last = [], 0.0
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self._on_pose, 10)
        self._draw(5.5, 5.5, 0.0)
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        _state["turtle"] = self

    def _draw(self, x, y, theta):
        S, size = self.S, 11 * self.S
        pts = " ".join(f"{a:.1f},{b:.1f}" for a, b in self.trail)
        self.screen.value = (
            f'<svg width="{size}" height="{size}" style="background:#4556ff">'
            f'<polyline points="{pts}" fill="none" stroke="white" stroke-width="2"/>'
            f'<polygon points="16,0 -9,-7 -9,7" fill="#2e9e4f" '
            f'transform="translate({x * S:.1f},{(11 - y) * S:.1f}) rotate({-math.degrees(theta):.1f})"/></svg>')
        self.label.value = f"x={x:.2f}  y={y:.2f}  \u03b8={theta:.2f}"

    def _on_pose(self, p):
        self.trail = (self.trail + [(p.x * self.S, (11 - p.y) * self.S)])[-2000:]
        now = time.time()
        if now - self._last >= 0.05:  # about 20 updates per second
            self._last = now
            self._draw(p.x, p.y, p.theta)

    def drive(self, linear, angular):
        msg = self._Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.pub.publish(msg)

    def close(self):
        try:
            self._executor.shutdown()
            self.destroy_node()
        except Exception:
            pass

    def show(self):
        """Display the turtle screen and the keypad."""
        import ipywidgets as w
        from IPython.display import display

        def key(text, lin, ang):
            b = w.Button(description=text, layout=w.Layout(width="60px"))
            b.on_click(lambda _: self.drive(lin, ang))
            return b

        def clear(_):
            self.trail = []

        reset = w.Button(description="Clear trail", layout=w.Layout(width="100px"))
        reset.on_click(clear)
        pad = w.HBox([key("\u2190", 0, 1.5), key("\u2191", 1.5, 0), key("\u2193", -1.5, 0),
                      key("\u2192", 0, -1.5), reset])
        display(w.VBox([self.screen, self.label, pad]))


# ====================================================================== rviz2
import io
import os


def _quat_to_matrix(x, y, z, w):
    import numpy as np
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    m = np.eye(4)
    m[:3, :3] = [[1 - yy - zz, xy - wz, xz + wy],
                 [xy + wz, 1 - xx - zz, yz - wx],
                 [xz - wy, yz + wx, 1 - xx - yy]]
    return m


def _transform_to_matrix(t):
    """geometry_msgs/Transform -> 4x4 numpy matrix."""
    m = _quat_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def _resolve_uri(uri):
    """package://pkg/file -> a real path on disk."""
    if uri.startswith("package://"):
        pkg, _, rest = uri[len("package://"):].partition("/")
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(pkg), rest)
    return uri[len("file://"):] if uri.startswith("file://") else uri


def _viewer_html(vis, height=520):
    """HTML for the notebook iframe that shows the Meshcat viewer.

    Behind JupyterHub the stock Meshcat page tries to open its websocket at the
    root of the site, which fails. So the page is rebuilt here with a websocket
    address that goes through the hub proxy. Set the environment variable
    ROSFORGE_MESHCAT_URL to use a different address instead."""
    import html
    style = f'width="100%" height="{height}" style="border:1px solid #9ca3af;border-radius:6px"'
    override = os.environ.get("ROSFORGE_MESHCAT_URL")
    if override:
        return f'<iframe src="{override}" {style}></iframe>'
    url = vis.url()  # e.g. http://127.0.0.1:7000/static/
    port = url.split("://")[1].split("/")[0].split(":")[-1]
    prefix = os.environ.get("JUPYTERHUB_SERVICE_PREFIX")
    if not prefix:
        return f'<iframe src="{url}" {style}></iframe>'
    base = f"{prefix}proxy/{port}/"
    page = ("<!DOCTYPE html><html><body style='margin:0'>"
            "<div id='pane' style='width:100vw;height:100vh;overflow:hidden'></div>"
            f"<script src='{base}static/main.min.js'></script>"
            "<script>var v=new MeshCat.Viewer(document.getElementById('pane'));"
            "var p=parent.location;"
            f"v.connect((p.protocol==='https:'?'wss://':'ws://')+p.host+'{base}');</script>"
            "</body></html>")
    return f'<iframe srcdoc="{html.escape(page, quote=True)}" {style}></iframe>'


class RvizScene:
    """The 3D scene: robot model, TF frames, laser scan and map (no ROS inside)."""

    def __init__(self, vis):
        import threading
        self.vis, self.lock = vis, threading.Lock()
        self.links, self.frames = set(), set()
        self.vis["rviz"].delete()

    # -- robot model
    def load_urdf(self, xml):
        import numpy as np
        import yourdfpy
        import meshcat.geometry as g
        urdf = yourdfpy.URDF.load(io.StringIO(xml), build_scene_graph=False, load_meshes=False)
        named = {m.name: m for m in urdf.robot.materials}
        with self.lock:
            self.vis["rviz/robot"].delete()
            self.links = set()
            for link in urdf.robot.links:
                self.links.add(link.name)
                for i, vis_ in enumerate(link.visuals):
                    geom = vis_.geometry
                    mat = vis_.material
                    if mat is not None and mat.color is None and mat.name in named:
                        mat = named[mat.name]
                    rgba = mat.color.rgba if (mat is not None and mat.color is not None) else [0.7, 0.7, 0.75, 1.0]
                    color = (int(rgba[0] * 255) << 16) + (int(rgba[1] * 255) << 8) + int(rgba[2] * 255)
                    material = g.MeshLambertMaterial(color=color, opacity=float(rgba[3]), transparent=bool(rgba[3] < 1))
                    local = np.eye(4) if vis_.origin is None else np.array(vis_.origin)
                    obj = None
                    if geom.box is not None:
                        obj = g.Box([float(v) for v in geom.box.size])
                    elif geom.sphere is not None:
                        obj = g.Sphere(float(geom.sphere.radius))
                    elif geom.cylinder is not None:
                        obj = g.Cylinder(float(geom.cylinder.length), float(geom.cylinder.radius))
                        local = local @ np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])
                    elif geom.mesh is not None:
                        try:
                            import trimesh
                            mesh = trimesh.load(_resolve_uri(geom.mesh.filename), force="mesh")
                            scale = np.ones(3) if geom.mesh.scale is None else np.array(geom.mesh.scale)
                            obj = g.TriangularMeshGeometry(np.array(mesh.vertices) * scale, np.array(mesh.faces))
                        except Exception as err:
                            print(f"rviz2: could not load mesh {geom.mesh.filename}: {err}")
                    if obj is not None:
                        node = self.vis[f"rviz/robot/{link.name}/v{i}"]
                        node.set_object(obj, material)
                        node.set_transform(local)

    # -- poses
    def set_link_pose(self, link, T):
        if link in self.links:
            with self.lock:
                self.vis[f"rviz/robot/{link}"].set_transform(T)

    def set_frame_pose(self, frame, T, size=0.15):
        import numpy as np
        import meshcat.geometry as g
        with self.lock:
            node = self.vis[f"rviz/tf/{frame}"]
            if frame not in self.frames:
                self.frames.add(frame)
                pts = np.array([[0, 0, 0], [size, 0, 0], [0, 0, 0], [0, size, 0], [0, 0, 0], [0, 0, size]], float).T
                cols = np.array([[1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 0, 1]], float).T
                node.set_object(g.LineSegments(g.PointsGeometry(pts, color=cols),
                                               g.LineBasicMaterial(vertexColors=True, linewidth=2)))
            node.set_transform(T)

    # -- sensor data
    def set_scan(self, points_xyz):
        import numpy as np
        import meshcat.geometry as g
        if len(points_xyz) == 0:
            return
        pts = np.asarray(points_xyz, float).T
        cols = np.tile(np.array([[1.0], [0.2], [0.2]]), (1, pts.shape[1]))
        with self.lock:
            self.vis["rviz/scan"].set_object(g.PointCloud(pts, cols, size=0.04))

    def set_map(self, data, width, height, resolution, T_origin):
        """data: occupancy values (-1 unknown, 0 free, 100 occupied), row-major."""
        import numpy as np
        import meshcat.geometry as g
        from PIL import Image
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        rgba = np.zeros((height, width, 4), np.uint8)
        rgba[grid == 0] = (235, 235, 235, 255)
        rgba[grid >= 65] = (20, 20, 20, 255)
        buf = io.BytesIO()
        Image.fromarray(np.flipud(rgba), "RGBA").save(buf, "PNG")
        tex = g.ImageTexture(image=g.PngImage(buf.getvalue()))
        plane = g.Box([width * resolution, height * resolution, 0.002])
        T = np.array(T_origin, float).copy()
        T[:3, 3] += T[:3, :3] @ np.array([width * resolution / 2, height * resolution / 2, -0.01])
        with self.lock:
            node = self.vis["rviz/map"]
            node.set_object(plane, g.MeshBasicMaterial(map=tex, transparent=True))
            node.set_transform(T)

    def set_visible(self, name, visible):
        with self.lock:
            self.vis[f"rviz/{name}"].set_property("visible", bool(visible))


class Rviz2(Node):
    """Feeds the Meshcat scene from ROS 2 topics, like the displays of RViz."""

    def __init__(self, fixed_frame=None, height=520):
        import ipywidgets as w
        import meshcat
        import tf2_ros
        from IPython.display import display
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data)
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        if _state.get("rviz"):
            _state["rviz"].close()
        super().__init__("rosforge_rviz2")
        if _state.get("vis") is None:
            import contextlib
            with contextlib.redirect_stdout(io.StringIO()):
                _state["vis"] = meshcat.Visualizer()
        self.fixed, self._auto = fixed_frame, fixed_frame is None
        self.scene = RvizScene(_state["vis"])
        self._scan, self._map, self._err = None, None, None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self._on_description, latched)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "_scan", m), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "_map", m), latched)
        self.create_timer(0.1, self._tick)

        self.status = w.HTML()
        boxes = {}
        for label, key in (("Robot model", "robot"), ("TF", "tf"), ("LaserScan", "scan"), ("Map", "map")):
            box = w.Checkbox(value=True, description=label, indent=False, layout=w.Layout(width="auto"))
            box.observe(lambda ch, k=key: self.scene.set_visible(k, ch["new"]), names="value")
            boxes[key] = box
        frame = w.HTML(_viewer_html(_state["vis"], height))
        display(w.VBox([w.HTML("<b>Displays</b>"), w.HBox(list(boxes.values())), frame, self.status]))
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        _state["rviz"] = self

    def _on_description(self, msg):
        try:
            self.scene.load_urdf(msg.data)
        except Exception as err:
            self._err = f"robot model: {err}"

    def _pose(self, frame):
        t = self.buffer.lookup_transform(self.fixed, frame, rclpy.time.Time())
        return _transform_to_matrix(t.transform)

    def _tick(self):
        import numpy as np
        import yaml
        problems = []
        try:
            frames = yaml.safe_load(self.buffer.all_frames_as_yaml()) or {}
            names = set(frames) | {v.get("parent") for v in frames.values() if v.get("parent")}
            if self._auto and names:  # re-check every time: /tf_static can arrive late
                roots = [f for f in names if f not in frames]
                self.fixed = "map" if "map" in names else sorted(roots)[0]
            if self.fixed is None:
                self.status.value = "Waiting for TF frames..."
                return
            if self.fixed not in names:
                self.status.value = f"The fixed frame '{self.fixed}' does not exist. Frames: {sorted(names)}"
                return
            for frame in names:
                try:
                    T = self._pose(frame)
                except Exception:
                    continue
                self.scene.set_frame_pose(frame, T)
                self.scene.set_link_pose(frame, T)
        except Exception as err:
            self.status.value = f"rviz2: {err}"
            return

        # a message is dropped only after it has been drawn, so nothing is lost
        # if the frames it needs have not arrived yet
        if self._scan is not None:
            try:
                scan = self._scan
                a = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
                r = np.array(scan.ranges, float)
                ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
                pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum()), np.ones(ok.sum())])
                self.scene.set_scan((self._pose(scan.header.frame_id) @ pts)[:3].T)
                if self._scan is scan:
                    self._scan = None
            except Exception as err:
                problems.append(f"scan: {err}")
        if self._map is not None:
            try:
                grid = self._map
                o = grid.info.origin
                origin = _quat_to_matrix(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
                origin[:3, 3] = [o.position.x, o.position.y, o.position.z]
                T = self._pose(grid.header.frame_id) @ origin
                self.scene.set_map(grid.data, grid.info.width, grid.info.height, grid.info.resolution, T)
                if self._map is grid:
                    self._map = None
            except Exception as err:
                problems.append(f"map: {err}")
        if self._err:
            problems.append(self._err)
        note = f" | warning: {'; '.join(problems)}" if problems else ""
        self.status.value = (f"Fixed frame: <b>{self.fixed}</b> | frames: {len(names)} | "
                             f"robot links: {len(self.scene.links)}{note}")

    def close(self):
        try:
            self._executor.shutdown()
            self.destroy_node()
        except Exception:
            pass


def rviz2(fixed_frame=None, height=520):
    """Open an RViz-like 3D view of the running robot inside the notebook."""
    Rviz2(fixed_frame, height)


# ================================================================ demo_robot
DEMO_URDF = """<robot name="demo_bot">
 <material name="blue"><color rgba="0.2 0.4 0.9 1"/></material>
 <material name="red"><color rgba="0.9 0.2 0.2 1"/></material>
 <material name="dark"><color rgba="0.15 0.15 0.18 1"/></material>
 <link name="base_link">
  <visual><origin xyz="0 0 0.06"/><geometry><box size="0.30 0.20 0.08"/></geometry><material name="blue"/></visual>
  <visual><origin xyz="0.16 0 0.06"/><geometry><box size="0.04 0.10 0.04"/></geometry><material name="red"/></visual>
 </link>
 <link name="left_wheel"><visual><origin rpy="1.5708 0 0"/><geometry><cylinder radius="0.05" length="0.03"/></geometry><material name="dark"/></visual></link>
 <link name="right_wheel"><visual><origin rpy="1.5708 0 0"/><geometry><cylinder radius="0.05" length="0.03"/></geometry><material name="dark"/></visual></link>
 <link name="laser"><visual><geometry><cylinder radius="0.04" length="0.04"/></geometry><material name="dark"/></visual></link>
 <joint name="left_joint" type="fixed"><parent link="base_link"/><child link="left_wheel"/><origin xyz="0 0.115 0"/></joint>
 <joint name="right_joint" type="fixed"><parent link="base_link"/><child link="right_wheel"/><origin xyz="0 -0.115 0"/></joint>
 <joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0 0 0.12"/></joint>
</robot>"""

ROOM_X, ROOM_Y, PILLAR_R = 3.0, 2.0, 0.3  # room is 6 m x 4 m, pillar at the center


def _demo_scan(px, py, yaw, n=360, noise=0.01):
    """Ranges seen by a 360-degree laser at (px, py) facing yaw, in the demo room."""
    import numpy as np
    ang = -math.pi + np.arange(n) * (2 * math.pi / n)
    dx, dy = np.cos(yaw + ang), np.sin(yaw + ang)
    with np.errstate(divide="ignore", invalid="ignore"):
        tx = np.where(dx > 1e-9, (ROOM_X - px) / dx, np.where(dx < -1e-9, (-ROOM_X - px) / dx, np.inf))
        ty = np.where(dy > 1e-9, (ROOM_Y - py) / dy, np.where(dy < -1e-9, (-ROOM_Y - py) / dy, np.inf))
    t = np.minimum(tx, ty)
    b = px * dx + py * dy
    disc = b * b - (px * px + py * py - PILLAR_R ** 2)
    hit = np.where(disc > 0, -b - np.sqrt(np.maximum(disc, 0)), np.inf)
    t = np.where(hit > 0, np.minimum(t, hit), t)
    return np.clip(t + np.random.normal(0, noise, n), 0.05, 8.0)


def _demo_map():
    """The demo room as an occupancy grid: -1 unknown, 0 free, 100 occupied."""
    import numpy as np
    w, h, res, ox, oy = 140, 100, 0.05, -3.5, -2.5
    X, Y = np.meshgrid(ox + (np.arange(w) + 0.5) * res, oy + (np.arange(h) + 0.5) * res)
    inside = (np.abs(X) < ROOM_X) & (np.abs(Y) < ROOM_Y)
    wall = inside & ((np.abs(X) > ROOM_X - res) | (np.abs(Y) > ROOM_Y - res))
    grid = np.full((h, w), -1, np.int8)
    grid[inside] = 0
    grid[wall | (X * X + Y * Y < PILLAR_R ** 2)] = 100
    return w, h, res, ox, oy, grid


class DemoRobot(Node):
    """A stand-in robot: drives circles around a pillar and publishes what a real robot would."""

    def __init__(self):
        import tf2_ros
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        if _state.get("demo"):
            _state["demo"].close()
        super().__init__("demo_robot")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._LaserScan = LaserScan
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        desc = self.create_publisher(String, "/robot_description", latched)
        desc.publish(String(data=DEMO_URDF))
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        w, h, res, ox, oy, grid = _demo_map()
        m = OccupancyGrid()
        m.header.frame_id = "map"
        m.info.resolution, m.info.width, m.info.height = res, w, h
        m.info.origin.position.x, m.info.origin.position.y, m.info.origin.orientation.w = ox, oy, 1.0
        m.data = grid.ravel().tolist()
        self.map_pub.publish(m)
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.static = tf2_ros.StaticTransformBroadcaster(self)
        self.static.sendTransform([self._tf("map", "odom", 0, 0, 0), self._tf("base_link", "laser", 0, 0, 0.12),
                                   self._tf("base_link", "left_wheel", 0, 0.115, 0),
                                   self._tf("base_link", "right_wheel", 0, -0.115, 0)])
        self._t0, self._n = time.time(), 0
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self.create_timer(0.05, self._tick)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        _state["demo"] = self

    def _tf(self, parent, child, x, y, z, yaw=0.0):
        from geometry_msgs.msg import TransformStamped
        m = TransformStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = parent, child
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = float(x), float(y), float(z)
        m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return m

    def _tick(self):
        angle = 0.3 * (time.time() - self._t0)  # position on a circle around the pillar
        x, y, yaw = 1.3 * math.cos(angle), 1.3 * math.sin(angle), angle + math.pi / 2
        self.tf.sendTransform(self._tf("odom", "base_link", x, y, 0.05, yaw))
        self._n += 1
        if self._n % 2 == 0:  # laser at 10 Hz
            n = 360
            msg = self._LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "laser"
            msg.angle_min, msg.angle_max = -math.pi, math.pi - 2 * math.pi / n
            msg.angle_increment, msg.range_min, msg.range_max = 2 * math.pi / n, 0.05, 8.0
            msg.ranges = _demo_scan(x, y, yaw, n).astype("float32").tolist()
            self.scan_pub.publish(msg)

    def close(self):
        try:
            self._executor.shutdown()
            self.destroy_node()
        except Exception:
            pass


def demo_robot():
    """Start a stand-in robot that publishes /robot_description, /tf, /scan and /map."""
    DemoRobot()


def stop_demo_robot():
    """Stop the stand-in robot."""
    if _state.get("demo"):
        _state["demo"].close()
        _state["demo"] = None


# ============================================================ URDF tools
import atexit
import os
import signal


def _check_urdf(xml):
    """Parse the URDF and return the yourdfpy model (raises on errors)."""
    import xml.etree.ElementTree as ET
    import yourdfpy
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as err:
        raise ValueError(f"the XML is not valid ({err}). Check that every tag is closed.")
    if root.tag != "robot":
        raise ValueError("the file must start with <robot name=\"...\">")
    return yourdfpy.URDF.load(io.StringIO(xml), build_scene_graph=False, load_meshes=False)


def stop_robot():
    """Stop the robot_state_publisher started by start_robot()."""
    proc = _state.get("rsp")
    if proc and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=3)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass
    _state["rsp"] = None


atexit.register(stop_robot)


def start_robot(urdf_file):
    """Run robot_state_publisher on a URDF file.

    It is the same as typing, in a Terminal:
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat file.urdf)"
    """
    import subprocess
    import tempfile
    import yaml
    with open(urdf_file) as f:
        xml = f.read()
    try:
        _check_urdf(xml)
    except Exception as err:
        print(f"There is a problem in {urdf_file}:\n  {err}")
        return
    stop_robot()
    params = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump({"robot_state_publisher": {"ros__parameters": {"robot_description": xml}}}, params)
    params.close()
    log_path = "/tmp/robot_state_publisher.log"
    log = open(log_path, "w")
    _state["rsp"] = subprocess.Popen(
        ["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args", "--params-file", params.name],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(1.5)
    if _state["rsp"].poll() is not None:
        print("robot_state_publisher stopped. Last lines of its log:")
        print("".join(open(log_path).readlines()[-12:]))
    else:
        print(f"robot_state_publisher is running with {urdf_file}")


class JointSliders(Node):
    """One slider per movable joint. Publishes /joint_states, like joint_state_publisher_gui."""

    def __init__(self):
        import ipywidgets as w
        from IPython.display import display
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        if _state.get("sliders"):
            _state["sliders"].close()
        super().__init__("rosforge_joint_sliders")
        self._JointState, self._w = JointState, w
        self.values = {}
        self.box = w.VBox([w.HTML("Waiting for a robot (run start_robot first)...")])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self._on_description, latched)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.05, self._publish)
        display(self.box)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        _state["sliders"] = self

    def _on_description(self, msg):
        w = self._w
        try:
            joints = [j for j in _check_urdf(msg.data).robot.joints if j.type in ("continuous", "revolute", "prismatic")]
        except Exception:
            return
        sliders, values = [], {}
        for j in joints:
            lo, hi = -math.pi, math.pi
            if j.type != "continuous" and j.limit is not None:
                lo, hi = float(j.limit.lower or 0.0), float(j.limit.upper or 0.0)
            values[j.name] = 0.0 if lo <= 0.0 <= hi else lo
            s = w.FloatSlider(value=values[j.name], min=lo, max=hi, step=0.01, description=j.name,
                              layout=w.Layout(width="560px"), style={"description_width": "170px"})
            s.observe(lambda ch, n=j.name: self.values.__setitem__(n, ch["new"]), names="value")
            sliders.append(s)
        self.values = values
        self.box.children = sliders or [w.HTML("This robot has no movable joints.")]

    def _publish(self):
        if self.values:
            msg = self._JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.values)
            msg.position = [float(v) for v in self.values.values()]
            self.pub.publish(msg)

    def close(self):
        try:
            self._executor.shutdown()
            self.destroy_node()
        except Exception:
            pass


def joint_sliders():
    """Show one slider per movable joint of the robot."""
    JointSliders()


# ================================================================ TF tools
def render_tree(frames):
    """SVG of the TF tree. `frames` is the dict from tf2's all_frames_as_yaml()."""
    names = set(frames) | {v.get("parent") for v in frames.values() if v.get("parent")}
    if not names:
        return ('<svg width="520" height="70"><text x="10" y="40" font-family="sans-serif" '
                'font-size="15" fill="#6b7280">No frames yet.</text></svg>')
    children = defaultdict(list)
    for child, v in frames.items():
        children[v.get("parent")].append(child)
    roots = sorted(n for n in names if n not in frames)
    pos, row = {}, [0]

    def place(name, depth):
        kids = sorted(children.get(name, []))
        if kids:
            ys = [place(k, depth + 1) for k in kids]
            y = sum(ys) / len(ys)
        else:
            y, row[0] = row[0], row[0] + 1
        pos[name] = (depth, y)
        return y

    for r in roots:
        place(r, 0)
    depth_w = defaultdict(float)
    for n, (d, _) in pos.items():
        depth_w[d] = max(depth_w[d], _width(n))
    xs, x = {}, 20
    for d in range(max(depth_w) + 1):
        xs[d] = x + depth_w[d] / 2
        x += depth_w[d] + 150
    row_h = 62
    width, height = x - 150 + 20, max(row[0], 1) * row_h + 30
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}">',
           '<defs><marker id="tfarrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" '
           'orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#6b7280"/></marker></defs>']
    centers = {n: (xs[d], 25 + y * row_h + row_h / 2 - 15) for n, (d, y) in pos.items()}
    for child, v in frames.items():
        parent = v.get("parent")
        if parent not in centers or child not in centers:
            continue
        (px, py), (cx, cy) = centers[parent], centers[child]
        x1, x2 = px + _width(parent) / 2, cx - _width(child) / 2 - 2
        mid = (x1 + x2) / 2
        out.append(f'<path d="M{x1:.0f},{py:.0f} C{mid:.0f},{py:.0f} {mid:.0f},{cy:.0f} {x2:.0f},{cy:.0f}" fill="none" '
                   f'stroke="#6b7280" stroke-width="1.8" marker-end="url(#tfarrow)"/>')
        rate = float(v.get("rate", 0) or 0)
        label = "static" if rate >= 1000 else f"{rate:.0f} Hz"
        out.append(f'<text x="{mid:.0f}" y="{(py + cy) / 2 - 8:.0f}" text-anchor="middle" font-family="sans-serif" '
                   f'font-size="12" fill="#2563eb">{label}</text>')
    for n, (cx, cy) in centers.items():
        w_ = _width(n)
        out.append(f'<rect x="{cx - w_ / 2:.0f}" y="{cy - 17:.0f}" width="{w_:.0f}" height="34" rx="8" '
                   f'fill="#dbeafe" stroke="#2563eb" stroke-width="2"/>')
        out.append(f'<text x="{cx:.0f}" y="{cy + 5:.0f}" text-anchor="middle" font-family="sans-serif" '
                   f'font-size="13" fill="#111827">{n}</text>')
    out.append("</svg>")
    return "".join(out)


def view_frames(wait=2.0):
    """Draw the TF tree: every frame, its parent, and how often it is updated.

    Like: ros2 run tf2_tools view_frames"""
    import tf2_ros
    import yaml
    from IPython.display import SVG, display
    ros = _ensure_ros()
    if _state.get("tf", (None,))[0] is not ros:
        buffer = tf2_ros.Buffer()
        _state["tf"] = (ros, buffer, tf2_ros.TransformListener(buffer, ros, spin_thread=True))
        time.sleep(wait)  # collect some frames
    frames = yaml.safe_load(_state["tf"][1].all_frames_as_yaml()) or {}
    display(SVG(render_tree(frames)))


def _draw_frames(ax, x, y, deg, cup=(1.0, 0.4)):
    """Plot the map frame, a robot frame at (x, y, deg) and a cup seen by the robot."""
    import numpy as np
    th = math.radians(deg)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    p_robot = np.array(cup, float)
    p_map = R @ p_robot + np.array([x, y])
    ax.axhline(0, color="#d1d5db", lw=1)
    ax.axvline(0, color="#d1d5db", lw=1)
    ax.annotate("", xy=(0.8, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#dc2626", lw=2))
    ax.annotate("", xy=(0, 0.8), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#16a34a", lw=2))
    ax.text(0.05, -0.3, "map", color="#374151", fontsize=11, fontweight="bold")
    body = np.array([[0.35, 0], [-0.2, 0.2], [-0.2, -0.2]]).T
    ax.fill(*(R @ body + np.array([[x], [y]])), color="#2563eb", alpha=0.85)
    for axis, col in (((0.7, 0.0), "#dc2626"), ((0.0, 0.7), "#16a34a")):
        end = R @ np.array(axis) + np.array([x, y])
        ax.annotate("", xy=end, xytext=(x, y), arrowprops=dict(arrowstyle="->", color=col, lw=2))
    ax.text(x - 0.1, y - 0.5, "base_link", color="#1d4ed8", fontsize=11, fontweight="bold")
    ax.plot([x, p_map[0]], [y, p_map[1]], "--", color="#9ca3af")
    ax.plot(*p_map, "*", color="#f59e0b", markersize=18)
    ax.text(p_map[0] + 0.1, p_map[1] + 0.1, "cup", fontsize=11)
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 3)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title(f"cup in base_link: ({p_robot[0]:.2f}, {p_robot[1]:.2f})     "
                 f"cup in map: ({p_map[0]:.2f}, {p_map[1]:.2f})", fontsize=11)
    return p_map


def frames_demo():
    """Move a robot around a map and see the same cup in two frames."""
    import ipywidgets as w
    import matplotlib.pyplot as plt
    from IPython.display import clear_output, display
    out = w.Output()
    style = {"description_width": "90px"}
    sx = w.FloatSlider(value=2.0, min=-3, max=3, step=0.1, description="robot x", style=style)
    sy = w.FloatSlider(value=1.0, min=-2, max=2, step=0.1, description="robot y", style=style)
    sd = w.FloatSlider(value=30, min=-180, max=180, step=5, description="heading (°)", style=style)

    def draw(*_):
        with out:
            clear_output(wait=True)
            fig, ax = plt.subplots(figsize=(7, 5))
            _draw_frames(ax, sx.value, sy.value, sd.value)
            plt.show()
            plt.close(fig)

    for s in (sx, sy, sd):
        s.observe(draw, names="value")
    display(w.VBox([sx, sy, sd, out]))
    draw()
