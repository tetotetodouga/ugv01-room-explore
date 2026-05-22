#!/usr/bin/env python3

import json
import mimetypes
import os
import signal
import subprocess
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse

REPO_ROOT = Path(__file__).resolve().parents[1]
WEB_DIR = REPO_ROOT / "web"
STATIC_DIR = WEB_DIR / "static"
ENV_PATH = WEB_DIR / ".env"

AUTO_PROCESS = None
MODE = "JOY"

ROS_LOG_PATH = "/tmp/ugv01_server_ros_commands.log"


def load_env(path: Path):
    if not path.exists():
        return

    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


load_env(ENV_PATH)

WEB_HOST = os.getenv("WEB_HOST", "0.0.0.0")
WEB_PORT = int(os.getenv("WEB_PORT", "8080"))
DASHBOARD_TOKEN = os.getenv("DASHBOARD_TOKEN", "")

ROS_SETUP = os.getenv("ROS_SETUP", "/opt/ros/jazzy/setup.bash")
WORKSPACE_SETUP = os.getenv("WORKSPACE_SETUP", str(Path.home() / "ros2_ws/install/setup.bash"))

AUTO_CMD = os.getenv(
    "AUTO_CMD",
    "ros2 run ugv01_room_explore auto_explore --ros-args "
    "-r /cmd_vel:=/cmd_vel_auto "
    "-r cmd_vel:=/cmd_vel_auto"
)


def ros_cmd(command: str):
    return [
        "bash",
        "-lc",
        f"source {ROS_SETUP} && source {WORKSPACE_SETUP} && {command}",
    ]


def ros_run(command: str, timeout: float = 8.0):
    return subprocess.run(
        ros_cmd(command),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=timeout,
    )


def ros_fire(command: str):
    log = open(ROS_LOG_PATH, "a", buffering=1)
    return subprocess.Popen(
        ros_cmd(command),
        stdout=log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def publish_mode(mode: str):
    # Do not wait for ros2 CLI; it may take a few seconds for DDS discovery.
    ros_fire(f'ros2 topic pub --once /ugv01/mode std_msgs/msg/String "{{data: \'{mode}\'}}"')


def publish_stop():
    # Non-blocking STOP commands. Mode manager + watchdog will enforce zero velocity.
    ros_fire("ros2 topic pub --once /ugv01/stop std_msgs/msg/Empty '{}'")

    zero = "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    for _ in range(5):
        ros_fire(f"ros2 topic pub --once /cmd_vel_joy geometry_msgs/msg/Twist '{zero}'")
        ros_fire(f"ros2 topic pub --once /cmd_vel_auto geometry_msgs/msg/Twist '{zero}'")
        time.sleep(0.03)


def auto_running():
    global AUTO_PROCESS
    return AUTO_PROCESS is not None and AUTO_PROCESS.poll() is None


def start_auto_process():
    global AUTO_PROCESS

    if auto_running():
        return

    log = open("/tmp/ugv01_auto_explore.log", "a", buffering=1)

    AUTO_PROCESS = subprocess.Popen(
        ros_cmd(AUTO_CMD),
        stdout=log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def stop_auto_process():
    global AUTO_PROCESS

    if AUTO_PROCESS is None:
        return

    if AUTO_PROCESS.poll() is None:
        try:
            os.killpg(os.getpgid(AUTO_PROCESS.pid), signal.SIGTERM)
            AUTO_PROCESS.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(AUTO_PROCESS.pid), signal.SIGKILL)
            except Exception:
                pass

    AUTO_PROCESS = None


def status():
    return {
        "ok": True,
        "mode": MODE,
        "auto_running": auto_running(),
        "auto_pid": AUTO_PROCESS.pid if auto_running() else None,
        "cmd_chain": "/cmd_vel_joy or /cmd_vel_auto -> /cmd_vel_web -> /cmd_vel",
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print("[server]", fmt % args)

    def send_json(self, code: int, payload: dict):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def authorized(self):
        if not DASHBOARD_TOKEN:
            return True
        return self.headers.get("X-Dashboard-Token", "") == DASHBOARD_TOKEN

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/api/status":
            self.send_json(200, status())
            return

        if path == "/":
            path = "/index.html"

        file_path = (STATIC_DIR / path.lstrip("/")).resolve()

        if not str(file_path).startswith(str(STATIC_DIR.resolve())):
            self.send_error(403)
            return

        if not file_path.exists() or not file_path.is_file():
            self.send_error(404)
            return

        data = file_path.read_bytes()
        content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        global MODE

        if not self.authorized():
            self.send_json(401, {"ok": False, "error": "unauthorized"})
            return

        path = urlparse(self.path).path

        try:
            if path == "/api/mode/auto":
                # JOY -> AUTO: без стопа. Просто запускаем автономку и просим manager ждать auto-команды.
                start_auto_process()
                publish_mode("AUTO")
                MODE = "AUTO"
                self.send_json(200, status())
                return

            if path == "/api/mode/joy":
                # AUTO -> JOY: выключаем автономку и останавливаемся.
                stop_auto_process()
                publish_mode("JOY")
                publish_stop()
                MODE = "JOY"
                self.send_json(200, status())
                return

            if path in ("/api/stop", "/api/stop_robot"):
                stop_auto_process()
                publish_mode("STOP")
                publish_stop()
                MODE = "JOY"
                self.send_json(200, status())
                return

            self.send_json(404, {"ok": False, "error": "unknown endpoint"})
        except Exception as e:
            self.send_json(500, {"ok": False, "error": str(e)})


def main():
    print("=== UGV01-X3 Dashboard Server ===")
    print(f"HTTP: http://{WEB_HOST}:{WEB_PORT}")
    print(f"STATIC: {STATIC_DIR}")
    print(f"TOKEN: {'enabled' if DASHBOARD_TOKEN else 'disabled'}")
    print(f"AUTO_CMD: {AUTO_CMD}")

    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), Handler)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        stop_auto_process()
        publish_mode("STOP")
        publish_stop()
        server.server_close()


if __name__ == "__main__":
    main()
