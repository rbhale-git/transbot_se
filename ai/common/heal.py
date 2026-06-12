"""Cure the robot's rosbridge silent-drop bug from the laptop.

The Melodic rosbridge (0.11) recurrently half-fails publisher registration
per topic and silently discards every command on it; the only reliable cure
is restarting rosbridge-dashboard.service over SSH (passwordless `sudo -n`
is configured and verified — docs/AI_NOTEBOOK.md). The service runs the
WHOLE robot stack (bringup + usb_cam + web_video_server + rosbridge), so a
heal drops the camera and every client connection for ~15-30 s. Callers
must only run it with the robot stationary.

Used by: the behavior preflight (ai/common/connect.py), the dashboard
server's POST /api/heal (tools/serve_dashboard.py), and the manual CLI
(tools/heal_rosbridge.py). It also clears the wedged-/voltage-publisher
quirk seen after power cycles.
"""

import socket
import subprocess
import time

SSH_USER = "jetson"
SERVICE = "rosbridge-dashboard.service"
ROSBRIDGE_PORT = 9090
SSH_TIMEOUT_S = 30
POLL_INTERVAL_S = 2.0


def host_from_url(url):
    """ws://192.168.0.109:9090 -> 192.168.0.109 (dashboard-style URL)."""
    stripped = url.replace("ws://", "").rstrip("/")
    return stripped.partition(":")[0]


def restart_command(host):
    # BatchMode: fail fast instead of hanging on a password prompt if the
    # key is missing; sudo -n likewise (passwordless sudo is set up).
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
            f"{SSH_USER}@{host}",
            "sudo", "-n", "systemctl", "restart", SERVICE]


def _port_open(host, port, timeout_s=2.0):
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def heal(host, timeout_s=60.0, run=subprocess.run, port_open=_port_open,
         sleep=time.sleep, clock=time.monotonic):
    """Restart the robot stack, wait for rosbridge to accept connections.

    Returns (ok, detail). run/port_open/sleep/clock are injection points
    for the unit tests; production callers pass only host.
    """
    try:
        result = run(restart_command(host), capture_output=True, text=True,
                     timeout=SSH_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return False, "ssh timed out"
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        return False, detail or f"ssh exited {result.returncode}"
    deadline = clock() + timeout_s
    while clock() < deadline:
        if port_open(host, ROSBRIDGE_PORT):
            return True, f"{SERVICE} restarted; rosbridge port is back"
        sleep(POLL_INTERVAL_S)
    return False, (f"{SERVICE} restarted but port {ROSBRIDGE_PORT} "
                   f"not back within {timeout_s:.0f}s")
