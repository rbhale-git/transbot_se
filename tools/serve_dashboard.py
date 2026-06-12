"""Static server for the dashboard with caching disabled + behavior API.

Plain `python -m http.server` sends no cache headers, so browsers heuristically
cache ES modules — after an update, a stale module mixed into the new graph
breaks the entire import chain and the dashboard goes dead until a hard
refresh. This server sends Cache-Control: no-store so every load is current.

It also exposes a tiny LOCALHOST-ONLY API so the dashboard's AI panel can
start/stop the person-following runner: a web page cannot spawn local
processes (browser sandbox), but this server can. One behavior process at a
time; output goes to runner.log in the repo root.

  GET  /api/behavior        -> {running, pid, args, started_s_ago, last_exit}
  POST /api/behavior/start  -> body {} or {"target_class": "dog", "cap_scale": 0.5}
  POST /api/behavior/stop
  POST /api/heal            -> body {} or {"profile": "home"}; SSH-restarts
                               rosbridge-dashboard.service on the robot and
                               waits for it (409 while a behavior is running)

Stopping is a hard terminate: the runner's parting-zero `finally` does not
run, but the robot-side mux zeroes /cmd_vel on AI silence and the watchdog
backs that up — stop is safe by architecture, not by cleanup.

Run from the repo root:  python tools/serve_dashboard.py  [port]
"""

import http.server
import json
import os
import re
import subprocess
import sys
import threading
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.join(REPO, "dashboard")
RUNNER_LOG = os.path.join(REPO, "runner.log")

sys.path.insert(0, REPO)
from ai import config as ai_config  # noqa: E402
from ai.common.heal import heal, host_from_url  # noqa: E402

# Only values matching these ever reach the command line (list-form Popen,
# no shell — this is belt and braces, not the only defense).
TARGET_CLASS_RE = re.compile(r"^[a-z ]{1,30}$")
CAP_SCALE_RANGE = (0.1, 1.0)


class BehaviorRunner:
    """Owns at most one AI behavior subprocess."""

    def __init__(self):
        self._lock = threading.Lock()
        self._proc = None
        self._args = []
        self._started_at = None
        self._last_exit = None

    def status(self):
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            if self._proc is not None and not running:
                self._last_exit = self._proc.returncode
            return {
                "running": running,
                "pid": self._proc.pid if running else None,
                "args": self._args if running else [],
                "started_s_ago": int(time.time() - self._started_at)
                if running and self._started_at else None,
                "last_exit": None if running else self._last_exit,
            }

    def start(self, target_class=None, cap_scale=None, preview=False):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False, "already running"
            # The spawned process inherits this server's desktop session, so
            # the cv2 preview window appears normally when requested (q in
            # the preview quits the runner; the STOP button always works).
            cmd = [sys.executable, "-u", "-m", "ai.person_following"]
            if not preview:
                cmd.append("--no-preview")
            if target_class is not None:
                if not TARGET_CLASS_RE.match(target_class):
                    return False, "bad target_class"
                cmd += ["--target-class", target_class]
            if cap_scale is not None:
                try:
                    cap_scale = float(cap_scale)
                except (TypeError, ValueError):
                    return False, "bad cap_scale"
                if not CAP_SCALE_RANGE[0] <= cap_scale <= CAP_SCALE_RANGE[1]:
                    return False, "bad cap_scale"
                cmd += ["--cap-scale", str(cap_scale)]
            log = open(RUNNER_LOG, "w", encoding="utf-8")
            self._proc = subprocess.Popen(
                cmd, cwd=REPO, stdout=log, stderr=subprocess.STDOUT)
            self._args = cmd[2:]  # drop interpreter + -u for display
            self._started_at = time.time()
            self._last_exit = None
            print(f"behavior started: pid {self._proc.pid}  {' '.join(self._args)}")
            return True, None

    def stop(self):
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                return False, "not running"
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait(timeout=5)
            self._last_exit = self._proc.returncode
            print(f"behavior stopped (exit {self._last_exit})")
            return True, None


RUNNER = BehaviorRunner()

# One robot restart at a time — a second heal click (or stray client) must
# not stack systemctl restarts while one is already in flight.
HEAL_LOCK = threading.Lock()


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Expires", "0")
        super().end_headers()

    # ---- behavior API (localhost only) ------------------------------------

    def _api_allowed(self):
        return self.client_address[0] in ("127.0.0.1", "::1")

    def _send_json(self, payload, code=200):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/behavior":
            if not self._api_allowed():
                return self._send_json({"error": "localhost only"}, 403)
            return self._send_json(RUNNER.status())
        super().do_GET()

    def do_POST(self):
        if not self.path.startswith("/api/"):
            return self._send_json({"error": "not found"}, 404)
        if not self._api_allowed():
            return self._send_json({"error": "localhost only"}, 403)
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            return self._send_json({"error": "bad json"}, 400)

        if self.path == "/api/behavior/start":
            ok, err = RUNNER.start(body.get("target_class"), body.get("cap_scale"),
                                   preview=bool(body.get("preview")))
        elif self.path == "/api/behavior/stop":
            ok, err = RUNNER.stop()
        elif self.path == "/api/heal":
            return self._heal(body)
        else:
            return self._send_json({"error": "not found"}, 404)
        if not ok:
            return self._send_json({"error": err}, 409)
        return self._send_json(RUNNER.status())

    def _heal(self, body):
        # A heal restarts the WHOLE robot stack; never yank it out from
        # under a live behavior — the runner has its own preflight heal
        # anyway. A START racing in during the ~30 s heal is accepted: the
        # runner's own preflight ladder absorbs a mid-restart connect.
        if RUNNER.status()["running"]:
            return self._send_json({"error": "stop the runner first"}, 409)
        if not HEAL_LOCK.acquire(blocking=False):
            return self._send_json({"error": "heal already in progress"}, 409)
        try:
            profile = body.get("profile") or ai_config.DEFAULT_PROFILE
            if profile not in ai_config.PROFILES:
                return self._send_json({"error": "unknown profile"}, 400)
            host = host_from_url(ai_config.PROFILES[profile]["rosbridge_url"])
            print(f"heal requested: restarting the robot stack on {host} ...")
            t0 = time.time()
            ok, detail = heal(host)
            print(f"heal {'succeeded' if ok else 'FAILED'}: {detail}")
            return self._send_json(
                {"ok": ok, "detail": detail, "seconds": int(time.time() - t0)},
                200 if ok else 502)
        finally:
            HEAL_LOCK.release()


if __name__ == "__main__":
    with http.server.ThreadingHTTPServer(("", PORT), NoCacheHandler) as httpd:
        print(f"Dashboard at http://localhost:{PORT}  (no-cache, behavior API, Ctrl+C to stop)")
        try:
            httpd.serve_forever()
        finally:
            # Never orphan a chassis-driving process when the server dies.
            RUNNER.stop()
