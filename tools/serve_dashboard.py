"""Static server for the dashboard with caching disabled.

Plain `python -m http.server` sends no cache headers, so browsers heuristically
cache ES modules — after an update, a stale module mixed into the new graph
breaks the entire import chain and the dashboard goes dead until a hard
refresh. This server sends Cache-Control: no-store so every load is current.

Run from the repo root:  python tools/serve_dashboard.py  [port]
"""

import http.server
import os
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard")


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Expires", "0")
        super().end_headers()


if __name__ == "__main__":
    with http.server.ThreadingHTTPServer(("", PORT), NoCacheHandler) as httpd:
        print(f"Dashboard at http://localhost:{PORT}  (no-cache, Ctrl+C to stop)")
        httpd.serve_forever()
