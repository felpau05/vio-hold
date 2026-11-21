#!/usr/bin/env python3
"""Stick view for the D/C rehearsal: the FC channel values as radio sticks, like the Betaflight Receiver tab.
Reads http://192.168.75.1:8080/state (apb_live.py --rc) 5 times a second. Changes nothing on the drone.
Run in WSL: python3 stick_view.py   then open http://localhost:8090 in the Windows browser."""
import json
import os
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

APB = "http://192.168.75.1:8080/state"
PORT = 8090

HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stick_view.html")   # read on each request


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        b = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith("/state"):
            try:
                with urllib.request.urlopen(APB, timeout=1.0) as r:
                    self._send(200, r.read().decode(), "application/json")
            except Exception as ex:                     # noqa: BLE001
                self._send(200, json.dumps({"error": f"{type(ex).__name__}: {ex}"}), "application/json")
        else:
            self._send(200, open(HTML).read(), "text/html; charset=utf-8")


if __name__ == "__main__":
    print(f"stick view on http://localhost:{PORT}")
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
