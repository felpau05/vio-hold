"""Live preview of the deck: rebuilds index.html when a slide, deck.css, deck.json or a figure changes,
and the open browser tab reloads by itself (staying on the same slide).

  ~/.venvs/vio/bin/python talk/deck/serve.py        then open http://localhost:8000/deck/
  Ctrl+C to stop. Only for editing: present from index.html as usual.
"""
import glob
import http.server
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TALK = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import build  # noqa: E402

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
RELOAD = """<script>
(() => { let v = null;
  setInterval(async () => {
    try { const n = await (await fetch('/__version', {cache: 'no-store'})).text();
      if (v !== null && n !== v) location.reload(); v = n; } catch (e) {}
  }, 700); })();
</script></body>"""


def sources():
    return (glob.glob(os.path.join(HERE, "project", "**", "*"), recursive=True)
            + glob.glob(os.path.join(TALK, "stage2", "*.png")) + [os.path.join(HERE, "build.py")])


def version():
    return str(max(os.path.getmtime(p) for p in sources() if os.path.isfile(p)))


built = None


def rebuild_if_needed():
    global built
    v = version()
    if v != built:
        try:
            build.main()
        except Exception as e:  # keep serving the last good build
            print(f"build failed: {e}")
        built = v
    return v


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=TALK, **k)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        path = self.path.split("?")[0].split("#")[0]
        if path == "/__version":
            body = rebuild_if_needed().encode()
        elif path in ("/", "/deck", "/deck/", "/deck/index.html"):
            if path in ("/", "/deck"):
                self.send_response(302)
                self.send_header("Location", "/deck/")
                self.end_headers()
                return
            rebuild_if_needed()
            body = open(os.path.join(HERE, "index.html")).read().replace("</body>", RELOAD, 1).encode()
        else:
            return super().do_GET()
        self.send_response(200)
        self.send_header("Content-Type", "text/plain" if path == "/__version" else "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        if "/__version" not in (args[0] if args else ""):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    rebuild_if_needed()
    print(f"deck preview: http://localhost:{PORT}/deck/  (edit talk/deck/project/slides/*.html; Ctrl+C to stop)")
    http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
