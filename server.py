from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

HOST = "127.0.0.1"
PORT = 8765
ROOT = Path(__file__).resolve().parent

class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

if __name__ == "__main__":
    print(f"Cable Tray Designer running at http://{HOST}:{PORT}")
    print(f"Project directory: {ROOT}")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
