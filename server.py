from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse
import json
import os
import shutil
import subprocess
import tempfile
import time
import uuid

HOST = "127.0.0.1"
PORT = 8765
ROOT = Path(__file__).resolve().parent
SOLIDWORKS_RESULTS = {}
RESULT_TTL_SECONDS = 3600


def cleanup_solidworks_results():
    now = time.time()
    for token, info in list(SOLIDWORKS_RESULTS.items()):
        if now - info["created_at"] > RESULT_TTL_SECONDS:
            try:
                Path(info["path"]).unlink(missing_ok=True)
            except OSError:
                pass
            SOLIDWORKS_RESULTS.pop(token, None)


def json_response(handler, status_code, payload):
    data = json.dumps(payload).encode("utf-8")
    handler.send_response(status_code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(data)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(data)


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_POST(self):
        route = urlparse(self.path).path
        if route == "/api/solidworks/pick":
            try:
                import tkinter as tk
                from tkinter import filedialog

                root = tk.Tk()
                root.withdraw()
                root.attributes("-topmost", True)
                selected = filedialog.askopenfilename(
                    title="Select SolidWorks Assembly or Part",
                    filetypes=[
                        ("SolidWorks Assembly", "*.sldasm"),
                        ("SolidWorks Part", "*.sldprt"),
                        ("SolidWorks files", "*.sldasm;*.sldprt"),
                        ("All files", "*.*"),
                    ],
                )
                root.destroy()
                if not selected:
                    return json_response(self, 400, {"error": "No SolidWorks file selected."})
                p = Path(selected).resolve()
                if p.suffix.lower() not in {".sldasm", ".sldprt"}:
                    return json_response(self, 400, {"error": "Please select an .SLDASM or .SLDPRT file."})
                return json_response(self, 200, {"path": str(p), "name": p.name})
            except Exception as exc:
                return json_response(self, 500, {"error": f"Windows file picker failed: {exc}"})

        if route == "/api/solidworks/import":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 32 * 1024:
                    return json_response(self, 400, {"error": "Invalid request."})
                body = self.rfile.read(length)
                payload = json.loads(body.decode("utf-8"))
                source = Path(str(payload.get("path", ""))).expanduser().resolve()
                if not source.exists():
                    return json_response(self, 400, {"error": "Selected SolidWorks file does not exist."})
                if source.suffix.lower() not in {".sldasm", ".sldprt"}:
                    return json_response(self, 400, {"error": "Only .SLDASM and .SLDPRT are supported by this importer."})

                cache_dir = Path(tempfile.gettempdir()) / "Cable_tray" / "solidworks"
                cache_dir.mkdir(parents=True, exist_ok=True)
                token = uuid.uuid4().hex
                output = cache_dir / f"{token}.stl"

                bridge = ROOT / "solidworks_bridge.vbs"
                if not bridge.exists():
                    return json_response(self, 500, {"error": "SolidWorks bridge script is missing."})

                command = ["cscript.exe", "//nologo", str(bridge), str(source), str(output)]
                completed = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=900,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )

                stdout = (completed.stdout or "").strip()
                stderr = (completed.stderr or "").strip()
                if completed.returncode != 0 or not output.exists() or output.stat().st_size == 0:
                    detail = stdout or stderr or f"cscript exited with code {completed.returncode}"
                    return json_response(self, 500, {"error": detail})

                load_errors = 0
                warnings = 0
                for line in stdout.splitlines():
                    if line.startswith("OK|"):
                        parts = line.split("|")
                        if len(parts) >= 3:
                            try:
                                load_errors = int(parts[1])
                                warnings = int(parts[2])
                            except ValueError:
                                pass

                SOLIDWORKS_RESULTS[token] = {
                    "path": str(output),
                    "created_at": time.time(),
                }
                cleanup_solidworks_results()
                return json_response(self, 200, {
                    "name": source.name,
                    "format": "SLDASM -> STL" if source.suffix.lower() == ".sldasm" else "SLDPRT -> STL",
                    "native_format": source.suffix.upper().lstrip("."),
                    "url": f"/api/solidworks/result/{token}.stl",
                    "load_errors": load_errors,
                    "warnings": warnings,
                    "bridge_output": stdout[-2000:],
                })
            except subprocess.TimeoutExpired:
                return json_response(self, 500, {"error": "SolidWorks conversion timed out after 15 minutes."})
            except Exception as exc:
                return json_response(self, 500, {"error": f"SolidWorks import failed: {exc}"})

        return json_response(self, 404, {"error": "Not found."})

    def do_GET(self):
        route = urlparse(self.path).path
        prefix = "/api/solidworks/result/"
        if route.startswith(prefix) and route.endswith(".stl"):
            cleanup_solidworks_results()
            token = route[len(prefix):-4]
            info = SOLIDWORKS_RESULTS.get(token)
            if not info:
                return json_response(self, 404, {"error": "Converted SolidWorks model expired or was not found."})
            path = Path(info["path"])
            if not path.exists():
                SOLIDWORKS_RESULTS.pop(token, None)
                return json_response(self, 404, {"error": "Converted SolidWorks model is no longer available."})
            try:
                data = path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "model/stl")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return
            except OSError as exc:
                return json_response(self, 500, {"error": str(exc)})
        super().do_GET()


if __name__ == "__main__":
    print(f"Cable Tray Designer running at http://{HOST}:{PORT}")
    print(f"Project directory: {ROOT}")
    print("SolidWorks import bridge: enabled")
    print("Requirement for SLDASM/SLDPRT import: SOLIDWORKS installed on this Windows PC.")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
