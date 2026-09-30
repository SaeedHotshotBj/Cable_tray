from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse
import json
import logging
from logging.handlers import RotatingFileHandler
import os
import shutil
import subprocess
import sys
import platform
import tempfile
import time
import uuid

HOST = "127.0.0.1"
PORT = 8765
ROOT = Path(__file__).resolve().parent
SOLIDWORKS_RESULTS = {}
RESULT_TTL_SECONDS = 3600
LOG_DIR = ROOT / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
LOG_FILE = LOG_DIR / "cable_tray_debug.log"
logger = logging.getLogger("cable_tray")
logger.setLevel(logging.INFO)
if not logger.handlers:
    handler = RotatingFileHandler(LOG_FILE, maxBytes=2*1024*1024, backupCount=3, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
    logger.addHandler(handler)

def log_event(event, **details):
    payload = {"event": event, **details}
    logger.info(json.dumps(payload, ensure_ascii=False, default=str))

log_event("SERVER_STARTUP", python=sys.version.split()[0], platform=platform.platform(), executable=sys.executable, root=str(ROOT))


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
    log_event("HTTP_RESPONSE", method=getattr(handler, "command", ""), path=getattr(handler, "path", ""), status=status_code, keys=list(payload.keys()) if isinstance(payload, dict) else None)
    data = json.dumps(payload).encode("utf-8")
    handler.send_response(status_code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(data)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(data)


def convert_solidworks_file(source):
    cache_dir = Path(tempfile.gettempdir()) / "Cable_tray" / "solidworks"
    cache_dir.mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex
    output = cache_dir / f"{token}.stl"
    bridge = ROOT / "solidworks_bridge.vbs"
    if not bridge.exists():
        raise RuntimeError("SolidWorks bridge script is missing.")

    command = ["cscript.exe", "//nologo", str(bridge), str(source), str(output)]
    log_event("SOLIDWORKS_COMMAND_START", source=str(source), output=str(output), bridge=str(bridge), bridge_exists=bridge.exists())
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=900,
    )

    stdout = (completed.stdout or "").strip()
    stderr = (completed.stderr or "").strip()
    print(f"SOLIDWORKS IMPORT: file={source} exit={completed.returncode}", flush=True)
    log_event("SOLIDWORKS_PROCESS_RESULT", returncode=completed.returncode, stdout=stdout[-6000:], stderr=stderr[-6000:], output_exists=output.exists(), output_size=output.stat().st_size if output.exists() else 0)
    if stdout:
        print(f"SOLIDWORKS STDOUT: {stdout[-4000:]}", flush=True)
    if stderr:
        print(f"SOLIDWORKS STDERR: {stderr[-4000:]}", flush=True)

    if completed.returncode != 0 or not output.exists() or output.stat().st_size == 0:
        log_event("SOLIDWORKS_PROCESS_FAILED", returncode=completed.returncode, output_exists=output.exists(), output_size=output.stat().st_size if output.exists() else 0)
        detail = stdout or stderr or f"cscript exited with code {completed.returncode}"
        raise RuntimeError(detail)

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
        "kind": "solidworks",
    }
    cleanup_solidworks_results()
    log_event("SOLIDWORKS_CONVERT_OK", token=token, source=str(source), output=str(output), output_size=output.stat().st_size)
    return {
        "name": source.name,
        "native_format": source.suffix.upper().lstrip("."),
        "format": "STL",
        "extension": ".stl",
        "url": f"/api/solidworks/result/{token}.stl",
        "load_errors": load_errors,
        "warnings": warnings,
    }


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def pick_model(self):
        # Launch the native Windows OpenFileDialog from PowerShell in STA mode.
        # The helper writes the selected path to stdout; cancellation returns no path.
        ps = r'''
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()
$dialog = New-Object System.Windows.Forms.OpenFileDialog
$dialog.Title = "Load 3D / CAD Model"
$dialog.Filter = "Supported 3D/CAD models|*.glb;*.gltf;*.obj;*.stl;*.sldasm;*.sldprt|GLB / GLTF|*.glb;*.gltf|OBJ|*.obj|STL|*.stl|SolidWorks Assembly|*.sldasm|SolidWorks Part|*.sldprt|All files|*.*"
$dialog.Multiselect = $false
$dialog.CheckFileExists = $true
$dialog.InitialDirectory = [Environment]::GetFolderPath("Desktop")
[void]$dialog.ShowDialog()
if ($dialog.FileName) { [Console]::Out.WriteLine($dialog.FileName) }
$dialog.Dispose()
'''
        completed = subprocess.run(
            ["powershell.exe", "-NoLogo", "-NoProfile", "-STA", "-ExecutionPolicy", "Bypass", "-Command", ps],
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=300,
        )
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout or "").strip()
            raise RuntimeError(detail or f"Windows file dialog exited with code {completed.returncode}")
        lines = [line.strip() for line in (completed.stdout or "").splitlines() if line.strip()]
        if not lines:
            raise ValueError("No model file selected.")
        p = Path(lines[-1]).resolve()
        ext = p.suffix.lower()
        allowed = {".glb", ".gltf", ".obj", ".stl", ".sldasm", ".sldprt"}
        if ext not in allowed:
            raise ValueError("This model format is not enabled yet.")
        return p

    def do_POST(self):
        route = urlparse(self.path).path
        log_event("HTTP_REQUEST", method="POST", path=route, client=str(self.client_address))
        if route == "/api/debug/log":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(length) if 0 < length <= 64 * 1024 else b"{}"
                payload = json.loads(body.decode("utf-8"))
                log_event("CLIENT_EVENT", level=payload.get("level", "INFO"), event=payload.get("event", "UNKNOWN"), details=payload.get("details"))
                return json_response(self, 204, {})
            except Exception as exc:
                log_event("CLIENT_LOG_ERROR", error=repr(exc))
                return json_response(self, 500, {"error": str(exc)})

        if route == "/api/model/pick":
            try:
                p = self.pick_model()
                ext = p.suffix.lower()
                token = uuid.uuid4().hex
                if ext in {".sldasm", ".sldprt"}:
                    converted = convert_solidworks_file(p)
                    converted["path"] = str(p)
                    return json_response(self, 200, converted)
                token_path = Path(tempfile.gettempdir()) / "Cable_tray" / "models"
                token_path.mkdir(parents=True, exist_ok=True)
                cached = token_path / f"{token}{ext}"
                shutil.copy2(p, cached)
                SOLIDWORKS_RESULTS[token] = {
                    "path": str(cached),
                    "created_at": time.time(),
                    "kind": "source",
                }
                return json_response(self, 200, {
                    "name": p.name,
                    "extension": ext,
                    "format": {".glb":"GLB", ".gltf":"GLTF", ".obj":"OBJ", ".stl":"STL"}[ext],
                    "url": f"/api/model/source/{token}{ext}"
                })
            except Exception as exc:
                log_event("MODEL_PICKER_ERROR", error=repr(exc))
                print(f"MODEL PICKER ERROR: {exc}", flush=True)
                return json_response(self, 500, {"error": f"Model picker failed: {exc}"})

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
                result = convert_solidworks_file(source)
                result["path"] = str(source)
                return json_response(self, 200, result)
            except subprocess.TimeoutExpired:
                log_event("SOLIDWORKS_IMPORT_TIMEOUT")
                return json_response(self, 500, {"error": "SolidWorks conversion timed out after 15 minutes."})
            except Exception as exc:
                log_event("SOLIDWORKS_IMPORT_ERROR", error=repr(exc))
                print(f"SOLIDWORKS IMPORT ERROR: {exc}", flush=True)
                return json_response(self, 500, {"error": f"SolidWorks import failed: {exc}"})

        return json_response(self, 404, {"error": "Not found."})

    def do_GET(self):
        route = urlparse(self.path).path
        log_event("HTTP_REQUEST", method="GET", path=route, client=str(self.client_address))
        if route == "/api/debug/log":
            try:
                raw = LOG_FILE.read_text(encoding="utf-8")
                data = raw.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return
            except OSError as exc:
                return json_response(self, 500, {"error": str(exc)})

        source_prefix = "/api/model/source/"
        if route.startswith(source_prefix):
            filename = route[len(source_prefix):]
            token = Path(filename).stem
            info = SOLIDWORKS_RESULTS.get(token)
            if not info:
                return json_response(self, 404, {"error": "Model source expired or was not found."})
            path = Path(info["path"])
            if not path.exists():
                SOLIDWORKS_RESULTS.pop(token, None)
                return json_response(self, 404, {"error": "Model source is no longer available."})
            content_types = {
                ".glb": "model/gltf-binary",
                ".gltf": "model/gltf+json",
                ".obj": "text/plain; charset=utf-8",
                ".stl": "model/stl",
            }
            try:
                data = path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", content_types.get(path.suffix.lower(), "application/octet-stream"))
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return
            except OSError as exc:
                return json_response(self, 500, {"error": str(exc)})

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
