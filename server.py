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
import struct

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


def _stl_binary_records(path):
    data = path.read_bytes()
    if len(data) < 84:
        return None
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + count * 50
    if expected != len(data):
        return None
    return [data[offset:offset + 50] for offset in range(84, len(data), 50)]


def _stl_ascii_records(path):
    text = path.read_text(encoding="utf-8", errors="ignore")
    vertices = []
    normal = (0.0, 0.0, 0.0)
    records = []
    for raw in text.splitlines():
        line = raw.strip()
        lower = line.lower()
        if lower.startswith("facet normal"):
            parts = line.split()
            if len(parts) >= 5:
                try:
                    normal = (float(parts[2]), float(parts[3]), float(parts[4]))
                except ValueError:
                    normal = (0.0, 0.0, 0.0)
        elif lower.startswith("vertex"):
            parts = line.split()
            if len(parts) >= 4:
                try:
                    vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
                except ValueError:
                    pass
                if len(vertices) == 3:
                    records.append(struct.pack(
                        "<3f3f3f3fH",
                        normal[0], normal[1], normal[2],
                        vertices[0][0], vertices[0][1], vertices[0][2],
                        vertices[1][0], vertices[1][1], vertices[1][2],
                        vertices[2][0], vertices[2][1], vertices[2][2],
                        0,
                    ))
                    vertices = []
    return records


def _read_stl_records(path):
    records = _stl_binary_records(path)
    if records is not None:
        return records, "binary"
    return _stl_ascii_records(path), "ascii"


def combine_stl_files(parts, output):
    valid = []
    total_triangles = 0
    formats = {}
    for part in parts:
        try:
            records, fmt = _read_stl_records(part)
        except (OSError, UnicodeError, ValueError, struct.error):
            continue
        if not records:
            continue
        valid.append((part, records))
        formats[fmt] = formats.get(fmt, 0) + 1
        total_triangles += len(records)

    if not valid:
        raise RuntimeError("SolidWorks produced STL files, but none contained readable triangles.")

    if total_triangles > 0xFFFFFFFF:
        raise RuntimeError("Combined STL contains too many triangles.")

    header = bytearray(80)
    label = b"Cable_tray SolidWorks combined STL"
    header[:len(label)] = label
    with output.open("wb") as stream:
        stream.write(header)
        stream.write(struct.pack("<I", total_triangles))
        for _, records in valid:
            for record in records:
                stream.write(record)

    size = output.stat().st_size
    expected = 84 + total_triangles * 50
    if size != expected:
        raise RuntimeError(
            f"Combined STL size mismatch: expected {expected}, got {size}."
        )
    return {
        "source_files": len(valid),
        "triangles": total_triangles,
        "formats": formats,
        "size": size,
    }


def convert_solidworks_file(source):
    cache_root = Path(tempfile.gettempdir()) / "Cable_tray" / "solidworks"
    cache_root.mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex
    export_dir = cache_root / token
    export_dir.mkdir(parents=True, exist_ok=True)
    output = export_dir / "__cable_tray_combined__.stl"
    bridge = ROOT / "solidworks_bridge.vbs"
    if not bridge.exists():
        raise RuntimeError("SolidWorks bridge script is missing.")

    command = ["cscript.exe", "//nologo", str(bridge), str(source), str(output)]
    log_event("SOLIDWORKS_COMMAND_START", source=str(source), output=str(output), export_dir=str(export_dir), bridge=str(bridge), bridge_exists=bridge.exists())
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=900,
    )

    stdout = (completed.stdout or "").strip()
    stderr = (completed.stderr or "").strip()
    stl_files = sorted(p for p in export_dir.rglob("*") if p.is_file() and p.suffix.lower() == ".stl")
    log_event(
        "SOLIDWORKS_PROCESS_RESULT",
        returncode=completed.returncode,
        stdout=stdout[-6000:],
        stderr=stderr[-6000:],
        output_exists=output.exists(),
        output_size=output.stat().st_size if output.exists() else 0,
        stl_files=[str(p) for p in stl_files],
    )
    print(f"SOLIDWORKS IMPORT: file={source} exit={completed.returncode}", flush=True)
    if stdout:
        print(f"SOLIDWORKS STDOUT: {stdout[-4000:]}", flush=True)
    if stderr:
        print(f"SOLIDWORKS STDERR: {stderr[-4000:]}", flush=True)

    if completed.returncode != 0:
        log_event("SOLIDWORKS_PROCESS_FAILED", returncode=completed.returncode, output_exists=output.exists(), output_size=output.stat().st_size if output.exists() else 0)
        detail = stdout or stderr or f"cscript exited with code {completed.returncode}"
        raise RuntimeError(detail)

    # Prefer the exact combined output produced by SolidWorks. If it did not
    # honor the one-file preference but generated component STLs, combine those
    # components in their assembly coordinates into one browser-ready STL.
    if not output.exists() or output.stat().st_size == 0:
        component_files = [p for p in stl_files if p.resolve() != output.resolve()]
        if not component_files:
            raise RuntimeError("SolidWorks completed without producing a usable STL file.")
        merge = combine_stl_files(component_files, output)
        log_event(
            "SOLIDWORKS_COMPONENT_STL_COMBINED",
            source=str(source),
            output=str(output),
            source_files=merge["source_files"],
            triangles=merge["triangles"],
            formats=merge["formats"],
            output_size=merge["size"],
        )

    if output.stat().st_size == 0:
        raise RuntimeError("The final SolidWorks STL is empty.")

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
        # Use the Windows native Common Dialog directly. This avoids both
        # PowerShell and Tk child-process/window-focus issues and returns the
        # real filesystem path needed by the SolidWorks automation bridge.
        if os.name != "nt":
            raise RuntimeError("The native model picker is only supported on Windows.")

        import ctypes
        from ctypes import wintypes

        class OPENFILENAMEW(ctypes.Structure):
            _fields_ = [
                ("lStructSize", wintypes.DWORD),
                ("hwndOwner", wintypes.HWND),
                ("hInstance", wintypes.HINSTANCE),
                ("lpstrFilter", wintypes.LPCWSTR),
                ("lpstrCustomFilter", wintypes.LPWSTR),
                ("nMaxCustFilter", wintypes.DWORD),
                ("nFilterIndex", wintypes.DWORD),
                ("lpstrFile", wintypes.LPWSTR),
                ("nMaxFile", wintypes.DWORD),
                ("lpstrFileTitle", wintypes.LPWSTR),
                ("nMaxFileTitle", wintypes.DWORD),
                ("lpstrInitialDir", wintypes.LPCWSTR),
                ("lpstrTitle", wintypes.LPCWSTR),
                ("Flags", wintypes.DWORD),
                ("nFileOffset", wintypes.WORD),
                ("nFileExtension", wintypes.WORD),
                ("lpstrDefExt", wintypes.LPCWSTR),
                ("lCustData", wintypes.LPARAM),
                ("lpfnHook", ctypes.c_void_p),
                ("lpTemplateName", wintypes.LPCWSTR),
            ]

        desktop = Path.home() / "Desktop"
        if not desktop.is_dir():
            desktop = Path.home()

        # Windows Common Dialog filter syntax uses semicolon-separated masks.
        filter_spec = (
            "Supported 3D/CAD models\x00"
            "*.glb;*.gltf;*.obj;*.stl;*.sldasm;*.sldprt\x00"
            "GLB / GLTF\x00*.glb;*.gltf\x00"
            "OBJ\x00*.obj\x00"
            "STL\x00*.stl\x00"
            "SolidWorks Assembly\x00*.sldasm\x00"
            "SolidWorks Part\x00*.sldprt\x00"
            "All files\x00*.*\x00"
            "\x00"
        )

        filename_buffer = ctypes.create_unicode_buffer(32768)
        title_buffer = ctypes.create_unicode_buffer(260)

        user32 = ctypes.WinDLL("user32", use_last_error=True)
        comdlg32 = ctypes.WinDLL("comdlg32", use_last_error=True)
        user32.GetForegroundWindow.restype = wintypes.HWND
        comdlg32.GetOpenFileNameW.argtypes = [ctypes.POINTER(OPENFILENAMEW)]
        comdlg32.GetOpenFileNameW.restype = wintypes.BOOL
        comdlg32.CommDlgExtendedError.restype = wintypes.DWORD

        flags = (
            0x00001000  # OFN_FILEMUSTEXIST
            | 0x00000800  # OFN_PATHMUSTEXIST
            | 0x00080000  # OFN_EXPLORER
            | 0x00000008  # OFN_NOCHANGEDIR
            | 0x00000004  # OFN_HIDEREADONLY
        )

        dialog = OPENFILENAMEW()
        dialog.lStructSize = ctypes.sizeof(OPENFILENAMEW)
        dialog.hwndOwner = user32.GetForegroundWindow()
        dialog.lpstrFilter = filter_spec
        dialog.nFilterIndex = 1
        dialog.lpstrFile = ctypes.cast(filename_buffer, wintypes.LPWSTR)
        dialog.nMaxFile = len(filename_buffer)
        dialog.lpstrFileTitle = ctypes.cast(title_buffer, wintypes.LPWSTR)
        dialog.nMaxFileTitle = len(title_buffer)
        dialog.lpstrInitialDir = str(desktop)
        dialog.lpstrTitle = "Load 3D / CAD Model"
        dialog.Flags = flags

        log_event(
            "MODEL_PICKER_START",
            picker="windows_common_dialog",
            initial_directory=str(desktop),
            owner_hwnd=int(dialog.hwndOwner or 0),
        )

        if not comdlg32.GetOpenFileNameW(ctypes.byref(dialog)):
            error_code = int(comdlg32.CommDlgExtendedError())
            log_event(
                "MODEL_PICKER_CANCELLED",
                picker="windows_common_dialog",
                common_dialog_error=error_code,
            )
            if error_code:
                raise RuntimeError(
                    f"Windows file dialog failed. CommonDialogError={error_code}"
                )
            raise ValueError("No model file selected.")

        selected = filename_buffer.value.strip()
        log_event(
            "MODEL_PICKER_SELECTED",
            picker="windows_common_dialog",
            selected=selected,
        )

        if not selected:
            raise ValueError("Windows returned an empty model path.")

        p = Path(selected).expanduser().resolve()
        allowed = {".glb", ".gltf", ".obj", ".stl", ".sldasm", ".sldprt"}
        ext = p.suffix.lower()
        if ext not in allowed:
            raise ValueError("This model format is not enabled yet.")
        if not p.is_file():
            raise ValueError("The selected model file no longer exists.")
        return p

    def do_POST(self):
        route = urlparse(self.path).path
        log_event("HTTP_REQUEST", method="POST", path=route, client=str(self.client_address))
        if route == "/api/debug/log":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(length) if 0 < length <= 64 * 1024 else b"{}"
                payload = json.loads(body.decode("utf-8"))
                try:
                    log_event(payload.get("event", "UNKNOWN"), level=str(payload.get("level", "INFO")), details=payload.get("details"))
                except Exception as exc:
                    print(f"CLIENT LOG WRITE ERROR: {exc!r}", flush=True)
                # Keep the browser logger completely non-blocking/non-fatal.
                data = b'{"ok":true}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return
            except Exception as exc:
                print(f"CLIENT LOG REQUEST ERROR: {exc!r}", flush=True)
                try:
                    log_event("CLIENT_LOG_ERROR", error=repr(exc))
                except Exception:
                    pass
                return json_response(self, 400, {"error": str(exc)})

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

        if route == "/api/cad/import":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 32 * 1024:
                    return json_response(self, 400, {"error": "Invalid request."})
                body = self.rfile.read(length)
                payload = json.loads(body.decode("utf-8"))
                source = Path(str(payload.get("path", ""))).expanduser().resolve()
                if not source.exists():
                    return json_response(self, 400, {"error": "Selected CAD file does not exist."})
                ext = source.suffix.lower()
                if ext in {".sldasm", ".sldprt"}:
                    result = convert_solidworks_file(source)
                elif ext in {".dwg", ".dxf"}:
                    result = convert_autocad_file(source)
                else:
                    return json_response(
                        self,
                        400,
                        {"error": "Only SLDASM, SLDPRT, DWG and DXF are supported by this importer."},
                    )
                result["path"] = str(source)
                return json_response(self, 200, result)
            except subprocess.TimeoutExpired:
                log_event("CAD_IMPORT_TIMEOUT")
                return json_response(self, 500, {"error": "CAD conversion timed out after 15 minutes."})
            except Exception as exc:
                log_event("CAD_IMPORT_ERROR", error=repr(exc))
                print(f"CAD IMPORT ERROR: {exc}", flush=True)
                return json_response(self, 500, {"error": f"CAD import failed: {exc}"})

        if route == "/api/solidworks/import":
            return json_response(
                self,
                410,
                {"error": "SolidWorks endpoint moved to /api/cad/import."},
            )

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
