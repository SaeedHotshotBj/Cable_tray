import ctypes
from ctypes import wintypes
import os

OFN_EXPLORER = 0x00080000
OFN_FILEMUSTEXIST = 0x00001000
OFN_PATHMUSTEXIST = 0x00000800
MAX_PATH = 32768


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
        ("lpfnHook", wintypes.LPVOID),
        ("lpTemplateName", wintypes.LPCWSTR),
        ("pvReserved", wintypes.LPVOID),
        ("dwReserved", wintypes.DWORD),
        ("FlagsEx", wintypes.DWORD),
    ]


def main():
    if os.name != "nt":
        raise RuntimeError("This model picker is available only on Windows.")

    file_buffer = ctypes.create_unicode_buffer(MAX_PATH)
    filter_text = (
        "Supported 3D/CAD models\0"
        "*.glb;*.gltf;*.obj;*.stl;*.sldasm;*.sldprt\0"
        "GLB / GLTF\0*.glb;*.gltf\0"
        "OBJ\0*.obj\0"
        "STL\0*.stl\0"
        "SolidWorks Assembly\0*.sldasm\0"
        "SolidWorks Part\0*.sldprt\0"
        "All files\0*.*\0\0"
    )

    dialog = OPENFILENAMEW()
    dialog.lStructSize = ctypes.sizeof(OPENFILENAMEW)
    dialog.lpstrFilter = filter_text
    dialog.nFilterIndex = 1
    dialog.lpstrFile = file_buffer
    dialog.nMaxFile = MAX_PATH
    dialog.lpstrTitle = "Load 3D / CAD Model"
    dialog.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST

    common_dialog = ctypes.windll.comdlg32.GetOpenFileNameW
    common_dialog.argtypes = [ctypes.POINTER(OPENFILENAMEW)]
    common_dialog.restype = wintypes.BOOL

    if common_dialog(ctypes.byref(dialog)):
        print(file_buffer.value)
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
