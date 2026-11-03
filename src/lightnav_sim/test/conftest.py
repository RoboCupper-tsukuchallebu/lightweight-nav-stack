"""Preload lightnav_msgs dylibs: macOS SIP strips DYLD_LIBRARY_PATH inside `colcon test`."""
import ctypes
import importlib.util
import pathlib

_spec = importlib.util.find_spec("lightnav_msgs")
if _spec is not None and _spec.submodule_search_locations:
    # <prefix>/lib/pythonX.Y/site-packages/lightnav_msgs -> <prefix>/lib
    _library_directory = pathlib.Path(next(iter(_spec.submodule_search_locations))).parents[2]
    # Dependencies among these libraries are unordered, so retry until none is left.
    _pending = sorted(_library_directory.glob("liblightnav_msgs__*.dylib"))
    while _pending:
        _still_pending = []
        for _library in _pending:
            try:
                ctypes.CDLL(str(_library), mode=ctypes.RTLD_GLOBAL)
            except OSError:
                _still_pending.append(_library)
        if len(_still_pending) == len(_pending):
            raise OSError(f"cannot preload {_still_pending}")
        _pending = _still_pending
