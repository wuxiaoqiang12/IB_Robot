# tensormsg/utils.py
import re
from typing import Any, List, Optional, Tuple
import numpy as np

def dot_get(obj, path: str):
    """
    Resolve a dotted attribute path on a ROS message or nested object.
    Supports a special JointState-style pattern: "<field>.<joint_name>".
    """
    parts = path.split(".")
    # JointState-like fast path
    if len(parts) == 2 and hasattr(obj, "name") and hasattr(obj, parts[0]):
        field, key = parts
        try:
            idx = list(obj.name).index(key)
            return getattr(obj, field)[idx]
        except Exception:
            raise

    # Generic nested getattr walk
    cur = obj
    for p in parts:
        cur = getattr(cur, p)
    return cur

def dot_set(obj, path: str, value: float):
    """
    Set a value on a ROS message or nested object using a dotted attribute path.
    Supports JointState-style pattern: "<field>.<joint_name>".
    """
    parts = path.split(".")

    if len(parts) == 2 and hasattr(obj, "name"):
        field, key = parts
        names = list(obj.name)

        field_arr = getattr(obj, field, [])
        if len(field_arr) == 0:
            setattr(obj, field, [0.0] * len(names))

        if key in names:
            idx = names.index(key)
            arr = getattr(obj, field)
            arr[idx] = float(value)
        return

    cur = obj
    for p in parts[:-1]:
        cur = getattr(cur, p)
    
    setattr(cur, parts[-1], float(value))

def nearest_resize_any(img: np.ndarray, rh: int, rw: int) -> np.ndarray:
    """Pure-numpy nearest-neighbor resize for HxW or HxWxC arrays."""
    H, W = img.shape[:2]
    if H == rh and W == rw:
        return img
    y = np.clip(np.linspace(0, H - 1, rh), 0, H - 1).astype(np.int64)
    x = np.clip(np.linspace(0, W - 1, rw), 0, W - 1).astype(np.int64)
    if img.ndim == 2:
        return img[np.ix_(y, x)]
    else:  # HxWxC
        return img[y][:, x, :]

def nearest_resize_rgb(img: np.ndarray, rh: int, rw: int) -> np.ndarray:
    """Pure-numpy nearest-neighbor resize for HxWxC arrays (uint8)."""
    return nearest_resize_any(img, rh, rw)
