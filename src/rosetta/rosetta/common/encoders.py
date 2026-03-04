# rosetta/common/encoders.py
from __future__ import annotations

"""
ROS message encoders for converting numpy arrays to ROS messages.

This module contains all registered encoders for converting policy outputs
(numpy arrays) into ROS messages for publishing. Encoders are registered using
the @register_encoder decorator and called via encode_value() in processing_utils.py.

Supported message types:
- geometry_msgs/msg/Twist: Convert to Twist messages for robot control
- std_msgs/msg/Float32MultiArray: Convert to Float32MultiArray messages
- std_msgs/msg/Int32MultiArray: Convert to Int32MultiArray messages
"""

from typing import Any, Dict, List, Optional, Sequence, Tuple
import numpy as np
from rosidl_runtime_py.utilities import get_message

from rosetta.common.contract_utils import register_encoder
from torch import Tensor
import torch

# ---------- Helper functions ----------

def dot_set(obj, path: str, value: float):
    """
    FYI: This function is adjusted for ROS joint_state style dotted paths, referred to the dot_get in decoders.py

    Set a value on a ROS message or nested object using a dotted attribute path.
    Inverse operation of dot_get.

    Supports a special JointState-style pattern: "<field>.<joint_name>".
    Example:
        path = "position.elbow_joint"
        -> looks up index of "elbow_joint" inside msg.name and sets msg.position[idx] = value
    """
    cur = obj
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

def _encode_via_dotted_paths(
    ros_type: str,
    names: List[str],
    action_vec: Sequence[float],
    clamp: Optional[Tuple[float, float]] = None,
) -> Any:
    """Fallback: explicit dotted-path assignment into a freshly-constructed ROS message."""
    if not names:
        raise ValueError(
            f"encode_value: no encoder registered for '{ros_type}' and no dotted-path names provided"
        )
    msg_cls = get_message(ros_type)
    msg = msg_cls()
    arr = np.asarray(action_vec, dtype=np.float32).reshape(-1)
    if clamp:
        arr = np.clip(arr, clamp[0], clamp[1])
    if len(names) > arr.size:
        raise ValueError(
            f"encode_value: names length ({len(names)}) exceeds action vector length ({arr.size})"
        )
    for i, path in enumerate(names):
        v = float(arr[i]) if i < arr.size else 0.0
        dot_set(msg, path, v)
    return msg


# ---------- Geometry encoders ----------


@register_encoder("geometry_msgs/msg/Twist")
def _enc_twist(
    names: List[str], action_vec: Sequence[float], clamp: Optional[Tuple[float, float]]
):
    """Twist encoder with sensible defaults when names are absent."""
    if names:
        return _encode_via_dotted_paths(
            "geometry_msgs/msg/Twist", names, action_vec, clamp
        )

    # Default mapping when no names specified
    msg_cls = get_message("geometry_msgs/msg/Twist")
    msg = msg_cls()

    # Apply clamping if specified
    arr = np.asarray(action_vec, dtype=np.float32).reshape(-1)
    if clamp:
        arr = np.clip(arr, clamp[0], clamp[1])

    # Map to twist fields (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
    if len(arr) >= 1:
        msg.linear.x = float(arr[0])
    if len(arr) >= 2:
        msg.angular.z = float(arr[1])  # Common pattern: linear.x, angular.z
    if len(arr) >= 3:
        msg.linear.y = float(arr[2])
    if len(arr) >= 4:
        msg.linear.z = float(arr[3])
    if len(arr) >= 5:
        msg.angular.x = float(arr[4])
    if len(arr) >= 6:
        msg.angular.y = float(arr[5])

    return msg


# ---------- Array encoders ----------


@register_encoder("std_msgs/msg/Float32MultiArray")
def _enc_f32_array(
    names: List[str], action_vec: Sequence[float], clamp: Optional[Tuple[float, float]]
):
    if names:
        return _encode_via_dotted_paths(
            "std_msgs/msg/Float32MultiArray", names, action_vec, clamp
        )
    msg_cls = get_message("std_msgs/msg/Float32MultiArray")
    msg = msg_cls()
    arr = np.asarray(action_vec, dtype=np.float32).reshape(-1)
    if clamp:
        arr = np.clip(arr, clamp[0], clamp[1])
    msg.data = [float(x) for x in arr.tolist()]
    return msg


@register_encoder("std_msgs/msg/Int32MultiArray")
def _enc_i32_array(
    names: List[str], action_vec: Sequence[float], clamp: Optional[Tuple[float, float]]
):
    if names:
        return _encode_via_dotted_paths(
            "std_msgs/msg/Int32MultiArray", names, action_vec, clamp
        )
    msg_cls = get_message("std_msgs/msg/Int32MultiArray")
    msg = msg_cls()
    arr = np.asarray(action_vec, dtype=np.float32).reshape(-1)
    if clamp:
        arr = np.clip(arr, clamp[0], clamp[1])
    msg.data = [int(round(float(x))) for x in arr.tolist()]
    return msg


@register_encoder("control_msgs/msg/MultiDOFCommand")
def _enc_multidof_command(
    names: List[str], action_vec: Sequence[float], clamp: Optional[Tuple[float, float]]
):
    """MultiDOFCommand encoder with special handling for values and values_dot fields."""
    msg_cls = get_message("control_msgs/msg/MultiDOFCommand")
    msg = msg_cls()
    
    # Apply clamping if specified
    arr = np.asarray(action_vec, dtype=np.float32).reshape(-1)
    if clamp:
        arr = np.clip(arr, clamp[0], clamp[1])
    
    if not names:
        # Default behavior: use all values for values field, empty values_dot
        msg.dof_names = [f"dof_{i}" for i in range(len(arr))]
        msg.values = [float(x) for x in arr.tolist()]
        msg.values_dot = []
        return msg
    
    # Parse names to separate values and values_dot
    values_names = []
    values_dot_names = []
    values_indices = []
    values_dot_indices = []
    
    for i, name in enumerate(names):
        if name.startswith("values_dot."):
            # Extract DOF name from "values_dot.dof_name"
            dof_name = name[11:]  # Remove "values_dot." prefix
            values_dot_names.append(dof_name)
            values_dot_indices.append(i)
        elif name.startswith("values."):
            # Extract DOF name from "values.dof_name"
            dof_name = name[7:]  # Remove "values." prefix
            values_names.append(dof_name)
            values_indices.append(i)
        else:
            # Default to values if no prefix
            values_names.append(name)
            values_indices.append(i)
    
    # Set DOF names (union of all DOF names from both values and values_dot)
    all_dof_names = list(dict.fromkeys(values_names + values_dot_names))  # Preserve order, remove duplicates
    msg.dof_names = all_dof_names
    
    # Map values to DOF names
    msg.values = []
    for dof_name in all_dof_names:
        if dof_name in values_names:
            idx = values_names.index(dof_name)
            if idx < len(values_indices) and values_indices[idx] < len(arr):
                msg.values.append(float(arr[values_indices[idx]]))
            else:
                msg.values.append(0.0)
        else:
            msg.values.append(0.0)
    
    # Map values_dot to DOF names
    msg.values_dot = []
    for dof_name in all_dof_names:
        if dof_name in values_dot_names:
            idx = values_dot_names.index(dof_name)
            if idx < len(values_dot_indices) and values_dot_indices[idx] < len(arr):
                msg.values_dot.append(float(arr[values_dot_indices[idx]]))
            else:
                msg.values_dot.append(0.0)
        else:
            msg.values_dot.append(0.0)
    
    return msg

# ---------- General encoders ----------
@register_encoder("sensor_msgs/msg/JointState")
def _enc_joint_state(
    names: List[str], action_vec: Sequence[float], clamp: Optional[Tuple[float, float]]
):
    if not names or len(names) == 0 or len(names[0].split('.')) != 2:
        raise ValueError(
            f"no joint name provided for sensor_msgs/msg/JointState or not as dotted path format"
        )

    msg_cls = get_message("sensor_msgs/msg/JointState")
    msg = msg_cls()
    msg.name = [names[i].split('.')[1] for i in range(len(names))]

    arr = np.asarray(action_vec, dtype=np.float32).reshape(-1)
    if clamp:
        arr = np.clip(arr, clamp[0], clamp[1])

    for index, path in enumerate(names):
        dot_set(msg, path, arr[index])

    return msg

# ---------- Variant encoder ----------
def _create_multiarray_msg(vec: np.ndarray, msg_type: str):
    msg_cls_name = f"std_msgs/msg/{msg_type}MultiArray"
    msg = get_message(msg_cls_name)()
    
    msg.data = vec.reshape(-1).tolist()
    
    for size in vec.shape:
        dim = get_message("std_msgs/msg/MultiArrayDimension")()
        dim.size = size
        msg.layout.dim.append(dim)
    
    return msg

def _enc_tensor_to_variant(vec: Tensor):
    """
    Encode Tensor into Variant message.
    """
    msg_cls = get_message("rosetta_interfaces/msg/Variant")
    variant_msg = msg_cls()

    if vec.dtype == torch.bool:
        variant_msg.type = "bool_array"
        variant_msg.bool_array = vec.reshape(-1).tolist()
    elif vec.dtype == torch.int32:
        variant_msg.type = "int_32_array"
        variant_msg.int_32_array = _create_multiarray_msg(vec, "Int32")
    elif vec.dtype == torch.int64:
        variant_msg.type = "int_64_array"
        variant_msg.int_64_array = _create_multiarray_msg(vec, "Int64")
    elif vec.dtype == torch.float32:
        variant_msg.type = "float_32_array"
        variant_msg.float_32_array = _create_multiarray_msg(vec, "Float32")
    elif vec.dtype == torch.float64:
        variant_msg.type = "float_64_array"
        variant_msg.float_64_array = _create_multiarray_msg(vec, "Float64")
    else:
        raise ValueError(f"Unsupported data type for variant encoding: {vec.dtype}")

    return variant_msg

def _enc_list_to_variant(vec: list):
    """
    Encode list into Variant message.
    Currently only supports list of strings.
    """
    if all(isinstance(item, str) for item in vec):
        msg_cls = get_message("rosetta_interfaces/msg/Variant")
        variant_msg = msg_cls()
        variant_msg.type = "string_array"
        variant_msg.string_array = vec
        return variant_msg
    else:
        raise ValueError("Unsupported list type for variant encoding. Only list of strings is supported.")

def enc_variant_list(batch: Dict[str, Any]):
    """Encode a batch into a VariantsList message."""

    msg_cls = get_message("rosetta_interfaces/msg/VariantsList")
    msg = msg_cls()
    msg.variants = []
    
    for key, value in batch.items():
        ## TODO: hardcoded filter, may be configurable in contract
        if not key.startswith('task') and not key.startswith('observation') and not key.startswith('action'):
            continue
        if isinstance(value, Tensor):
            variant_msg = _enc_tensor_to_variant(value)
        elif isinstance(value, list):
            variant_msg = _enc_list_to_variant(value)
        else:
            continue
        variant_msg.key = key
        msg.variants.append(variant_msg)

    return msg
