"""
Independent pre-processors and post-processors for Lerobot policies.
"""

import os
import json
from rclpy.node import Node
from typing import List, Any, Dict, Optional
from pathlib import Path
from dataclasses import dataclass
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message
from lerobot.policies.factory import make_pre_post_processors
from rosetta.common.contract_utils import (
    load_contract,
    iter_specs,
    SpecView,
    feature_from_spec,
    zero_pad,
    qos_profile_from_dict,    
    contract_fingerprint,
    decode_value,
    StreamBuffer,
    stamp_from_header_ns,
    encode_value,
)
import torch

# Prefix to indicate data has been processed
PROCESSED_PREFIX = "processed"

def _device_from_param(requested: Optional[str] = None) -> torch.device:
    r = (requested or "auto").lower().strip()

    def mps_available() -> bool:
        return bool(getattr(torch.backends, "mps", None)) and torch.backends.mps.is_available()

    if r == "auto":
        if torch.cuda.is_available():
            return torch.device("cuda")
        if mps_available():
            return torch.device("mps")
        return torch.device("cpu")

    # Explicit CUDA (supports 'cuda' and 'cuda:N')
    if r.startswith("cuda"):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but not available.")
        return torch.device(r)  # 'cuda' or 'cuda:N'

    # Explicit MPS (or 'metal' alias)
    if r in {"mps", "metal"}:
        if not mps_available():
            raise RuntimeError("MPS requested but not available.")
        return torch.device("mps")

    # Anything else: try to parse ('cpu', 'xpu', etc.), otherwise fallback
    try:
        return torch.device(r)
    except (TypeError, ValueError, RuntimeError):
        # Invalid device requested, fallback to CPU
        return torch.device("cpu")

@dataclass(slots=True)
class _SubState:
    spec: SpecView
    msg_type: Any
    buf: StreamBuffer
    stamp_src: str  # 'receive' or 'header'

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        # ---------------- Parameters ----------------
        self.declare_parameter("contract_path", "")        
        self.declare_parameter("policy_path", "")
        # TODO: refector by using common device_from_param function
        self.declare_parameter("policy_device", "cuda")

        # ---------------- Contract ----------------
        contract_path = str(self.get_parameter("contract_path").value or "")
        if not contract_path:
            raise RuntimeError("policy_bridge: 'contract_path' is required")
        self._contract = load_contract(Path(contract_path))
        self._obs_qos_by_key: Dict[str, Optional[Dict[str, Any]]] = {
            o.key: o.qos for o in (self._contract.observations or [])
        }
        self._specs: List[SpecView] = list(iter_specs(self._contract))
        self._obs_specs = [s for s in self._specs if not s.is_action]
        self._cbg = ReentrantCallbackGroup()
        self._ros_sub_handles = []
        self._ros_pub_dict = {}
        self._subs = {}
        self._obs_zero = {}
        self._state_specs = [s for s in self._obs_specs if s.key == "observation.state"]
        self.fps = int(self._contract.rate_hz)
        self.device = _device_from_param(str(self.get_parameter("policy_device").value))

        if self.fps <= 0:
            raise ValueError("Contract rate_hz must be >= 1")
        self.step_ns = int(round(1e9 / self.fps))
        self.step_sec = 1.0 / self.fps

        # TODO: load policy config for processors
        # ---------------- Policy load ----------------
        policy_path = str(self.get_parameter("policy_path").value or "")
        if not policy_path:
            raise RuntimeError("policy_bridge: 'policy_path' is required")
        
        if not os.path.exists(policy_path):
            raise FileNotFoundError(f"Policy path does not exist: {policy_path}")

        # For local paths, try to read config.json
        cfg_json = os.path.join(policy_path, "config.json")
        policy_cfg = {}
        try:
            if os.path.exists(cfg_json):
                with open(cfg_json, "r", encoding="utf-8") as f:
                    policy_cfg = json.load(f)
        except (OSError, json.JSONDecodeError, KeyError) as e:
            self.get_logger().warning(
                f"Could not read policy config.json: {e!r}"
                )
        
        # ------------ pre-post processors init ----------------
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=policy_path,
            preprocessor_overrides={
                "device_processor": {"device": str(self.device)}},
            postprocessor_overrides={
                "device_processor": {"device": str(self.device)}},
        )

        # TODO: refector init function
        """
        SpecView(
           key='observation.images.top', 
           topic='/camera/top', 
           ros_type='sensor_msgs/msg/Image', 
           is_action=False, names=[], 
           image_resize=(480, 640), 
           image_encoding='bgr8', 
           image_channels=3, 
           resample_policy='hold', 
           asof_tol_ms=1500
           stamp_src='header'
           clamp=None
           safety_behavior=None
        )
        """
        # ---------------- Sub & Pubs ----------------
        for s in self._obs_specs:
            k, meta, _ = feature_from_spec(s, use_videos=False)
            msg_cls = get_message(s.ros_type)
            dict_key = self._make_dict_key(s)
            
            self._obs_zero[dict_key] = zero_pad(meta)

            sub = self.create_subscription(
                msg_cls, s.topic, lambda m, sv=s: self._obs_cb(m, sv),
                qos_profile_from_dict(self._obs_qos_by_key.get(s.key)),
                callback_group=self._cbg,
            )
            self._ros_sub_handles.append(sub)
            
            tol_ns = int(max(0, s.asof_tol_ms)) * 1_000_000
            self._subs[dict_key] = _SubState(
                spec=s,
                msg_type=msg_cls,
                buf=StreamBuffer(policy=s.resample_policy, step_ns=self.step_ns, tol_ns=tol_ns),
                stamp_src=s.stamp_src,
            )
            
            pub = self.create_publisher(
                msg_cls, PROCESSED_PREFIX + s.topic,
                qos_profile_from_dict(self._obs_qos_by_key.get(s.key)),
                callback_group=self._cbg,
            )
            self._ros_pub_dict[dict_key] = pub

    def _process(self, val):
        return val

    # ---------------- Sub callback ----------------
    def _obs_cb(self, msg, spec: SpecView) -> None:
        use_header = (spec.stamp_src ==
                      "header") or self._params.use_header_time
        ts = stamp_from_header_ns(msg) if use_header else None
        ts_ns = int(
            ts) if ts is not None else self.get_clock().now().nanoseconds
        val = decode_value(spec.ros_type, msg, spec)

        pub = self._ros_pub_dict[self._make_dict_key(spec)]
        processed_val = self._process(val)
        # TODO: encode
        pub.publish(msg)
    
    def _make_dict_key(self, spec: SpecView) -> str:
        """Create unique dict key for multiple observation.state specs."""
        if spec.key == "observation.state" and len(self._state_specs) > 1:
            return f"{spec.key}_{spec.topic.replace('/', '_')}"
        return spec.key

def main():
    """Main function to run the processors pipeline node."""
    try:
        rclpy.init()
        node = ProcessorNode()
        exe = SingleThreadedExecutor()
        exe.add_node(node)
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
