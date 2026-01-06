"""
Independent pre-processors and post-processors for Lerobot policies.
"""

from rclpy.node import Node
from typing import List, Any, Dict, Optional
from pathlib import Path
from dataclasses import dataclass
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message
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

# Prefix to indicate data has been processed
PROCESSED_PREFIX = "processed"

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
        if self.fps <= 0:
            raise ValueError("Contract rate_hz must be >= 1")
        self.step_ns = int(round(1e9 / self.fps))
        self.step_sec = 1.0 / self.fps

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
