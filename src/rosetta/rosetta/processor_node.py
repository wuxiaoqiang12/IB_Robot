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
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message
from lerobot.policies.factory import make_pre_post_processors
import numpy as np
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
from tensormsg.converter import TensorMsgConverter
import torch
from time import perf_counter

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
        self.declare_parameter("use_header_time", True)

        # ---------------- Contract ----------------
        contract_path = str(self.get_parameter("contract_path").value or "")
        self._use_header_time = bool(self.get_parameter("use_header_time").value)
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

        # TODO: read from robot interface instead of hardcoding
        self._prompt = "Grasp the banana and put it on the plate"
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
        
        # Get variant topics from contract
        input_topic = self._contract.process.get("input_topic_name", "/rosetta/batch")
        output_topic = self._contract.process.get("output_topic_name", "/rosetta/action")
        
        self._variant_pub = self.create_publisher(
            get_message("rosetta_interfaces/msg/VariantsList"),
            input_topic,
            10,
        )
        self.get_logger().info(f"Publishing pre-processed variants to: {input_topic}")
        
        # Subscribe to inference output for post-processing
        self._action_specs = [s for s in self._specs if s.is_action]
        self._act_pubs: Dict[str, Any] = {}
        for spec in self._action_specs:
            msg_cls = get_message(spec.ros_type)
            pub = self.create_publisher(msg_cls, spec.topic, 10)
            self._act_pubs[spec.topic] = pub
            self.get_logger().info(f"Created action publisher: {spec.topic}")
        
        variant_msg_cls = get_message("rosetta_interfaces/msg/VariantsList")
        self.create_subscription(
            variant_msg_cls,
            output_topic,
            self._action_cb,
            10,
            callback_group=self._cbg,
        )
        self.get_logger().info(f"Subscribed to inference output: {output_topic}")
        
        # ---------------- Timer ----------------
        self._cbg_timers = ReentrantCallbackGroup()
        self._producer_timer = self.create_timer(
            self.step_sec, self._process_tick, callback_group=self._cbg_timers
        )

    # ---------------- Sub callback ----------------
    def _obs_cb(self, msg, spec: SpecView) -> None:
        use_header = (spec.stamp_src ==
                      "header")
        ts = stamp_from_header_ns(msg) if use_header else None
        ts_ns = int(
            ts) if ts is not None else self.get_clock().now().nanoseconds
        val = decode_value(spec.ros_type, msg, spec)
        if val is not None:
            dict_key = self._make_dict_key(spec)
            self._subs[dict_key].buf.push(ts_ns, val)
    
    def _action_cb(self, msg) -> None:
        """Callback for inference output - apply post-processing and publish actions."""
        try:
            batch = TensorMsgConverter.from_variant(msg, self.device)
            action = batch.get("action")
            if action is None:
                self.get_logger().warning("No 'action' key in inference output")
                return
            
            # Apply post-processing
            action = self.postprocessor(action)
            
            # Publish actions
            self._publish_actions(action)
        except Exception as e:
            self.get_logger().error(f"Failed to post-process action: {e!r}")
    
    def _publish_actions(self, action: Any) -> None:
        """Encode and publish action vectors."""
        # Convert action to numpy
        if torch.is_tensor(action):
            action_np = action.detach().cpu().numpy()
        else:
            action_np = np.asarray(action)
        
        # Flatten if needed
        action_np = action_np.ravel()
        
        # Publish each action spec
        start_idx = 0
        for spec in self._action_specs:
            spec_len = len(spec.names) if spec.names else 0
            if spec_len == 0:
                continue
            
            end_idx = start_idx + spec_len
            if end_idx > len(action_np):
                self.get_logger().error(
                    f"Action vector too short for {spec.key}: "
                    f"need {spec_len}, have {len(action_np) - start_idx}"
                )
                break
            
            spec_action = action_np[start_idx:end_idx]
            
            ros_msg = encode_value(
                ros_type=spec.ros_type,
                names=spec.names,
                action_vec=spec_action,
                clamp=getattr(spec, "clamp", None),
            )
            
            pub = self._act_pubs[spec.topic]
            pub.publish(ros_msg)
            
            start_idx = end_idx
    
    def _make_dict_key(self, spec: SpecView) -> str:
        """Create unique dict key for multiple observation.state specs."""
        if spec.key == "observation.state" and len(self._state_specs) > 1:
            return f"{spec.key}_{spec.topic.replace('/', '_')}"
        return spec.key
    
    def _process_tick(self) -> None:
        sample_t_ns = None
        if self._use_header_time:
            ts = self._get_most_recent_image_timestamp()
            # Guard: if header time is too stale relative to node clock, ignore it
            if ts is not None:
                skew = self.get_clock().now().nanoseconds - ts
                if 0 <= skew <= int(500e6):  # <= 500 ms stale is OK
                    sample_t_ns = ts
        if sample_t_ns is None:
            sample_t_ns = self.get_clock().now().nanoseconds

        obs_frame = self._sample_obs_frame(sample_t_ns)
        batch = self._prepare(obs_frame)
        batch = self.preprocessor(batch)
        start_time = perf_counter()
        variant_msg = TensorMsgConverter.to_variant(batch)
        end_time = perf_counter()
        # self.get_logger().info(f"Encode time: {(end_time - start_time)*1000:.4f} ms")
        self._variant_pub.publish(variant_msg)
    
    def _sample_obs_frame(self, sample_t_ns: int) -> Dict[str, Any]:
        obs_frame: Dict[str, Any] = {}
        
        # Handle multiple observation.state specs by consolidating them
        if len(self._state_specs) > 1:
            state_parts = []
            for sv in self._state_specs:
                dict_key = f"{sv.key}_{sv.topic.replace('/', '_')}"
                if dict_key in self._subs:
                    v = self._subs[dict_key].buf.sample(sample_t_ns)
                    if v is None:
                        zp = self._obs_zero[dict_key]
                        v = zp.copy() if isinstance(zp, np.ndarray) else zp
                        self.get_logger().warning(f"Observation {dict_key} is None, zero padding")
                    state_parts.append(v)
                else:
                    # Fallback to zero padding if subscription missing
                    zp = self._obs_zero.get(dict_key, np.zeros((len(sv.names),), dtype=np.float32))
                    state_parts.append(zp.copy() if isinstance(zp, np.ndarray) else zp)
            
            # Concatenate all state parts in contract order
            if state_parts:
                obs_frame["observation.state"] = np.concatenate(state_parts, axis=0)
            else:
                obs_frame["observation.state"] = np.zeros((0,), dtype=np.float32)
        
        # Handle all other observations
        for key, st in self._subs.items():
            # Skip individual state keys if we have multiple state specs (already handled above)
            if key.startswith("observation.state_") and len(self._state_specs) > 1:
                continue
                
            v = st.buf.sample(sample_t_ns)
            if v is None:
                zp = self._obs_zero[key]
                obs_frame[key] = zp.copy() if isinstance(
                    zp, np.ndarray) else zp
                self.get_logger().warning(f"Observation {key} is None, zero padding")
            else:
                obs_frame[key] = v
        obs_frame["task"] = self._prompt
        return obs_frame

    def _prepare(self, obs_frame: Dict[str, Any]) -> Dict[str, Any]:
        batch: Dict[str, Any] = {}
        for k, v in obs_frame.items():
            if v is None:
                continue
            if isinstance(v, str):
                batch[k] = v
                continue
            if isinstance(v, np.ndarray):
                t = torch.from_numpy(v)
                if t.ndim == 3 and t.shape[2] in (1, 3, 4):
                    t = t.permute(2, 0, 1).unsqueeze(0).contiguous()
                    if np.issubdtype(v.dtype, np.integer):
                        max_val = float(np.iinfo(v.dtype).max)
                        t = t.to(self.device, dtype=torch.float32) / max_val
                    else:
                        t = t.to(self.device, dtype=torch.float32)
                    batch[k] = t
                    continue
                batch[k] = torch.as_tensor(
                    v, dtype=torch.float32, device=self.device)
                continue
            if torch.is_tensor(v):
                t = v
                if t.ndim == 3 and t.shape[2] in (1, 3, 4):
                    t = t.permute(2, 0, 1).unsqueeze(0).contiguous()
                batch[k] = t.to(self.device, dtype=torch.float32)
                continue
            try:
                batch[k] = torch.as_tensor(
                    v, dtype=torch.float32, device=self.device)
            except (ValueError, TypeError, RuntimeError):
                pass
        return batch

    def _get_most_recent_image_timestamp(self) -> Optional[int]:
        """Get the timestamp of the most recent primary image observation."""
        # Look for image observations by checking if the spec has image_resize set
        image_keys = []
        for key, sub_state in self._subs.items():
            if hasattr(sub_state.spec, 'image_resize') and sub_state.spec.image_resize is not None:
                image_keys.append(key)
        
        if not image_keys:
            return None
            
        # Get the most recent timestamp from image observations
        most_recent_ts = None
        for key in image_keys:
            latest_ts = getattr(self._subs[key].buf, 'last_ts', None)
            if latest_ts is not None:
                if most_recent_ts is None or latest_ts > most_recent_ts:
                    most_recent_ts = latest_ts
        
        # Optional: log clock skew for debugging
        if most_recent_ts is not None:
            skew_ms = (self.get_clock().now().nanoseconds - most_recent_ts) / 1e6
            # self.get_logger().info(f"obs-header skew: {skew_ms:.1f} ms")
                    
        return most_recent_ts

def main():
    """Main function to run the processors pipeline node."""
    try:
        rclpy.init()
        node = ProcessorNode()
        exe = MultiThreadedExecutor(num_threads=4)
        exe.add_node(node)
        exe.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
