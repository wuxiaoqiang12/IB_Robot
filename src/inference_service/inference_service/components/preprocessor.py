#!/usr/bin/env python3
"""
Preprocessor Component for LeRobot policies.

Can run as:
1. Independent ROS2 node (distributed mode)
2. Composed in same process with Inference component (zero-copy mode)

Responsibilities:
- Subscribe to raw sensor data (images, joint states)
- Apply normalization using model's dataset_stats
- Publish VariantsList (normalized tensors)
"""

from __future__ import annotations

import json
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np
import rclpy
import torch
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from robot_config.contract_utils import (
    StreamBuffer,
    SpecView,
    decode_value,
    feature_from_spec,
    iter_specs,
    load_contract,
    qos_profile_from_dict,
    stamp_from_header_ns,
    zero_pad,
)
from rosetta.common.encoders import enc_variant_list
from ibrobot_msgs.msg import VariantsList


@dataclass(slots=True)
class _SubState:
    spec: SpecView
    buf: StreamBuffer


class PreprocessorBase(ABC):
    """Abstract base for preprocessor implementations."""

    @abstractmethod
    def __call__(self, batch: Dict[str, Any]) -> Dict[str, Any]:
        """Apply preprocessing to batch."""
        pass


class LeRobotPreprocessor(PreprocessorBase):
    """LeRobot-specific preprocessor using make_pre_post_processors."""

    def __init__(self, policy_path: str, device: torch.device):
        from lerobot.policies.factory import make_pre_post_processors

        self.device = device
        policy_cfg = self._load_policy_config(policy_path)

        self._preprocessor, _ = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=policy_path,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )

    def _load_policy_config(self, policy_path: str) -> dict:
        cfg_json = os.path.join(policy_path, "config.json")
        if os.path.exists(cfg_json):
            with open(cfg_json, "r", encoding="utf-8") as f:
                return json.load(f)
        return {}

    def __call__(self, batch: Dict[str, Any]) -> Dict[str, Any]:
        return self._preprocessor(batch)


class PreprocessorComponent(Node):
    """
    Preprocessor component that can run standalone or composed.

    Subscribes to raw sensor topics and publishes normalized VariantsList.
    """

    def __init__(
        self,
        node_name: str = "preprocessor",
        contract_path: Optional[str] = None,
        policy_path: Optional[str] = None,
        output_topic: str = "/preprocessed/batch",
        device: str = "auto",
        preprocessor: Optional[PreprocessorBase] = None,
        use_header_time: bool = True,
    ):
        super().__init__(node_name)

        self._use_header_time = use_header_time
        self._device = self._resolve_device(device)

        # Load contract
        if contract_path:
            self._load_contract(contract_path)
        else:
            self._contract = None
            self._obs_specs = []
            self._subs = {}
            self._obs_zero = {}
            self._state_specs = []

        # Setup preprocessor
        if preprocessor:
            self._preprocessor = preprocessor
        elif policy_path:
            self._preprocessor = LeRobotPreprocessor(policy_path, self._device)
        else:
            self._preprocessor = None

        # Publisher for preprocessed data
        self._pub = self.create_publisher(VariantsList, output_topic, 10)

        # Timer for processing
        if self._contract:
            self._timer = self.create_timer(
                1.0 / self._contract.rate_hz,
                self._process_tick,
                callback_group=ReentrantCallbackGroup(),
            )

        self.get_logger().info(
            f"Preprocessor ready: device={self._device}, "
            f"output={output_topic}, specs={len(self._obs_specs)}"
        )

    def _resolve_device(self, device: str) -> torch.device:
        if device == "auto":
            if torch.cuda.is_available():
                return torch.device("cuda")
            return torch.device("cpu")
        return torch.device(device)

    def _load_contract(self, contract_path: str):
        self._contract = load_contract(Path(contract_path))
        self._obs_specs = [s for s in iter_specs(self._contract) if not s.is_action]
        self._state_specs = [s for s in self._obs_specs if s.key == "observation.state"]

        qos_map = {o.key: o.qos for o in (self._contract.observations or [])}
        self._subs: Dict[str, _SubState] = {}
        self._obs_zero: Dict[str, np.ndarray] = {}
        step_ns = int(1e9 / self._contract.rate_hz)

        for s in self._obs_specs:
            _, meta, _ = feature_from_spec(s, use_videos=False)
            dict_key = self._make_dict_key(s)
            self._obs_zero[dict_key] = zero_pad(meta)

            msg_cls = get_message(s.ros_type)
            qos = qos_profile_from_dict(qos_map.get(s.key)) or 10

            self.create_subscription(
                msg_cls,
                s.topic,
                lambda m, sv=s: self._obs_cb(m, sv),
                qos,
                callback_group=ReentrantCallbackGroup(),
            )

            self._subs[dict_key] = _SubState(
                spec=s,
                buf=StreamBuffer(
                    policy=getattr(s, "resample_policy", "hold"),
                    step_ns=step_ns,
                    tol_ns=int(max(0, getattr(s, "asof_tol_ms", 0)) * 1_000_000),
                ),
            )

    def _make_dict_key(self, spec: SpecView) -> str:
        if spec.key == "observation.state" and len(self._state_specs) > 1:
            return f"{spec.key}_{spec.topic.replace('/', '_')}"
        return spec.key

    def _obs_cb(self, msg, spec: SpecView):
        use_header = (
            getattr(spec, "stamp_src", "header") == "header" or self._use_header_time
        )
        ts = stamp_from_header_ns(msg) if use_header else None
        ts_ns = int(ts) if ts is not None else self.get_clock().now().nanoseconds

        val = decode_value(spec.ros_type, msg, spec)
        if val is not None:
            self._subs[self._make_dict_key(spec)].buf.push(ts_ns, val)

    def _sample_obs_frame(self, sample_t_ns: int) -> Dict[str, Any]:
        obs_frame: Dict[str, Any] = {}

        # Handle multiple observation.state specs
        if len(self._state_specs) > 1:
            parts = []
            for sv in self._state_specs:
                key = self._make_dict_key(sv)
                v = (
                    self._subs[key].buf.sample(sample_t_ns)
                    if key in self._subs
                    else None
                )
                parts.append(
                    v if v is not None else self._obs_zero.get(key, np.zeros(1))
                )
            obs_frame["observation.state"] = np.concatenate(parts)

        for key, st in self._subs.items():
            if key.startswith("observation.state_") and len(self._state_specs) > 1:
                continue
            v = st.buf.sample(sample_t_ns)
            obs_frame[key] = (
                v if v is not None else self._obs_zero.get(key, np.zeros(1))
            )

        return obs_frame

    def _prepare_batch(self, obs_frame: Dict[str, Any]) -> Dict[str, Any]:
        batch: Dict[str, Any] = {}
        for k, v in obs_frame.items():
            if isinstance(v, str):
                batch[k] = v
            elif isinstance(v, np.ndarray):
                t = torch.from_numpy(v)
                if t.ndim == 3 and t.shape[2] in (1, 3, 4):
                    t = t.permute(2, 0, 1).unsqueeze(0).contiguous()
                    if np.issubdtype(v.dtype, np.integer):
                        t = t.to(self._device, dtype=torch.float32) / float(
                            np.iinfo(v.dtype).max
                        )
                    else:
                        t = t.to(self._device, dtype=torch.float32)
                else:
                    t = t.to(self._device, dtype=torch.float32)
                batch[k] = t
            elif torch.is_tensor(v):
                batch[k] = v.to(self._device, dtype=torch.float32)
        return batch

    def _process_tick(self):
        sample_t_ns = self.get_clock().now().nanoseconds
        obs_frame = self._sample_obs_frame(sample_t_ns)
        batch = self._prepare_batch(obs_frame)

        if self._preprocessor:
            batch = self._preprocessor(batch)

        msg = enc_variant_list(batch)
        self._pub.publish(msg)

    def process(self, obs_frame: Dict[str, Any]) -> Dict[str, Any]:
        """Direct processing for zero-copy mode."""
        batch = self._prepare_batch(obs_frame)
        if self._preprocessor:
            batch = self._preprocessor(batch)
        return batch


def main():
    rclpy.init()

    from rclpy.node import Node

    # Use temp node to read ROS parameters
    temp = Node("_preprocessor_param_reader")
    temp.declare_parameter("contract_path", "")
    temp.declare_parameter("policy_path", "")
    temp.declare_parameter("output_topic", "/preprocessed/batch")
    temp.declare_parameter("device", "auto")
    temp.declare_parameter("use_sim_time", False)

    params = {
        "contract_path": temp.get_parameter("contract_path").value or None,
        "policy_path": temp.get_parameter("policy_path").value or None,
        "output_topic": temp.get_parameter("output_topic").value,
        "device": temp.get_parameter("device").value,
        "use_sim_time": temp.get_parameter("use_sim_time").value,
    }
    temp.destroy_node()

    node = PreprocessorComponent(
        node_name="preprocessor",
        contract_path=params["contract_path"],
        policy_path=params["policy_path"],
        output_topic=params["output_topic"],
        device=params["device"],
    )

    if params["use_sim_time"]:
        node._use_sim_time = True

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
