#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Base inference node for running ML models as ROS2 nodes.

This base class extracts reusable patterns from PolicyBridge:
- Contract-driven observation subscriptions
- Observation sampling with temporal alignment
- Batch preparation
- Health monitoring
- Standard service interfaces
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import QoSProfile
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from rosetta.common.contract_utils import (
    load_contract,
    iter_specs,
    SpecView,
    feature_from_spec,
    zero_pad,
    qos_profile_from_dict,
    StreamBuffer,
    decode_value,
)
from ibrobot_msgs.msg import VariantsList


@dataclass
class _SubState:
    """Subscription state for a single observation stream."""

    spec: SpecView
    buf: StreamBuffer


@dataclass
class _ModelConfig:
    """Configuration for a model node."""

    name: str
    node_name: str
    model_type: str  # "lerobot_policy" or "custom"
    repo_id: Optional[str] = None  # For LeRobot policies
    checkpoint: Optional[str] = None  # For custom models
    device: str = "auto"
    frequency: float = 10.0  # Hz
    contract_path: Optional[str] = None  # YAML contract file
    use_header_time: bool = True
    passive_mode: bool = False  # If True, disable timer-based inference


class BaseInferenceNode(Node, ABC):
    """
    Base class for model inference nodes.

    Extracts reusable patterns from PolicyBridge:
    - Contract-driven observation subscriptions with StreamBuffer
    - Observation sampling with temporal alignment
    - Batch preparation (numpy → torch)
    - Health monitoring

    Subclasses must implement:
    - _load_model(): Load the model
    - _inference(): Run inference on a batch
    """

    def __init__(self, model_config: Dict):
        super().__init__(model_config["node_name"])

        self._config = _ModelConfig(**model_config)
        self.get_logger().info(f"Initializing {self._config.name} node")

        # ---------------- Health State ----------------
        self._last_inference_time: Optional[float] = None
        self._inference_count = 0
        self._health_status = DiagnosticStatus.OK
        self._error_message = ""

        # ---------------- Parameters ----------------
        self.declare_parameter("model_type", self._config.model_type)
        self.declare_parameter("device", self._config.device)
        self.declare_parameter("frequency", self._config.frequency)

        # ---------------- Contract Loading ----------------
        self._contract = None
        self._obs_specs: List[SpecView] = []
        self._obs_zero: Dict[str, np.ndarray] = {}
        self._subs: Dict[str, _SubState] = {}
        self._state_specs: List[SpecView] = []

        if self._config.contract_path:
            self._load_contract(self._config.contract_path)
            self._setup_observation_subscriptions()

        # ---------------- Model Loading ----------------
        self._load_model(model_config)

        # ---------------- Publishers ----------------
        self._setup_publishers()

        # ---------------- Services ----------------
        self._setup_services()

        # ---------------- Timers ----------------
        if not self._config.passive_mode:
            freq = float(
                self.get_parameter("frequency").value or self._config.frequency
            )
            self._inference_timer = self.create_timer(
                1.0 / freq, self._inference_callback
            )
        else:
            self._inference_timer = None
            self.get_logger().info("Passive mode: timer-based inference disabled")

        self._health_timer = self.create_timer(
            1.0,  # 1 Hz
            self._health_callback,
        )

        self.get_logger().info(f"{self._config.name} node ready")

    def _load_contract(self, contract_path: str):
        """Load contract from YAML file."""
        p = Path(contract_path)
        if not p.exists():
            raise RuntimeError(f"Contract file not found: {contract_path}")

        self._contract = load_contract(contract_path)
        self._obs_specs = [s for s in iter_specs(self._contract) if not s.is_action]
        self._state_specs = [s for s in self._obs_specs if s.key == "observation.state"]

        # Build topic to qos map
        self._topic_to_qos = {}
        for obs in self._contract.observations or []:
            self._topic_to_qos[obs.topic] = obs.qos

        self.get_logger().info(
            f"Loaded contract with {len(self._obs_specs)} observation specs"
        )

    def _setup_observation_subscriptions(self):
        """Setup observation subscriptions from loaded contract."""
        if not self._contract:
            self.get_logger().warn(
                "No contract loaded, skipping observation subscriptions"
            )
            return

        for s in self._obs_specs:
            k, meta, _ = feature_from_spec(s, use_videos=False)

            # Create unique key for multiple observation.state specs
            if s.key == "observation.state" and len(self._state_specs) > 1:
                dict_key = f"{s.key}_{s.topic.replace('/', '_')}"
            else:
                dict_key = s.key

            self._obs_zero[dict_key] = zero_pad(meta)

            msg_cls = self.get_message_class(s.ros_type)
            qos_dict = self._topic_to_qos.get(s.topic, {})
            qos = qos_profile_from_dict(qos_dict) or QoSProfile(depth=10)

            self.create_subscription(
                msg_cls,
                s.topic,
                lambda m, sv=s: self._obs_cb(m, sv),
                qos,
            )

            tol_ns = int(max(0, getattr(s, "asof_tol_ms", 0)) * 1_000_000)

            self._subs[dict_key] = _SubState(
                spec=s,
                buf=StreamBuffer(
                    policy=getattr(s, "resample_policy", "hold"),
                    step_ns=int(1e9 / self._config.frequency),
                    tol_ns=tol_ns,
                ),
            )

        self.get_logger().info(f"Subscribed to {len(self._subs)} observation streams")

    def get_message_class(self, ros_type: str):
        """Get ROS message class from string type."""
        from rosidl_runtime_py.utilities import get_message

        return get_message(ros_type)

    @abstractmethod
    def _load_model(self, config: Dict):
        """Load the model. Must be implemented by subclasses."""
        pass

    @abstractmethod
    def _inference(self, batch: Dict[str, Any]) -> np.ndarray:
        """Run inference and return action.

        Args:
            batch: Dictionary with observations

        Returns:
            Action as numpy array
        """
        pass

    def _setup_publishers(self):
        """Setup action publishers. Can be overridden by subclasses."""
        self._action_pub = self.create_publisher(
            VariantsList,
            f"/actions/{self._config.name}",
            10,
        )

        self._health_pub = self.create_publisher(
            DiagnosticStatus,
            f"/{self._config.node_name}/health",
            10,
        )

    def _setup_services(self):
        """Setup ROS2 services. Can be extended by subclasses."""
        pass

    # ---------------- Callbacks ----------------

    def _obs_cb(self, msg, spec: SpecView):
        """Observation callback (reused from PolicyBridge)."""
        use_header = (spec.stamp_src == "header") or self._config.use_header_time

        if use_header:
            from rosetta.common.contract_utils import stamp_from_header_ns

            ts = stamp_from_header_ns(msg)
            ts_ns = int(ts) if ts is not None else self.get_clock().now().nanoseconds
        else:
            ts_ns = self.get_clock().now().nanoseconds

        val = decode_value(spec.ros_type, msg, spec)
        if val is not None:
            if spec.key == "observation.state" and len(self._state_specs) > 1:
                dict_key = f"{spec.key}_{spec.topic.replace('/', '_')}"
            else:
                dict_key = spec.key
            self._subs[dict_key].buf.push(ts_ns, val)

    def _inference_callback(self):
        """Inference timer callback."""
        try:
            obs_frame = self._sample_obs_frame()
            batch = self._prepare_batch(obs_frame)

            action = self._inference(batch)

            self._publish_action(action)

            self._last_inference_time = time.time()
            self._inference_count += 1

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            self._health_status = DiagnosticStatus.ERROR
            self._error_message = str(e)

    def _health_callback(self):
        """Health monitoring timer callback."""
        if self._last_inference_time is None:
            self._health_status = DiagnosticStatus.OK
        elif time.time() - self._last_inference_time > self._config.frequency * 2:
            self._health_status = DiagnosticStatus.WARN
            self._error_message = (
                f"No inference for {time.time() - self._last_inference_time:.1f}s"
            )
        else:
            self._health_status = DiagnosticStatus.OK

        health_msg = DiagnosticStatus()
        health_msg.level = self._health_status
        health_msg.name = self._config.node_name
        health_msg.message = (
            self._error_message or f"{self._config.name} operating normally"
        )
        health_msg.hardware_id = self._config.node_name
        health_msg.values = [
            KeyValue(key="inference_count", value=str(self._inference_count)),
            KeyValue(key="model_type", value=self._config.model_type),
        ]

        self._health_pub.publish(health_msg)

    # ---------------- Observation Sampling ----------------

    def _sample_obs_frame(self, sample_t_ns: Optional[int] = None) -> Dict[str, Any]:
        """
        Sample observation frame at a given timestamp.
        """
        if sample_t_ns is None:
            sample_t_ns = self.get_clock().now().nanoseconds

        obs_frame: Dict[str, Any] = {}

        for key, st in self._subs.items():
            v = st.buf.sample(sample_t_ns)

            if v is None:
                v = self._obs_zero.get(key)
                if v is None:
                    v = np.zeros((1,), dtype=np.float32)
                self.get_logger().warning(
                    f"MISSING: {key} (Topic: {st.spec.topic}). Using zeros."
                )

            obs_frame[key] = v

        return obs_frame

    def _prepare_batch(self, obs_frame: Dict[str, Any]) -> Dict[str, Any]:
        """
        Prepare observations for model input (reused from PolicyBridge).

        Args:
            obs_frame: Dictionary of observations

        Returns:
            Batch dictionary with torch tensors
        """
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
                        t = t.to(self.get_device(), dtype=torch.float32) / max_val
                    else:
                        t = t.to(self.get_device(), dtype=torch.float32)
                    batch[k] = t
                    continue
                batch[k] = torch.as_tensor(
                    v, dtype=torch.float32, device=self.get_device()
                )
                continue
            if torch.is_tensor(v):
                t = v
                if t.ndim == 3 and t.shape[2] in (1, 3, 4):
                    t = t.permute(2, 0, 1).unsqueeze(0).contiguous()
                    batch[k] = t.to(self.get_device(), dtype=torch.float32)
                    continue
            try:
                batch[k] = torch.as_tensor(
                    v, dtype=torch.float32, device=self.get_device()
                )
            except (ValueError, TypeError, RuntimeError):
                pass

        return batch

    def get_device(self) -> Any:
        """Get the device for model computation."""
        return torch.device(self.get_parameter("device").value or "auto")

    # ---------------- Publishing ----------------

    def _create_action_msg(self, action) -> "VariantsList":
        """Create VariantsList message from action array."""
        from ibrobot_msgs.msg import Variant
        from std_msgs.msg import MultiArrayDimension

        if torch.is_tensor(action):
            action = action.cpu().numpy()

        msg = VariantsList()
        variant = Variant()
        variant.key = "action"
        variant.type = "float_32_array"

        array_msg = variant.float_32_array
        array_msg.layout.dim = []

        for i, dim in enumerate(action.shape):
            dim_msg = MultiArrayDimension()
            dim_msg.label = f"dim_{i}"
            dim_msg.size = int(dim)
            dim_msg.stride = (
                1 if i == len(action.shape) - 1 else int(np.prod(action.shape[i + 1 :]))
            )
            array_msg.layout.dim.append(dim_msg)

        array_msg.data = action.flatten().tolist()
        msg.variants.append(variant)
        return msg

    def _publish_action(self, action: np.ndarray):
        """Publish action to ROS topic."""
        msg = self._create_action_msg(action)
        self._action_pub.publish(msg)
