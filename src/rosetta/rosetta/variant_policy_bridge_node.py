#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VariantPolicyBridge: Policy inference node for VariantsList messages.

This node:
1. Subscribes to rosetta_interfaces/msg/VariantsList (pre-processed observations)
2. Runs policy inference
3. Publishes raw action output as VariantsList (for post-processing by processor_node)
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
import torch

from lerobot.policies.factory import get_policy_class

from rosetta.common.contract_utils import (
    load_contract,
    iter_specs,
    SpecView,
    StreamBuffer,
)
from tensormsg.converter import TensorMsgConverter


def _device_from_param(requested: Optional[str] = None) -> torch.device:
    """Parse device parameter and return torch device."""
    r = (requested or "auto").lower().strip()

    def mps_available() -> bool:
        return bool(getattr(torch.backends, "mps", None)) and torch.backends.mps.is_available()

    if r == "auto":
        if torch.cuda.is_available():
            return torch.device("cuda")
        if mps_available():
            return torch.device("mps")
        return torch.device("cpu")

    if r.startswith("cuda"):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but not available.")
        return torch.device(r)

    if r in {"mps", "metal"}:
        if not mps_available():
            raise RuntimeError("MPS requested but not available.")
        return torch.device("mps")

    try:
        return torch.device(r)
    except (TypeError, ValueError, RuntimeError):
        return torch.device("cpu")


class VariantPolicyBridge(Node):
    """Policy inference bridge - receives pre-processed observations, outputs raw actions."""

    def __init__(self) -> None:
        super().__init__("variant_policy_bridge")

        # Parameters
        self.declare_parameter("contract_path", "")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("policy_device", "auto")

        # Load contract
        contract_path = str(self.get_parameter("contract_path").value or "")
        if not contract_path:
            raise RuntimeError("contract_path is required")
        self._contract = load_contract(Path(contract_path))

        # Get variant topics from contract
        input_topic = self._contract.process.get("input_topic_name", "/rosetta/batch")
        output_topic = self._contract.process.get("output_topic_name", "/rosetta/action")

        # Setup specs
        self._specs: List[SpecView] = list(iter_specs(self._contract))
        self._action_specs = [s for s in self._specs if s.is_action]

        # Setup device
        self.device = _device_from_param(str(self.get_parameter("policy_device").value))
        self.get_logger().info(f"Using device: {self.device}")

        # Setup execution frequency
        self.fps = int(self._contract.rate_hz)
        if self.fps <= 0:
            raise ValueError("Contract rate_hz must be >= 1")
        self.step_ns = int(round(1e9 / self.fps))
        self.step_sec = 1.0 / self.fps

        # Load policy
        policy_path = str(self.get_parameter("policy_path").value or "")
        if not policy_path:
            raise RuntimeError("policy_path is required")
        
        if not os.path.exists(policy_path):
            raise FileNotFoundError(f"Policy path does not exist: {policy_path}")

        cfg_json = os.path.join(policy_path, "config.json")
        policy_cfg = {}
        policy_type = ""
        try:
            if os.path.exists(cfg_json):
                with open(cfg_json, "r", encoding="utf-8") as f:
                    policy_cfg = json.load(f)
                    policy_type = str(policy_cfg.get("type", "")).lower()
        except (OSError, json.JSONDecodeError) as e:
            self.get_logger().warning(f"Could not read policy config.json: {e!r}")

        # Initialize policy
        if not policy_type:
            raise ValueError("Could not determine policy type from config.json")
        
        policy_class = get_policy_class(policy_type)
        self.policy = policy_class.from_pretrained(policy_path)
        self.get_logger().info(f"Loaded policy: {policy_class.__name__}")

        # TODO: convert to streambuffer
        self._variant_buffer = {}

        # Setup publisher for inference output (raw action)
        self._cbg = ReentrantCallbackGroup()
        from ibrobot_msgs.msg import VariantsList
        variant_msg_cls = VariantsList
        
        self._action_pub = self.create_publisher(
            variant_msg_cls,
            output_topic,
            10,
        )
        self.get_logger().info(f"Publishing inference output to: {output_topic}")

        # Setup subscription for pre-processed observations
        self.create_subscription(
            variant_msg_cls,
            input_topic,
            self._variant_cb,
            10,
            callback_group=self._cbg,
        )
        self.get_logger().info(f"Subscribed to: {input_topic}")

        # Inference loop timer
        self._timer = self.create_timer(self.step_sec, self._inference_tick, callback_group=self._cbg)

        self.get_logger().info(f"VariantPolicyBridge ready at {self.fps} Hz")

    def _variant_cb(self, msg) -> None:
        """Callback for VariantsList messages."""
        try:
            batch = TensorMsgConverter.from_variant(msg, self.device)
            self._variant_buffer = batch
        except Exception as e:
            self.get_logger().error(f"Failed to decode VariantsList: {e!r}")

    def _inference_tick(self) -> None:
        """Main inference loop."""
        batch = self._variant_buffer
        if not batch:
            self.get_logger().warning("No variants in buffer, skipping inference")
            return

        with torch.inference_mode():
            action = self.policy.select_action(batch)
        
        # Publish raw action as VariantsList (no post-processing here)
        action_batch = {"action": action}
        variant_msg = TensorMsgConverter.to_variant(action_batch)
        self._action_pub.publish(variant_msg)


def main():
    """Main entry point."""
    try:
        rclpy.init()
        node = VariantPolicyBridge()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
