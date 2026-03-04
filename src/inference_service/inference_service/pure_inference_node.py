#!/usr/bin/env python3
"""
Pure inference node for distributed/composed mode.

This node:
- Subscribes to preprocessed VariantsList
- Runs pure inference (no preprocessing/postprocessing)
- Publishes raw action as VariantsList

Designed to work with PreprocessorComponent and PostprocessorComponent.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import rclpy
import torch
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rosetta.common.decoders import dec_variant_list
from rosetta.common.encoders import enc_variant_list
from rosetta_interfaces.msg import VariantsList


def _resolve_device(device: str) -> torch.device:
    """Resolve device string to torch.device."""
    if device == "auto":
        if torch.cuda.is_available():
            return torch.device("cuda")
        return torch.device("cpu")
    return torch.device(device)


class PureInferenceNode(Node):
    """
    Pure inference node without preprocessing/postprocessing.

    Subscribes: /preprocessed/batch (VariantsList)
    Publishes: /inference/action (VariantsList)
    """

    def __init__(
        self,
        node_name: str = "pure_inference",
        policy_path: Optional[str] = None,
        input_topic: str = "/preprocessed/batch",
        output_topic: str = "/inference/action",
        device: str = "auto",
    ):
        super().__init__(node_name)

        self._device = _resolve_device(device)
        self._policy = None
        self._policy_type = ""
        self._use_action_chunking = False
        self._chunk_size = 1

        # Load policy
        if policy_path:
            self._load_policy(policy_path)

        # Subscriber for preprocessed input
        self._sub = self.create_subscription(
            VariantsList,
            input_topic,
            self._inference_cb,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # Publisher for raw action output
        self._pub = self.create_publisher(VariantsList, output_topic, 10)

        self.get_logger().info(
            f"PureInferenceNode ready: device={self._device}, "
            f"input={input_topic}, output={output_topic}"
        )

    def _load_policy(self, policy_path: str):
        """Load LeRobot policy."""
        from lerobot.policies.factory import get_policy_class

        is_hf_repo = "/" in policy_path and not os.path.exists(policy_path)

        if is_hf_repo:
            for policy_type in [
                "act",
                "diffusion",
                "pi0",
                "pi05",
                "smolvla",
                "tdmpc",
                "vqbet",
            ]:
                try:
                    PolicyCls = get_policy_class(policy_type)
                    self._policy = PolicyCls.from_pretrained(policy_path)
                    self._policy.to(self._device)
                    self._policy.eval()
                    self._policy_type = policy_type
                    self.get_logger().info(f"Loaded {policy_type} from {policy_path}")
                    break
                except Exception as e:
                    self.get_logger().debug(f"Not {policy_type}: {e}")
        else:
            cfg_json = os.path.join(policy_path, "config.json")
            cfg_type = ""
            if os.path.exists(cfg_json):
                with open(cfg_json, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                    cfg_type = str(cfg.get("type", "")).lower()

            if cfg_type:
                PolicyCls = get_policy_class(cfg_type)
                self._policy = PolicyCls.from_pretrained(policy_path)
                self._policy.to(self._device)
                self._policy.eval()
                self._policy_type = cfg_type

        if not self._policy:
            raise RuntimeError(f"Failed to load policy from {policy_path}")

        self._use_action_chunking = self._policy_type in ("act", "tdmpc", "vqbet")
        if hasattr(self._policy.config, "chunk_size"):
            self._chunk_size = self._policy.config.chunk_size
        elif hasattr(self._policy.config, "action_chunk_size"):
            self._chunk_size = self._policy.config.action_chunk_size

        self.get_logger().info(
            f"Policy loaded: type={self._policy_type}, "
            f"chunking={self._use_action_chunking}, chunk_size={self._chunk_size}"
        )

    def _inference_cb(self, msg: VariantsList):
        """Run inference on preprocessed input."""
        try:
            batch = dec_variant_list(msg, self._device)

            with torch.no_grad():
                if self._use_action_chunking:
                    action = self._policy.predict_action_chunk(batch)
                    action = action.squeeze(0)
                else:
                    action = self._policy.select_action(batch)[0]

            # Publish as VariantsList
            result = {"action": action}
            out_msg = enc_variant_list(result)
            self._pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")

    def infer(self, batch: Dict[str, Any]) -> Dict[str, Any]:
        """Direct inference for zero-copy mode."""
        with torch.no_grad():
            if self._use_action_chunking:
                action = self._policy.predict_action_chunk(batch)
                action = action.squeeze(0)
            else:
                action = self._policy.select_action(batch)[0]
        return {"action": action}


def main():
    rclpy.init()

    from rclpy.node import Node

    # Use temp node to read ROS parameters
    temp = Node("_pure_inference_param_reader")
    temp.declare_parameter("policy_path", "")
    temp.declare_parameter("input_topic", "/preprocessed/batch")
    temp.declare_parameter("output_topic", "/inference/action")
    temp.declare_parameter("device", "auto")
    temp.declare_parameter("use_sim_time", False)

    params = {
        "policy_path": temp.get_parameter("policy_path").value or None,
        "input_topic": temp.get_parameter("input_topic").value,
        "output_topic": temp.get_parameter("output_topic").value,
        "device": temp.get_parameter("device").value,
    }
    temp.destroy_node()

    node = PureInferenceNode(
        node_name="pure_inference",
        policy_path=params["policy_path"],
        input_topic=params["input_topic"],
        output_topic=params["output_topic"],
        device=params["device"],
    )

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
