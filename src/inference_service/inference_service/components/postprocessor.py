#!/usr/bin/env python3
"""
Postprocessor Component for LeRobot policies.

Can run as:
1. Independent ROS2 node (distributed mode)
2. Composed in same process with Inference component (zero-copy mode)

Responsibilities:
- Subscribe to raw action output from inference
- Apply de-normalization using model's dataset_stats
- Publish to controller topics
"""

from __future__ import annotations

import json
import os
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np
import rclpy
import torch
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rosetta.common.contract_utils import (
    SpecView,
    encode_value,
    iter_specs,
    load_contract,
)
from tensormsg.converter import TensorMsgConverter
from ibrobot_msgs.msg import VariantsList


class PostprocessorBase(ABC):
    """Abstract base for postprocessor implementations."""

    @abstractmethod
    def __call__(self, action: Any) -> Any:
        """Apply postprocessing to action."""
        pass


class LeRobotPostprocessor(PostprocessorBase):
    """LeRobot-specific postprocessor using make_pre_post_processors."""

    def __init__(self, policy_path: str, device: torch.device):
        from lerobot.policies.factory import make_pre_post_processors

        self.device = device
        policy_cfg = self._load_policy_config(policy_path)

        _, self._postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=policy_path,
            postprocessor_overrides={"device_processor": {"device": str(device)}},
        )

    def _load_policy_config(self, policy_path: str) -> dict:
        cfg_json = os.path.join(policy_path, "config.json")
        if os.path.exists(cfg_json):
            with open(cfg_json, "r", encoding="utf-8") as f:
                return json.load(f)
        return {}

    def __call__(self, action: Any) -> Any:
        return self._postprocessor(action)


class PostprocessorComponent(Node):
    """
    Postprocessor component that can run standalone or composed.

    Subscribes to raw action output and publishes to controller topics.
    """

    def __init__(
        self,
        node_name: str = "postprocessor",
        contract_path: Optional[str] = None,
        policy_path: Optional[str] = None,
        input_topic: str = "/inference/action",
        device: str = "auto",
        postprocessor: Optional[PostprocessorBase] = None,
    ):
        super().__init__(node_name)

        self._device = self._resolve_device(device)

        # Load contract
        if contract_path:
            self._load_contract(contract_path)
        else:
            self._contract = None
            self._action_specs = []
            self._act_pubs = {}

        # Setup postprocessor
        if postprocessor:
            self._postprocessor = postprocessor
        elif policy_path:
            self._postprocessor = LeRobotPostprocessor(policy_path, self._device)
        else:
            self._postprocessor = None

        # Subscribe to raw action output
        self._sub = self.create_subscription(
            VariantsList,
            input_topic,
            self._action_cb,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"Postprocessor ready: device={self._device}, "
            f"input={input_topic}, specs={len(self._action_specs)}"
        )

    def _resolve_device(self, device: str) -> torch.device:
        if device == "auto":
            if torch.cuda.is_available():
                return torch.device("cuda")
            return torch.device("cpu")
        return torch.device(device)

    def _load_contract(self, contract_path: str):
        self._contract = load_contract(Path(contract_path))
        self._action_specs: List[SpecView] = [
            s for s in iter_specs(self._contract) if s.is_action
        ]

        # Create publishers for each action spec
        self._act_pubs: Dict[str, Any] = {}
        for spec in self._action_specs:
            from rosidl_runtime_py.utilities import get_message

            msg_cls = get_message(spec.ros_type)
            self._act_pubs[spec.topic] = self.create_publisher(msg_cls, spec.topic, 10)
            self.get_logger().info(f"Action publisher: {spec.topic}")

    def _action_cb(self, msg: VariantsList):
        """Callback for inference output - apply post-processing and publish."""
        try:
            batch = TensorMsgConverter.from_variant(msg, self._device)
            action = batch.get("action")
            if action is None:
                self.get_logger().warning("No 'action' key in VariantsList")
                return

            # Apply postprocessing
            if self._postprocessor:
                action = self._postprocessor(action)

            # Publish to controllers
            self._publish_actions(action)

        except Exception as e:
            self.get_logger().error(f"Postprocessing failed: {e}")

    def _publish_actions(self, action: Any):
        """Publish action to controller topics."""
        if torch.is_tensor(action):
            action = action.detach().cpu().numpy()
        action = np.asarray(action).ravel()

        start_idx = 0
        for spec in self._action_specs:
            spec_len = len(spec.names) if spec.names else 0
            if spec_len == 0:
                continue

            end_idx = start_idx + spec_len
            if end_idx > len(action):
                self.get_logger().error(
                    f"Action too short for {spec.key}: need {spec_len}, have {len(action) - start_idx}"
                )
                break

            spec_action = action[start_idx:end_idx]
            ros_msg = encode_value(
                ros_type=spec.ros_type,
                names=spec.names,
                action_vec=spec_action.tolist(),
                clamp=getattr(spec, "clamp", None),
            )

            self._act_pubs[spec.topic].publish(ros_msg)
            start_idx = end_idx

    def process(self, action: Any) -> Any:
        """Direct processing for zero-copy mode."""
        if self._postprocessor:
            action = self._postprocessor(action)
        self._publish_actions(action)
        return action


def main():
    rclpy.init()

    from rclpy.node import Node

    # Use temp node to read ROS parameters
    temp = Node("_postprocessor_param_reader")
    temp.declare_parameter("contract_path", "")
    temp.declare_parameter("policy_path", "")
    temp.declare_parameter("input_topic", "/inference/action")
    temp.declare_parameter("device", "auto")
    temp.declare_parameter("use_sim_time", False)

    params = {
        "contract_path": temp.get_parameter("contract_path").value or None,
        "policy_path": temp.get_parameter("policy_path").value or None,
        "input_topic": temp.get_parameter("input_topic").value,
        "device": temp.get_parameter("device").value,
        "use_sim_time": temp.get_parameter("use_sim_time").value,
    }
    temp.destroy_node()

    node = PostprocessorComponent(
        node_name="postprocessor",
        contract_path=params["contract_path"],
        policy_path=params["policy_path"],
        input_topic=params["input_topic"],
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
