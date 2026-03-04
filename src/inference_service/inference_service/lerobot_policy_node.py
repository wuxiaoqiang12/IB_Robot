#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot policy wrapper for ROS2 inference.

This node extends BaseInferenceNode to provide LeRobot policy integration.
Supports all LeRobot policies via HuggingFace.

IMPORTANT: This handles ALL LeRobot policy types, including VLA models like SmolVLA.
The key insight is that LeRobot provides a unified interface for all policy types:
- select_action(): Single action prediction
- predict_action_chunk(): Action chunking (for ACT, TDMPC, etc.)

Supported policy types:
- act: Action Chunking Transformer
- diffusion: Diffusion Policy
- tdmpc: TDMPC
- vqbet: VQ-BeT
- pi0, pi05: Pi-family policies
- smolvla: Small Vision-Language-Action model (VLA)

Note: SmolVLA is treated as a standard LeRobot policy, not a separate VLA class.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import torch

from inference_service.base_model_node import BaseInferenceNode, _ModelConfig
from inference_service.passive_inference_node import PassiveInferenceNode


def _device_from_param(requested: Optional[str] = None) -> torch.device:
    """Get torch device from parameter string."""
    r = (requested or "auto").lower().strip()

    def mps_available() -> bool:
        return (
            bool(getattr(torch.backends, "mps", None))
            and torch.backends.mps.is_available()
        )

    if r == "auto":
        if torch.cuda.is_available():
            return torch.device("cuda")
        if mps_available():
            return torch.device("mps")
        return torch.device("cpu")

    # Explicit CUDA (supports 'cuda' and 'cuda:N')
    if r.startswith("cuda"):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but not available")
        device_idx = r[5:] or "0"
        return torch.device(f"cuda:{device_idx}" if device_idx else "cuda")

    # Explicit MPS (Metal Performance Shaders on macOS)
    if r in ("mps", "metal"):
        if not mps_available():
            raise RuntimeError("MPS requested but not available")
        return torch.device("mps")

    # Explicit CPU
    if r == "cpu":
        return torch.device("cpu")

    # Ascend NPU support (if available)
    if r.startswith("npu"):
        try:
            import torch_npu

            if torch_npu.npu.is_available():
                device_idx = r[3:] or "0"
                return torch.device(f"npu:{device_idx}" if device_idx else "npu")
        except ImportError:
            pass
        raise RuntimeError("NPU requested but torch_npu not available")

    raise ValueError(f"Unknown device request: {requested}")


class LeRobotPolicyNode(PassiveInferenceNode):
    """
    LeRobot policy inference node.

    This is the PRIMARY node for ALL LeRobot-compatible policies.

    Supports loading policies from:
    - HuggingFace repo IDs (e.g., "lerobot/act_example")
    - Local paths with pre-trained policies

    Policy types supported (unified interface):
    - act: Action Chunking Transformer
    - diffusion: Diffusion Policy
    - tdmpc: TDMPC
    - vqbet: VQ-BeT
    - pi0, pi05: Pi-family policies
    - smolvla: Small Vision-Language-Action model (VLA is just another LeRobot policy!)

    NOTE: SmolVLA and other VLA models are handled here, not in a separate VLAPolicyNode.
    This follows the PolicyBridge pattern: all LeRobot policies use the same interface.
    """

    def __init__(self, model_config: Dict):
        # Override device handling before parent init
        device_req = model_config.get("device", "auto")
        self._lerobot_device = _device_from_param(device_req)
        model_config["device"] = str(self._lerobot_device)

        # Policy-specific state (MUST be initialized before super().__init__)
        # because parent __init__ will call self._load_model() which sets these
        self._policy: Any = None
        self._preprocessor: Any = None
        self._postprocessor: Any = None
        self._policy_type: str = ""
        self._use_action_chunking: bool = False
        self._chunk_size: int = 1

        # Call parent __init__ (PassiveInferenceNode -> BaseInferenceNode)
        # This will call self._load_model() which sets the variables above
        super().__init__(model_config)

    def _load_model(self, config: Dict):
        """Load LeRobot policy from HuggingFace or local path."""
        from lerobot.policies.factory import get_policy_class
        from lerobot.policies.factory import make_pre_post_processors

        policy_path = config.get("repo_id") or config.get("checkpoint") or ""
        if not policy_path:
            raise RuntimeError(
                "LeRobotPolicyNode: 'repo_id' or 'checkpoint' is required"
            )

        is_hf_repo = "/" in policy_path and not os.path.exists(policy_path)
        cfg_type = ""

        self.get_logger().info(f"Using device: {self._lerobot_device}")

        if is_hf_repo:
            self.get_logger().info(f"Loading from Hugging Face: {policy_path}")
            policy_types_to_try = [
                "act",
                "diffusion",
                "pi0",
                "pi05",
                "smolvla",
                "tdmpc",
                "vqbet",
            ]
            for policy_type in policy_types_to_try:
                try:
                    PolicyCls = get_policy_class(policy_type)
                    self._policy = PolicyCls.from_pretrained(policy_path)
                    self._policy.to(self._lerobot_device)
                    self._policy.eval()
                    cfg_type = policy_type
                    self.get_logger().info(
                        f"Loaded {policy_type} policy from {policy_path}"
                    )
                    break
                except Exception as e:
                    self.get_logger().debug(f"Not {policy_type}: {e}")
                    continue

            if not self._policy:
                raise RuntimeError(
                    f"Could not load policy from {policy_path} with any known policy type"
                )
        else:
            cfg_json = os.path.join(policy_path, "config.json")
            if os.path.exists(cfg_json):
                try:
                    with open(cfg_json, "r", encoding="utf-8") as f:
                        cfg = json.load(f)
                        cfg_type = str(cfg.get("type", "")).lower()
                except (OSError, json.JSONDecodeError) as e:
                    self.get_logger().warning(f"Could not read config.json: {e}")

            if not cfg_type:
                raise RuntimeError(
                    f"Could not determine policy type from {policy_path}"
                )

            self.get_logger().info(f"Loading {cfg_type} from: {policy_path}")
            PolicyCls = get_policy_class(cfg_type)
            self._policy = PolicyCls.from_pretrained(policy_path)
            self._policy.to(self._lerobot_device)
            self._policy.eval()

        self._policy_type = cfg_type

        ds_stats = None
        for cand in ("dataset_stats.json", "stats.json", "meta/stats.json"):
            p = Path(policy_path) / cand
            if p.exists():
                try:
                    with p.open("r", encoding="utf-8") as f:
                        ds_stats = json.load(f)
                    self.get_logger().info(f"Loaded dataset stats from {p}")
                    break
                except Exception as e:
                    self.get_logger().warning(f"Failed to read {p}: {e}")

        if hasattr(self, "_contract"):
            try:
                from rosetta.common.contract_utils import contract_fingerprint

                current_fp = contract_fingerprint(self._contract)
                policy_fp_path = Path(policy_path) / "contract_fingerprint.txt"
                if policy_fp_path.exists():
                    stored_fp = policy_fp_path.read_text().strip()
                    if stored_fp != current_fp:
                        self.get_logger().warning(
                            f"Contract fingerprint mismatch! Policy: {stored_fp}, Current: {current_fp}"
                        )
                    else:
                        self.get_logger().info("Contract fingerprint matches")
            except Exception as e:
                self.get_logger().warning(f"Fingerprint validation failed: {e}")

        self._preprocessor, self._postprocessor = make_pre_post_processors(
            policy_cfg=self._policy.config,
            pretrained_path=policy_path,
            dataset_stats=ds_stats,
            preprocessor_overrides={
                "device_processor": {"device": str(self._lerobot_device)}
            },
            postprocessor_overrides={
                "device_processor": {"device": str(self._lerobot_device)}
            },
        )

        self._use_action_chunking = self._policy_type in ("act", "tdmpc", "vqbet")
        if hasattr(self._policy.config, "chunk_size"):
            self._chunk_size = self._policy.config.chunk_size
        elif hasattr(self._policy.config, "action_chunk_size"):
            self._chunk_size = self._policy.config.action_chunk_size
        else:
            self._chunk_size = 1

        self.get_logger().info(
            f"Policy ready: type={self._policy_type}, "
            f"chunking={self._use_action_chunking}, chunk_size={self._chunk_size}"
        )

    def _inference(self, batch: Dict[str, Any]) -> np.ndarray:
        """
        Run inference and return action.

        Args:
            batch: Dictionary with preprocessed observations

        Returns:
            Action as numpy array (1D for single action, 2D for chunk)
        """
        if self._policy is None:
            raise RuntimeError("Policy is None - model loading may have failed")

        if self._preprocessor:
            batch = self._preprocessor(batch)

        with torch.no_grad():
            if self._use_action_chunking:
                chunk = self._policy.predict_action_chunk(batch)
                self.get_logger().debug(f"Action chunk shape: {chunk.shape}")
                chunk = chunk.squeeze(0)
                chunk = self._postprocess_actions(chunk)
                return chunk.detach().cpu().numpy().astype(np.float32)
            else:
                action = self._policy.select_action(batch)
                self.get_logger().debug(f"Action shape: {action.shape}")
                action = self._postprocess_actions(action)
                return action[0].detach().cpu().numpy().astype(np.float32)

    def _postprocess_actions(self, x: torch.Tensor) -> torch.Tensor:
        """Post-process actions using the policy's postprocessor."""
        x = x.to(self._lerobot_device)
        return self._postprocessor(x)

    def get_device(self) -> Any:
        """Override to return the LeRobot-specific device."""
        return self._lerobot_device


def main() -> None:
    """Main entry point for LeRobot policy node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    rclpy.init()

    try:
        temp_node = Node("_param_reader")
        for p in [
            "name",
            "node_name",
            "model_type",
            "repo_id",
            "checkpoint",
            "contract_path",
            "device",
            "frequency",
            "use_header_time",
        ]:
            temp_node.declare_parameter(
                p,
                ""
                if p in ["repo_id", "checkpoint", "contract_path"]
                else "auto"
                if p == "device"
                else 10.0
                if p == "frequency"
                else True
                if p == "use_header_time"
                else f"lerobot_policy{'_node' if p == 'node_name' else ''}",
            )

        config = {
            p: temp_node.get_parameter(p).value or None
            for p in [
                "name",
                "node_name",
                "model_type",
                "repo_id",
                "checkpoint",
                "contract_path",
            ]
        }
        config["device"] = temp_node.get_parameter("device").value
        config["frequency"] = temp_node.get_parameter("frequency").value
        config["use_header_time"] = temp_node.get_parameter("use_header_time").value
        temp_node.destroy_node()

        node = LeRobotPolicyNode(config)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        node.get_logger().info("LeRobot policy node started")

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
