#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LossCompareNode: ROS2 Node for comparing model outputs.

This node compares PyTorch model outputs with target outputs and computes L1 loss.
Supports two modes:
- generate_target: Generate target outputs from batch data
- compute_loss: Compute L1 loss between predictions and targets

Usage:
    # Generate target mode
    ros2 run rosetta loss_compare_node --ros-args \
        -p mode:="generate_target" \
        -p batch_path:="/path/to/batches.json" \
        -p policy_path:="/path/to/policy"
        
    # Compute loss mode
    ros2 run rosetta loss_compare_node --ros-args \
        -p mode:="compute_loss" \
        -p batch_path:="/path/to/batches.json" \
        -p target_path:="/path/to/targets.json" \
        -p policy_path:="/path/to/policy"
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from tqdm import tqdm

from lerobot.policies.factory import make_pre_post_processors
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device


class LossCompareNode(Node):
    """ROS2 Node for comparing model outputs and computing loss."""

    def __init__(self) -> None:
        super().__init__("loss_compare_node")

        # Declare parameters
        self.declare_parameter("mode", "compute_loss")
        self.declare_parameter("batch_path", "")
        self.declare_parameter("target_path", "")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("policy_type", "act")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("output_dir", "/tmp/loss_compare")

        # Get parameters
        mode = self.get_parameter("mode").value
        batch_path = self.get_parameter("batch_path").value
        target_path = self.get_parameter("target_path").value
        policy_path = self.get_parameter("policy_path").value
        policy_type = self.get_parameter("policy_type").value
        device = self.get_parameter("device").value
        output_dir = self.get_parameter("output_dir").value

        # Validate required parameters
        if not policy_path:
            self.get_logger().error("Parameter 'policy_path' is required but not set!")
            self.get_logger().error("Usage: ros2 run rosetta loss_compare_node --ros-args -p policy_path:=\"/path/to/policy\"")
            sys.exit(1)

        if not batch_path:
            self.get_logger().error("Parameter 'batch_path' is required but not set!")
            sys.exit(1)

        if not Path(policy_path).exists():
            self.get_logger().error(f"Policy path does not exist: {policy_path}")
            sys.exit(1)

        if not Path(batch_path).exists():
            self.get_logger().error(f"Batch path does not exist: {batch_path}")
            sys.exit(1)

        self.get_logger().info(f"Starting LossCompare in '{mode}' mode...")
        self.get_logger().info(f"  Batch path: {batch_path}")
        self.get_logger().info(f"  Policy path: {policy_path}")
        self.get_logger().info(f"  Policy type: {policy_type}")
        self.get_logger().info(f"  Device: {device}")

        try:
            if mode == "generate_target":
                self.generate_target(batch_path, policy_path, policy_type, device, output_dir)
            elif mode == "compute_loss":
                if not target_path:
                    self.get_logger().error("Parameter 'target_path' is required for compute_loss mode!")
                    sys.exit(1)
                if not Path(target_path).exists():
                    self.get_logger().error(f"Target path does not exist: {target_path}")
                    sys.exit(1)
                self.compute_loss(batch_path, target_path, policy_path, policy_type, device)
            else:
                self.get_logger().error(f"Invalid mode: {mode}. Use 'generate_target' or 'compute_loss'")
                sys.exit(1)
            
            self.get_logger().info("Operation completed successfully!")
            
        except Exception as e:
            self.get_logger().error(f"Operation failed: {e!r}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            sys.exit(1)

    def prepare_policy(self, policy_path: str, policy_type: str, device: str):
        """Load and prepare the policy model."""
        policy_path_obj = Path(policy_path)
        
        # Auto-detect pretrained_model subdirectory if config.json not found in the given path
        if not (policy_path_obj / "config.json").exists():
            alt_path = policy_path_obj / "pretrained_model"
            if alt_path.exists() and (alt_path / "config.json").exists():
                self.get_logger().info(f"Config not found in {policy_path}, using {alt_path} instead")
                policy_path = str(alt_path)
            else:
                raise FileNotFoundError(
                    f"config.json not found in {policy_path} or {alt_path}. "
                    "Please provide a valid policy path containing config.json"
                )
        
        if policy_type == "act":
            from lerobot.policies.act.modeling_act import ACTPolicy
            policy = ACTPolicy.from_pretrained(policy_path)
        else:
            raise NotImplementedError(f"Policy type {policy_type} not implemented")

        self.get_logger().info(f"Model loaded: {policy_path}")
        return policy

    def load_batches_as_tensors(self, batch_path: str) -> list[dict]:
        """Load batches from JSON file and convert to tensors."""
        self.get_logger().info(f"Loading batches from {batch_path}...")
        with open(batch_path, encoding="utf-8") as f:
            raw_batches = json.load(f)

        processed_batches = []
        for b in raw_batches:
            processed_batch = {}
            for k, v in b.items():
                if "side_view" in k:
                    continue
                elif k == "observation.images.hand_view":
                    processed_batch["observation.images.wrist"] = np.array(v).astype(np.float32)
                elif k == "observation.images.top_view":
                    processed_batch["observation.images.top"] = np.array(v).astype(np.float32)
                else:
                    processed_batch[k] = np.array(v).astype(np.float32)
            processed_batches.append(processed_batch)

        self.get_logger().info(f"Loaded {len(processed_batches)} batches")
        return processed_batches

    def forward(
        self,
        batches: list[dict],
        policy,
        policy_path: str,
        device: torch.device
    ) -> list[torch.Tensor]:
        """Run forward pass on all batches."""
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy.config,
            pretrained_path=policy_path
        )

        outputs = []
        for i in tqdm(range(len(batches)), desc="forwarding"):
            output = predict_action(
                observation=batches[i],
                policy=policy,
                device=device,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                use_amp=policy.config.use_amp,
            )
            outputs.append(output)

        return outputs

    def generate_target(
        self,
        batch_path: str,
        policy_path: str,
        policy_type: str,
        device: str,
        output_dir: str
    ) -> None:
        """Generate target outputs from batch data."""
        self.get_logger().info("Generating target outputs...")

        policy = self.prepare_policy(policy_path, policy_type, device)
        device_obj = get_safe_torch_device(device)

        batches = self.load_batches_as_tensors(batch_path)

        start_time = time.perf_counter()
        outputs = self.forward(batches, policy, policy_path, device_obj)
        end_time = time.perf_counter()
        inference_time = end_time - start_time

        # Convert outputs to list for JSON serialization
        output_list = [out.tolist() for out in outputs]

        # Save to output file
        output_dir_path = Path(output_dir)
        output_dir_path.mkdir(parents=True, exist_ok=True)
        output_path = output_dir_path / "generated_targets.json"

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(output_list, f, indent=4)

        self.get_logger().info(f"Generated {len(outputs)} targets")
        self.get_logger().info(f"Inference time: {inference_time:.3f}s")
        self.get_logger().info(f"Target saved to: {output_path}")

    def compute_loss(
        self,
        batch_path: str,
        target_path: str,
        policy_path: str,
        policy_type: str,
        device: str
    ) -> None:
        """Compute L1 loss between predictions and targets."""
        self.get_logger().info("Computing loss...")

        # Load targets
        self.get_logger().info(f"Loading targets from {target_path}...")
        with open(target_path, encoding="utf-8") as f:
            targets = json.load(f)
        targets = [torch.tensor(t) for t in targets]

        policy = self.prepare_policy(policy_path, policy_type, device)
        device_obj = get_safe_torch_device(device)

        batches = self.load_batches_as_tensors(batch_path)

        if len(targets) != len(batches):
            raise ValueError(f"Length mismatch: targets {len(targets)} vs batches {len(batches)}")

        # Forward pass
        start_time = time.perf_counter()
        preds = self.forward(batches, policy, policy_path, device_obj)
        end_time = time.perf_counter()
        inference_time = end_time - start_time

        # Compute losses
        losses = []
        self.get_logger().info("Computing L1 losses...")
        for i in range(len(targets)):
            loss = torch.nn.functional.l1_loss(preds[i], targets[i], reduction="mean")
            losses.append(loss.item())
            self.get_logger().info(f"  Batch {i}: loss = {loss.item():.6f}")

        avg_loss = sum(losses) / len(losses)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Results:")
        self.get_logger().info(f"  Number of batches: {len(losses)}")
        self.get_logger().info(f"  Inference time: {inference_time:.3f}s")
        self.get_logger().info(f"  Average loss: {avg_loss:.6f}")
        self.get_logger().info("=" * 50)


def main(args: list[str] | None = None) -> None:
    """Main function to run the LossCompareNode."""
    rclpy.init(args=args)
    node = LossCompareNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
