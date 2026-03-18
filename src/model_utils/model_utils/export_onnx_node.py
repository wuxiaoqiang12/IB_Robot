#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExportOnnxNode: ROS2 Node for exporting LeRobot policies to ONNX format.

This node exports ACT and other policy models from PyTorch to ONNX format
at startup, then exits.

Usage:
    ros2 run rosetta export_onnx_node --ros-args \
        -p policy_path:="/path/to/policy" \
        -p policy_type:="act" \
        -p device:="cuda" \
        -p output_name:="model.onnx"
        
    Or use launch file:
    ros2 launch rosetta export_onnx.launch.py policy_path:=/path/to/policy
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import onnx
import rclpy
import torch
from onnxsim import simplify
from rclpy.node import Node


def logger(msg: str) -> None:
    print(f"[export_onnx]: {msg}")


class ExportOnnxNode(Node):
    """ROS2 Node for exporting LeRobot policies to ONNX format."""

    def __init__(self) -> None:
        super().__init__("export_onnx_node")

        # Declare parameters
        self.declare_parameter("policy_path", "")
        self.declare_parameter("policy_type", "act")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("output_name", "act_ros2.onnx")
        self.declare_parameter("simplify_onnx", True)

        # Get parameters
        policy_path = self.get_parameter("policy_path").value
        policy_type = self.get_parameter("policy_type").value
        device = self.get_parameter("device").value
        output_name = self.get_parameter("output_name").value
        simplify_model = self.get_parameter("simplify_onnx").value

        # Validate required parameters
        if not policy_path:
            self.get_logger().error("Parameter 'policy_path' is required but not set!")
            self.get_logger().error("Usage: ros2 run rosetta export_onnx_node --ros-args -p policy_path:=\"/path/to/policy\"")
            sys.exit(1)

        if not os.path.exists(policy_path):
            self.get_logger().error(f"Policy path does not exist: {policy_path}")
            sys.exit(1)

        self.get_logger().info(f"Starting ONNX export...")
        self.get_logger().info(f"  Policy path: {policy_path}")
        self.get_logger().info(f"  Policy type: {policy_type}")
        self.get_logger().info(f"  Device: {device}")
        self.get_logger().info(f"  Output name: {output_name}")
        self.get_logger().info(f"  Simplify: {simplify_model}")

        try:
            if policy_type == "act":
                self.export_act(policy_path, device, output_name, simplify_model)
                self.get_logger().info("Export completed successfully!")
            else:
                self.get_logger().error(f"Unsupported policy type: {policy_type}")
                sys.exit(1)
        except Exception as e:
            self.get_logger().error(f"Export failed: {e!r}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            sys.exit(1)

    def export_act(self, policy_path, device, output_name, simplify_model):
        from lerobot.policies.act.modeling_act import ACTPolicy

        model_folder = policy_path
        model_path = os.path.join(model_folder, "pretrained_model")
        onnx_path = os.path.join(model_folder, "act_ros2.onnx")

        policy = ACTPolicy.from_pretrained(model_path)
        policy.model.eval()

        # 准备 dummy input
        dummy_batch = {
            "observation.state": torch.randn(1, 6, dtype=torch.float32, device=device),
            "observation.images.top": torch.randn(1, 3, 240, 320, dtype=torch.float32, device=device),
            "observation.images.wrist": torch.randn(1, 3, 240, 320, dtype=torch.float32, device=device),
        }
        dummy_batch['observation.images'] = [
            dummy_batch['observation.images.top'],
            dummy_batch['observation.images.wrist']
        ]

        # 先测试前向传播
        example_output = policy.model(dummy_batch)
        self.get_logger().info(f"Example output shape: {example_output[0].shape}")

        # ✅ 关键修改：正确准备 ONNX 导出的输入
        # 方法：使用包装器将 dict 输入转换为位置参数
        
        class DictWrapper(torch.nn.Module):
            """将 dict 输入包装为位置参数的包装器，用于 ONNX 导出"""
            
            def __init__(self, model, input_keys):
                super().__init__()
                self.model = model
                self.input_keys = input_keys
                
            def forward(self, *args):
                # 将位置参数重新组合为字典
                batch = {key: arg for key, arg in zip(self.input_keys, args)}
                # 特殊处理：需要重建 observation.images 列表
                if 'observation.images.top' in batch and 'observation.images.wrist' in batch:
                    batch['observation.images'] = [
                        batch.pop('observation.images.top'),
                        batch.pop('observation.images.wrist')
                    ]
                return self.model(batch)
        
        # 准备输入键列表（对应 ONNX 输入节点）
        input_keys = [
            'observation.state',
            'observation.images.top',
            'observation.images.wrist'
        ]
        
        # 创建包装器
        wrapped_model = DictWrapper(policy.model, input_keys)
        
        # 准备位置参数形式的输入
        example_inputs = (
            dummy_batch['observation.state'],
            dummy_batch['observation.images.top'],
            dummy_batch['observation.images.wrist'],
        )
        
        self.get_logger().info("Exporting onnx")
        torch.onnx.export(
            wrapped_model,           # ✅ 使用包装后的模型
            example_inputs,          # ✅ 位置参数形式的输入
            onnx_path,
            input_names=input_keys,  # ✅ 明确的输入节点名称
            opset_version=13,
            output_names=["action"],
            dynamic_axes={
                'observation.state': {0: 'batch_size'},
                'observation.images.top': {0: 'batch_size'},
                'observation.images.wrist': {0: 'batch_size'},
                'action': {0: 'batch_size'}
            },
            external_data=True,
            verbose=False,
            do_constant_folding=False,
        )

        self.get_logger().info("Simplify onnx")
        onnx_model = onnx.load(onnx_path)
        model_simp, check = simplify(onnx_model)
        if not check:
            raise ValueError("Simplified ONNX model could not be validated")
        onnx.save(model_simp, model_folder + "act_ros2_simplified.onnx")
        self.get_logger().info("finished exporting onnx")

def main(args: list[str] | None = None) -> None:
    """Main function to run the ExportOnnxNode."""
    rclpy.init(args=args)
    node = ExportOnnxNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
