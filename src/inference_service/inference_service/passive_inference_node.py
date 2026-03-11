#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Passive inference node extension for pull-based inference.

Extends BaseInferenceNode with DispatchInfer Action Server support
for dispatcher-initiated inference requests.
"""

from __future__ import annotations

import time
import traceback
from typing import Dict

import numpy as np
import rclpy
import rclpy.action
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from diagnostic_msgs.msg import DiagnosticStatus

from ibrobot_msgs.action import DispatchInfer
from ibrobot_msgs.msg import VariantsList
from inference_service.base_model_node import BaseInferenceNode


class PassiveInferenceNode(BaseInferenceNode):
    """
    Extended inference node supporting both timer-based and pull-based inference.

    Adds DispatchInfer Action Server for dispatcher-initiated inference requests,
    while maintaining backward compatibility with timer-based mode.
    """

    def __init__(self, model_config: Dict):
        model_config.setdefault("passive_mode", True)
        super().__init__(model_config)

        self._action_server = rclpy.action.ActionServer(
            self,
            DispatchInfer,
            "~/DispatchInfer",
            execute_callback=self._dispatch_infer_callback,
            goal_callback=lambda req: rclpy.action.GoalResponse.ACCEPT,
            cancel_callback=lambda handle: rclpy.action.CancelResponse.ACCEPT,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info("DispatchInfer Action Server ready")

    def _dispatch_infer_callback(self, goal_handle):
        """Execute inference requested by dispatcher."""
        goal = goal_handle.request
        obs_timestamp_ns = goal.obs_timestamp.sec * 10**9 + goal.obs_timestamp.nanosec

        try:
            obs_frame = self._sample_obs_frame(obs_timestamp_ns)
            batch = self._prepare_batch(obs_frame)
            action = self._inference(batch)
            self._publish_action(action)

            result = DispatchInfer.Result()
            result.action_chunk = self._create_action_msg(action)
            result.chunk_size = action.shape[0] if action.ndim == 2 else 1
            result.success = True
            result.message = "OK"
            result.inference_latency_ms = (
                self.get_clock().now().nanoseconds / 1e9 - obs_timestamp_ns / 1e9
            ) * 1000.0

            goal_handle.succeed()
            self._last_inference_time = time.time()
            self._inference_count += 1

            self.get_logger().debug(
                f"Inference complete: {goal.inference_id}, latency: {result.inference_latency_ms:.1f}ms"
            )
            return result

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}\n{traceback.format_exc()}")

            result = DispatchInfer.Result()
            result.action_chunk = VariantsList()
            result.chunk_size = 0
            result.success = False
            result.message = str(e)
            result.inference_latency_ms = 0.0

            goal_handle.abort()
            self._health_status = DiagnosticStatus.ERROR
            self._error_message = str(e)

            return result
