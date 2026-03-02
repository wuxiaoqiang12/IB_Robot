#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal Action Dispatcher Node.

Maintains a queue of actions and triggers inference when low.
Publishes actions to ros2_control via TopicExecutor at a fixed frequency.
"""

import time
import collections
from pathlib import Path
from typing import Optional, Dict, Any, List

import numpy as np
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from rosetta_interfaces.action import DispatchInfer
from rosetta_interfaces.msg import VariantsList
from rosetta.common.contract_utils import load_contract, iter_specs
from rosetta.common.decoders import dec_variant_list

from .topic_executor import TopicExecutor


class ActionDispatcherNode(Node):
    """
    Simplified action dispatcher.
    - Queue: collections.deque
    - Trigger: Simple watermark check
    - Execution: TopicExecutor (100Hz streaming)
    """

    def __init__(self):
        super().__init__('action_dispatcher')
        self.get_logger().info("Initializing Minimal Action Dispatcher")

        # 1. Parameters
        self.declare_parameter('queue_size', 100)
        self.declare_parameter('watermark_threshold', 20)
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('inference_action_server', '/act_inference_node/DispatchInfer')
        self.declare_parameter('contract_path', '')
        self.declare_parameter('joint_state_topic', '/joint_states')

        self._queue_limit = self.get_parameter('queue_size').value
        self._watermark = self.get_parameter('watermark_threshold').value
        self._control_hz = self.get_parameter('control_frequency').value
        self._server_name = self.get_parameter('inference_action_server').value

        # 2. State & Queue
        self._queue = collections.deque(maxlen=self._queue_limit)
        self._last_action: Optional[np.ndarray] = None
        self._inference_in_progress = False
        self._is_running = True

        # 3. Load Contract (Essential for TopicExecutor mapping)
        contract_path = self.get_parameter('contract_path').value
        self._action_specs = []
        if contract_path:
            try:
                self._contract = load_contract(Path(contract_path))
                self._action_specs = [s for s in iter_specs(self._contract) if s.is_action]
                self.get_logger().info(f"Loaded {len(self._action_specs)} action specs from contract")
            except Exception as e:
                self.get_logger().error(f"Failed to load contract from {contract_path}: {e}")
        else:
            self.get_logger().warn("No contract_path provided! TopicExecutor will use defaults.")

        # 4. Executor (Topic-based)
        self._executor = TopicExecutor(self, {'action_specs': self._action_specs})
        if not self._executor.initialize():
            raise RuntimeError("Failed to initialize TopicExecutor")

        # 5. Communication
        self._infer_client = rclpy.action.ActionClient(self, DispatchInfer, self._server_name)
        
        # Subscriptions
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joint_sub = self.create_subscription(JointState, self.get_parameter('joint_state_topic').value, self._joint_cb, qos)
        
        # Public Queue Size
        self._queue_size_pub = self.create_publisher(Int32, '~/queue_size', 10)

        # 6. Timers
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(1.0 / self._control_hz, self._control_loop, callback_group=self._cb_group)
        
        # Service
        self._reset_srv = self.create_service(Empty, '~/reset', self._reset_cb)

        self.get_logger().info(f"Dispatcher ready. Hz: {self._control_hz}, Watermark: {self._watermark}")

    def _joint_cb(self, msg):
        """Optional: could use current state for safety or initialization."""
        pass

    def _control_loop(self):
        if not self._is_running:
            return

        q_size = len(self._queue)
        self._queue_size_pub.publish(Int32(data=q_size))

        # A. Trigger Inference if queue is low
        if q_size < self._watermark and not self._inference_in_progress:
            self._request_inference()

        # B. Get Action
        action = None
        if q_size > 0:
            action = self._queue.popleft()
            self._last_action = action
        elif self._last_action is not None:
            # Hold last action if queue empty
            action = self._last_action
        
        # C. Execute
        if action is not None:
            self._executor.execute(action)

    def _request_inference(self):
        """Send async goal to inference service."""
        if not self._infer_client.wait_for_server(timeout_sec=0.1):
            return

        self._inference_in_progress = True
        goal = DispatchInfer.Goal()
        goal.obs_timestamp = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f"Requesting inference @ {goal.obs_timestamp.sec}")
        
        send_goal_future = self._infer_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Inference goal REJECTED")
            self._inference_in_progress = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._inference_in_progress = False
        result = future.result().result
        
        if not result.success:
            self.get_logger().error(f"Inference FAILED: {result.message}")
            return

        # Decode VariantsList to Numpy
        # Assuming result.action_chunk is VariantsList containing 'action' key
        batch = dec_variant_list(result.action_chunk)
        if 'action' in batch:
            action_chunk = batch['action'] # Could be Tensor or Numpy
            
            # 1. Convert to Numpy if it's a Torch Tensor
            if hasattr(action_chunk, 'detach'):
                action_chunk = action_chunk.detach().cpu().numpy()
            
            # 2. Reshape to (N, action_dim)
            if action_chunk.ndim == 1:
                action_chunk = action_chunk.reshape(1, -1)
            
            self._queue.extend(action_chunk)
            self.get_logger().info(f"Queued {len(action_chunk)} new actions. Total: {len(self._queue)}")

    def _reset_cb(self, request, response):
        self.get_logger().info("Resetting queue")
        self._queue.clear()
        self._inference_in_progress = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ActionDispatcherNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
