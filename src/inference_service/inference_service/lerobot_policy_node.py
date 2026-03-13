#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot policy inference node - Supports both Monolithic and Distributed modes.

This node provides ROS 2 integration for LeRobot policies with two execution modes:

    Mode A: Monolithic (default)
    ─────────────────────────────
    DispatchInfer Action Server → InferenceCoordinator (Pre → Infer → Post)
    
    All processing in one process for zero-copy tensor passing.

    Mode B: Distributed (cloud-edge)
    ────────────────────────────────
    Edge Node (this node):
        - Preprocessing (local CPU)
        - Publish to cloud via /preprocessed/batch
        - Await cloud result (async)
        - Postprocessing (local CPU)
        - Return to action_dispatch
    
    Cloud Node (pure_inference_node):
        - Subscribe /preprocessed/batch
        - GPU inference
        - Publish to /inference/action

ROS Interface Compatibility (MUST NOT CHANGE):
- Action: ibrobot_msgs/action/DispatchInfer
- Parameters: name, node_name, model_type, repo_id, checkpoint,
              contract_path, device, frequency, use_header_time,
              execution_mode, request_timeout, cloud_inference_topic,
              cloud_result_topic
"""

from __future__ import annotations

import threading
import time
import traceback
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
import rclpy.action
import torch
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from ibrobot_msgs.action import DispatchInfer
from ibrobot_msgs.msg import VariantsList
from robot_config.contract_utils import (
    load_contract,
    iter_specs,
    SpecView,
    feature_from_spec,
    zero_pad,
    qos_profile_from_dict,
    StreamBuffer,
    decode_value,
    stamp_from_header_ns,
)

from inference_service.core import (
    InferenceCoordinator,
    CoordinatorConfig,
    CoordinatorResult,
    resolve_device,
)
from inference_service.core.preprocessor import TensorPreprocessor
from inference_service.core.postprocessor import TensorPostprocessor
from tensormsg.converter import TensorMsgConverter


@dataclass
class _SubState:
    """Subscription state for a single observation stream."""
    spec: SpecView
    buf: StreamBuffer


@dataclass
class _NodeConfig:
    """Configuration parsed from ROS parameters."""
    name: str = "lerobot_policy"
    node_name: str = "lerobot_policy_node"
    model_type: str = "lerobot_policy"
    repo_id: Optional[str] = None
    checkpoint: Optional[str] = None
    contract_path: Optional[str] = None
    device: str = "auto"
    frequency: float = 10.0
    use_header_time: bool = True
    execution_mode: str = "monolithic"
    request_timeout: float = 5.0
    cloud_inference_topic: str = "/preprocessed/batch"
    cloud_result_topic: str = "/inference/action"


class LeRobotPolicyNode(Node):
    """
    LeRobot policy inference node with DispatchInfer Action Server.
    
    Supports two execution modes:
    
    1. Monolithic (default): All inference in one process (zero-copy)
    2. Distributed: Edge preprocessing → Cloud inference → Edge postprocessing
    
    The distributed mode is transparent to action_dispatch - it always sees
    the same Action Server interface.
    """
    
    def __init__(self, model_config: Dict):
        super().__init__(model_config.get("node_name", "lerobot_policy_node"))
        
        self._config = _NodeConfig(**{
            k: v for k, v in model_config.items()
            if k in _NodeConfig.__dataclass_fields__
        })
        
        self.get_logger().info(f"Initializing {self._config.name} node")
        self.get_logger().info(f"Execution mode: {self._config.execution_mode}")
        
        self._device = resolve_device(self._config.device)
        self.get_logger().info(f"Using device: {self._device}")
        
        self._last_inference_time: Optional[float] = None
        self._inference_count = 0
        self._health_status = DiagnosticStatus.OK
        self._error_message = ""
        
        self._contract = None
        self._obs_specs: List[SpecView] = []
        self._obs_zero: Dict[str, np.ndarray] = {}
        self._subs: Dict[str, _SubState] = {}
        self._state_specs: List[SpecView] = []
        
        if self._config.contract_path:
            self._load_contract(self._config.contract_path)
            self._setup_observation_subscriptions()
        
        if self._config.execution_mode == "distributed":
            self._setup_distributed_mode()
        else:
            self._setup_monolithic_mode()
        
        self._setup_publishers()
        
        self._setup_action_server()
        
        self._health_timer = self.create_timer(1.0, self._health_callback)
        
        mode_str = "distributed (edge proxy)" if self._config.execution_mode == "distributed" else "monolithic"
        self.get_logger().info(
            f"{self._config.name} node ready ({mode_str}): "
            f"policy_type={self._policy_type}, "
            f"chunk_size={self._chunk_size}"
        )
    
    def _load_contract(self, contract_path: str):
        """Load contract from YAML file."""
        p = Path(contract_path)
        if not p.exists():
            raise RuntimeError(f"Contract file not found: {contract_path}")
        
        self._contract = load_contract(contract_path)
        self._obs_specs = [s for s in iter_specs(self._contract) if not s.is_action]
        self._state_specs = [s for s in self._obs_specs if s.key == "observation.state"]
        
        self._topic_to_qos = {}
        for obs in self._contract.observations or []:
            self._topic_to_qos[obs.topic] = obs.qos
        
        self.get_logger().info(
            f"Loaded contract with {len(self._obs_specs)} observation specs"
        )
    
    def _setup_observation_subscriptions(self):
        """Setup observation subscriptions from loaded contract."""
        if not self._contract:
            self.get_logger().warn("No contract loaded, skipping observation subscriptions")
            return
        
        from rosidl_runtime_py.utilities import get_message
        
        for s in self._obs_specs:
            k, meta, _ = feature_from_spec(s, use_videos=False)
            
            if s.key == "observation.state" and len(self._state_specs) > 1:
                dict_key = f"{s.key}_{s.topic.replace('/', '_')}"
            else:
                dict_key = s.key
            
            self._obs_zero[dict_key] = zero_pad(meta)
            
            msg_cls = get_message(s.ros_type)
            qos_dict = self._topic_to_qos.get(s.topic, {})
            qos = qos_profile_from_dict(qos_dict) or QoSProfile(depth=10)
            
            self.create_subscription(
                msg_cls,
                s.topic,
                lambda m, sv=s: self._obs_cb(m, sv),
                qos,
                callback_group=ReentrantCallbackGroup(),
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
    
    def _setup_monolithic_mode(self):
        """Setup for monolithic (single-process) inference."""
        policy_path = self._config.repo_id or self._config.checkpoint
        if not policy_path:
            raise RuntimeError("LeRobotPolicyNode: 'repo_id' or 'checkpoint' is required")
        
        self._coordinator = InferenceCoordinator(
            policy_path=policy_path,
            device=str(self._device),
        )
        
        self._policy_type = self._coordinator.policy_type
        self._chunk_size = self._coordinator.chunk_size
        self._use_action_chunking = self._coordinator.use_action_chunking
        
        self._preprocessor = None
        self._postprocessor = None
        self._pending_requests: Dict[str, Any] = {}
        self._pub_batch = None
        self._sub_result = None
    
    def _setup_distributed_mode(self):
        """Setup for distributed (cloud-edge) inference."""
        policy_path = self._config.repo_id or self._config.checkpoint
        if not policy_path:
            raise RuntimeError("LeRobotPolicyNode: 'repo_id' or 'checkpoint' is required")
        
        self._preprocessor = TensorPreprocessor(
            policy_path=policy_path,
            device=self._device,
        )
        self._postprocessor = TensorPostprocessor(
            policy_path=policy_path,
            device=self._device,
        )
        
        self._coordinator = None
        
        temp_engine = InferenceCoordinator(
            policy_path=policy_path,
            device="cpu",
        )
        self._policy_type = temp_engine.policy_type
        self._chunk_size = temp_engine.chunk_size
        self._use_action_chunking = temp_engine.use_action_chunking
        
        self._pending_requests: Dict[str, Any] = {}
        
        self._pub_batch = self.create_publisher(
            VariantsList,
            self._config.cloud_inference_topic,
            10,
        )
        
        self._sub_result = self.create_subscription(
            VariantsList,
            self._config.cloud_result_topic,
            self._cloud_result_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        
        self.get_logger().info(
            f"Distributed mode: "
            f"publishing to {self._config.cloud_inference_topic}, "
            f"subscribed to {self._config.cloud_result_topic}"
        )
    
    def _setup_publishers(self):
        """Setup ROS publishers."""
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
    
    def _setup_action_server(self):
        """Setup DispatchInfer Action Server."""
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
    
    def _obs_cb(self, msg, spec: SpecView):
        """Observation callback - push to StreamBuffer."""
        use_header = (spec.stamp_src == "header") or self._config.use_header_time
        
        if use_header:
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
    
    def _sample_obs_frame(self, sample_t_ns: Optional[int] = None) -> Dict[str, Any]:
        """Sample observation frame at a given timestamp."""
        if sample_t_ns is None:
            sample_t_ns = self.get_clock().now().nanoseconds
        
        obs_frame: Dict[str, Any] = {}
        
        if len(self._state_specs) > 1:
            parts = []
            for sv in self._state_specs:
                key = f"{sv.key}_{sv.topic.replace('/', '_')}"
                v = self._subs[key].buf.sample(sample_t_ns) if key in self._subs else None
                parts.append(v if v is not None else self._obs_zero.get(key, np.zeros(1)))
            obs_frame["observation.state"] = np.concatenate(parts)
        
        for key, st in self._subs.items():
            if key.startswith("observation.state_") and len(self._state_specs) > 1:
                continue
            v = st.buf.sample(sample_t_ns)
            obs_frame[key] = v if v is not None else self._obs_zero.get(key, np.zeros(1))
        
        return obs_frame
    
    def _dispatch_infer_callback(self, goal_handle):
        """Execute inference requested by dispatcher."""
        goal = goal_handle.request
        obs_timestamp_ns = goal.obs_timestamp.sec * 10**9 + goal.obs_timestamp.nanosec

        try:
            obs_frame = self._sample_obs_frame(obs_timestamp_ns)

            if self._config.execution_mode == "distributed":
                result = self._execute_distributed(obs_frame, goal.inference_id)
            else:
                result = self._execute_monolithic(obs_frame)

            action_msg = self._create_action_msg(result.action)
            self._action_pub.publish(action_msg)
            
            response = DispatchInfer.Result()
            response.action_chunk = action_msg
            response.chunk_size = result.chunk_size
            response.success = True
            response.message = "OK"
            response.inference_latency_ms = result.total_latency_ms
            
            goal_handle.succeed()
            self._last_inference_time = time.time()
            self._inference_count += 1
            
            self.get_logger().debug(
                f"Inference complete: {goal.inference_id}, "
                f"latency: {result.total_latency_ms:.1f}ms "
                f"(pre: {result.preprocess_latency_ms:.1f}ms, "
                f"inf: {result.inference_latency_ms:.1f}ms, "
                f"post: {result.postprocess_latency_ms:.1f}ms)"
            )
            
            return response
            
        except TimeoutError:
            self.get_logger().error(f"Inference timeout for request {goal.inference_id}")
            
            response = DispatchInfer.Result()
            response.action_chunk = VariantsList()
            response.chunk_size = 0
            response.success = False
            response.message = "Inference timeout - cloud did not respond"
            response.inference_latency_ms = self._config.request_timeout * 1000.0
            
            goal_handle.abort()
            self._health_status = DiagnosticStatus.WARN
            self._error_message = "Cloud inference timeout"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}\n{traceback.format_exc()}")
            
            response = DispatchInfer.Result()
            response.action_chunk = VariantsList()
            response.chunk_size = 0
            response.success = False
            response.message = str(e)
            response.inference_latency_ms = 0.0
            
            goal_handle.abort()
            self._health_status = DiagnosticStatus.ERROR
            self._error_message = str(e)
            
            return response
    
    def _execute_monolithic(self, obs_frame: Dict[str, Any]) -> CoordinatorResult:
        """Execute inference in monolithic mode (zero-copy)."""
        return self._coordinator(obs_frame)
    
    def _execute_distributed(
        self,
        obs_frame: Dict[str, Any],
        inference_id: str,
    ) -> CoordinatorResult:
        """
        Execute inference in distributed mode.
        
        Flow:
        1. Preprocess locally (edge CPU)
        2. Publish to cloud with request_id
        3. Block thread and wait for cloud result (with timeout)
        4. Postprocess locally (edge CPU)
        """
        total_start = time.perf_counter()
        
        preprocess_start = time.perf_counter()
        batch = self._preprocessor(obs_frame)
        preprocess_latency = (time.perf_counter() - preprocess_start) * 1000.0
        
        request_id = str(uuid.uuid4())
        batch["task.request_id"] = [request_id]
        
        msg = TensorMsgConverter.to_variant(batch)
        
        event = threading.Event()
        self._pending_requests[request_id] = [event, None]
        
        self._pub_batch.publish(msg)
        self.get_logger().debug(f"Published batch to cloud, request_id={request_id}")
        
        success = event.wait(timeout=self._config.request_timeout)
        
        if not success:
            self._pending_requests.pop(request_id, None)
            raise TimeoutError(f"Inference timeout for request {request_id}")
            
        req_data = self._pending_requests.pop(request_id, None)
        if not req_data or req_data[1] is None:
            raise RuntimeError(f"Event set but no cloud result found for {request_id}")
            
        cloud_result = req_data[1]
        
        inference_latency = cloud_result.get("_latency_ms", 0.0)
        
        postprocess_start = time.perf_counter()
        action = self._postprocessor(cloud_result["action"])
        postprocess_latency = (time.perf_counter() - postprocess_start) * 1000.0
        
        total_latency = (time.perf_counter() - total_start) * 1000.0
        
        return CoordinatorResult(
            action=action,
            chunk_size=self._chunk_size,
            total_latency_ms=total_latency,
            preprocess_latency_ms=preprocess_latency,
            inference_latency_ms=inference_latency,
            postprocess_latency_ms=postprocess_latency,
            policy_type=self._policy_type,
        )
    
    def _cloud_result_callback(self, msg: VariantsList):
        """
        Callback for cloud inference results.
        
        Matches the result to the pending request using action.request_id
        and completes the corresponding Event.
        """
        try:
            batch = TensorMsgConverter.from_variant(msg, self._device)
            
            req_list = batch.pop("action.request_id", None)
            request_id = req_list[0] if req_list and isinstance(req_list, list) else None
            
            if request_id is None:
                self.get_logger().warn("Received cloud result without action.request_id")
                return
            
            if request_id in self._pending_requests:
                req = self._pending_requests[request_id]
                req[1] = batch
                req[0].set()
                self.get_logger().debug(f"Cloud result received for request_id={request_id}")
            else:
                self.get_logger().warn(
                    f"No pending request found for request_id={request_id}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Error processing cloud result: {e}")
    
    def _create_action_msg(self, action: torch.Tensor) -> VariantsList:
        """Create VariantsList message from action tensor."""
        from ibrobot_msgs.msg import Variant
        from std_msgs.msg import MultiArrayDimension
        
        if torch.is_tensor(action):
            action = action.detach().cpu().numpy()
        
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
                1 if i == len(action.shape) - 1 else int(np.prod(action.shape[i + 1:]))
            )
            array_msg.layout.dim.append(dim_msg)
        
        array_msg.data = action.flatten().tolist()
        msg.variants.append(variant)
        return msg
    
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
        health_msg.message = self._error_message or f"{self._config.name} operating normally"
        health_msg.hardware_id = self._config.node_name
        health_msg.values = [
            KeyValue(key="inference_count", value=str(self._inference_count)),
            KeyValue(key="model_type", value=self._config.model_type),
            KeyValue(key="policy_type", value=self._policy_type),
            KeyValue(key="chunk_size", value=str(self._chunk_size)),
            KeyValue(key="execution_mode", value=self._config.execution_mode),
        ]
        
        self._health_pub.publish(health_msg)


def main() -> None:
    """Main entry point for LeRobot policy node."""
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
            "execution_mode",
            "request_timeout",
            "cloud_inference_topic",
            "cloud_result_topic",
        ]:
            if p == "execution_mode":
                default = "monolithic"
            elif p == "request_timeout":
                default = 5.0
            elif p == "cloud_inference_topic":
                default = "/preprocessed/batch"
            elif p == "cloud_result_topic":
                default = "/inference/action"
            elif p in ["repo_id", "checkpoint", "contract_path"]:
                default = ""
            elif p == "device":
                default = "auto"
            elif p == "frequency":
                default = 10.0
            elif p == "use_header_time":
                default = True
            elif p == "node_name":
                default = "lerobot_policy_node"
            else:
                default = "lerobot_policy"
            temp_node.declare_parameter(p, default)
        
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
        config["execution_mode"] = temp_node.get_parameter("execution_mode").value
        config["request_timeout"] = temp_node.get_parameter("request_timeout").value
        config["cloud_inference_topic"] = temp_node.get_parameter("cloud_inference_topic").value
        config["cloud_result_topic"] = temp_node.get_parameter("cloud_result_topic").value
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
