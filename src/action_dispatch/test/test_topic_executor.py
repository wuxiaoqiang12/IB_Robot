"""
Unit tests for TopicExecutor.

Tests the topic-based action executor for high-frequency position control.
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from action_dispatch.base_executor import ExecutorType, RobotStatus
from action_dispatch.topic_executor import TopicExecutor


class TestTopicExecutor(unittest.TestCase):
    """Test suite for TopicExecutor."""

    def setUp(self):
        """Set up test fixtures."""
        # Create mock node
        self.mock_node = Mock(spec=Node)
        self.mock_logger = Mock()
        self.mock_node.get_logger.return_value = self.mock_logger

        # Create mock publisher
        self.mock_publisher = Mock()

        # Default configuration
        self.config = {
            'action_specs': [],
            'hold_last_action': True,
        }

    def test_initialization_basic(self):
        """Test basic initialization without action_specs."""
        executor = TopicExecutor(self.mock_node, self.config)

        self.assertEqual(executor.get_executor_type(), ExecutorType.TOPIC)
        self.assertTrue(executor.is_healthy())
        self.assertFalse(executor.is_busy())

    def test_initialization_with_action_specs(self):
        """Test initialization with action_specs."""
        # Create action spec
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]

        # Mock create_publisher
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        success = executor.initialize()

        self.assertTrue(success)
        self.assertTrue(executor.is_healthy())

        # Verify publisher was created
        self.mock_node.create_publisher.assert_called_once()
        call_args = self.mock_node.create_publisher.call_args
        self.assertEqual(call_args[0][0], Float64MultiArray)
        self.assertEqual(call_args[0][1], "/test/command")

    def test_execute_float64_multi_array(self):
        """Test executing action with Float64MultiArray."""
        # Setup
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute
        action = np.array([1.0, 2.0, 3.0])
        success = executor.execute(action)

        self.assertTrue(success)
        self.assertTrue(executor.is_busy())

        # Verify publish was called
        self.mock_publisher.publish.assert_called_once()
        published_msg = self.mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, Float64MultiArray)
        self.assertEqual(list(published_msg.data), [1.0, 2.0, 3.0])

    def test_execute_joint_trajectory(self):
        """Test executing action with JointTrajectory."""
        # Setup
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/trajectory"
        action_spec.ros_type = "trajectory_msgs/msg/JointTrajectory"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute
        action = np.array([0.5, 1.0, 1.5])
        success = executor.execute(action)

        self.assertTrue(success)
        self.assertTrue(executor.is_busy())

        # Verify publish was called
        self.mock_publisher.publish.assert_called_once()
        published_msg = self.mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, JointTrajectory)
        self.assertEqual(len(published_msg.points), 1)
        self.assertEqual(list(published_msg.points[0].positions), [0.5, 1.0, 1.5])

    def test_execute_with_slicing(self):
        """Test action slicing based on spec.names."""
        # Setup
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = [2, 5]  # Slice indices [2:5]

        self.config['action_specs'] = [action_spec]
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute full action
        full_action = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
        success = executor.execute(full_action)

        self.assertTrue(success)

        # Verify sliced action was published
        published_msg = self.mock_publisher.publish.call_args[0][0]
        self.assertEqual(list(published_msg.data), [2.0, 3.0, 4.0])

    def test_execute_without_initialization(self):
        """Test that execute fails without initialization."""
        executor = TopicExecutor(self.mock_node, self.config)

        action = np.array([1.0, 2.0, 3.0])
        success = executor.execute(action)

        self.assertFalse(success)

    def test_hold_last_action(self):
        """Test holding last action."""
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]
        self.config['hold_last_action'] = True
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute first action
        action1 = np.array([1.0, 2.0, 3.0])
        executor.execute(action1)

        # Execute second action
        action2 = np.array([4.0, 5.0, 6.0])
        executor.execute(action2)

        # Get last action
        last_action = executor.get_last_action()
        self.assertIsNotNone(last_action)
        np.testing.assert_array_equal(last_action, action2)

    def test_cancel(self):
        """Test cancellation."""
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute action
        action = np.array([1.0, 2.0, 3.0])
        executor.execute(action)
        self.assertTrue(executor.is_busy())

        # Cancel
        success = executor.cancel()
        self.assertTrue(success)
        self.assertFalse(executor.is_busy())

    def test_get_status(self):
        """Test status reporting."""
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Get initial status
        status = executor.get_status()
        self.assertIsInstance(status, RobotStatus)
        self.assertEqual(status.executor_type, ExecutorType.TOPIC)
        self.assertTrue(status.is_healthy)
        self.assertFalse(status.is_moving)

        # Execute action
        action = np.array([1.0, 2.0, 3.0])
        executor.execute(action)

        # Get status after execution
        status = executor.get_status()
        self.assertTrue(status.is_moving)

    def test_cleanup(self):
        """Test cleanup destroys publishers."""
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Cleanup
        executor.cleanup()

        # Verify publisher was destroyed
        self.mock_node.destroy_publisher.assert_called_once_with(self.mock_publisher)

    def test_backward_compatibility_no_action_specs(self):
        """Test backward compatibility when no action_specs provided."""
        # No action_specs in config
        self.config['action_specs'] = []

        executor = TopicExecutor(self.mock_node, self.config)
        success = executor.initialize()

        # Should create default publisher
        self.assertTrue(success)
        self.mock_node.create_publisher.assert_called_once()

        # Verify default topic
        call_args = self.mock_node.create_publisher.call_args
        self.assertEqual(call_args[0][1], "/position_controller/command")


class TestTopicExecutorEdgeCases(unittest.TestCase):
    """Test edge cases and error handling."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = Mock(spec=Node)
        self.mock_logger = Mock()
        self.mock_node.get_logger.return_value = self.mock_logger
        self.mock_publisher = Mock()
        self.config = {'action_specs': []}

    def test_execute_empty_action(self):
        """Test executing empty action array."""
        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        action = np.array([])
        success = executor.execute(action)

        # Should handle gracefully
        self.assertTrue(success)

    def test_unsupported_message_type(self):
        """Test handling unsupported message type."""
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "unsupported/MessageType"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]

        executor = TopicExecutor(self.mock_node, self.config)
        success = executor.initialize()

        # Should initialize successfully but skip unsupported type
        self.assertTrue(success)
        self.mock_logger.warn.assert_called()

    def test_multiple_publishers(self):
        """Test creating multiple publishers for different action specs."""
        # Create multiple action specs
        spec1 = Mock()
        spec1.key = "action1"
        spec1.topic = "/topic1"
        spec1.ros_type = "std_msgs/msg/Float64MultiArray"
        spec1.names = None

        spec2 = Mock()
        spec2.key = "action2"
        spec2.topic = "/topic2"
        spec2.ros_type = "trajectory_msgs/msg/JointTrajectory"
        spec2.names = None

        self.config['action_specs'] = [spec1, spec2]
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        success = executor.initialize()

        self.assertTrue(success)
        # Should create 2 publishers
        self.assertEqual(self.mock_node.create_publisher.call_count, 2)

    def test_execution_error_handling(self):
        """Test error handling during execution."""
        action_spec = Mock()
        action_spec.key = "test_action"
        action_spec.topic = "/test/command"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]

        # Make publisher raise exception
        self.mock_publisher.publish.side_effect = Exception("Publish failed")
        self.mock_node.create_publisher.return_value = self.mock_publisher

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        action = np.array([1.0, 2.0, 3.0])
        success = executor.execute(action)

        # Should handle error gracefully
        self.assertFalse(success)

        status = executor.get_status()
        self.assertIsNotNone(status.error_message)
        self.assertEqual(status.error_code, 1)


class TestTopicExecutorHighFrequency(unittest.TestCase):
    """Test high-frequency position command execution (Task 9.6)."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = Mock(spec=Node)
        self.mock_logger = Mock()
        self.mock_node.get_logger.return_value = self.mock_logger
        self.mock_publisher = Mock()

        action_spec = Mock()
        action_spec.key = "arm_action"
        action_spec.topic = "/arm_position_controller/commands"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config = {'action_specs': [action_spec]}
        self.mock_node.create_publisher.return_value = self.mock_publisher

    def test_single_command_latency(self):
        """Test single command execution latency (<1ms overhead)."""
        import time

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute single command
        action = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        start_time = time.time()
        success = executor.execute(action)
        elapsed_ms = (time.time() - start_time) * 1000

        self.assertTrue(success)
        # Verify publish was called
        self.mock_publisher.publish.assert_called_once()

        # Log latency for information (should be <1ms)
        print(f"\nSingle command latency: {elapsed_ms:.3f}ms")
        self.assertLess(elapsed_ms, 10.0, "Single command should execute in <10ms")

    def test_50hz_continuous_streaming(self):
        """Test continuous 50Hz position streaming (simulating ACT model output)."""
        import time

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Simulate 50Hz streaming for 1 second (50 commands)
        num_commands = 50
        target_period_ms = 20.0  # 1000ms / 50Hz = 20ms per command

        start_time = time.time()
        successful_commands = 0
        latencies = []

        for i in range(num_commands):
            cmd_start = time.time()

            # Generate action (simulating ACT model output)
            action = np.array([1.0 + i*0.01, 2.0 + i*0.01, 3.0 + i*0.01, 4.0 + i*0.01, 5.0 + i*0.01])
            success = executor.execute(action)

            cmd_elapsed_ms = (time.time() - cmd_start) * 1000
            latencies.append(cmd_elapsed_ms)

            if success:
                successful_commands += 1

            # Maintain 50Hz rate (sleep to achieve target period)
            elapsed_ms = (time.time() - cmd_start) * 1000
            if elapsed_ms < target_period_ms:
                time.sleep((target_period_ms - elapsed_ms) / 1000.0)

        total_time_ms = (time.time() - start_time) * 1000
        avg_latency_ms = sum(latencies) / len(latencies)
        success_rate = successful_commands / num_commands * 100.0

        # Verify all commands were successful
        self.assertEqual(successful_commands, num_commands, "All commands should succeed")
        self.assertEqual(self.mock_publisher.publish.call_count, num_commands)

        # Log performance metrics
        print(f"\n50Hz Streaming Test Results:")
        print(f"  Total commands: {num_commands}")
        print(f"  Successful: {successful_commands} ({success_rate:.1f}%)")
        print(f"  Total time: {total_time_ms:.1f}ms (target: {num_commands * target_period_ms:.1f}ms)")
        print(f"  Average latency: {avg_latency_ms:.3f}ms")
        print(f"  Max latency: {max(latencies):.3f}ms")
        print(f"  Min latency: {min(latencies):.3f}ms")

        # Verify performance requirements
        self.assertGreater(success_rate, 99.0, "Success rate should be >99%")
        self.assertLess(avg_latency_ms, 5.0, "Average latency should be <5ms")

    def test_100hz_continuous_streaming(self):
        """Test continuous 100Hz position streaming (stress test)."""
        import time

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Simulate 100Hz streaming for 0.5 second (50 commands)
        num_commands = 50
        target_period_ms = 10.0  # 1000ms / 100Hz = 10ms per command

        start_time = time.time()
        successful_commands = 0
        latencies = []

        for i in range(num_commands):
            cmd_start = time.time()

            # Generate action
            action = np.array([1.0 + i*0.005, 2.0 + i*0.005, 3.0 + i*0.005, 4.0 + i*0.005, 5.0 + i*0.005])
            success = executor.execute(action)

            cmd_elapsed_ms = (time.time() - cmd_start) * 1000
            latencies.append(cmd_elapsed_ms)

            if success:
                successful_commands += 1

            # Maintain 100Hz rate
            elapsed_ms = (time.time() - cmd_start) * 1000
            if elapsed_ms < target_period_ms:
                time.sleep((target_period_ms - elapsed_ms) / 1000.0)

        total_time_ms = (time.time() - start_time) * 1000
        avg_latency_ms = sum(latencies) / len(latencies)
        success_rate = successful_commands / num_commands * 100.0

        # Verify all commands were successful
        self.assertEqual(successful_commands, num_commands)
        self.assertEqual(self.mock_publisher.publish.call_count, num_commands)

        # Log performance metrics
        print(f"\n100Hz Streaming Test Results:")
        print(f"  Total commands: {num_commands}")
        print(f"  Successful: {successful_commands} ({success_rate:.1f}%)")
        print(f"  Total time: {total_time_ms:.1f}ms (target: {num_commands * target_period_ms:.1f}ms)")
        print(f"  Average latency: {avg_latency_ms:.3f}ms")

        # Verify performance requirements (more relaxed for 100Hz)
        self.assertGreater(success_rate, 95.0, "Success rate should be >95% at 100Hz")
        self.assertLess(avg_latency_ms, 8.0, "Average latency should be <8ms at 100Hz")

    def test_burst_commands(self):
        """Test burst of commands without rate limiting."""
        import time

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Send burst of 100 commands as fast as possible
        num_commands = 100

        start_time = time.time()
        successful_commands = 0

        for i in range(num_commands):
            action = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
            success = executor.execute(action)
            if success:
                successful_commands += 1

        total_time_ms = (time.time() - start_time) * 1000
        avg_time_per_cmd = total_time_ms / num_commands

        # Verify all commands successful
        self.assertEqual(successful_commands, num_commands)
        self.assertEqual(self.mock_publisher.publish.call_count, num_commands)

        # Log performance
        print(f"\nBurst Commands Test Results:")
        print(f"  Total commands: {num_commands}")
        print(f"  Successful: {successful_commands}")
        print(f"  Total time: {total_time_ms:.1f}ms")
        print(f"  Average time per command: {avg_time_per_cmd:.3f}ms")
        print(f"  Equivalent frequency: {1000.0/avg_time_per_cmd:.1f}Hz")

        # Verify no performance degradation
        self.assertLess(avg_time_per_cmd, 1.0, "Burst commands should average <1ms each")


class TestTopicExecutorQueueManagement(unittest.TestCase):
    """Test queue management and interpolation functionality (Task 9.7)."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_node = Mock(spec=Node)
        self.mock_logger = Mock()
        self.mock_node.get_logger.return_value = self.mock_logger
        self.mock_publisher = Mock()

        action_spec = Mock()
        action_spec.key = "arm_action"
        action_spec.topic = "/arm_position_controller/commands"
        action_spec.ros_type = "std_msgs/msg/Float64MultiArray"
        action_spec.names = None

        self.config = {'action_specs': [action_spec]}
        self.mock_node.create_publisher.return_value = self.mock_publisher

    def test_queue_size_tracking_via_metadata(self):
        """Test that queue size is tracked via metadata."""
        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute with queue size metadata
        action = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        metadata = {'queue_size': 10}
        success = executor.execute(action, metadata)

        self.assertTrue(success)

        # Verify status includes queue size
        status = executor.get_status()
        self.assertEqual(status.queue_size, 10)

    def test_hold_last_action_on_queue_empty(self):
        """Test holding last action when queue becomes empty."""
        executor = TopicExecutor(self.mock_node, self.config)
        executor.hold_last_action = True
        executor.initialize()

        # Execute action
        action1 = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        executor.execute(action1)

        # Execute another action
        action2 = np.array([2.0, 3.0, 4.0, 5.0, 6.0])
        executor.execute(action2)

        # Get last action
        last_action = executor.get_last_action()
        self.assertIsNotNone(last_action)
        np.testing.assert_array_equal(last_action, action2)

        # Verify last action is held even after multiple publishes
        self.assertEqual(self.mock_publisher.publish.call_count, 2)

    def test_interpolation_via_joint_trajectory(self):
        """Test that JointTrajectory messages support interpolation."""
        # Configure for JointTrajectory
        action_spec = Mock()
        action_spec.key = "arm_trajectory"
        action_spec.topic = "/arm_trajectory_controller/joint_trajectory"
        action_spec.ros_type = "trajectory_msgs/msg/JointTrajectory"
        action_spec.names = None

        self.config['action_specs'] = [action_spec]
        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Execute action
        action = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        success = executor.execute(action)

        self.assertTrue(success)

        # Verify JointTrajectory was published with time_from_start
        published_msg = self.mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, JointTrajectory)
        self.assertEqual(len(published_msg.points), 1)

        # Verify time_from_start is set (enables interpolation by controller)
        point = published_msg.points[0]
        self.assertGreater(point.time_from_start.nanosec, 0)

    def test_queue_watermark_monitoring(self):
        """Test queue watermark monitoring via status."""
        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Simulate queue filling up
        for i in range(10):
            action = np.array([1.0 + i*0.1, 2.0 + i*0.1, 3.0 + i*0.1, 4.0 + i*0.1, 5.0 + i*0.1])
            metadata = {'queue_size': 10 - i}  # Simulate decreasing queue
            executor.execute(action, metadata)

            status = executor.get_status()
            self.assertEqual(status.queue_size, 10 - i)
            self.assertTrue(status.is_moving)

        # Verify all commands were published
        self.assertEqual(self.mock_publisher.publish.call_count, 10)

    def test_action_chunking_simulation(self):
        """Test action chunking (receiving chunks and managing execution)."""
        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Simulate receiving action chunk (e.g., 16 points from ACT model)
        chunk_size = 16
        actions_chunk = [np.array([1.0 + i*0.1, 2.0 + i*0.1, 3.0 + i*0.1, 4.0 + i*0.1, 5.0 + i*0.1])
                        for i in range(chunk_size)]

        # Execute each action in chunk
        for idx, action in enumerate(actions_chunk):
            metadata = {'queue_size': chunk_size - idx - 1}
            success = executor.execute(action, metadata)
            self.assertTrue(success)

        # Verify all actions in chunk were published
        self.assertEqual(self.mock_publisher.publish.call_count, chunk_size)

        # Verify last action is held
        last_action = executor.get_last_action()
        np.testing.assert_array_equal(last_action, actions_chunk[-1])

    def test_continuous_streaming_with_queue_feedback(self):
        """Test continuous streaming with queue feedback (simulating real ACT deployment)."""
        import time

        executor = TopicExecutor(self.mock_node, self.config)
        executor.initialize()

        # Simulate continuous streaming with queue feedback
        num_chunks = 5
        chunk_size = 16
        commands_per_chunk = 16

        for chunk_idx in range(num_chunks):
            # Receive new chunk when queue is low
            for cmd_idx in range(commands_per_chunk):
                action = np.array([1.0 + cmd_idx*0.1, 2.0 + cmd_idx*0.1, 3.0 + cmd_idx*0.1,
                                 4.0 + cmd_idx*0.1, 5.0 + cmd_idx*0.1])
                queue_remaining = chunk_size - cmd_idx - 1
                metadata = {'queue_size': queue_remaining}
                success = executor.execute(action, metadata)

                self.assertTrue(success)

                # Simulate 50Hz rate
                time.sleep(0.02)

        total_commands = num_chunks * commands_per_chunk
        self.assertEqual(self.mock_publisher.publish.call_count, total_commands)

        # Log test completion
        print(f"\nContinuous Streaming with Queue Feedback:")
        print(f"  Total chunks: {num_chunks}")
        print(f"  Commands per chunk: {commands_per_chunk}")
        print(f"  Total commands: {total_commands}")


if __name__ == '__main__':
    unittest.main()
