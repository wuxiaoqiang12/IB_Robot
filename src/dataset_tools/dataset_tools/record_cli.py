#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Command Line Interface for triggering episodic recordings.
Sends Action goals to the EpisodeRecorderServer.
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

# Import the global interface
from ibrobot_msgs.action import RecordEpisode


class RecordCLI(Node):
    def __init__(self):
        super().__init__('record_cli')
        
        # Action client to start recording
        self._action_client = ActionClient(self, RecordEpisode, 'record_episode')
        
        # Service client to stop recording early
        self._cancel_client = self.create_client(Trigger, 'record_episode/cancel')
        
        self.get_logger().info("Record CLI started. Waiting for Action Server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to Episode Recorder Server!")

    def send_goal(self, prompt_text: str):
        goal_msg = RecordEpisode.Goal()
        goal_msg.prompt = prompt_text

        self.get_logger().info(f"Sending goal with prompt: '{prompt_text}'")
        
        # We don't block here so the user can cancel it
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected by server (Is it already recording?)")
            return

        self.get_logger().info("🔴 RECORDING STARTED. (Press Enter to stop early)")
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"✅ RECORDING SAVED: {result.message}")
        else:
            self.get_logger().error(f"❌ RECORDING FAILED/CANCELLED: {result.message}")
            
        print("\n----------------------------------------")
        print("Ready for next episode.")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Optional: Print progress on the same line
        sys.stdout.write(f"\r[Time Left: {feedback.seconds_remaining}s] {feedback.feedback_message}   ")
        sys.stdout.flush()

    def cancel_recording(self):
        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Cancel service not available")
            return
            
        req = Trigger.Request()
        future = self._cancel_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Stop signal sent successfully.")
        else:
            self.get_logger().error("Failed to send stop signal.")


def cli_loop(node):
    """Run the interactive prompt in a separate thread."""
    last_prompt = "default_task"
    
    while rclpy.ok():
        print("\n========================================")
        print("Dataset Collection CLI")
        print(f"Enter prompt text to start recording. (Press Enter to reuse: '{last_prompt}')")
        print("Type 'q' or 'quit' to exit.")
        print("========================================")
        
        try:
            prompt = input("Prompt > ")
            if prompt.strip().lower() in ['q', 'quit']:
                print("Exiting...")
                rclpy.shutdown()
                break
                
            if not prompt.strip():
                prompt = last_prompt
            else:
                last_prompt = prompt.strip()
                
            # Send the start command
            node.send_goal(prompt)
            
            # Wait for user to press Enter to stop
            input() 
            
            # Send cancel command
            node.cancel_recording()
            
        except EOFError:
            break
        except Exception as e:
            print(f"CLI Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RecordCLI()
    
    # Run ROS spinning in a background thread so input() doesn't block callbacks
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        # Run the interactive CLI in the main thread
        cli_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
