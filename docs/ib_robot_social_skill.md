# IB_Robot General Control

## When to Use

Use this skill when the user wants to interact with, control, or get the status of the **IB_Robot** (SO-101 robotic arm). 
- "Move the arm to a safe position"
- "Open the gripper"
- "What do the cameras see?"
- "Run the AI inference task"

## System Architecture & Data Formats

### 1. Joint States & Control (`/joint_states` and Controller Commands)
- The SO-101 arm has **6 joints**:
  - `arm` (5 joints): Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll.
  - `gripper` (1 joint): Jaw.
- **IMPORTANT UNIT CONVENTION:** 
  - All joint positions are in **RADIANS** (for the arm) or normalized absolute values `0.0 to 1.0` (for the gripper). 
  - **Do NOT send degrees.** If the user asks for "90 degrees", you MUST convert it to `1.5708` radians before publishing.
- **Topics**:
  - To read current position: `ros2_subscribe_once` on `/joint_states` (Type: `sensor_msgs/msg/JointState`).
  - To send arm commands: `ros2_publish` to `/arm_position_controller/commands` (Type: `std_msgs/msg/Float64MultiArray`, requires array of 5 floats in radians).
  - To send gripper commands: `ros2_publish` to `/gripper_position_controller/commands` (Type: `std_msgs/msg/Float64MultiArray`, requires array of 1 float, `0.0` is closed, `1.0` is open).

### 2. Vision / Cameras
- The robot has three cameras:
  - Top: `/camera/top/image_raw`
  - Wrist: `/camera/wrist/image_raw`
  - Front: `/camera/front/image_raw` (or `/world/demo/.../image` in simulation)
- To take a picture and analyze it, use the tool `ros2_camera_snapshot` and specify the topic.

### 3. AI Inference (End-to-End Control)
- To execute a complex task (like "pick up the cup"), the robot relies on a VLA (Vision-Language-Action) model via an Action Server.
- Use `ros2_action_goal` on the action `/act_inference_node/DispatchInfer` (Type: `ibrobot_msgs/action/DispatchInfer`).
  - Set `prompt` to the user's natural language instruction (e.g., "pick up the red block").
  - Set `max_steps` if requested (default is usually 0 for continuous).

## Examples

### Example 1: Move arm to home position (straight up)
```
Tool: ros2_publish
Topic: /arm_position_controller/commands
Type: std_msgs/msg/Float64MultiArray
Payload: {"data": [0.0, 0.0, 0.0, 0.0, 0.0]}
```

### Example 2: Open the gripper
```
Tool: ros2_publish
Topic: /gripper_position_controller/commands
Type: std_msgs/msg/Float64MultiArray
Payload: {"data": [1.0]}
```

### Example 3: Ask the robot to perform an AI task
```
Tool: ros2_action_goal
Action: /act_inference_node/DispatchInfer
Type: ibrobot_msgs/action/DispatchInfer
Payload: {"prompt": "grab the pan handle"}
```

## Tips
- Always check the current `/joint_states` before moving to avoid sudden jumps.
- If the user asks for angles, reply stating that you are converting their requested degrees into radians for the robot's hardware interface.
