# inference_service

# Lazy import to avoid torch dependency when not needed
# Users should import specific modules directly:
# - from inference_service.simple_mock_inference import SimpleMockInferenceNode
# - from inference_service.base_model_node import BaseInferenceNode (requires torch)

__all__ = []

# NOTE: SmolVLA and other VLA models are handled by LeRobotPolicyNode
# because they are standard LeRobot policies with the same interface.
