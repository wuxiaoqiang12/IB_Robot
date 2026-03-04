"""
Inference components for modular pipeline architecture.

Components can run as:
1. Independent ROS2 nodes (distributed mode)
2. Composed nodes in same process (zero-copy mode)
"""

from inference_service.components.preprocessor import PreprocessorComponent
from inference_service.components.postprocessor import PostprocessorComponent

__all__ = ["PreprocessorComponent", "PostprocessorComponent"]
