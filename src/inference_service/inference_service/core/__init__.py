"""
Core inference components - Pure Python, zero ROS dependencies.

This module provides the building blocks for inference pipelines:
- PureInferenceEngine: Stateless GPU inference engine
- TensorPreprocessor: Tensor normalization
- TensorPostprocessor: Tensor denormalization
- InferenceCoordinator: Zero-copy composition of all components

All components can be tested independently in Jupyter/PyTest without ROS.
"""

from inference_service.core.pure_inference_engine import (
    PureInferenceEngine,
    InferenceResult,
    PolicyWrapper,
    MockPolicyWrapper,
    resolve_device,
)
from inference_service.core.preprocessor import (
    TensorPreprocessor,
    PreprocessorBase,
    MockPreprocessor,
)
from inference_service.core.postprocessor import (
    TensorPostprocessor,
    PostprocessorBase,
    MockPostprocessor,
)
from inference_service.core.coordinator import (
    InferenceCoordinator,
    CoordinatorConfig,
    CoordinatorResult,
)

__all__ = [
    "PureInferenceEngine",
    "InferenceResult",
    "PolicyWrapper",
    "MockPolicyWrapper",
    "resolve_device",
    "TensorPreprocessor",
    "PreprocessorBase",
    "MockPreprocessor",
    "TensorPostprocessor",
    "PostprocessorBase",
    "MockPostprocessor",
    "InferenceCoordinator",
    "CoordinatorConfig",
    "CoordinatorResult",
]
