"""
Action Dispatch Package.

Pull-based action distribution layer between inference models and ros2_control.

Provides:
- ActionDispatcherNode: Main node for action dispatching with optional temporal smoothing
- TopicExecutor: High-frequency topic-based action execution
- TemporalSmoother: Cross-frame exponential smoothing for action chunks
- TemporalSmootherManager: Convenient manager with runtime toggle support
"""

__version__ = '0.2.0'

from .temporal_smoother import (
    TemporalSmoother,
    TemporalSmootherConfig,
    TemporalSmootherManager,
)
from .topic_executor import TopicExecutor
from .action_dispatcher_node import ActionDispatcherNode

__all__ = [
    'TemporalSmoother',
    'TemporalSmootherConfig',
    'TemporalSmootherManager',
    'TopicExecutor',
    'ActionDispatcherNode',
]
