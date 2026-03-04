# tensormsg/registry.py
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Type
import numpy as np

# Encoders: ROS Type -> function(names, data, clamp) -> ROS Message
ENCODER_REGISTRY: Dict[str, Callable] = {}

# Decoders: ROS Type -> function(msg, spec) -> numpy/tensor
DECODER_REGISTRY: Dict[str, Callable] = {}

def register_encoder(ros_type: str):
    """Decorator to register a ROS message encoder."""
    def decorator(func: Callable):
        ENCODER_REGISTRY[ros_type] = func
        return func
    return decorator

def register_decoder(ros_type: str):
    """Decorator to register a ROS message decoder."""
    def decorator(func: Callable):
        DECODER_REGISTRY[ros_type] = func
        return func
    return decorator

def get_encoder(ros_type: str) -> Optional[Callable]:
    return ENCODER_REGISTRY.get(ros_type)

def get_decoder(ros_type: str) -> Optional[Callable]:
    return DECODER_REGISTRY.get(ros_type)
