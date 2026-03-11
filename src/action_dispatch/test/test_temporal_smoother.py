#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for TemporalSmoother module.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import torch
import pytest

from action_dispatch.temporal_smoother import (
    TemporalSmoother,
    TemporalSmootherConfig,
    TemporalSmootherManager,
)


class TestTemporalSmootherConfig:
    def test_default_config(self):
        config = TemporalSmootherConfig()
        assert config.enabled is True
        assert config.chunk_size == 100
        assert config.temporal_ensemble_coeff == 0.01
        assert config.device is None

    def test_custom_config(self):
        config = TemporalSmootherConfig(
            enabled=False,
            chunk_size=50,
            temporal_ensemble_coeff=0.05,
            device='cuda:0',
        )
        assert config.enabled is False
        assert config.chunk_size == 50
        assert config.temporal_ensemble_coeff == 0.05
        assert config.device == 'cuda:0'

    def test_invalid_chunk_size(self):
        with pytest.raises(ValueError):
            TemporalSmootherConfig(chunk_size=0)
        with pytest.raises(ValueError):
            TemporalSmootherConfig(chunk_size=-1)


class TestTemporalSmoother:
    def test_basic_update_and_get(self):
        config = TemporalSmootherConfig(chunk_size=10)
        smoother = TemporalSmoother(config)
        
        actions = np.random.randn(10, 7)
        smoother.update(actions, 0)
        
        assert smoother.plan_length == 10
        
        for i in range(10):
            action = smoother.get_next_action()
            assert action.shape == (7,)
            assert smoother.plan_length == 9 - i

    def test_disabled_smoothing(self):
        config = TemporalSmootherConfig(enabled=False, chunk_size=10)
        smoother = TemporalSmoother(config)
        
        actions1 = np.ones((10, 7))
        smoother.update(actions1, 0)
        
        for _ in range(5):
            smoother.get_next_action()
        
        assert smoother.plan_length == 5
        
        plan_len_at_inference_start = 5
        for _ in range(3):
            smoother.get_next_action()
        actions_executed = plan_len_at_inference_start - smoother.plan_length
        
        actions2 = np.zeros((10, 7))
        smoother.update(actions2, actions_executed)
        
        assert smoother.plan_length == 7
        
        for _ in range(7):
            action = smoother.get_next_action()
            np.testing.assert_array_almost_equal(action, np.zeros(7))

    def test_cross_frame_smoothing(self):
        config = TemporalSmootherConfig(enabled=True, chunk_size=10, temporal_ensemble_coeff=0.01)
        smoother = TemporalSmoother(config)
        
        actions1 = np.ones((10, 7)) * 1.0
        smoother.update(actions1, 0)
        
        for _ in range(3):
            smoother.get_next_action()
        
        assert smoother.plan_length == 7
        
        plan_len_at_inference_start = 7
        for _ in range(2):
            smoother.get_next_action()
        actions_executed = plan_len_at_inference_start - smoother.plan_length
        
        actions2 = np.ones((10, 7)) * 2.0
        smoother.update(actions2, actions_executed)
        
        assert smoother.plan_length == 8
        
        first_action = smoother.peek_next_action()
        assert first_action is not None
        
        assert not np.allclose(first_action.numpy(), np.ones(7) * 1.0)
        assert not np.allclose(first_action.numpy(), np.ones(7) * 2.0)

    def test_tensor_input(self):
        config = TemporalSmootherConfig(chunk_size=10)
        smoother = TemporalSmoother(config)
        
        actions = torch.randn(10, 7)
        smoother.update(actions, 0)
        
        assert smoother.plan_length == 10
        
        action = smoother.get_next_action()
        assert isinstance(action, torch.Tensor)
        assert action.shape == (7,)

    def test_reset(self):
        config = TemporalSmootherConfig(chunk_size=10)
        smoother = TemporalSmoother(config)
        
        actions = np.random.randn(10, 7)
        smoother.update(actions, 0)
        
        assert smoother.plan_length == 10
        
        smoother.reset()
        
        assert smoother.plan_length == 0
        assert smoother._smoothed_actions is None

    def test_empty_input(self):
        config = TemporalSmootherConfig(chunk_size=10)
        smoother = TemporalSmoother(config)
        
        actions = np.array([]).reshape(0, 7)
        result = smoother.update(actions, 0)
        
        assert result == 0
        assert smoother.plan_length == 0

    def test_get_next_action_raises_on_empty(self):
        config = TemporalSmootherConfig(chunk_size=10)
        smoother = TemporalSmoother(config)
        
        with pytest.raises(IndexError):
            smoother.get_next_action()

    def test_1d_input_reshaped(self):
        config = TemporalSmootherConfig(chunk_size=10)
        smoother = TemporalSmoother(config)
        
        actions = np.random.randn(7)
        smoother.update(actions, 0)
        
        assert smoother.plan_length == 1
        
        action = smoother.get_next_action()
        assert action.shape == (7,)


class TestTemporalSmootherManager:
    def test_manager_basic(self):
        manager = TemporalSmootherManager(
            enabled=True,
            chunk_size=10,
            temporal_ensemble_coeff=0.01,
        )
        
        assert manager.is_enabled is True
        assert manager.plan_length == 0
        
        actions = np.random.randn(10, 7)
        manager.update(actions, 0)
        
        assert manager.plan_length == 10

    def test_manager_toggle(self):
        manager = TemporalSmootherManager(enabled=True, chunk_size=10)
        
        assert manager.is_enabled is True
        
        manager.set_enabled(False)
        assert manager.is_enabled is False
        
        manager.set_enabled(True)
        assert manager.is_enabled is True

    def test_manager_peek(self):
        manager = TemporalSmootherManager(enabled=True, chunk_size=10)
        
        actions = np.random.randn(10, 7)
        manager.update(actions, 0)
        
        peeked = manager.peek_next_action()
        assert peeked is not None
        
        assert manager.plan_length == 10
        
        gotten = manager.get_next_action()
        assert manager.plan_length == 9


class TestSmoothingFormula:
    def test_weight_calculation(self):
        config = TemporalSmootherConfig(chunk_size=5, temporal_ensemble_coeff=0.0)
        smoother = TemporalSmoother(config)
        
        expected_weights = torch.ones(5)
        np.testing.assert_array_almost_equal(
            smoother._weights.numpy(), expected_weights.numpy()
        )

    def test_positive_coeff_weights(self):
        config = TemporalSmootherConfig(chunk_size=5, temporal_ensemble_coeff=0.1)
        smoother = TemporalSmoother(config)
        
        assert smoother._weights[0] > smoother._weights[4]
        assert smoother._weights[0] == 1.0

    def test_cumulative_weights(self):
        config = TemporalSmootherConfig(chunk_size=5, temporal_ensemble_coeff=0.0)
        smoother = TemporalSmoother(config)
        
        expected_cumsum = torch.tensor([1.0, 2.0, 3.0, 4.0, 5.0])
        np.testing.assert_array_almost_equal(
            smoother._weights_cumsum.numpy(), expected_cumsum.numpy()
        )


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
