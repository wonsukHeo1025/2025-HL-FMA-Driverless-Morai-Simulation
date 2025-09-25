#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Scenario package for data collection."""

from .base import BaseScenario
from .step_accel import StepAccelScenario
from .step_accel_reset import StepAccelResetScenario
from .step_brake import StepBrakeScenario
from .prbs import PRBSScenario
from .chirp import ChirpScenario
from .steady_state_cornering import SteadyStateCornering
from .step_steer import StepSteerScenario
from .sine_sweep import SineSweepScenario
from .double_lane_change import DoubleLaneChangeScenario

from typing import Optional, Dict, Any


class ScenarioFactory:
    """Factory class for creating scenario instances."""
    
    # Registry of available scenarios
    _scenarios = {
        'step_accel': StepAccelScenario,
        'step_accel_reset': StepAccelResetScenario,
        'step_brake': StepBrakeScenario,
        'prbs': PRBSScenario,
        'chirp': ChirpScenario,
        # Lateral control scenarios
        'steady_state_cornering': SteadyStateCornering,
        'step_steer': StepSteerScenario,
        'sine_sweep': SineSweepScenario,
        'double_lane_change': DoubleLaneChangeScenario,
    }
    
    @classmethod
    def create(cls, scenario_type: str, 
               params: Optional[Dict[str, Any]] = None) -> BaseScenario:
        """Create a scenario instance.
        
        Args:
            scenario_type: Type of scenario to create
            params: Optional parameters for scenario
            
        Returns:
            Scenario instance
            
        Raises:
            ValueError: If scenario type is unknown
        """
        if scenario_type not in cls._scenarios:
            raise ValueError(f"Unknown scenario type: {scenario_type}. "
                           f"Available: {list(cls._scenarios.keys())}")
        
        scenario_class = cls._scenarios[scenario_type]
        return scenario_class(params)
    
    @classmethod
    def list_scenarios(cls) -> list:
        """Get list of available scenario types.
        
        Returns:
            List of scenario type names
        """
        return list(cls._scenarios.keys())
    
    @classmethod
    def register(cls, name: str, scenario_class: type) -> None:
        """Register a new scenario type.
        
        Args:
            name: Name for the scenario type
            scenario_class: Scenario class (must inherit from BaseScenario)
        """
        if not issubclass(scenario_class, BaseScenario):
            raise TypeError(f"{scenario_class} must inherit from BaseScenario")
        
        cls._scenarios[name] = scenario_class


# Export main classes
__all__ = [
    'BaseScenario',
    'StepAccelScenario',
    'StepAccelResetScenario',
    'StepBrakeScenario',
    'PRBSScenario',
    'ChirpScenario',
    'SteadyStateCornering',
    'StepSteerScenario',
    'SineSweepScenario',
    'DoubleLaneChangeScenario',
    'ScenarioFactory',
]