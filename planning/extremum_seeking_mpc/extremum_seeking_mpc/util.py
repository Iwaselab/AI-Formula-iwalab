from dataclasses import dataclass, field
from enum import IntEnum
import numpy as np


def first_scalar(value, parameter_name: str) -> float:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size == 0:
        raise ValueError(f"{parameter_name} must not be empty.")
    return float(array[0])


@dataclass
class Position2d:
    x: float = 0.0
    y: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y])


@dataclass
class Pose:
    pos: Position2d = field(default_factory=Position2d)
    yaw: float = 0.0


@dataclass
class Velocity:
    linear: float
    angular: float


@dataclass
class ControllerParameters:
    seek_gain: float
    seek_amp: float
    curvature_max: float
    curvature_min: float
    feedback_gain: float
    sin_period: float


@dataclass
class LowPassFilterParameters:
    A: float
    B: float
    C: float


class Side(IntEnum):
    LEFT = 0
    RIGHT = 1
    NUM_SIDES = 2


class Vector2(IntEnum):
    X = 0
    Y = 1
