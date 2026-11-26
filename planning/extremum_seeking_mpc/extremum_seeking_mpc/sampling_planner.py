from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np

from .lane_center_estimator import LaneCenterEstimator


@dataclass(frozen=True)
class SamplingPlannerConfig:
    """Configuration for sampling-based two-segment arc MPC."""

    curvature_limit: float = 0.15
    num_candidates: int = 21  # per axis (kappa1 x kappa2 candidates in total)
    horizon_min_distance: float = 1.0
    horizon_max_distance: float = 5.0
    num_horizon_points: int = 8
    switch_distance: float = 2.0
    far_weight: float = 0.4
    min_valid_points: int = 3
    center_weight: float = 1.0
    boundary_weight: float = 10.0
    road_margin: float = 0.3
    smoothness_weight: float = 0.05
    invalid_penalty: float = 3.0  # cost per weighted horizon point without lane info [m^2]
    max_total_yaw: float = 1.57   # candidates turning more than this [rad] are rejected
    object_variance_x: float = 1.0
    object_variance_y: float = 1.0
    object_gain: float = 15.0
    obstacle_class_ids: Tuple[int, ...] = (1, 2, 3, 4, 7)


@dataclass(frozen=True)
class PlanResult:
    curvature: float
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    has_valid_lane: bool


class SamplingPlanner:
    """Vectorized sampling MPC over two-segment constant-curvature arcs.

    kappa1 is used for the first `switch_distance` meters (and is the command),
    kappa2 for the rest (look-ahead only).
    """

    def __init__(self, config: SamplingPlannerConfig) -> None:
        if config.curvature_limit <= 0.0:
            raise ValueError("curvature_limit must be positive.")
        if config.num_candidates < 3 or config.num_candidates % 2 == 0:
            raise ValueError("num_candidates must be an odd integer >= 3.")
        if config.horizon_min_distance <= 0.0:
            raise ValueError("horizon_min_distance must be positive.")
        if config.horizon_max_distance <= config.horizon_min_distance:
            raise ValueError(
                "horizon_max_distance must be greater than horizon_min_distance."
            )
        if config.num_horizon_points < 1:
            raise ValueError("num_horizon_points must be at least 1.")
        if config.switch_distance <= 0.0:
            raise ValueError("switch_distance must be positive.")
        if not 0.0 <= config.far_weight <= 1.0:
            raise ValueError("far_weight must be in [0, 1].")
        if config.invalid_penalty < 0.0:
            raise ValueError("invalid_penalty must be non-negative.")
        if config.max_total_yaw <= 0.0:
            raise ValueError("max_total_yaw must be positive.")
        if config.min_valid_points < 1:
            raise ValueError("min_valid_points must be at least 1.")
        if config.object_variance_x <= 0.0 or config.object_variance_y <= 0.0:
            raise ValueError("object_variance_x and object_variance_y must be positive.")

        self.config = config
        k_axis = np.linspace(
            -config.curvature_limit, config.curvature_limit, config.num_candidates
        )
        k1, k2 = np.meshgrid(k_axis, k_axis, indexing="ij")
        self.curvatures = k1.ravel()  # command curvature kappa1
        self._k2 = k2.ravel()
        self.arc_lengths = np.linspace(
            config.horizon_min_distance,
            config.horizon_max_distance,
            config.num_horizon_points,
        )
        self._cand_x, self._cand_y, self._cand_yaw = self._rollout_two_arcs(
            self.curvatures, self._k2, self.arc_lengths, config.switch_distance
        )
        self._w = np.linspace(1.0, config.far_weight, config.num_horizon_points)

    @staticmethod
    def _arc(k, s, th0=0.0, x0=0.0, y0=0.0):
        ks = k * s
        small = np.abs(k) < 1e-6
        k_safe = np.where(small, 1.0, k)
        x = np.where(
            small,
            x0 + s * np.cos(th0),
            x0 + (np.sin(th0 + ks) - np.sin(th0)) / k_safe,
        )
        y = np.where(
            small,
            y0 + s * np.sin(th0),
            y0 - (np.cos(th0 + ks) - np.cos(th0)) / k_safe,
        )
        return x, y, th0 + ks

    @classmethod
    def _rollout_two_arcs(cls, k1, k2, arc_lengths, switch):
        s = arc_lengths[np.newaxis, :]
        x1, y1, th1 = cls._arc(k1[:, np.newaxis], np.minimum(s, switch))
        return cls._arc(k2[:, np.newaxis], np.maximum(s - switch, 0.0), th1, x1, y1)

    def _compute_object_cost(self, objects: Iterable) -> np.ndarray:
        obs_xy = []
        obs_weights = []
        allowed = set(self.config.obstacle_class_ids)
        for obj in objects:
            if obj.class_id not in allowed:
                continue
            weight = self.config.object_gain * float(obj.width) * (float(obj.confidence) ** 2)
            if weight <= 0.0:
                continue
            obs_xy.append((float(obj.x), float(obj.y)))
            obs_weights.append(weight)

        if not obs_xy:
            return np.zeros(len(self.curvatures), dtype=float)

        obs_arr = np.asarray(obs_xy, dtype=float)
        w_arr = np.asarray(obs_weights, dtype=float)

        dx = self._cand_x[:, :, np.newaxis] - obs_arr[np.newaxis, np.newaxis, :, 0]
        dy = self._cand_y[:, :, np.newaxis] - obs_arr[np.newaxis, np.newaxis, :, 1]

        inv_var_x = 1.0 / (self.config.object_variance_x ** 2)
        inv_var_y = 1.0 / (self.config.object_variance_y ** 2)
        norm_const = 1.0 / (
            2.0 * np.pi * self.config.object_variance_x * self.config.object_variance_y
        )

        exponent = -0.5 * ((dx ** 2) * inv_var_x + (dy ** 2) * inv_var_y)
        risk_per_point = np.sum(
            norm_const * np.exp(exponent) * w_arr[np.newaxis, np.newaxis, :], axis=2
        )
        return np.max(risk_per_point, axis=1)

    def plan(
        self,
        lane_estimator: LaneCenterEstimator,
        objects: Iterable,
        prev_curvature: float,
    ) -> PlanResult:
        center_sq_err, boundary_sq_err, valid = lane_estimator.evaluate_points(
            self._cand_x, self._cand_y, self.config.road_margin
        )
        valid_counts = np.sum(valid, axis=1)
        max_valid = int(np.max(valid_counts))

        if max_valid == 0:
            idx = int(np.argmin(
                np.abs(self.curvatures - prev_curvature) + np.abs(self._k2)
            ))
            return PlanResult(
                curvature=float(self.curvatures[idx]),
                x=self._cand_x[idx].copy(),
                y=self._cand_y[idx].copy(),
                yaw=self._cand_yaw[idx].copy(),
                has_valid_lane=False,
            )

        required_valid = min(self.config.min_valid_points, max_valid)
        # Reject candidates that U-turn (e.g. to escape the lane evaluation region)
        yaw_ok = np.max(np.abs(self._cand_yaw), axis=1) <= self.config.max_total_yaw
        eligible = (valid_counts >= required_valid) & yaw_ok
        if not np.any(eligible):
            eligible = valid_counts >= required_valid

        # Points without lane information are NOT free: they cost invalid_penalty.
        # Otherwise candidates that leave the observed region early look cheaper.
        w_all = np.broadcast_to(self._w[np.newaxis, :], valid.shape)
        denom = np.sum(w_all, axis=1)
        center_cost = np.sum(
            w_all * np.where(valid, center_sq_err, self.config.invalid_penalty), axis=1
        ) / denom
        boundary_cost = np.sum(
            w_all * np.where(valid, boundary_sq_err, 0.0), axis=1
        ) / denom

        object_cost = self._compute_object_cost(objects)
        smooth_cost = self.config.smoothness_weight * (self.curvatures - prev_curvature) ** 2

        total_cost = (
            self.config.center_weight * center_cost
            + self.config.boundary_weight * boundary_cost
            + object_cost
            + smooth_cost
        )
        total_cost = np.where(eligible, total_cost, np.inf)

        best_idx = int(np.argmin(total_cost))
        return PlanResult(
            curvature=float(self.curvatures[best_idx]),
            x=self._cand_x[best_idx].copy(),
            y=self._cand_y[best_idx].copy(),
            yaw=self._cand_yaw[best_idx].copy(),
            has_valid_lane=True,
        )