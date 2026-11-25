from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np

from .lane_center_estimator import LaneCenterEstimator


@dataclass(frozen=True)
class SamplingPlannerConfig:
    """Configuration for sampling-based single-curvature arc MPC."""

    curvature_limit: float = 0.15
    num_candidates: int = 41
    horizon_min_distance: float = 1.5
    horizon_max_distance: float = 8.0
    num_horizon_points: int = 8
    min_valid_points: int = 3
    center_weight: float = 1.0
    boundary_weight: float = 10.0
    road_margin: float = 0.3
    smoothness_weight: float = 0.05
    object_variance_x: float = 1.0
    object_variance_y: float = 1.0
    object_gain: float = 15.0
    obstacle_class_ids: Tuple[int, ...] = (1, 2, 3, 4, 6, 7)


@dataclass(frozen=True)
class PlanResult:
    """Selected curvature and corresponding predicted trajectory."""

    curvature: float
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    has_valid_lane: bool


class SamplingPlanner:
    """Vectorized sampling MPC over constant-curvature circular arcs."""

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
        if config.min_valid_points < 1:
            raise ValueError("min_valid_points must be at least 1.")
        if config.object_variance_x <= 0.0 or config.object_variance_y <= 0.0:
            raise ValueError("object_variance_x and object_variance_y must be positive.")

        self.config = config
        self.curvatures = np.linspace(
            -config.curvature_limit, config.curvature_limit, config.num_candidates
        )
        self.arc_lengths = np.linspace(
            config.horizon_min_distance,
            config.horizon_max_distance,
            config.num_horizon_points,
        )
        self._cand_x, self._cand_y, self._cand_yaw = self._rollout_arcs(
            self.curvatures, self.arc_lengths
        )

    @staticmethod
    def _rollout_arcs(
        curvatures: np.ndarray, arc_lengths: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute exact (x, y, yaw) in base_footprint for constant-curvature arcs."""
        k = curvatures[:, np.newaxis]
        s = arc_lengths[np.newaxis, :]
        ks = k * s

        x = np.empty_like(ks, dtype=float)
        y = np.empty_like(ks, dtype=float)
        yaw = ks.copy()

        straight = np.abs(k[:, 0]) < 1e-6
        x[straight] = s
        y[straight] = 0.0

        curved = ~straight
        if np.any(curved):
            k_c = k[curved]
            ks_c = ks[curved]
            x[curved] = np.sin(ks_c) / k_c
            y[curved] = (1.0 - np.cos(ks_c)) / k_c

        return x, y, yaw

    def _compute_object_cost(self, objects: Iterable) -> np.ndarray:
        """Compute mean Gaussian obstacle risk across horizon points for each candidate."""
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

        obs_arr = np.asarray(obs_xy, dtype=float)  # (M, 2)
        w_arr = np.asarray(obs_weights, dtype=float)  # (M,)

        dx = self._cand_x[:, :, np.newaxis] - obs_arr[np.newaxis, np.newaxis, :, 0]
        dy = self._cand_y[:, :, np.newaxis] - obs_arr[np.newaxis, np.newaxis, :, 1]

        inv_var_x = 1.0 / (self.config.object_variance_x ** 2)
        inv_var_y = 1.0 / (self.config.object_variance_y ** 2)
        norm_const = 1.0 / (
            2.0 * np.pi * self.config.object_variance_x * self.config.object_variance_y
        )

        exponent = -0.5 * ((dx ** 2) * inv_var_x + (dy ** 2) * inv_var_y)
        risk_per_point = np.sum(norm_const * np.exp(exponent) * w_arr[np.newaxis, np.newaxis, :], axis=2)
        return np.max(risk_per_point, axis=1)

    def plan(
        self,
        lane_estimator: LaneCenterEstimator,
        objects: Iterable,
        prev_curvature: float,
    ) -> PlanResult:
        """Select the optimal curvature candidate minimizing path, boundary, and object costs."""
        y_center, y_left_bound, y_right_bound, valid = lane_estimator.estimate_road_bounds(
            self._cand_x
        )
        valid_counts = np.sum(valid, axis=1)
        max_valid = int(np.max(valid_counts))

        if max_valid == 0:
            idx = int(np.argmin(np.abs(self.curvatures - prev_curvature)))
            return PlanResult(
                curvature=float(self.curvatures[idx]),
                x=self._cand_x[idx].copy(),
                y=self._cand_y[idx].copy(),
                yaw=self._cand_yaw[idx].copy(),
                has_valid_lane=False,
            )

        required_valid = min(self.config.min_valid_points, max_valid)
        eligible = valid_counts >= required_valid
        denom = np.maximum(valid_counts, 1)

        center_sq_err = np.where(valid, (self._cand_y - y_center) ** 2, 0.0)
        center_cost = np.sum(center_sq_err, axis=1) / denom

        left_excess = np.maximum(0.0, self._cand_y - (y_left_bound - self.config.road_margin))
        right_excess = np.maximum(0.0, (y_right_bound + self.config.road_margin) - self._cand_y)
        boundary_sq_err = np.where(valid, left_excess ** 2 + right_excess ** 2, 0.0)
        boundary_cost = np.sum(boundary_sq_err, axis=1) / denom

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
