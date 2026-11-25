from types import SimpleNamespace
import numpy as np

from extremum_seeking_mpc.lane_center_estimator import LaneCenterEstimator
from extremum_seeking_mpc.sampling_planner import SamplingPlanner, SamplingPlannerConfig
from extremum_seeking_mpc.util import Side


def test_lane_center_estimator_straight_both_sides():
    est = LaneCenterEstimator(lane_width=3.5)
    x = np.linspace(1.0, 10.0, 15)
    est.update_lane(Side.LEFT, np.column_stack([x, np.full_like(x, 1.75)]))
    est.update_lane(Side.RIGHT, np.column_stack([x, np.full_like(x, -1.75)]))

    y_c, y_l, y_r, valid = est.estimate_road_bounds(np.array([2.0, 5.0, 12.0]))
    assert list(valid) == [True, True, False]
    assert np.allclose(y_c[:2], 0.0)
    assert np.allclose(y_l[:2], 1.75)
    assert np.allclose(y_r[:2], -1.75)


def test_lane_center_estimator_single_side_fallback():
    est = LaneCenterEstimator(lane_width=3.5)
    x = np.linspace(1.0, 10.0, 15)
    est.update_lane(Side.LEFT, np.column_stack([x, np.full_like(x, 1.75)]))

    y_c, y_l, y_r, valid = est.estimate_road_bounds(np.array([3.0, 8.0]))
    assert np.all(valid)
    assert np.allclose(y_c, 0.0)
    assert np.allclose(y_l, 1.75)
    assert np.allclose(y_r, -1.75)


def test_lane_center_estimator_uses_linear_fit_when_short():
    est = LaneCenterEstimator(
        lane_width=3.5, min_points_for_quadratic=6, min_x_range_for_quadratic=4.0
    )
    x = np.array([1.0, 2.0, 3.0])
    est.update_lane(Side.LEFT, np.column_stack([x, 1.5 + 0.1 * x]))
    assert len(est._lanes[Side.LEFT].coeffs) == 2  # degree 1


def test_sampling_planner_follows_curved_lane():
    est = LaneCenterEstimator(lane_width=3.5)
    x = np.linspace(1.0, 12.0, 25)
    target_k = 0.08
    y_center_true = 0.5 * target_k * (x ** 2)
    est.update_lane(Side.LEFT, np.column_stack([x, y_center_true + 1.75]))
    est.update_lane(Side.RIGHT, np.column_stack([x, y_center_true - 1.75]))

    planner = SamplingPlanner(SamplingPlannerConfig(curvature_limit=0.15, num_candidates=41))
    result = planner.plan(est, objects=[], prev_curvature=0.0)

    assert result.has_valid_lane
    assert result.curvature > 0.04
    assert abs(result.curvature - target_k) < 0.03


def test_sampling_planner_avoids_obstacle():
    est = LaneCenterEstimator(lane_width=3.5)
    x = np.linspace(1.0, 12.0, 25)
    est.update_lane(Side.LEFT, np.column_stack([x, np.full_like(x, 1.75)]))
    est.update_lane(Side.RIGHT, np.column_stack([x, np.full_like(x, -1.75)]))

    # Obstacle slightly right of center at x=4.0 -> should steer left (curvature > 0)
    obstacle = SimpleNamespace(class_id=1, x=4.0, y=-0.1, width=20.0, confidence=1.0)
    planner = SamplingPlanner(SamplingPlannerConfig(curvature_limit=0.15, num_candidates=41))
    result = planner.plan(est, objects=[obstacle], prev_curvature=0.0)

    assert result.has_valid_lane
    assert result.curvature > 0.0


def test_sampling_planner_rejects_candidates_with_too_few_valid_points():
    est = LaneCenterEstimator(lane_width=3.5)
    # Only x in [1.0, 3.5] is valid -> straight arc has 3 valid points (1.5, 2.43, 3.36)
    x = np.linspace(1.0, 3.5, 10)
    est.update_lane(Side.LEFT, np.column_stack([x, np.full_like(x, 1.75)]))
    est.update_lane(Side.RIGHT, np.column_stack([x, np.full_like(x, -1.75)]))

    planner = SamplingPlanner(
        SamplingPlannerConfig(
            curvature_limit=0.15,
            num_candidates=41,
            horizon_min_distance=1.5,
            horizon_max_distance=8.0,
            num_horizon_points=8,
            min_valid_points=3,
        )
    )
    result = planner.plan(est, objects=[], prev_curvature=0.0)
    assert result.has_valid_lane
    assert abs(result.curvature) < 0.02
