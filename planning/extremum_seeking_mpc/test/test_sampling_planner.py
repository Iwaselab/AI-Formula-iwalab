from types import SimpleNamespace
import numpy as np

from extremum_seeking_mpc.lane_center_estimator import (
    LaneCenterEstimator,
    order_lane_polyline,
    signed_distance_to_polyline,
)
from extremum_seeking_mpc.sampling_planner import SamplingPlanner, SamplingPlannerConfig
from extremum_seeking_mpc.util import Side


def test_signed_distance_distinguishes_inside_and_outside():
    x = np.linspace(0.0, 10.0, 11)
    left_poly = order_lane_polyline(np.column_stack([x, np.full_like(x, 1.75)]))
    right_poly = order_lane_polyline(np.column_stack([x, np.full_like(x, -1.75)]))

    # Points: inside center (5, 0), outside left (5, 3.5), outside right (5, -3.5)
    pts = np.array([[5.0, 0.0], [5.0, 3.5], [5.0, -3.5]])
    d_l = signed_distance_to_polyline(pts, left_poly, Side.LEFT)
    d_r = signed_distance_to_polyline(pts, right_poly, Side.RIGHT)

    assert np.isclose(d_l[0], 1.75)
    assert np.isclose(d_r[0], 1.75)
    assert np.isclose(d_l[1], -1.75)  # outside left line -> negative
    assert np.isclose(d_r[2], -1.75)  # outside right line -> negative


def test_outside_road_never_beats_inside_when_single_side():
    est = LaneCenterEstimator(lane_width=3.5)
    x = np.linspace(0.5, 10.0, 20)
    est.update_lane(Side.LEFT, np.column_stack([x, np.full_like(x, 1.75)]))

    # Candidate A: road center (y=0.0) vs Candidate B: outside left line (y=+3.5)
    x_pts = np.array([[2.0, 5.0, 8.0], [2.0, 5.0, 8.0]])
    y_pts = np.array([[0.0, 0.0, 0.0], [3.5, 3.5, 3.5]])
    center_sq, boundary_sq, valid = est.evaluate_points(x_pts, y_pts, road_margin=0.3)

    assert np.all(valid)
    assert np.allclose(center_sq[0], 0.0)
    assert np.allclose(boundary_sq[0], 0.0)
    assert np.all(center_sq[1] > 10.0)
    assert np.all(boundary_sq[1] > 3.0)


def test_sampling_planner_follows_90_degree_sharp_curve():
    # R = 5.0 m left turn (curvature = 0.20 rad/m), curving beyond 90 degrees
    R_center = 5.0
    lane_width = 3.5
    angles = np.linspace(0.0, 0.65 * np.pi, 40)  # > 90 deg, x folds back
    left_pts = np.column_stack([
        (R_center - 0.5 * lane_width) * np.sin(angles),
        R_center - (R_center - 0.5 * lane_width) * np.cos(angles),
    ])
    right_pts = np.column_stack([
        (R_center + 0.5 * lane_width) * np.sin(angles),
        R_center - (R_center + 0.5 * lane_width) * np.cos(angles),
    ])

    est = LaneCenterEstimator(max_point_length=15.0, lane_width=lane_width)
    est.update_lane(Side.LEFT, left_pts)
    est.update_lane(Side.RIGHT, right_pts)

    planner = SamplingPlanner(
        SamplingPlannerConfig(
            curvature_limit=0.25,
            num_candidates=41,
            horizon_min_distance=1.5,
            horizon_max_distance=7.0,
        )
    )
    result = planner.plan(est, objects=[], prev_curvature=0.0)

    assert result.has_valid_lane
    assert abs(result.curvature - 0.20) < 0.03


def test_sampling_planner_single_side_outer_right_only():
    # Inner (left) line missing; only outer (right) line of a R=5m left turn is visible
    R_center = 5.0
    lane_width = 3.5
    angles = np.linspace(0.0, 0.55 * np.pi, 30)
    right_pts = np.column_stack([
        (R_center + 0.5 * lane_width) * np.sin(angles),
        R_center - (R_center + 0.5 * lane_width) * np.cos(angles),
    ])

    est = LaneCenterEstimator(max_point_length=15.0, lane_width=lane_width)
    est.update_lane(Side.RIGHT, right_pts)

    planner = SamplingPlanner(
        SamplingPlannerConfig(
            curvature_limit=0.25,
            num_candidates=41,
            horizon_min_distance=1.5,
            horizon_max_distance=7.0,
        )
    )
    result = planner.plan(est, objects=[], prev_curvature=0.0)

    assert result.has_valid_lane
    assert abs(result.curvature - 0.20) < 0.03


def test_sampling_planner_avoids_obstacle():
    est = LaneCenterEstimator(lane_width=3.5)
    x = np.linspace(0.5, 12.0, 25)
    est.update_lane(Side.LEFT, np.column_stack([x, np.full_like(x, 1.75)]))
    est.update_lane(Side.RIGHT, np.column_stack([x, np.full_like(x, -1.75)]))

    # Obstacle slightly right of center at x=4.0 -> should steer left (curvature > 0)
    obstacle = SimpleNamespace(class_id=1, x=4.0, y=-0.1, width=20.0, confidence=1.0)
    planner = SamplingPlanner(SamplingPlannerConfig(curvature_limit=0.25, num_candidates=41))
    result = planner.plan(est, objects=[obstacle], prev_curvature=0.0)

    assert result.has_valid_lane
    assert result.curvature > 0.0


def test_sampling_planner_rejects_candidates_with_too_few_valid_points():
    est = LaneCenterEstimator(lane_width=3.5)
    # Only x in [1.0, 3.0] is valid; points beyond 7.2 m from polyline are invalid
    x = np.linspace(1.0, 3.0, 10)
    est.update_lane(Side.LEFT, np.column_stack([x, np.full_like(x, 1.75)]))
    est.update_lane(Side.RIGHT, np.column_stack([x, np.full_like(x, -1.75)]))

    planner = SamplingPlanner(
        SamplingPlannerConfig(
            curvature_limit=0.25,
            num_candidates=41,
            horizon_min_distance=1.5,
            horizon_max_distance=12.0,
            num_horizon_points=8,
            min_valid_points=3,
        )
    )
    result = planner.plan(est, objects=[], prev_curvature=0.0)
    assert result.has_valid_lane
    assert abs(result.curvature) < 0.03

