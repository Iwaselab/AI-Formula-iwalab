from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .util import Side


@dataclass(frozen=True)
class FittedLaneLine:
    """Least-squares polynomial fit for a single lane line without extrapolation."""

    coeffs: np.ndarray  # Highest power first, compatible with np.polyval
    x_min: float
    x_max: float


class LaneCenterEstimator:
    """Stateless per-frame road center and boundary estimator from left/right lane points."""

    def __init__(
        self,
        max_point_length: float = 50.0,
        lane_width: float = 3.5,
        min_points_for_quadratic: int = 6,
        min_x_range_for_quadratic: float = 4.0,
    ) -> None:
        if max_point_length <= 0.0:
            raise ValueError("max_point_length must be positive.")
        if lane_width <= 0.0:
            raise ValueError("lane_width must be positive.")
        if min_points_for_quadratic < 3:
            raise ValueError("min_points_for_quadratic must be at least 3.")
        if min_x_range_for_quadratic <= 0.0:
            raise ValueError("min_x_range_for_quadratic must be positive.")

        self.max_point_length = float(max_point_length)
        self.lane_width = float(lane_width)
        self.min_points_for_quadratic = int(min_points_for_quadratic)
        self.min_x_range_for_quadratic = float(min_x_range_for_quadratic)

        self._lanes: list[Optional[FittedLaneLine]] = [None, None]

    def update_lane(self, side: Side, lane_points: np.ndarray) -> None:
        """Fit a 1st- or 2nd-order polynomial to filtered lane points for one side."""
        points = np.asarray(lane_points, dtype=float)
        if points.ndim != 2 or points.shape[1] < 2 or len(points) == 0:
            self._lanes[side] = None
            return

        mask = (points[:, 0] >= 0.0) & (points[:, 0] <= self.max_point_length)
        filtered = points[mask]
        if len(filtered) < 2:
            self._lanes[side] = None
            return

        x = filtered[:, 0]
        y = filtered[:, 1]
        x_min = float(np.min(x))
        x_max = float(np.max(x))
        x_range = x_max - x_min
        if x_range <= 1e-3:
            self._lanes[side] = None
            return

        use_quadratic = (
            len(filtered) >= self.min_points_for_quadratic
            and x_range >= self.min_x_range_for_quadratic
        )
        degree = 2 if use_quadratic else 1
        coeffs = np.polyfit(x, y, degree)
        self._lanes[side] = FittedLaneLine(coeffs=coeffs, x_min=x_min, x_max=x_max)

    def has_any_lane(self) -> bool:
        """Return True if at least one lane line is currently fitted."""
        return self._lanes[Side.LEFT] is not None or self._lanes[Side.RIGHT] is not None

    @staticmethod
    def _eval_side(
        lane: Optional[FittedLaneLine], x_query: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        if lane is None:
            zeros = np.zeros_like(x_query, dtype=float)
            invalid = np.zeros_like(x_query, dtype=bool)
            return zeros, invalid

        valid = (x_query >= lane.x_min) & (x_query <= lane.x_max)
        y_vals = np.zeros_like(x_query, dtype=float)
        if np.any(valid):
            y_vals[valid] = np.polyval(lane.coeffs, x_query[valid])
        return y_vals, valid

    def estimate_road_bounds(
        self, x_query: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate road center, left bound, right bound, and valid mask at x_query.

        Never extrapolates beyond a side's detected [x_min, x_max]. When only one
        side is valid at a query x, the opposite bound and center are offset by
        lane_width / 2.
        """
        x_arr = np.asarray(x_query, dtype=float)
        y_l, valid_l = self._eval_side(self._lanes[Side.LEFT], x_arr)
        y_r, valid_r = self._eval_side(self._lanes[Side.RIGHT], x_arr)

        both = valid_l & valid_r
        only_l = valid_l & ~valid_r
        only_r = valid_r & ~valid_l
        valid = valid_l | valid_r

        half_w = 0.5 * self.lane_width
        y_center = np.zeros_like(x_arr, dtype=float)
        y_left_bound = np.zeros_like(x_arr, dtype=float)
        y_right_bound = np.zeros_like(x_arr, dtype=float)

        y_center[both] = 0.5 * (y_l[both] + y_r[both])
        y_left_bound[both] = y_l[both]
        y_right_bound[both] = y_r[both]

        y_center[only_l] = y_l[only_l] - half_w
        y_left_bound[only_l] = y_l[only_l]
        y_right_bound[only_l] = y_l[only_l] - self.lane_width

        y_center[only_r] = y_r[only_r] + half_w
        y_left_bound[only_r] = y_r[only_r] + self.lane_width
        y_right_bound[only_r] = y_r[only_r]

        return y_center, y_left_bound, y_right_bound, valid
