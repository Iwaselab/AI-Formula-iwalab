from typing import Optional, Tuple

import numpy as np

from .util import Side


def order_lane_polyline(
    lane_points: np.ndarray,
    max_point_length: float = 15.0,
    min_segment_length: float = 0.2,
    max_gap: float = 1.5,
) -> Optional[np.ndarray]:
    """Filter and order 2D lane points into a forward polyline starting near the vehicle.

    The chain stops when the nearest remaining point is farther than max_gap,
    so that no fake segment jumping back across the road is created.
    """
    points = np.asarray(lane_points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2 or len(points) == 0:
        return None

    xy = points[:, :2]
    dist_sq = xy[:, 0] ** 2 + xy[:, 1] ** 2
    mask = (xy[:, 0] >= -0.5) & (dist_sq <= max_point_length ** 2)
    filtered = xy[mask]
    if len(filtered) < 2:
        return None

    start_idx = int(np.argmin(np.sum(filtered ** 2, axis=1)))
    remaining = np.ones(len(filtered), dtype=bool)
    remaining[start_idx] = False
    min_seg_sq = min_segment_length ** 2
    max_gap_sq = max_gap ** 2

    def extend(chain):
        """Greedily extend the chain from its last point using unused points."""
        while np.any(remaining):
            last = chain[-1]
            cand_indices = np.flatnonzero(remaining)
            d_sq = np.sum((filtered[cand_indices] - last) ** 2, axis=1)
            nearest_pos = int(np.argmin(d_sq))
            if d_sq[nearest_pos] > max_gap_sq:
                break  # do not jump to distant points
            next_idx = cand_indices[nearest_pos]
            remaining[next_idx] = False
            if d_sq[nearest_pos] >= min_seg_sq:
                chain.append(filtered[next_idx])
        return chain

    # Chain in both directions from the closest point, so the polyline covers the
    # whole line even when the closest point lies in the middle of it.
    first = extend([filtered[start_idx]])
    second = extend([filtered[start_idx]])
    ordered = list(reversed(second[1:])) + first
    if len(ordered) < 2:
        return None

    polyline = np.asarray(ordered, dtype=float)
    # Orient the polyline rear -> front (the signed distance depends on the direction).
    if polyline[-1, 0] < polyline[0, 0]:
        polyline = polyline[::-1].copy()
    return polyline


def signed_distance_to_polyline(
    query_xy: np.ndarray,
    polyline_xy: np.ndarray,
    side: Side,
    return_beyond_end: bool = False,
):
    """Signed distance from query_xy (Q, 2) to polyline_xy (M, 2).

    Positive means inside the road; negative means outside the white line.
    If return_beyond_end is True, also returns a bool mask that is True for
    queries whose nearest point is the far end of the polyline and that
    project beyond it (i.e. the line is not observed there).
    """
    pts = np.asarray(query_xy, dtype=float)
    seg_a = polyline_xy[:-1]
    seg_b = polyline_xy[1:]
    ab = seg_b - seg_a
    len_sq = np.maximum(np.sum(ab ** 2, axis=1), 1e-12)

    ap = pts[:, np.newaxis, :] - seg_a[np.newaxis, :, :]
    t_raw = np.sum(ap * ab[np.newaxis, :, :], axis=2) / len_sq[np.newaxis, :]
    t = np.clip(t_raw, 0.0, 1.0)
    closest = seg_a[np.newaxis, :, :] + t[:, :, np.newaxis] * ab[np.newaxis, :, :]

    diff = pts[:, np.newaxis, :] - closest
    dist_sq = np.sum(diff ** 2, axis=2)
    best_seg = np.argmin(dist_sq, axis=1)

    q_idx = np.arange(len(pts))
    min_dist = np.sqrt(dist_sq[q_idx, best_seg])
    best_diff = diff[q_idx, best_seg]
    best_ab = ab[best_seg]

    cross = best_ab[:, 0] * best_diff[:, 1] - best_ab[:, 1] * best_diff[:, 0]
    if side == Side.LEFT:
        inside_sign = -np.sign(cross)
    else:
        inside_sign = np.sign(cross)

    signed = inside_sign * min_dist
    if not return_beyond_end:
        return signed
    beyond_end = (best_seg == len(seg_a) - 1) & (t_raw[q_idx, best_seg] > 1.0)
    return signed, beyond_end


class LaneCenterEstimator:
    """Stateless per-frame road evaluator using signed point-to-polyline distances."""

    def __init__(
        self,
        max_point_length: float = 15.0,
        lane_width: float = 3.5,
        min_segment_length: float = 0.2,
        max_gap: float = 1.5,
    ) -> None:
        if max_point_length <= 0.0:
            raise ValueError("max_point_length must be positive.")
        if lane_width <= 0.0:
            raise ValueError("lane_width must be positive.")
        if min_segment_length <= 0.0:
            raise ValueError("min_segment_length must be positive.")
        if max_gap <= 0.0:
            raise ValueError("max_gap must be positive.")

        self.max_point_length = float(max_point_length)
        self.lane_width = float(lane_width)
        self.min_segment_length = float(min_segment_length)
        self.max_gap = float(max_gap)
        self.max_assoc_distance = 1.2 * self.lane_width

        self._polylines: list = [None, None]

    def update_lane(self, side: Side, lane_points: np.ndarray) -> None:
        self._polylines[side] = order_lane_polyline(
            lane_points,
            max_point_length=self.max_point_length,
            min_segment_length=self.min_segment_length,
            max_gap=self.max_gap,
        )

    def get_polyline(self, side: Side) -> Optional[np.ndarray]:
        """Return the current ordered polyline (near -> far) of one side, or None."""
        return self._polylines[side]

    def visible_length(self) -> float:
        """Approximate observed path length [m] of the longer lane polyline (0 if none)."""
        best = 0.0
        for poly in self._polylines:
            if poly is None:
                continue
            seg = np.diff(poly, axis=0)
            length = float(np.linalg.norm(poly[0])) + float(np.sum(np.linalg.norm(seg, axis=1)))
            best = max(best, length)
        return best

    def has_any_lane(self) -> bool:
        return (
            self._polylines[Side.LEFT] is not None
            or self._polylines[Side.RIGHT] is not None
        )

    def evaluate_points(
        self,
        x_points: np.ndarray,
        y_points: np.ndarray,
        road_margin: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return center_sq_err, boundary_sq_err, valid_mask, each of shape (C, H)."""
        x_arr = np.asarray(x_points, dtype=float)
        y_arr = np.asarray(y_points, dtype=float)
        shape = x_arr.shape
        query_xy = np.column_stack([x_arr.ravel(), y_arr.ravel()])
        n_pts = len(query_xy)

        left_poly = self._polylines[Side.LEFT]
        right_poly = self._polylines[Side.RIGHT]

        if left_poly is not None:
            d_l, beyond_l = signed_distance_to_polyline(
                query_xy, left_poly, Side.LEFT, return_beyond_end=True
            )
            valid_l = (np.abs(d_l) <= self.max_assoc_distance) & ~beyond_l
        else:
            d_l = np.zeros(n_pts, dtype=float)
            valid_l = np.zeros(n_pts, dtype=bool)

        if right_poly is not None:
            d_r, beyond_r = signed_distance_to_polyline(
                query_xy, right_poly, Side.RIGHT, return_beyond_end=True
            )
            valid_r = (np.abs(d_r) <= self.max_assoc_distance) & ~beyond_r
        else:
            d_r = np.zeros(n_pts, dtype=float)
            valid_r = np.zeros(n_pts, dtype=bool)

        both = valid_l & valid_r
        only_l = valid_l & ~valid_r
        only_r = valid_r & ~valid_l
        valid = valid_l | valid_r

        half_w = 0.5 * self.lane_width
        center_err = np.zeros(n_pts, dtype=float)
        center_err[both] = 0.5 * (d_l[both] - d_r[both])
        center_err[only_l] = d_l[only_l] - half_w
        center_err[only_r] = d_r[only_r] - half_w

        left_excess = np.where(valid_l, np.maximum(0.0, road_margin - d_l), 0.0)
        right_excess = np.where(valid_r, np.maximum(0.0, road_margin - d_r), 0.0)
        boundary_sq = left_excess ** 2 + right_excess ** 2

        return (
            (center_err ** 2).reshape(shape),
            boundary_sq.reshape(shape),
            valid.reshape(shape),
        )