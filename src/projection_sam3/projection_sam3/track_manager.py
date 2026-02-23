#!/usr/bin/env python3
"""Track management for 3D geometric tracking with TTL and accumulation."""

import numpy as np
import time
from enum import Enum
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field

try:
    from scipy.optimize import linear_sum_assignment
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class TrackState(Enum):
    """Track state machine."""
    TENTATIVE = 0     # Just created, < min_hits
    CONFIRMED = 1     # Has ≥ min_hits and recent detections
    LOST = 2          # No detection in last min_frames, but TTL not expired
    REMOVED = 3       # TTL expired, will be discarded


@dataclass
class Track:
    """Single object track."""
    track_id: int
    class_name: str
    center_3d: np.ndarray  # EMA-smoothed 3D position
    state: TrackState = TrackState.TENTATIVE
    hit_count: int = 0
    miss_count: int = 0
    last_detection_time: float = field(default_factory=time.time)
    created_time: float = field(default_factory=time.time)
    last_2d_det: Optional[object] = None  # Reference to last Detection2D
    confidence: float = 1.0  # Latest detection confidence
    lost_time: Optional[float] = None  # When track entered LOST state (for TTL)

    def age(self, current_time: float) -> float:
        """Time since track creation (seconds)."""
        return current_time - self.created_time

    def time_since_detection(self, current_time: float) -> float:
        """Time since last detection (seconds)."""
        return current_time - self.last_detection_time


@dataclass
class InventoryItem:
    """Long-lived inventory entry for seen objects."""
    inventory_id: int
    class_name: str
    center_3d_history: List[np.ndarray]  # All seen 3D centers
    last_seen_time: float = field(default_factory=time.time)
    seen_count: int = 0
    last_confidence: float = 1.0

    def update(self, center_3d: np.ndarray, confidence: float):
        """Update inventory with new detection."""
        self.center_3d_history.append(np.array(center_3d))
        self.last_seen_time = time.time()
        self.seen_count += 1
        self.last_confidence = confidence

    def get_avg_center(self) -> np.ndarray:
        """Get average center over all history."""
        if not self.center_3d_history:
            return np.zeros(3)
        return np.mean(self.center_3d_history, axis=0)


class TrackManager:
    """Manage multiple tracks with data association, TTL, and accumulation."""

    def __init__(
        self,
        min_hits: int = 2,
        ttl_sec: float = 2.0,
        max_3d_distance: float = 0.25,
        merge_radius_3d: float = 0.2,
        ema_alpha: float = 0.3,
        allow_unconfirmed_output: bool = False,
    ):
        """
        Initialize track manager.

        Args:
            min_hits: Minimum detections to confirm track
            ttl_sec: Time to live for lost tracks (seconds)
            max_3d_distance: Max 3D Euclidean distance for association (meters)
            merge_radius_3d: Distance threshold for merging into inventory (meters)
            ema_alpha: EMA smoothing coefficient for track centers
            allow_unconfirmed_output: Include tentative tracks in output
        """
        self.min_hits = min_hits
        self.ttl_sec = ttl_sec
        self.max_3d_distance = max_3d_distance
        self.merge_radius_3d = merge_radius_3d
        self.ema_alpha = ema_alpha
        self.allow_unconfirmed_output = allow_unconfirmed_output

        self.tracks: Dict[int, Track] = {}
        self.inventory: Dict[int, InventoryItem] = {}
        self.next_track_id = 1
        self.next_inventory_id = 1000

    def update(
        self,
        detections_3d: List[Tuple[object, np.ndarray, str, float]],
        current_time: float,
    ) -> Tuple[List[Track], List[InventoryItem]]:
        """
        Update tracks with new detections.

        Args:
            detections_3d: List of (detection_2d, center_3d, class_name, confidence)
            current_time: Current timestamp (seconds since epoch)

        Returns:
            (confirmed_and_tentative_tracks, inventory_items)
        """
        # 1. Perform data association
        unmatched_dets, matched_tracks = self._associate_detections(
            detections_3d, current_time
        )

        # 2. Update matched tracks
        for track_id, det_idx in matched_tracks:
            det_2d, center_3d, class_name, confidence = detections_3d[det_idx]
            track = self.tracks[track_id]

            # EMA smooth center
            track.center_3d = (
                self.ema_alpha * center_3d
                + (1 - self.ema_alpha) * track.center_3d
            )
            track.hit_count += 1
            track.miss_count = 0
            track.last_detection_time = current_time
            track.last_2d_det = det_2d
            track.confidence = confidence

            # State transition
            if track.state == TrackState.TENTATIVE and track.hit_count >= self.min_hits:
                track.state = TrackState.CONFIRMED
            elif track.state == TrackState.LOST:
                track.state = TrackState.CONFIRMED

            # Accumulate to inventory
            self._merge_to_inventory(track)

        # 3. Create new tracks from unmatched detections
        for det_idx in unmatched_dets:
            det_2d, center_3d, class_name, confidence = detections_3d[det_idx]
            track = Track(
                track_id=self.next_track_id,
                class_name=class_name,
                center_3d=np.array(center_3d, dtype=np.float64),
                state=TrackState.TENTATIVE,
                hit_count=1,
                last_2d_det=det_2d,
                confidence=confidence,
            )
            self.tracks[self.next_track_id] = track
            self.next_track_id += 1

            # Also add to inventory
            self._merge_to_inventory(track)

        # 4. Age missed tracks and handle TTL
        for track_id, track in list(self.tracks.items()):
            if track_id not in [t for t, _ in matched_tracks]:
                # This track was not matched this frame
                track.miss_count += 1

            # Check TTL
            time_since_det = track.time_since_detection(current_time)

            # Transition CONFIRMED → LOST when no detection for ttl_sec
            if track.state == TrackState.CONFIRMED and time_since_det > self.ttl_sec:
                track.state = TrackState.LOST
                track.lost_time = current_time  # Mark when entered LOST

            # Transition LOST → REMOVED when TTL expires (ttl_sec after entering LOST)
            if track.state == TrackState.LOST and track.lost_time is not None:
                time_in_lost = current_time - track.lost_time
                if time_in_lost > self.ttl_sec:
                    track.state = TrackState.REMOVED

            # Remove old tracks
            if track.state == TrackState.REMOVED:
                del self.tracks[track_id]

        # 5. Compile output
        # Include CONFIRMED, LOST, and optionally TENTATIVE tracks
        # LOST tracks show TTL persistence (object being tracked through detection gaps)
        output_tracks = [
            t
            for t in self.tracks.values()
            if t.state in (TrackState.CONFIRMED, TrackState.LOST, TrackState.TENTATIVE)
            if not (t.state == TrackState.TENTATIVE and not self.allow_unconfirmed_output)
        ]

        return output_tracks, list(self.inventory.values())

    def _associate_detections(
        self, detections_3d: List[Tuple[object, np.ndarray, str, float]], current_time: float
    ) -> Tuple[List[int], List[Tuple[int, int]]]:
        """
        Data association: match detections to existing tracks.

        Args:
            detections_3d: List of (detection_2d, center_3d, class_name, confidence)
            current_time: Current timestamp

        Returns:
            (unmatched_det_indices, [(track_id, det_idx), ...])
        """
        n_dets = len(detections_3d)
        n_tracks = len(self.tracks)

        if n_dets == 0 or n_tracks == 0:
            return list(range(n_dets)), []

        # Build cost matrix
        cost_matrix = np.ones((n_tracks, n_dets)) * np.inf
        track_ids = list(self.tracks.keys())

        for track_idx, track_id in enumerate(track_ids):
            track = self.tracks[track_id]
            for det_idx, (det_2d, center_3d, class_name, confidence) in enumerate(
                detections_3d
            ):
                # Gate: class must match
                if track.class_name != class_name:
                    continue

                # Gate: 3D distance
                try:
                    dist_3d = np.linalg.norm(track.center_3d - center_3d)
                    if np.isnan(dist_3d) or np.isinf(dist_3d):
                        continue
                    if dist_3d > self.max_3d_distance:
                        continue
                    # Cost is 3D distance (lower is better)
                    cost_matrix[track_idx, det_idx] = dist_3d
                except Exception:
                    continue

        # Check if any finite costs exist
        has_finite_cost = np.any(np.isfinite(cost_matrix))

        if not has_finite_cost:
            # No valid matches possible (all gated out)
            # Treat all detections as unmatched
            return list(range(n_dets)), []

        # Hungarian algorithm or greedy matching
        matched = []
        try:
            if HAS_SCIPY and n_tracks > 0 and n_dets > 0:
                track_indices, det_indices = linear_sum_assignment(cost_matrix)
                matched = [
                    (track_ids[t_idx], d_idx)
                    for t_idx, d_idx in zip(track_indices, det_indices)
                    if cost_matrix[t_idx, d_idx] < np.inf
                ]
            else:
                # Fallback greedy matching
                matched = self._greedy_match(cost_matrix, track_ids)
        except ValueError:
            # linear_sum_assignment can fail if matrix is degenerate
            # Fallback to greedy
            matched = self._greedy_match(cost_matrix, track_ids)

        matched_det_indices = set(d_idx for _, d_idx in matched)
        unmatched_dets = [i for i in range(n_dets) if i not in matched_det_indices]

        return unmatched_dets, matched

    @staticmethod
    def _greedy_match(
        cost_matrix: np.ndarray, track_ids: List[int]
    ) -> List[Tuple[int, int]]:
        """
        Greedy matching fallback (no scipy).

        Args:
            cost_matrix: (n_tracks, n_dets) cost array
            track_ids: List of track IDs

        Returns:
            [(track_id, det_idx), ...]
        """
        matches = []
        used_dets = set()

        # Sort by cost (ascending)
        flat_costs = [
            (cost_matrix[i, j], i, j)
            for i in range(cost_matrix.shape[0])
            for j in range(cost_matrix.shape[1])
            if cost_matrix[i, j] < np.inf
        ]
        flat_costs.sort()

        used_tracks = set()
        for cost, track_idx, det_idx in flat_costs:
            if track_idx not in used_tracks and det_idx not in used_dets:
                matches.append((track_ids[track_idx], det_idx))
                used_tracks.add(track_idx)
                used_dets.add(det_idx)

        return matches

    def _merge_to_inventory(self, track: Track):
        """Merge track into inventory if not already present."""
        # Find nearby inventory entry
        best_inventory_id = None
        best_dist = self.merge_radius_3d

        for inv_id, inv_item in self.inventory.items():
            avg_center = inv_item.get_avg_center()
            dist = np.linalg.norm(track.center_3d - avg_center)
            if dist < best_dist:
                best_dist = dist
                best_inventory_id = inv_id

        if best_inventory_id is not None:
            # Merge into existing
            self.inventory[best_inventory_id].update(
                track.center_3d, track.confidence
            )
        else:
            # Create new inventory entry
            inv = InventoryItem(
                inventory_id=self.next_inventory_id,
                class_name=track.class_name,
                center_3d_history=[np.array(track.center_3d)],
                seen_count=1,
                last_confidence=track.confidence,
            )
            self.inventory[self.next_inventory_id] = inv
            self.next_inventory_id += 1

    def get_debug_summary(self) -> str:
        """Return JSON-like debug summary."""
        import json

        summary = {
            "tracks": {
                str(t.track_id): {
                    "class": t.class_name,
                    "state": t.state.name,
                    "hit_count": t.hit_count,
                    "confidence": round(t.confidence, 3),
                    "center_3d": [round(x, 3) for x in t.center_3d],
                }
                for t in self.tracks.values()
            },
            "inventory_size": len(self.inventory),
            "inventory": {
                str(inv.inventory_id): {
                    "class": inv.class_name,
                    "seen_count": inv.seen_count,
                    "avg_center": [round(x, 3) for x in inv.get_avg_center()],
                }
                for inv in self.inventory.values()
            },
        }
        return json.dumps(summary, indent=2)

    def reset(self):
        """Clear all tracks and inventory."""
        self.tracks.clear()
        self.inventory.clear()
        self.next_track_id = 1
        self.next_inventory_id = 1000
