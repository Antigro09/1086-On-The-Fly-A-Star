#!/usr/bin/env python3
"""Author tiny synthetic recordings; no planner, network, hardware or extra packages."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def frame(sequence):
    now_us = 1_000_000 + 200_000 * sequence
    return {
        "schema_version": "frc-planner-dashboard/1",
        "source_kind": "synthetic_mock",
        "session_id": "fixture-synthetic-session",
        "sequence": str(sequence), "robot_us": str(now_us), "captured_us": str(now_us),
        "epoch": "1", "snapshot_id": "1", "obstacle_map_version": "1",
        "field": {"season": "offseason-synthetic", "map_id": "dashboard-lab-8x4", "geometry_revision": "1"},
        "robot": {"pose": {"x_m": 1 + .1 * sequence, "y_m": 1, "heading_rad": 0},
                  "velocity": {"vx_mps": .5, "vy_mps": 0, "omega_radps": 0}},
        "obstacles": [],
        "plan": {
            "status": "SUCCESS", "generation": "1", "request_id": "fixture-authored-path",
            "task_id": "fixture-synthetic-display", "epoch": "1", "snapshot_id": "1", "obstacle_map_version": "1",
            "issued_us": "1000000", "valid_until_us": "5000000", "solver_duration_ns": "0",
            "backend_id": "synthetic-recording/not-a-solver",
            "detail": "Authored SYNTHETIC geometry; not a planner measurement or hardware evidence",
            "positions": [{"x_m": 1, "y_m": 1, "heading_rad": 0}, {"x_m": 7, "y_m": 1, "heading_rad": 0}]
        }
    }


if __name__ == "__main__":
    sample = frame(0)
    replay = {"schema_version": "frc-planner-dashboard-replay/1", "source_kind": "synthetic_mock",
              "frames": [frame(n) for n in range(3)]}
    (ROOT / "synthetic-telemetry.json").write_text(json.dumps(sample, indent=2) + "\n")
    (ROOT / "synthetic-replay.json").write_text(json.dumps(replay, indent=2) + "\n")
    print("Authored synthetic telemetry and three-frame replay fixtures; no solver or hardware claims.")
