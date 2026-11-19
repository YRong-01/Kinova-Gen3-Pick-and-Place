#!/usr/bin/env python3
"""Place blocks 1, 3, 5 at nearest-neighbour destinations using waypoints.

Run with --dry-run to preview without connecting to the robot. Otherwise use
the same connection options as pick_and_place_waypoints.py. Refresh measured
poses with update_config.py when tags move; no separate config is required.
Distances use only robot-base-frame X/Y in metres, starting at (0, 0).
The route measures placement tags only, excluding trips to retrieve blocks.
"""

import math
import sys

from config import PICK_PLACE_TASKS


def build_nn_tasks(tasks, start_xy=(0.0, 0.0)):
    """Keep pickup order 1/3/5; greedily assign unused destinations 2/4/6.

    A/B come from the pickup task; C/D come from the selected destination.
    Preserve all heights and orientations; never mutate the supplied config.
    Equal distances are resolved by destination tag ID.
    """
    picks = {task["pick_tag"]: task for task in tasks}
    destinations = {task["place_tag"]: task for task in tasks}
    if (len(tasks) != 3 or set(picks) != {1, 3, 5}
            or set(destinations) != {2, 4, 6}):
        raise ValueError("Expected three tasks with pick tags 1/3/5 and place tags 2/4/6")
    for task in tasks:
        for name in ("POSE_A", "POSE_B", "POSE_C", "POSE_D"):
            if not all(math.isfinite(task[name][axis]) for axis in
                       ("x", "y", "z", "theta_x", "theta_y", "theta_z")):
                raise ValueError(f"Non-finite coordinate in {name}")

    current_x, current_y = start_xy
    planned = []
    for pick_tag in (1, 3, 5):
        place_tag = min(destinations, key=lambda tag: (
            math.hypot(destinations[tag]["POSE_D"]["x"] - current_x,
                       destinations[tag]["POSE_D"]["y"] - current_y),
            tag,
        ))
        destination = destinations.pop(place_tag)
        source = picks[pick_tag]
        planned.append(dict(
            pick_tag=pick_tag, place_tag=place_tag,
            POSE_A=dict(source["POSE_A"]), POSE_B=dict(source["POSE_B"]),
            POSE_C=dict(destination["POSE_C"]), POSE_D=dict(destination["POSE_D"]),
        ))
        current_x = destination["POSE_D"]["x"]
        current_y = destination["POSE_D"]["y"]
    return planned


def main():
    tasks = build_nn_tasks(PICK_PLACE_TASKS)
    current_x, current_y = 0.0, 0.0
    total = 0.0
    print("Nearest-neighbour placement route from base XY (0, 0):")
    for task in tasks:
        pose = task["POSE_D"]
        distance = math.hypot(pose["x"] - current_x, pose["y"] - current_y)
        total += distance
        print(f"  Block {task['pick_tag']} -> tag {task['place_tag']}: "
              f"XY=({pose['x']:.6f}, {pose['y']:.6f}), leg={distance:.6f} m")
        current_x, current_y = pose["x"], pose["y"]
    print(f"Placement route length: {total:.6f} m")
    if "--dry-run" in sys.argv[1:]:
        return 0

    # Lazy import keeps planning/preview independent of the Kortex SDK.
    from pick_and_place_waypoints import main as execute_waypoints
    return execute_waypoints(tasks)


if __name__ == "__main__":
    sys.exit(main())
