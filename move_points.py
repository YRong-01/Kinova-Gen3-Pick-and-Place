#! /usr/bin/env python3

###
# Simple-Manipulation -- move through POINTS.
#
# Discrete point-to-point Cartesian moves (high-level reach_pose), adapted from
# example 102/01. Each call moves to one pose and the arm fully reaches and stops
# there before the next move starts.
#
# See move_waypoints.py for the single-continuous-trajectory alternative.
###

import threading

from kortex_api.autogen.messages import Base_pb2

from config import TIMEOUT_DURATION
from robot_common import check_for_end_or_abort


def move_cartesian(base, base_cyclic, pose, label=""):
    """Move the tool to an ABSOLUTE Cartesian `pose` dict. Adapted from example 102."""
    print("Moving to Cartesian pose {}: {}".format(label, pose))

    action = Base_pb2.Action()
    action.name = "move_cartesian {}".format(label).strip()
    action.application_data = ""

    target = action.reach_pose.target_pose
    target.x = pose["x"]                  # meters
    target.y = pose["y"]                  # meters
    target.z = pose["z"]                  # meters
    target.theta_x = pose["theta_x"]      # degrees
    target.theta_y = pose["theta_y"]      # degrees
    target.theta_z = pose["theta_z"]      # degrees

    e = threading.Event()
    result = {"event": None}
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e, result),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    # Only ACTION_END is success; ACTION_ABORT and timeout are failures.
    ok = finished and result["event"] == Base_pb2.ACTION_END
    if ok:
        print("Reached pose {}".format(label))
    elif result["event"] == Base_pb2.ACTION_ABORT:
        print("Move to pose {} was ABORTED".format(label))
    else:
        print("Timeout while moving to pose {}".format(label))
    return ok


def move_through_points(base, base_cyclic, poses, label=""):
    """Move through a sequence of pose dicts as SEPARATE point-to-point moves
    (one move_cartesian per pose, each finishing before the next starts).

    The point-based counterpart of move_cartesian_waypoints: same path, but the arm
    fully reaches and stops at every pose instead of running one trajectory. Stops
    early and returns False on the first failed move.

    Args:
        base, base_cyclic: Kortex clients on the TCP router.
        poses:             ordered list of pose dicts to visit.
        label:             short description for log messages.

    Returns:
        True only if every move succeeded.
    """
    ok = True
    for i, pose in enumerate(poses):
        step = "{} [{}/{}]".format(label, i + 1, len(poses)).strip()
        ok &= move_cartesian(base, base_cyclic, pose, label=step)
        if not ok:
            break
    return ok
