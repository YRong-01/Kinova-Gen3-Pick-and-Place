#! /usr/bin/env python3

###
# Simple-Manipulation -- move through WAYPOINTS.
#
# Cartesian waypoint trajectories (ExecuteWaypointTrajectory), adapted from example
# 110/02. A list of waypoints is validated up front and executed as ONE continuous
# trajectory.
#
# See move_points.py for the discrete point-to-point alternative.
###

import threading

from kortex_api.autogen.messages import Base_pb2

from config import TIMEOUT_DURATION
from robot_common import check_for_end_or_abort, set_single_level_servoing


def populate_cartesian_waypoint(pose, blending_radius):
    """Build a Base_pb2.CartesianWaypoint from a pose dict and a blending radius.
    Adapted from example 110/02's populateCartesianCoordinate (dict-based here so it
    reuses this project's pose dicts).

    blending_radius (m): 0 = pass exactly through the waypoint (the arm momentarily
    stops there); > 0 = round the corner for continuous, non-stop motion.
    """
    waypoint = Base_pb2.CartesianWaypoint()
    waypoint.pose.x = pose["x"]                # meters
    waypoint.pose.y = pose["y"]                # meters
    waypoint.pose.z = pose["z"]                # meters
    waypoint.pose.theta_x = pose["theta_x"]    # degrees
    waypoint.pose.theta_y = pose["theta_y"]    # degrees
    waypoint.pose.theta_z = pose["theta_z"]    # degrees
    waypoint.blending_radius = blending_radius
    waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    return waypoint


def move_cartesian_waypoints(base, waypoints, label=""):
    """Move the tool through a sequence of Cartesian waypoints as ONE continuous
    trajectory. Adapted from example 110/02 (send_cartesian_waypoint_trajectory).

    This is the waypoint-trajectory replacement for chaining several move_cartesian()
    calls: the whole path is validated up front and executed as a single trajectory.

    Args:
        base:      BaseClient on the TCP router (single-level servoing).
        waypoints: list of (pose_dict, blending_radius_m) pairs, in order. Give the
                   final waypoint blending_radius = 0 so the arm stops exactly on it
                   (important for pick/place); intermediate ones may use > 0 for
                   smooth, non-stop motion.
        label:     short description for log messages.

    Returns:
        True if the trajectory validated and completed (ACTION_END); False on a
        validation error, ABORT, or timeout.
    """
    print("Cartesian waypoint trajectory {}: {} waypoints".format(label, len(waypoints)))
    set_single_level_servoing(base)

    waypoint_list = Base_pb2.WaypointList()
    waypoint_list.duration = 0.0
    waypoint_list.use_optimal_blending = False
    for index, (pose, blending_radius) in enumerate(waypoints):
        wp = waypoint_list.waypoints.add()
        wp.name = "waypoint_{}".format(index)
        wp.cartesian_waypoint.CopyFrom(populate_cartesian_waypoint(pose, blending_radius))

    # Validate before executing (example 110/02): catches an unreachable or
    # malformed trajectory before any motion is commanded.
    result = base.ValidateWaypointList(waypoint_list)
    if len(result.trajectory_error_report.trajectory_error_elements):
        print("Trajectory {} is INVALID -- not executing:".format(label))
        result.trajectory_error_report.PrintDebugString()
        return False

    e = threading.Event()
    res = {"event": None}
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e, res), Base_pb2.NotificationOptions())

    print("Executing waypoint trajectory ...")
    base.ExecuteWaypointTrajectory(waypoint_list)

    print("Waiting for trajectory to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    ok = finished and res["event"] == Base_pb2.ACTION_END
    if ok:
        print("Trajectory {} completed".format(label))
    elif res["event"] == Base_pb2.ACTION_ABORT:
        print("Trajectory {} was ABORTED".format(label))
    else:
        print("Timeout while running trajectory {}".format(label))
    return ok
