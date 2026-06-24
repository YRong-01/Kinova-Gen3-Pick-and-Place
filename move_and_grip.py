#! /usr/bin/env python3

###
# Simple-Manipulation - Task 1
#
# Move a Kinova Gen3 arm to a user-defined Cartesian pose, close the gripper with
# a user-defined force, then move to a second user-defined Cartesian pose.
#
# References (Kinova Kortex Python examples):
#   - Cartesian move : 102-Movement_high_level/01-move_angular_and_cartesian.py
#   - Gripper force  : 107-Gripper_low_level_command/01-gripper_low_level_command.py
#
# Note on "force": the high-level GripperCommand only supports POSITION and SPEED
# modes (no force field). A user-defined force is therefore applied through the
# low-level cyclic command, where MotorCommand.force is a 0-100 % force *limit*
# used while closing -- the gripper stops once that limit is exceeded.
###

import os
import sys
import time
import threading

# --- Make the Kinova examples' utilities.py importable (connection helper) -----
KORTEX_EXAMPLES_DIR = "/home/jiandong/Kinova-kortex2_Gen3_G3L/api_python/examples"
sys.path.insert(0, KORTEX_EXAMPLES_DIR)
import utilities  # noqa: E402  (provides DeviceConnection)

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient            # noqa: E402
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient  # noqa: E402
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2                 # noqa: E402


# =============================================================================
# CONFIG -- edit these values, then run:  python3 move_and_grip.py
# =============================================================================

# Connection ------------------------------------------------------------------
ROBOT_IP = "192.168.1.10"
USERNAME = "admin"
PASSWORD = "admin"

# Target poses (ABSOLUTE Cartesian, robot base frame) -------------------------
#   x, y, z          in meters
#   theta_x/y/z      in degrees
#
# !!! Replace with real, reachable coordinates for your setup.
#     Set PRINT_POSE_AND_EXIT = True first to print the current pose, then pick
#     POSE_A / POSE_B values near it.
#
#     Targets must stay inside the arm's DEXTEROUS workspace, not just within max
#     reach: a pose can be < 0.9 m from the base yet still be unreachable because
#     the wrist cannot achieve the requested orientation there. Rule of thumb: the
#     farther out in x, the lower z must be. (e.g. at x ~= 0.645, keep z <= ~0.25;
#     (0.645, 0, 0.30) pointing down fails IK -> the move aborts.)
#     The reachability pre-check below catches bad targets before any motion.

# Gripper force ---------------------------------------------------------------
# 0-100 % force LIMIT applied while closing. Keep this low for first runs.
GRIPPER_FORCE = 20.0

# Joint speed -----------------------------------------------------------------
# example 102/04 style: per-joint angular speed (deg/s), commanded with
# base.SendJointSpeedsCommand() before the high-level moves (see send_joint_speeds
# and its call in main). example 04 uses SPEED = 20.0; raise to move faster.
JOINT_SPEED = 20.0
JOINT_SPEED_DURATION = 2.0   # seconds to hold the speed command before Stop()

# Waypoint trajectory ---------------------------------------------------------
# Blending radius (m) for INTERMEDIATE waypoints of a Cartesian trajectory (110/02
# style). 0 = pass exactly through each waypoint (momentary stop); raise (e.g. 0.05)
# for smooth, non-stop motion. The final waypoint of each segment always uses 0 so
# the arm stops precisely on the pick/place pose.
BLENDING_RADIUS = 0.0

PRINT_POSE_AND_EXIT = False  # just connect, print current pose, and exit (no motion)

# Maximum allowed waiting time during high-level actions (seconds)
TIMEOUT_DURATION = 30

# =============================================================================
# End of CONFIG
# =============================================================================


def _connection_args():
    """Build the argument object DeviceConnection expects, from the constants
    above (so the script needs no command-line arguments)."""
    class _Args:
        pass
    args = _Args()
    args.ip = ROBOT_IP
    args.username = USERNAME
    args.password = PASSWORD
    return args


def check_for_end_or_abort(e, result):
    """Return a closure that, on an END or ABORT action notification, records the
    event into `result['event']` and sets event `e`. `result` lets the caller tell
    a successful END apart from an ABORT (a plain Event cannot)."""
    def check(notification, e=e):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            result["event"] = notification.action_event
            e.set()
    return check


def print_current_pose(base_cyclic):
    """Print the current tool Cartesian pose and gripper position."""
    fb = base_cyclic.RefreshFeedback()
    print("--- Current pose (base frame) ---")
    print("  x={:.4f} m  y={:.4f} m  z={:.4f} m".format(
        fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z))
    print("  theta_x={:.2f}  theta_y={:.2f}  theta_z={:.2f} (deg)".format(
        fb.base.tool_pose_theta_x, fb.base.tool_pose_theta_y, fb.base.tool_pose_theta_z))
    if len(fb.interconnect.gripper_feedback.motor):
        print("  gripper position = {:.1f} % (0=open, 100=closed)".format(
            fb.interconnect.gripper_feedback.motor[0].position))
    print("---------------------------------")


def pose_is_reachable(base, base_cyclic, pose):
    """Read-only IK pre-check: return True if `pose` is reachable, False otherwise.

    Uses base.ComputeInverseKinematics (no motion), seeded with the current joint
    angles. A False here is exactly what would otherwise make the high-level
    Cartesian action abort with "The first convergence of the IK has failed".
    """
    ik = Base_pb2.IKData()
    ik.cartesian_pose.x = pose["x"]
    ik.cartesian_pose.y = pose["y"]
    ik.cartesian_pose.z = pose["z"]
    ik.cartesian_pose.theta_x = pose["theta_x"]
    ik.cartesian_pose.theta_y = pose["theta_y"]
    ik.cartesian_pose.theta_z = pose["theta_z"]

    feedback = base_cyclic.RefreshFeedback()
    for actuator in feedback.actuators:
        ik.guess.joint_angles.add().value = actuator.position

    try:
        base.ComputeInverseKinematics(ik)
        return True
    except Exception as ex:
        print("  IK pre-check failed: {}".format(str(ex).splitlines()[0]))
        return False


def set_single_level_servoing(base):
    """Put the arm in single-level servoing (required for high-level actions)."""
    mode = Base_pb2.ServoingModeInformation()
    mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(mode)


def move_to_home(base):
    """Move the arm to the built-in 'Home' position. Adapted from example 102."""
    set_single_level_servoing(base)
    print("Moving the arm to the Home position ...")

    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle
    if action_handle is None:
        print("Could not find the Home action. Skipping.")
        return False

    e = threading.Event()
    result = {"event": None}
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e, result), Base_pb2.NotificationOptions())
    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    ok = finished and result["event"] == Base_pb2.ACTION_END
    if ok:
        print("Home reached")
    elif result["event"] == Base_pb2.ACTION_ABORT:
        print("Move to Home was ABORTED")
    else:
        print("Timeout while moving to Home")
    return ok


def send_joint_speeds(base, speeds, duration=JOINT_SPEED_DURATION):
    """Command per-joint angular speeds via the high-level velocity API.
    Direct port of example 102/04's example_send_joint_speeds.

    OPEN-LOOP velocity control: each joint turns at its speed (deg/s) until
    `duration` elapses, then base.Stop() halts the arm. This sets how fast the
    joints move; it does not steer the tool to a target pose.

    NOTE: this is a velocity command, not a persistent limit. A high-level
    reach_pose/reach_joint_angles action issued afterwards plans its own
    trajectory and is not capped by a previous SendJointSpeedsCommand.

    Args:
        base:     BaseClient on the TCP router (single-level servoing).
        speeds:   list of per-joint speeds in deg/s, one entry per actuator.
        duration: seconds to hold the speed before stopping. 0 -> send and
                  return without stopping (next command/Stop ends the motion).
    """
    set_single_level_servoing(base)

    joint_speeds = Base_pb2.JointSpeeds()
    for joint_id, value in enumerate(speeds):
        js = joint_speeds.joint_speeds.add()
        js.joint_identifier = joint_id
        js.value = value      # deg/s

    print("Sending joint speeds (deg/s): {}".format([round(s, 1) for s in speeds]))
    base.SendJointSpeedsCommand(joint_speeds)
    if duration > 0:
        time.sleep(duration)
        base.Stop()
        print("Joint speeds stopped.")
    return True


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


def populate_cartesian_waypoint(pose, blending_radius):
    """Build a Base_pb2.CartesianWaypoint from a pose dict and a blending radius.
    Adapted from example 110/02's populateCartesianCoordinate (dict-based here so it
    reuses this file's pose dicts).

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


def close_gripper_with_force(base, base_cyclic_rt, force):
    """Close the gripper using a user-defined force LIMIT (0-100 %) via low-level
    cyclic commands. Stops when fully closed or when the force limit halts motion
    (object grasped). Adapted from example 107.

    Args:
        base:           BaseClient on the TCP router (servoing-mode changes).
        base_cyclic_rt: BaseCyclicClient on the UDP router (real-time cyclic).
        force:          force limit, 0-100 %.
    """
    force = max(0.0, min(100.0, float(force)))
    print("Closing gripper with force limit = {:.1f} % ...".format(force))

    # Save the current servoing mode so we can restore it afterwards.
    previous_mode = base.GetServoingMode()

    # Switch to low-level servoing for cyclic gripper control.
    low_level = Base_pb2.ServoingModeInformation()
    low_level.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
    base.SetServoingMode(low_level)

    try:
        # Build the cyclic command, holding every actuator at its current position
        # so the arm does not move while we servo the gripper.
        command = BaseCyclic_pb2.Command()
        command.frame_id = 0
        command.interconnect.command_id.identifier = 0
        command.interconnect.gripper_command.command_id.identifier = 0
        motorcmd = command.interconnect.gripper_command.motor_cmd.add()

        feedback = base_cyclic_rt.RefreshFeedback()
        motorcmd.position = feedback.interconnect.gripper_feedback.motor[0].position
        motorcmd.velocity = 0.0
        motorcmd.force = force

        for actuator in feedback.actuators:
            actuator_command = command.actuators.add()
            actuator_command.position = actuator.position
            actuator_command.velocity = 0.0
            actuator_command.torque_joint = 0.0
            actuator_command.command_id = 0

        # Command a full close (100 %). With the force limit set, the gripper will
        # stop early if it grasps an object before reaching the closed position.
        motorcmd.position = 100.0
        proportional_gain = 2.0

        last_position = motorcmd.position
        stall_cycles = 0
        STALL_LIMIT = 300          # ~0.3 s of no movement -> consider stopped
        POSITION_EPS = 0.10        # % change considered "no movement"
        start = time.time()

        while True:
            try:
                fb = base_cyclic_rt.Refresh(command)
            except Exception as ex:
                print("Error in cyclic refresh: " + str(ex))
                break

            position = fb.interconnect.gripper_feedback.motor[0].position
            error = 100.0 - position

            # Fully closed.
            if error < 1.5:
                print("Gripper fully closed.")
                break

            # Detect a stall (object grasped / force limit reached).
            if abs(position - last_position) < POSITION_EPS:
                stall_cycles += 1
                if stall_cycles >= STALL_LIMIT:
                    print("Gripper stopped at {:.1f} % (force limit reached).".format(position))
                    break
            else:
                stall_cycles = 0
            last_position = position

            # Safety timeout.
            if time.time() - start > TIMEOUT_DURATION:
                print("Timeout while closing gripper (at {:.1f} %).".format(position))
                break

            # Proportional closing speed toward the closed position.
            velocity = proportional_gain * abs(error)
            motorcmd.velocity = min(velocity, 100.0)
            motorcmd.position = 100.0
            time.sleep(0.001)

        # Stop the gripper motion.
        motorcmd.velocity = 0.0
        try:
            base_cyclic_rt.Refresh(command)
        except Exception:
            pass
    finally:
        # Always restore the previous servoing mode so later moves work.
        base.SetServoingMode(previous_mode)

    return True


def move_gripper_to_position(base, position, timeout=TIMEOUT_DURATION):
    """Move the gripper to an absolute position using the high-level GripperCommand
    (GRIPPER_POSITION mode). Adapted from example 106.

    Unlike close_gripper_with_force (low-level, force-limited), this is a simple
    open-loop position command: tell the gripper where to go and wait until the
    measured position reports it has arrived. No force limiting -- use this for
    free motion (e.g. fully open before a pick), not for grasping fragile objects.

    Args:
        base:     BaseClient on the TCP router. High-level commands require the arm
                  to be in single-level servoing (this function enforces it).
        position: target opening, 0-100 % (0 = fully open, 100 = fully closed).
                  The Kortex API uses a normalized 0.0-1.0 value; we convert here so
                  the interface matches the rest of this file's 0-100 % convention.
        timeout:  max seconds to wait for the gripper to reach the target.

    Returns:
        True if the gripper reached the target OR stopped on something (an object
        grasped / a hard stop); False only on the safety timeout.
    """
    position = max(0.0, min(100.0, float(position)))
    target = position / 100.0          # API expects 0.0 (open) .. 1.0 (closed)
    print("Moving gripper to {:.1f} % (0=open, 100=closed) ...".format(position))

    # High-level gripper commands require single-level servoing.
    set_single_level_servoing(base)

    # Build and send the position command (example 106: GRIPPER_POSITION mode).
    gripper_command = Base_pb2.GripperCommand()
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger = gripper_command.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = target
    base.SendGripperCommand(gripper_command)

    # Poll the measured position (example 106). Exit as soon as EITHER the target
    # is reached OR the fingers stop moving -- when closing onto an object wider
    # than the commanded gap, the fingers stall on the object and never reach the
    # target, so without stall detection this would block for the whole timeout.
    gripper_request = Base_pb2.GripperRequest()
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    POSITION_TOLERANCE = 0.01          # normalized (~1 %): close enough to target
    STALL_EPS = 0.005                  # normalized (~0.5 %): motion below this = stopped
    STALL_TIME = 0.4                   # s of no motion -> fingers have stopped

    start = time.time()
    last_position = None
    last_move_time = start
    while time.time() - start < timeout:
        measure = base.GetMeasuredGripperMovement(gripper_request)
        if not len(measure.finger):    # no finger reported -> nothing to wait on
            break
        current = measure.finger[0].value

        if abs(current - target) < POSITION_TOLERANCE:
            print("Gripper reached {:.1f} %.".format(current * 100.0))
            return True

        # Stall detection: once the fingers stop changing position they have either
        # grasped an object or hit a hard stop and will not reach the target -- stop
        # waiting instead of blocking until the timeout.
        now = time.time()
        if last_position is None or abs(current - last_position) >= STALL_EPS:
            last_position = current
            last_move_time = now
        elif now - last_move_time >= STALL_TIME:
            print("Gripper stopped at {:.1f} % (object grasped / hard stop)."
                  .format(current * 100.0))
            return True

        time.sleep(0.05)

    print("Timeout while moving gripper to {:.1f} %.".format(position))
    return False


def main():
    args = _connection_args()

    with utilities.DeviceConnection.createTcpConnection(args) as router:        # TCP: actions + mode
        with utilities.DeviceConnection.createUdpConnection(args) as router_rt:  # UDP: gripper cyclic
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            base_cyclic_rt = BaseCyclicClient(router_rt)

            # Show the current pose so the operator can sanity-check the frame.
            print_current_pose(base_cyclic)

            if PRINT_POSE_AND_EXIT:
                print("PRINT_POSE_AND_EXIT is set -- not moving. "
                      "Use the values above to fill POSE_A / POSE_B.")
                return 0

            success = True

            # High-level Cartesian moves require single-level servoing.
            set_single_level_servoing(base)

            # theta_y = -0.4 matches the proven-reachable pose from Kinova example
            # 102/01; at this workspace boundary even a ~0.4 deg orientation change
            # can flip the pose from reachable to IK-failure (which aborts the move).
            POSE_A = dict(x=0.645, y=0.012, z=0.184, theta_x=178, theta_y=-0.4, theta_z=87.4)  # pick
            POSE_B = dict(x=0.645, y=0.012, z=0.25, theta_x=178, theta_y=-0.4, theta_z=87.4)  # lift
            POSE_C = dict(x=0.478, y=-0.194, z=0.20, theta_x=0.5, theta_y=-177.8, theta_z=172.9)  # place
            POSE_D = dict(x=0.478, y=-0.194, z=0.056, theta_x=0.5, theta_y=-177.8, theta_z=172.9)  # ready for next

            # Fail fast: reject unreachable targets before any motion is commanded.
            for pose, label in ((POSE_A, "A (pick)"), (POSE_B, "B (lift)"), (POSE_C, "C (place)")):
                if not pose_is_reachable(base, base_cyclic, pose):
                    print("Pose {} is not reachable (IK pre-check failed). Aborting "
                          "before motion.".format(label))
                    return 1


            success &= move_to_home(base)

            # --- Previous discrete Cartesian moves (annotated; kept for reference) ---
            # Each step below moved to one pose via move_cartesian(). They are now
            # replaced by the waypoint trajectories below (110/02 style), which
            # validate and run each contiguous path as ONE continuous trajectory.
            #
            #   success &= move_cartesian(base, base_cyclic, POSE_B, label="B (ready)")
            #   success &= move_cartesian(base, base_cyclic, POSE_A, label="A (pick)")
            #   success &= move_gripper_to_position(base, 85.0)  # fully closed
            #   set_single_level_servoing(base)
            #   success &= move_cartesian(base, base_cyclic, POSE_B, label="B (ready)")
            #   success &= move_cartesian(base, base_cyclic, POSE_C, label="C (next ready)")
            #   success &= move_cartesian(base, base_cyclic, POSE_D, label="D (place)")
            #   success &= move_gripper_to_position(base, 0.0)  # fully open
            #   success &= move_cartesian(base, base_cyclic, POSE_C, label="C (next ready)")
            # ------------------------------------------------------------------------

            # 1) Approach + descend to the pick pose, as one trajectory (B -> A).
            success &= move_cartesian_waypoints(base, [
                (POSE_B, BLENDING_RADIUS),   # ready above the object
                (POSE_A, 0.0),               # descend onto the object (exact stop)
            ], label="pick approach (B->A)")

            # 2) Close the gripper on the object.
            success &= move_gripper_to_position(base, 85.0)  # grasp

            # 3) Lift, traverse, and descend to the place pose, as one trajectory.
            success &= move_cartesian_waypoints(base, [
                (POSE_B, BLENDING_RADIUS),   # lift straight up
                (POSE_C, BLENDING_RADIUS),   # ready above the place location
                (POSE_D, 0.0),               # descend to place (exact stop)
            ], label="carry + place (B->C->D)")

            # 4) Release the object.
            success &= move_gripper_to_position(base, 0.0)  # open

            # 5) Retreat above the place location, then go Home.
            success &= move_cartesian_waypoints(base, [
                (POSE_C, 0.0),               # retreat up (exact stop)
            ], label="retreat (C)")

            success &= move_to_home(base)

            print("Task completed successfully." if success
                  else "Task finished with one or more failures.")
            return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
