#! /usr/bin/env python3

###
# Simple-Manipulation -- shared robot helpers.
#
# Connection setup, action-notification helper, IK pre-check, servoing-mode helper,
# Home move, joint-speed command, and the two gripper functions. Imported by the
# movement modules (move_points, move_waypoints) and by the pick-and-place scripts.
#
# Gripper "force": the high-level GripperCommand only supports POSITION and SPEED
# modes (no force field). A user-defined force is applied through the low-level
# cyclic command, where MotorCommand.force is a 0-100 % force *limit* used while
# closing -- the gripper stops once that limit is exceeded.
###

import time
import threading

from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Session_pb2

from config import (ROBOT_IP, USERNAME, PASSWORD, TIMEOUT_DURATION,
                    JOINT_SPEED_DURATION)


class DeviceConnection:
    TCP_PORT = 10000
    UDP_PORT = 10001

    def __init__(self, ip_address, port=TCP_PORT, credentials=("admin", "admin"), use_udp=False):
        self.ip_address = ip_address
        self.port = port
        self.credentials = credentials
        self.transport = UDPTransport() if use_udp else TCPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
        self.session_manager = SessionManager(self.router)

    @staticmethod
    def createTcpConnection(args):
        return DeviceConnection(
            args.ip,
            port=DeviceConnection.TCP_PORT,
            credentials=(args.username, args.password),
            use_udp=False,
        )

    @staticmethod
    def createUdpConnection(args):
        return DeviceConnection(
            args.ip,
            port=DeviceConnection.UDP_PORT,
            credentials=(args.username, args.password),
            use_udp=True,
        )

    def __enter__(self):
        self.transport.connect(self.ip_address, self.port)

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = self.credentials[0]
        session_info.password = self.credentials[1]
        session_info.session_inactivity_timeout = 60000
        session_info.connection_inactivity_timeout = 2000

        self.session_manager.CreateSession(session_info)
        return self.router

    def __exit__(self, exc_type, exc_value, traceback):
        self.session_manager.CloseSession()
        self.transport.disconnect()


def connection_args():
    """Build the argument object DeviceConnection expects, from the config
    constants (so the scripts need no command-line arguments)."""
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


def print_action_control_error(ex):
    """Print a concise hint for common Kortex action-control failures."""
    message = str(ex).splitlines()[0]
    print("Action command failed: {}".format(message))
    if "SESSION_NOT_IN_CONTROL" in str(ex):
        print("The API session is connected but is not in control of the robot.")
        print("On the robot tablet/Web App, release or take control for this session,")
        print("close other control sessions, clear faults, then run the script again.")


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
    try:
        base.ExecuteActionFromReference(action_handle)
    except Exception as ex:
        base.Unsubscribe(notification_handle)
        print_action_control_error(ex)
        return False

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
                  the interface matches the rest of this project's 0-100 % convention.
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
    try:
        base.SendGripperCommand(gripper_command)
    except Exception as ex:
        print_action_control_error(ex)
        return False

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
        try:
            measure = base.GetMeasuredGripperMovement(gripper_request)
        except Exception as ex:
            print_action_control_error(ex)
            return False

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
