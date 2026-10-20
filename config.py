#! /usr/bin/env python3

###
# Simple-Manipulation -- shared configuration.
#
# Pure constants only (no Kortex imports), so every module and script can import
# this without side effects. Edit the values here, then run one of:
#   python3 pick_and_place_points.py      # discrete point-to-point moves (ex.102/01)
#   python3 pick_and_place_waypoints.py   # one continuous trajectory     (ex.110/02)
###

# Connection ------------------------------------------------------------------
ROBOT_IP = "192.168.2.10"
USERNAME = "admin"
PASSWORD = "admin"

# Gripper force ---------------------------------------------------------------
# 0-100 % force LIMIT applied while closing (used by close_gripper_with_force).
GRIPPER_FORCE = 20.0

# Joint speed -----------------------------------------------------------------
# example 102/04 style: per-joint angular speed (deg/s) for send_joint_speeds.
JOINT_SPEED = 20.0
JOINT_SPEED_DURATION = 2.0   # seconds to hold the speed command before Stop()

# Waypoint trajectory ---------------------------------------------------------
# Blending radius (m) for INTERMEDIATE waypoints of a Cartesian trajectory (110/02
# style). 0 = pass exactly through each waypoint (momentary stop); raise (e.g. 0.05)
# for smooth, non-stop motion. The final waypoint of each segment always uses 0 so
# the arm stops precisely on the pick/place pose.
BLENDING_RADIUS = 0.0

# Behaviour -------------------------------------------------------------------
PRINT_POSE_AND_EXIT = False  # just connect, print current pose, and exit (no motion)

# Maximum allowed waiting time during high-level actions (seconds)
TIMEOUT_DURATION = 30

# Target poses (ABSOLUTE Cartesian, robot base frame) -------------------------
#   x, y, z      in meters
#   theta_x/y/z  in degrees
#
#   These values can be regenerated from the CameraCalibration AprilTag pose
#   estimates by running:
#       python3 update_config_with_tag_positions.py
#
#   The updater detects pick/place pairs 1->2, 3->4, and 5->6, then writes
#   PICK_PLACE_TASKS for sequential point-to-point execution.
#
#   Targets must stay inside the arm's DEXTEROUS workspace, not just within max
#   reach: a pose can be < 0.9 m from the base yet still be unreachable because the
#   wrist cannot achieve the requested orientation there. Rule of thumb: the farther
#   out in x, the lower z must be (e.g. at x ~= 0.645, keep z <= ~0.25).
#
#   POSE_A uses the detected pick x/y/z. POSE_D uses placement-tag x/y/yaw but
#   preserves the pick tool Z because the flat placement tag's Z is the table
#   surface. POSE_B and POSE_C are 9 cm above their pick/place poses.
# BEGIN AUTO TAG TASKS
POSE_A = dict(x=0.433914, y=0.207461, z=0.055574, theta_x=178.0, theta_y=-0.4, theta_z=93.0)
POSE_B = dict(x=0.433914, y=0.207461, z=0.165574, theta_x=178.0, theta_y=-0.4, theta_z=93.0)
POSE_C = dict(x=0.398829, y=0.040169, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=70.1)
POSE_D = dict(x=0.398829, y=0.040169, z=0.085000, theta_x=178.0, theta_y=-0.4, theta_z=70.1)

PICK_PLACE_TASKS = [
    dict(
        pick_tag=1,
        place_tag=2,
        POSE_A=dict(x=0.433914, y=0.207461, z=0.055574, theta_x=178.0, theta_y=-0.4, theta_z=93.0),
        POSE_B=dict(x=0.433914, y=0.207461, z=0.165574, theta_x=178.0, theta_y=-0.4, theta_z=93.0),
        POSE_C=dict(x=0.398829, y=0.040169, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=70.1),
        POSE_D=dict(x=0.398829, y=0.040169, z=0.085000, theta_x=178.0, theta_y=-0.4, theta_z=70.1),
    ),
    dict(
        pick_tag=3,
        place_tag=4,
        POSE_A=dict(x=0.690852, y=-0.115961, z=0.046750, theta_x=178.0, theta_y=-0.4, theta_z=50.4),
        POSE_B=dict(x=0.690852, y=-0.115961, z=0.156750, theta_x=178.0, theta_y=-0.4, theta_z=50.4),
        POSE_C=dict(x=0.521274, y=-0.083684, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=80.7),
        POSE_D=dict(x=0.521274, y=-0.083684, z=0.085000, theta_x=178.0, theta_y=-0.4, theta_z=80.7),
    ),
    dict(
        pick_tag=5,
        place_tag=6,
        POSE_A=dict(x=0.645676, y=0.158129, z=0.050105, theta_x=178.0, theta_y=-0.4, theta_z=69.6),
        POSE_B=dict(x=0.645676, y=0.158129, z=0.160105, theta_x=178.0, theta_y=-0.4, theta_z=69.6),
        POSE_C=dict(x=0.535848, y=0.061320, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=79.1),
        POSE_D=dict(x=0.535848, y=0.061320, z=0.085000, theta_x=178.0, theta_y=-0.4, theta_z=79.1),
    ),
]
# END AUTO TAG TASKS
































