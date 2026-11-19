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
POSE_A = dict(x=0.404270, y=0.150682, z=0.048877, theta_x=178.0, theta_y=-0.4, theta_z=86.4)
POSE_B = dict(x=0.404270, y=0.150682, z=0.158877, theta_x=178.0, theta_y=-0.4, theta_z=86.4)
POSE_C = dict(x=0.508680, y=-0.057835, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=30.5)
POSE_D = dict(x=0.508680, y=-0.057835, z=0.065000, theta_x=178.0, theta_y=-0.4, theta_z=30.5)

PICK_PLACE_TASKS = [
    dict(
        pick_tag=1,
        place_tag=2,
        POSE_A=dict(x=0.404270, y=0.150682, z=0.048877, theta_x=178.0, theta_y=-0.4, theta_z=86.4),
        POSE_B=dict(x=0.404270, y=0.150682, z=0.158877, theta_x=178.0, theta_y=-0.4, theta_z=86.4),
        POSE_C=dict(x=0.508680, y=-0.057835, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=30.5),
        POSE_D=dict(x=0.508680, y=-0.057835, z=0.065000, theta_x=178.0, theta_y=-0.4, theta_z=30.5),
    ),
    dict(
        pick_tag=3,
        place_tag=4,
        POSE_A=dict(x=0.557339, y=0.106286, z=0.050021, theta_x=178.0, theta_y=-0.4, theta_z=62.2),
        POSE_B=dict(x=0.557339, y=0.106286, z=0.160021, theta_x=178.0, theta_y=-0.4, theta_z=62.2),
        POSE_C=dict(x=0.578489, y=-0.225526, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=-18.0),
        POSE_D=dict(x=0.578489, y=-0.225526, z=0.065000, theta_x=178.0, theta_y=-0.4, theta_z=-18.0),
    ),
    dict(
        pick_tag=5,
        place_tag=6,
        POSE_A=dict(x=0.671732, y=0.004628, z=0.052963, theta_x=178.0, theta_y=-0.4, theta_z=43.2),
        POSE_B=dict(x=0.671732, y=0.004628, z=0.162963, theta_x=178.0, theta_y=-0.4, theta_z=43.2),
        POSE_C=dict(x=0.401521, y=-0.117962, z=0.300000, theta_x=178.0, theta_y=-0.4, theta_z=35.5),
        POSE_D=dict(x=0.401521, y=-0.117962, z=0.065000, theta_x=178.0, theta_y=-0.4, theta_z=35.5),
    ),
]
# END AUTO TAG TASKS
































