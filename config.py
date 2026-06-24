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
ROBOT_IP = "192.168.1.10"
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
#   Targets must stay inside the arm's DEXTEROUS workspace, not just within max
#   reach: a pose can be < 0.9 m from the base yet still be unreachable because the
#   wrist cannot achieve the requested orientation there. Rule of thumb: the farther
#   out in x, the lower z must be (e.g. at x ~= 0.645, keep z <= ~0.25).
#
#   theta_y = -0.4 matches the proven-reachable pose from Kinova example 102/01; at
#   this workspace boundary even a ~0.4 deg orientation change can flip the pose from
#   reachable to IK-failure (which aborts the move).
POSE_A = dict(x=0.645, y=0.012,  z=0.184, theta_x=178, theta_y=-0.4,   theta_z=87.4)   # pick (down on object)
POSE_B = dict(x=0.645, y=0.012,  z=0.25,  theta_x=178, theta_y=-0.4,   theta_z=87.4)   # ready/lift above pick
POSE_C = dict(x=0.478, y=-0.194, z=0.20,  theta_x=0.5, theta_y=-177.8, theta_z=172.9)  # ready above place
POSE_D = dict(x=0.478, y=-0.194, z=0.056, theta_x=0.5, theta_y=-177.8, theta_z=172.9)  # place (down)
