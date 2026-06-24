#! /usr/bin/env python3

###
# Simple-Manipulation -- pick and place using WAYPOINTS.
#
# Pick an object at POSE_A and place it at POSE_D, moving each contiguous path as ONE
# continuous, validated Cartesian trajectory (move_waypoints.py / example 110/02).
# See pick_and_place_points.py for the discrete point-to-point version.
#
#   python3 pick_and_place_waypoints.py
###

import sys

from config import POSE_A, POSE_B, POSE_C, POSE_D, PRINT_POSE_AND_EXIT, BLENDING_RADIUS
from robot_common import (
    DeviceConnection, BaseClient, BaseCyclicClient,
    connection_args, print_current_pose, pose_is_reachable,
    set_single_level_servoing, move_to_home, move_gripper_to_position,
    # close_gripper_with_force,  # optional force-limited grasp (needs base_cyclic_rt)
)
from move_waypoints import move_cartesian_waypoints


def main():
    args = connection_args()

    with DeviceConnection.createTcpConnection(args) as router:        # TCP: actions + mode
        with DeviceConnection.createUdpConnection(args) as router_rt:  # UDP: gripper cyclic
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            base_cyclic_rt = BaseCyclicClient(router_rt)  # for close_gripper_with_force

            # Show the current pose so the operator can sanity-check the frame.
            print_current_pose(base_cyclic)

            if PRINT_POSE_AND_EXIT:
                print("PRINT_POSE_AND_EXIT is set -- not moving.")
                return 0

            success = True

            # High-level Cartesian moves require single-level servoing.
            set_single_level_servoing(base)

            # Fail fast: reject unreachable targets before any motion is commanded.
            for pose, label in ((POSE_A, "A (pick)"), (POSE_B, "B (lift)"), (POSE_C, "C (place)")):
                if not pose_is_reachable(base, base_cyclic, pose):
                    print("Pose {} is not reachable (IK pre-check failed). Aborting "
                          "before motion.".format(label))
                    return 1

            success &= move_to_home(base)

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
