#! /usr/bin/env python3

###
# Simple-Manipulation -- pick-only test.
#
# Move the arm from a safe approach pose (POSE_B) down to the pick pose
# (POSE_A), close the gripper, and then lift back to POSE_B. This is useful
# for testing pick coordinates without executing the place sequence.
#
# Usage:
#   python3 pick_only_points.py
###

import sys

from config import POSE_A, POSE_B, PRINT_POSE_AND_EXIT
from robot_common import (
    DeviceConnection, BaseClient, BaseCyclicClient,
    connection_args, print_current_pose, pose_is_reachable,
    set_single_level_servoing, move_to_home, move_gripper_to_position,
)
from move_points import move_through_points


def main():
    args = connection_args()

    with DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        print_current_pose(base_cyclic)

        if PRINT_POSE_AND_EXIT:
            print("PRINT_POSE_AND_EXIT is set -- not moving.")
            return 0

        set_single_level_servoing(base)

        for pose, label in ((POSE_A, "A (pick)"), (POSE_B, "B (lift)")):
            if not pose_is_reachable(base, base_cyclic, pose):
                print(f"Pose {label} is not reachable. Aborting.")
                return 1

        success = True
        success &= move_to_home(base)
        if not success:
            print("Pick-only test stopped before approach.")
            return 1

        success &= move_through_points(base, base_cyclic, [POSE_B, POSE_A], label="pick approach (B->A)")
        if not success:
            print("Pick-only test stopped before grasp.")
            return 1

        success &= move_gripper_to_position(base, 85.0)
        if not success:
            print("Pick-only test stopped before lift.")
            return 1

        success &= move_through_points(base, base_cyclic, [POSE_B], label="lift to B")

        print("Pick-only test completed successfully." if success else "Pick-only test finished with one or more failures.")
        return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
