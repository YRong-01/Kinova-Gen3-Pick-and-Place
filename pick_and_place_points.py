#! /usr/bin/env python3

###
# Simple-Manipulation -- pick and place using POINTS.
#
# Execute every entry in PICK_PLACE_TASKS as discrete point-to-point Cartesian
# moves. The arm fully reaches and stops at each pose.
#
#   python3 pick_and_place_points.py
###

import sys

from config import PICK_PLACE_TASKS, PRINT_POSE_AND_EXIT
from robot_common import (
    DeviceConnection, BaseClient, BaseCyclicClient,
    connection_args, print_current_pose, pose_is_reachable,
    set_single_level_servoing, move_to_home, move_gripper_to_position,
    # close_gripper_with_force,  # optional force-limited grasp (needs base_cyclic_rt)
)
from move_points import move_through_points


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

            if not PICK_PLACE_TASKS:
                print(
                    "PICK_PLACE_TASKS is empty. Run "
                    "update_config_with_tag_positions.py first."
                )
                return 1

            # High-level Cartesian moves require single-level servoing.
            set_single_level_servoing(base)

            # Validate every endpoint before moving any block.
            for task_number, task in enumerate(PICK_PLACE_TASKS, start=1):
                pair = "{}->{}".format(task["pick_tag"], task["place_tag"])
                for pose_name in ("POSE_A", "POSE_B", "POSE_C", "POSE_D"):
                    if not pose_is_reachable(base, base_cyclic, task[pose_name]):
                        print(
                            "Task {} ({}) {} is not reachable. Aborting before "
                            "motion.".format(task_number, pair, pose_name)
                        )
                        return 1

            if not move_to_home(base):
                print("Could not reach Home before starting the tasks.")
                return 1

            for task_number, task in enumerate(PICK_PLACE_TASKS, start=1):
                pair = "{}->{}".format(task["pick_tag"], task["place_tag"])
                print(
                    "\n=== Task {}/{}: tag {} ===".format(
                        task_number, len(PICK_PLACE_TASKS), pair
                    )
                )

                if not move_through_points(
                    base,
                    base_cyclic,
                    [task["POSE_B"], task["POSE_A"]],
                    label="{} pick approach (B->A)".format(pair),
                ):
                    print("Task {} stopped before grasp.".format(pair))
                    return 1

                if not move_gripper_to_position(base, 85.0):
                    print("Task {} grasp failed.".format(pair))
                    return 1

                if not move_through_points(
                    base,
                    base_cyclic,
                    [task["POSE_B"], task["POSE_C"], task["POSE_D"]],
                    label="{} carry + place (B->C->D)".format(pair),
                ):
                    print("Task {} stopped before release.".format(pair))
                    return 1

                if not move_gripper_to_position(base, 0.0):
                    print("Task {} release failed.".format(pair))
                    return 1

                if not move_through_points(
                    base,
                    base_cyclic,
                    [task["POSE_C"]],
                    label="{} retreat (C)".format(pair),
                ):
                    print("Task {} retreat failed.".format(pair))
                    return 1

                print("Task {} completed successfully.".format(pair))

            if not move_to_home(base):
                print("All tasks finished, but the final return Home failed.")
                return 1

            print("All pick-and-place tasks completed successfully.")
            return 0


if __name__ == "__main__":
    sys.exit(main())
