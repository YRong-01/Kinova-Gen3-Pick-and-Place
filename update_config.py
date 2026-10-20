#! /usr/bin/env python3

"""Update three AprilTag pick/place tasks in config.py."""

import argparse
import importlib.util
import re
from pathlib import Path


CONFIG_PATH = Path(__file__).resolve().parent / "config.py"
APRILTAG_SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "Camera"
    / "CameraCalibration"
    / "4AprilTag_position.py"
)
TAG_PAIRS = ((1, 2), (3, 4), (5, 6))
MANAGED_BLOCK_PATTERN = re.compile(
    r"^# BEGIN AUTO TAG TASKS$.*?^# END AUTO TAG TASKS$",
    re.MULTILINE | re.DOTALL,
)

TOOL_THETA_X = 178.0
TOOL_THETA_Y = -0.4
TAG_TO_GRIPPER_YAW = 90.0


class ConfigUpdateError(Exception):
    pass


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise ConfigUpdateError(f"Could not load module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_apriltag_module():
    if not APRILTAG_SCRIPT_PATH.exists():
        raise ConfigUpdateError(f"AprilTag script not found: {APRILTAG_SCRIPT_PATH}")
    module = load_module(APRILTAG_SCRIPT_PATH, "apriltag_pose_module")
    if not hasattr(module, "find_tag_poses"):
        raise ConfigUpdateError("4AprilTag_position.py has no find_tag_poses()")
    return module


def gripper_yaw(tag_result) -> float:
    """Return tag Z yaw plus 90 degrees, normalized to [-180, 180)."""
    tag_yaw = float(tag_result["euler_deg"][2])
    return (tag_yaw + TAG_TO_GRIPPER_YAW + 180.0) % 360.0 - 180.0


def tag_pose(tag_result) -> dict:
    x, y, z = (float(value) for value in tag_result["pos"])
    return dict(
        x=x,
        y=y,
        z=z,
        theta_x=TOOL_THETA_X,
        theta_y=TOOL_THETA_Y,
        theta_z=gripper_yaw(tag_result),
    )


def build_task(
    pick_tag: int,
    place_tag: int,
    pick_result,
    place_result,
    lift_height: float,
    place_approach_z: float,
    place_z: float,
) -> dict:
    """Build A/B from the pick tag and C/D X/Y/yaw from the place tag."""
    pose_a = tag_pose(pick_result)
    pose_b = dict(pose_a)
    pose_b["z"] += lift_height

    destination = tag_pose(place_result)
    pose_c = dict(destination)
    pose_c["z"] = place_approach_z
    pose_d = dict(destination)
    pose_d["z"] = place_z

    return dict(
        pick_tag=pick_tag,
        place_tag=place_tag,
        POSE_A=pose_a,
        POSE_B=pose_b,
        POSE_C=pose_c,
        POSE_D=pose_d,
    )


def format_pose(pose: dict) -> str:
    return (
        "dict("
        f"x={pose['x']:.6f}, y={pose['y']:.6f}, z={pose['z']:.6f}, "
        f"theta_x={pose['theta_x']:.1f}, "
        f"theta_y={pose['theta_y']:.1f}, "
        f"theta_z={pose['theta_z']:.1f})"
    )


def build_managed_block(tasks: list[dict]) -> str:
    first = tasks[0]
    lines = ["# BEGIN AUTO TAG TASKS"]
    for name in ("POSE_A", "POSE_B", "POSE_C", "POSE_D"):
        lines.append(f"{name} = {format_pose(first[name])}")

    lines.extend(("", "PICK_PLACE_TASKS = ["))
    for task in tasks:
        lines.extend((
            "    dict(",
            f"        pick_tag={task['pick_tag']},",
            f"        place_tag={task['place_tag']},",
            f"        POSE_A={format_pose(task['POSE_A'])},",
            f"        POSE_B={format_pose(task['POSE_B'])},",
            f"        POSE_C={format_pose(task['POSE_C'])},",
            f"        POSE_D={format_pose(task['POSE_D'])},",
            "    ),",
        ))
    lines.extend(("]", "# END AUTO TAG TASKS"))
    return "\n".join(lines)


def update_config_file(block: str) -> None:
    text = CONFIG_PATH.read_text()
    match = MANAGED_BLOCK_PATTERN.search(text)
    if match is None:
        raise ConfigUpdateError("Managed pose block was not found in config.py")
    CONFIG_PATH.write_text(text[:match.start()] + block + text[match.end():])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture AprilTag mappings 1->2, 3->4, and 5->6",
    )
    parser.add_argument("--camera-index", type=int, default=4)
    parser.add_argument("--lift-height", type=float, default=0.11) # lift height 11cm after piicking up
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.lift_height <= 0.0:
        raise ConfigUpdateError("--lift-height must be greater than zero")

    current_config = load_module(CONFIG_PATH, "current_manipulation_config")
    place_approach_z = float(current_config.POSE_C["z"])
    place_z = float(current_config.POSE_D["z"])

    required_ids = sorted({tag_id for pair in TAG_PAIRS for tag_id in pair})
    print(f"Capturing AprilTags {required_ids} with camera {args.camera_index}...")
    results = load_apriltag_module().find_tag_poses(
        tag_ids=required_ids,
        camera_index=args.camera_index,
        timeout_seconds=args.timeout,
    )

    missing_ids = [tag_id for tag_id in required_ids if tag_id not in results]
    if missing_ids:
        raise ConfigUpdateError(f"Could not detect AprilTag IDs: {missing_ids}")

    tasks = [
        build_task(
            pick_tag,
            place_tag,
            results[pick_tag],
            results[place_tag],
            args.lift_height,
            place_approach_z,
            place_z,
        )
        for pick_tag, place_tag in TAG_PAIRS
    ]
    block = build_managed_block(tasks)

    for task in tasks:
        print(
            f"{task['pick_tag']} -> {task['place_tag']}: "
            f"pick=({task['POSE_A']['x']:.3f}, {task['POSE_A']['y']:.3f}), "
            f"place=({task['POSE_D']['x']:.3f}, {task['POSE_D']['y']:.3f})"
        )
    print("\n=== New multi-block config ===\n")
    print(block)

    if args.dry_run:
        return 0
    update_config_file(block)
    print(f"Updated {CONFIG_PATH} with three pick/place tasks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
