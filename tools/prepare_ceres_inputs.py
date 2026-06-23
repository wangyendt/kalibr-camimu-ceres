#!/usr/bin/env python3
"""Prepare ceres_cam_imu CSV inputs from pkl, bag, or EuRoC-style data.

The C++ calibration binary intentionally consumes neutral CSV files.  This
wrapper runs Kalibr Docker for Kalibr pickle objects and ROS bags.  EuRoC image
folders are exported natively by default through cpp_tools apriltag detection;
the old Kalibr Docker EuRoC path remains available as a baseline backend.
"""

import argparse
import pathlib
import re
import shlex
import subprocess
import sys


KALIBR_LOCAL_IMAGE = "kalibr-camera-calibration:20.04"
KALIBR_DOCKERHUB_IMAGE = "wang121ye/kalibr-camera-calibration:20.04"
KALIBR_IMAGE = KALIBR_LOCAL_IMAGE

DEFAULT_CALIBRATION_FLAGS = [
    "--corner-defaults",
    "--estimate-time-shift-prior",
    "--estimate-orientation-gravity-prior",
    "--pose-fit-boundary-anchors",
    "--pose-motion-prior",
]

DEFAULT_CALIBRATION_VALUES = [
    ("--pose-fit-motion-lambda", "0.0001"),
    ("--time-shift-prior-sigma", "0.0001"),
    ("--pose-motion-translation-variance", "10"),
    ("--pose-motion-rotation-variance", "1"),
]


def repo_dir():
    return pathlib.Path(__file__).resolve().parents[1]


def quote_command(command):
    return " ".join(shlex.quote(str(part)) for part in command)


def run(command, print_only=False):
    print(quote_command(command), flush=True)
    if print_only:
        return 0
    return subprocess.call(command)


def docker_image_exists(image):
    # Use `docker images` rather than `docker image inspect <tag>`: a corrupted
    # reference store can break the tag->id lookup that `inspect` relies on, so
    # a locally-present image would be misreported as missing (and then a doomed
    # pull of a local-only image would be attempted).
    result = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode != 0:
        return False
    return image in {line.strip() for line in result.stdout.splitlines()}


def resolve_kalibr_image(args):
    if args.print_only:
        if args.image == KALIBR_LOCAL_IMAGE:
            return args.remote_image
        return args.image
    if docker_image_exists(args.image):
        return args.image
    pull_image = args.remote_image if args.image == KALIBR_LOCAL_IMAGE else args.image
    print(f"local Docker image not found: {args.image}; pulling {pull_image}", flush=True)
    rc = run(["docker", "pull", pull_image], print_only=False)
    if rc != 0:
        raise RuntimeError(f"failed to pull Docker image: {pull_image}")
    return pull_image


def has_option(tokens, option):
    prefix = option + "="
    return option in tokens or any(token.startswith(prefix) for token in tokens)


def default_calibration_args(passthrough_args):
    defaults = []
    for flag in DEFAULT_CALIBRATION_FLAGS:
        if not has_option(passthrough_args, flag):
            defaults.append(flag)
    for option, value in DEFAULT_CALIBRATION_VALUES:
        if not has_option(passthrough_args, option):
            defaults.extend([option, value])
    return defaults


def add_path_mounts(command, paths):
    """Append read-only parent mounts and return container paths."""
    mapped = []
    parent_to_mount = {}
    for path in paths:
        resolved = pathlib.Path(path).expanduser().resolve()
        parent = resolved.parent
        if parent not in parent_to_mount:
            mount_name = f"/mnt/input{len(parent_to_mount)}"
            parent_to_mount[parent] = mount_name
            command.extend(["-v", f"{parent}:{mount_name}:ro"])
        mapped.append(parent_to_mount[parent] + "/" + resolved.name)
    return mapped


def base_docker_command(args, out_dir):
    command = [
        "docker",
        "run",
        "--rm",
        "--platform",
        args.platform,
        "-v",
        f"{repo_dir()}:/repo:ro",
        "-v",
        f"{out_dir}:/out",
        "-w",
        "/out",
    ]
    return command


def prepare_pkl(args):
    if not args.corner_pkl:
        raise ValueError("--corner-pkl is required for --source-type pkl")
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    image = resolve_kalibr_image(args)
    for index, corner_pkl in enumerate(args.corner_pkl):
        command = base_docker_command(args, out_dir)
        (container_pkl,) = add_path_mounts(command, [corner_pkl])
        command.extend([image, "/bin/bash", "-lc"])
        corners_name = f"cam{index}_corners.csv"
        script = [
            "python3",
            "/repo/tools/export_kalibr_corners.py",
            "--corner-pkl",
            container_pkl,
            "--corners-csv",
            f"/out/{corners_name}",
        ]
        if index == 0:
            script.extend(["--poses-csv", "/out/cam0_corner_poses.csv"])
        command.append(quote_command(script))
        rc = run(command, args.print_only)
        if rc != 0:
            return rc
    return 0


def prepare_bag(args):
    for name, value in [
        ("--bag", args.bag),
        ("--cams", args.cams),
        ("--imu", args.imu),
        ("--target", args.target),
    ]:
        if not value:
            raise ValueError(f"{name} is required for --source-type bag")
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    image = resolve_kalibr_image(args)
    command = base_docker_command(args, out_dir)
    container_bag, container_cams, container_imu, container_target = add_path_mounts(
        command, [args.bag, args.cams, args.imu, args.target]
    )
    command.extend([image, "/bin/bash", "-lc"])
    script = [
        "python3",
        "/repo/tools/export_kalibr_bag_to_ceres.py",
        "--bag",
        container_bag,
        "--cams",
        container_cams,
        "--imu",
        container_imu,
        "--target",
        container_target,
        "--out-dir",
        "/out",
    ]
    if args.bag_from_to:
        script.extend(["--bag-from-to", str(args.bag_from_to[0]), str(args.bag_from_to[1])])
    if args.bag_freq is not None:
        script.extend(["--bag-freq", str(args.bag_freq)])
    if args.perform_synchronization:
        script.append("--perform-synchronization")
    command.append(quote_command(script))
    return run(command, args.print_only)


def validate_euroc_args(args):
    for name, value in [
        ("--euroc-dir", args.euroc_dir),
        ("--cams", args.cams),
        ("--imu", args.imu),
        ("--target", args.target),
    ]:
        if not value:
            raise ValueError(f"{name} is required for --source-type euroc")
    euroc_dir = pathlib.Path(args.euroc_dir).expanduser().resolve()
    mav0 = euroc_dir / "mav0"
    if not mav0.is_dir():
        mav0 = euroc_dir
    if not (mav0 / "cam0" / "data").is_dir():
        raise ValueError(f"EuRoC cam0 data directory not found under {mav0}")
    if not (mav0 / "imu0" / "data.csv").is_file():
        raise ValueError(f"EuRoC imu0/data.csv not found under {mav0}")
    return mav0


def prepare_euroc_native(args):
    validate_euroc_args(args)
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(repo_dir() / "tools" / "export_euroc_native_to_ceres.py"),
        "--euroc-dir",
        str(pathlib.Path(args.euroc_dir).expanduser().resolve()),
        "--cams",
        str(pathlib.Path(args.cams).expanduser().resolve()),
        "--target",
        str(pathlib.Path(args.target).expanduser().resolve()),
        "--out-dir",
        str(out_dir),
        "--cpp-tools-root",
        str(pathlib.Path(args.cpp_tools_root).expanduser().resolve()),
        "--tag-family",
        args.tag_family,
        "--tag-black-border",
        str(args.tag_black_border),
        "--max-hamming",
        str(args.max_hamming),
        "--imu-index",
        str(args.imu_index),
        "--min-pnp-corners",
        str(args.min_pnp_corners),
        "--corner-id-mode",
        args.corner_id_mode,
    ]
    if args.max_extraction_frames:
        command.extend(["--max-frames", str(args.max_extraction_frames)])
    if args.corner_permutation:
        command.append("--corner-permutation")
        command.extend(str(value) for value in args.corner_permutation)
    return run(command, args.print_only)


def prepare_euroc_kalibr_docker(args):
    mav0 = validate_euroc_args(args)
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    image = resolve_kalibr_image(args)
    command = base_docker_command(args, out_dir)
    container_mav0, container_cams, container_imu, container_target = add_path_mounts(
        command, [mav0, args.cams, args.imu, args.target]
    )
    command.extend([image, "/bin/bash", "-lc"])
    bag_name = pathlib.PurePosixPath(args.output_bag or "euroc_input.bag").name
    cam1_stage = ""
    if (mav0 / "cam1" / "data").is_dir():
        cam1_stage = "mkdir -p /out/euroc_stage/cam1; ln -sf {}/cam1/data/* /out/euroc_stage/cam1; ".format(
            shlex.quote(container_mav0)
        )
    export_args = [
        "python3",
        "/repo/tools/export_kalibr_bag_to_ceres.py",
        "--bag",
        f"/out/{bag_name}",
        "--cams",
        container_cams,
        "--imu",
        container_imu,
        "--target",
        container_target,
        "--out-dir",
        "/out",
    ]
    if args.bag_from_to:
        export_args.extend(["--bag-from-to", str(args.bag_from_to[0]), str(args.bag_from_to[1])])
    if args.bag_freq is not None:
        export_args.extend(["--bag-freq", str(args.bag_freq)])
    if args.perform_synchronization:
        export_args.append("--perform-synchronization")
    script = (
        "set -e; "
        "rm -rf /out/euroc_stage; "
        "mkdir -p /out/euroc_stage/cam0; "
        f"ln -sf {shlex.quote(container_mav0)}/cam0/data/* /out/euroc_stage/cam0; "
        + cam1_stage
        + f"cp {shlex.quote(container_mav0)}/imu0/data.csv /out/euroc_stage/imu0.csv; "
        + f"/catkin_ws/devel/lib/kalibr/kalibr_bagcreater --folder /out/euroc_stage --output-bag /out/{shlex.quote(bag_name)}; "
        + quote_command(export_args)
    )
    command.append(script)
    return run(command, args.print_only)


def prepare_euroc(args):
    if args.euroc_backend == "native":
        return prepare_euroc_native(args)
    if args.euroc_backend == "kalibr-docker":
        return prepare_euroc_kalibr_docker(args)
    raise ValueError(f"unsupported EuRoC backend: {args.euroc_backend}")


def camera_count_from_camchain(cams_path):
    path = pathlib.Path(cams_path).expanduser().resolve()
    if not path.is_file():
        return 1
    max_index = -1
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^cam([0-9]+):\s*$", line)
        if match:
            max_index = max(max_index, int(match.group(1)))
    return max_index + 1 if max_index >= 0 else 1


def expected_corner_count(args):
    if args.source_type == "pkl" and args.corner_pkl:
        return len(args.corner_pkl)
    return camera_count_from_camchain(args.cams)


def generated_corners(out_dir, count):
    return [out_dir / f"cam{index}_corners.csv" for index in range(count)]


def require_generated(path, label, print_only):
    if not print_only and not path.is_file():
        raise FileNotFoundError(f"{label} not found after conversion: {path}")
    return path


def run_calibration(args, passthrough_args):
    for name, value in [
        ("--cams", args.cams),
        ("--imu", args.imu),
        ("--target", args.target),
    ]:
        if not value:
            raise ValueError(f"{name} is required with --run-calibration")

    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    imu_data = out_dir / "imu.csv"
    if args.source_type == "pkl":
        if not args.imu_data:
            raise ValueError(
                "--imu-data is required with --source-type pkl --run-calibration"
            )
        imu_data = pathlib.Path(args.imu_data).expanduser().resolve()
    require_generated(imu_data, "IMU CSV", args.print_only)

    corner_count = expected_corner_count(args)
    corner_paths = generated_corners(out_dir, corner_count)
    for index, corner_path in enumerate(corner_paths):
        require_generated(corner_path, f"camera {index} corners CSV", args.print_only)
    corner_poses = require_generated(
        out_dir / "cam0_corner_poses.csv", "cam0 corner poses CSV", args.print_only
    )

    output_result = pathlib.Path(args.output_result).expanduser().resolve()
    if not args.print_only:
        output_result.parent.mkdir(parents=True, exist_ok=True)

    command = [
        str(pathlib.Path(args.calibrate_bin).expanduser().resolve()),
        "--cam",
        str(pathlib.Path(args.cams).expanduser().resolve()),
        "--imu",
        str(pathlib.Path(args.imu).expanduser().resolve()),
        "--target",
        str(pathlib.Path(args.target).expanduser().resolve()),
        "--imu-data",
        str(imu_data),
    ]
    for corner_path in corner_paths:
        command.extend(["--corners", str(corner_path)])
    if corner_count > 1 and "--init-from-camchain" not in passthrough_args:
        command.append("--init-from-camchain")
    command.extend(
        [
            "--corner-poses",
            str(corner_poses),
            "--output-result",
            str(output_result),
        ]
    )
    if not args.no_default_calibration_args:
        command.extend(default_calibration_args(passthrough_args))
    command.extend(passthrough_args)
    return run(command, args.print_only)


def run_two_stage_calibration(args, passthrough_args):
    for name, value in [
        ("--cams", args.cams),
        ("--imu", args.imu),
        ("--target", args.target),
    ]:
        if not value:
            raise ValueError(f"{name} is required with --run-two-stage")

    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    imu_data = out_dir / "imu.csv"
    if args.source_type == "pkl":
        if not args.imu_data:
            raise ValueError(
                "--imu-data is required with --source-type pkl --run-two-stage"
            )
        imu_data = pathlib.Path(args.imu_data).expanduser().resolve()
    require_generated(imu_data, "IMU CSV", args.print_only)

    corner_count = expected_corner_count(args)
    corner_paths = generated_corners(out_dir, corner_count)
    for index, corner_path in enumerate(corner_paths):
        require_generated(corner_path, f"camera {index} corners CSV", args.print_only)
    corner_poses = require_generated(
        out_dir / "cam0_corner_poses.csv", "cam0 corner poses CSV", args.print_only
    )

    command = [
        sys.executable,
        str(pathlib.Path(args.two_stage_bin).expanduser().resolve()),
        "--cam",
        str(pathlib.Path(args.cams).expanduser().resolve()),
        "--imu",
        str(pathlib.Path(args.imu).expanduser().resolve()),
        "--target",
        str(pathlib.Path(args.target).expanduser().resolve()),
        "--imu-data",
        str(imu_data),
    ]
    for corner_path in corner_paths:
        command.extend(["--corners", str(corner_path)])
    if corner_count > 1 and "--init-from-camchain" not in passthrough_args:
        command.append("--init-from-camchain")
    command.extend(
        [
            "--corner-poses",
            str(corner_poses),
            "--out-dir",
            str(out_dir / "two_stage"),
            "--calibrate-bin",
            str(pathlib.Path(args.calibrate_bin).expanduser().resolve()),
        ]
    )
    command.extend(passthrough_args)
    if args.print_only:
        command.append("--print-only")
    return run(command, args.print_only)


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    passthrough_args = []
    if "--" in argv:
        split_index = argv.index("--")
        passthrough_args = argv[split_index + 1 :]
        argv = argv[:split_index]

    parser = argparse.ArgumentParser(
        description=(
            "Convert pkl, bag, or EuRoC-style datasets to Ceres CSV inputs, "
            "optionally followed by native Ceres calibration."
        )
    )
    parser.add_argument("--source-type", choices=["pkl", "bag", "euroc"], required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--image", default=KALIBR_IMAGE)
    parser.add_argument(
        "--remote-image",
        default=KALIBR_DOCKERHUB_IMAGE,
        help=(
            "DockerHub image to pull when the default local Kalibr image is absent"
        ),
    )
    parser.add_argument("--platform", default="linux/amd64")
    parser.add_argument("--print-only", action="store_true")
    parser.add_argument("--corner-pkl", action="append", default=[])
    parser.add_argument("--bag")
    parser.add_argument("--euroc-dir")
    parser.add_argument("--output-bag")
    parser.add_argument(
        "--euroc-backend",
        choices=["native", "kalibr-docker"],
        default="native",
        help="EuRoC converter backend; native avoids Kalibr Docker",
    )
    parser.add_argument(
        "--cpp-tools-root",
        default=str(repo_dir() / "third_party" / "cpp_tools"),
        help="cpp_tools checkout used by the native EuRoC converter",
    )
    parser.add_argument("--tag-family", default="tag36h11")
    parser.add_argument("--tag-black-border", type=int, default=2)
    parser.add_argument("--max-hamming", type=int, default=-1)
    parser.add_argument("--imu-index", type=int, default=0)
    parser.add_argument("--max-extraction-frames", type=int, default=0)
    parser.add_argument("--min-pnp-corners", type=int, default=12)
    parser.add_argument("--corner-permutation", type=int, nargs=4, default=[0, 1, 2, 3])
    parser.add_argument("--corner-id-mode", choices=["kalibr", "tag"], default="kalibr")
    parser.add_argument("--cams")
    parser.add_argument("--imu")
    parser.add_argument("--imu-data")
    parser.add_argument("--target")
    parser.add_argument("--bag-from-to", type=float, nargs=2)
    parser.add_argument("--bag-freq", type=float)
    parser.add_argument("--perform-synchronization", action="store_true")
    parser.add_argument(
        "--run-calibration",
        action="store_true",
        help=(
            "after conversion, run native build/calibrate_cam_imu "
            "with the production single-stage defaults"
        ),
    )
    parser.add_argument(
        "--no-default-calibration-args",
        action="store_true",
        help="do not add the production single-stage defaults before passthrough args",
    )
    parser.add_argument(
        "--run-two-stage",
        action="store_true",
        help="after conversion, run the two-stage Ceres diagnostic workflow",
    )
    parser.add_argument(
        "--calibrate-bin",
        default=str(repo_dir() / "build" / "calibrate_cam_imu"),
    )
    parser.add_argument(
        "--two-stage-bin",
        default=str(repo_dir() / "tools" / "run_ceres_two_stage.py"),
    )
    parser.add_argument("--output-result")
    args = parser.parse_args(argv)
    if args.run_calibration and args.run_two_stage:
        parser.error("--run-calibration and --run-two-stage are mutually exclusive")
    if args.output_result is None:
        args.output_result = str(pathlib.Path(args.out_dir) / "result.yaml")
    args.passthrough_args = passthrough_args
    return args


def main():
    args = parse_args()
    try:
        if args.source_type == "pkl":
            rc = prepare_pkl(args)
        elif args.source_type == "bag":
            rc = prepare_bag(args)
        elif args.source_type == "euroc":
            rc = prepare_euroc(args)
        else:
            raise ValueError(f"unsupported source type: {args.source_type}")
        if rc != 0:
            return rc
        if args.run_two_stage:
            return run_two_stage_calibration(args, args.passthrough_args)
        if args.run_calibration:
            return run_calibration(args, args.passthrough_args)
        return rc
    except Exception as exc:
        print(f"prepare_ceres_inputs failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
