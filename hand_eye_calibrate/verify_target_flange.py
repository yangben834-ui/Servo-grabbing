#!/usr/bin/env python3
"""Use calibrated RGBD coordinates to verify a target with the Piper flange."""

import argparse
import os
import time
from platform import system

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config


def load_calibration(calib_dir):
    with open(os.path.join(calib_dir, "camera_intrinsics.yaml"), encoding="utf-8") as f:
        intrinsics = yaml.safe_load(f)
    with open(os.path.join(calib_dir, "hand_eye_extrinsics.yaml"), encoding="utf-8") as f:
        extrinsics = yaml.safe_load(f)

    convention = extrinsics.get("metadata", {}).get("frame_convention")
    if convention != "camera_to_base":
        raise RuntimeError(f"需要 camera_to_base 外参，当前为: {convention!r}")

    K = np.asarray(intrinsics["K"], dtype=np.float64)
    D = np.asarray(intrinsics["D"], dtype=np.float64).reshape(-1)
    T_camera_base = np.asarray(extrinsics["transform_matrix"], dtype=np.float64)
    if K.shape != (3, 3) or T_camera_base.shape != (4, 4):
        raise RuntimeError("标定 YAML 中的矩阵尺寸不正确。")
    return K, D, T_camera_base


def capture_aligned_rgbd(width, height, fps, warmup_frames):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
    try:
        color_image = None
        depth_image = None
        for _ in range(max(1, warmup_frames)):
            aligned = align.process(pipeline.wait_for_frames())
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data()).copy()
                depth_image = np.asanyarray(depth_frame.get_data()).copy()
        if color_image is None or depth_image is None:
            raise RuntimeError("未获取到有效的 RealSense RGBD 帧。")
        return color_image, depth_image, depth_scale
    finally:
        pipeline.stop()


def parse_pixel(value):
    parts = value.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("像素格式应为 u,v")
    return np.asarray([float(parts[0]), float(parts[1])], dtype=np.float64)


def select_pixel(image):
    selected = {"pixel": None}
    window = "click target"

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            selected["pixel"] = np.asarray([x, y], dtype=np.float64)

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    print("请在图像中单击目标点；按 Enter 确认，按 Esc/q 取消。")
    while True:
        view = image.copy()
        if selected["pixel"] is not None:
            u, v = np.round(selected["pixel"]).astype(int)
            cv2.drawMarker(view, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.imshow(window, view)
        key = cv2.waitKey(20) & 0xFF
        if key in (10, 13) and selected["pixel"] is not None:
            break
        if key in (27, ord("q")):
            cv2.destroyAllWindows()
            raise RuntimeError("用户取消选择目标点。")
    cv2.destroyAllWindows()
    return selected["pixel"]


def depth_at_pixel(depth_image, pixel, depth_scale, window):
    u, v = np.round(pixel).astype(int)
    height, width = depth_image.shape
    if not (0 <= u < width and 0 <= v < height):
        raise RuntimeError(f"像素超出图像范围: {(u, v)}")
    half = max(0, window // 2)
    patch = depth_image[
        max(0, v - half):min(height, v + half + 1),
        max(0, u - half):min(width, u + half + 1),
    ]
    values = patch[(patch > 0) & (patch < 65535)].astype(np.float64)
    if values.size == 0:
        raise RuntimeError("目标点附近没有有效深度。")
    return float(np.median(values) * depth_scale)


def pixel_to_camera(pixel, depth_m, K, D):
    ray = cv2.undistortPoints(np.asarray([[pixel]], dtype=np.float64), K, D).reshape(2)
    return np.asarray([ray[0] * depth_m, ray[1] * depth_m, depth_m], dtype=np.float64)


def camera_to_base(point_camera, transform):
    point = np.append(np.asarray(point_camera, dtype=np.float64), 1.0)
    return (transform @ point)[:3]


def create_robot():
    platform_name = system()
    if platform_name == "Linux":
        interface, channel = "socketcan", "can0"
    elif platform_name == "Windows":
        interface, channel = "agx_cando", "0"
    elif platform_name == "Darwin":
        interface, channel = "slcan", "/dev/ttyACM0"
    else:
        raise RuntimeError(f"不支持的平台: {platform_name}")
    config = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V183,
        interface=interface,
        channel=channel,
    )
    return AgxArmFactory.create_arm(config)


def get_message(robot, getter_name, retries=30):
    getter = getattr(robot, getter_name)
    for _ in range(retries):
        value = getter()
        if value is not None:
            return list(value.msg)
        time.sleep(0.1)
    raise RuntimeError(f"无法读取 {getter_name}")


def wait_until_enabled(robot, timeout=10.0):
    start = time.monotonic()
    last_states = None
    while time.monotonic() - start <= timeout:
        if robot.enable():
            states = robot.get_joints_enable_status_list()
            print("六轴使能状态:", states)
            return
        last_states = robot.get_joints_enable_status_list()
        time.sleep(0.05)
    status = robot.get_arm_status()
    raise RuntimeError(
        f"机械臂使能超时，六轴使能状态={last_states}, "
        f"arm_status={None if status is None else status.msg}"
    )


def wait_motion_done(robot, label, target_pose, start_pose, timeout=30.0):
    time.sleep(0.5)
    start = time.monotonic()
    motion_started = False
    last_status = None
    last_pose = None
    while time.monotonic() - start <= timeout:
        status = robot.get_arm_status()
        if status is not None:
            last_status = status.msg
            if status.msg.arm_status != robot.ARM_STATUS.ArmStatus.NORMAL:
                raise RuntimeError(
                    f"{label} 异常停止，arm_status={status.msg.arm_status}, "
                    f"err_status={status.msg.err_status}"
                )

        pose_message = robot.get_flange_pose()
        if pose_message is not None:
            last_pose = np.asarray(pose_message.msg, dtype=np.float64)
            start_error = np.linalg.norm(last_pose[:3] - np.asarray(start_pose[:3]))
            target_error = np.linalg.norm(last_pose[:3] - np.asarray(target_pose[:3]))
            motion_started = motion_started or start_error > 0.001
            if target_error <= 0.002:
                print(f"{label} 已完成")
                return
            if (
                motion_started
                and status is not None
                and status.msg.motion_status == 0
            ):
                print(f"{label} 已完成")
                return

        if time.monotonic() - start > 5.0 and not motion_started:
            raise RuntimeError(
                f"{label} 未开始，六轴使能状态={robot.get_joints_enable_status_list()}, "
                f"arm_status={last_status}, flange_pose={last_pose}"
            )
        time.sleep(0.1)
    raise TimeoutError(
        f"{label} 等待超时，arm_status={last_status}, flange_pose={last_pose}"
    )


def check_workspace(pose):
    x, y, z = pose[:3]
    if not (0.05 <= x <= 0.65 and -0.40 <= y <= 0.40 and -0.05 <= z <= 0.60):
        raise RuntimeError(
            "目标法兰位置超出脚本安全范围: "
            f"xyz={[x, y, z]}；请先检查标定结果，不要强制移动。"
        )


def save_debug(out_dir, color, depth, depth_scale, pixel, point_base, prefix):
    os.makedirs(out_dir, exist_ok=True)
    color_path = os.path.join(out_dir, f"{prefix}_Color.png")
    depth_path = os.path.join(out_dir, f"{prefix}_Depth_raw.png")
    debug_path = os.path.join(out_dir, f"{prefix}_debug.png")
    scale_path = os.path.join(out_dir, f"{prefix}_depth_scale.txt")
    debug = color.copy()
    u, v = np.round(pixel).astype(int)
    cv2.drawMarker(debug, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(
        debug,
        f"base xyz: {point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f} m",
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(color_path, color)
    cv2.imwrite(depth_path, depth)
    cv2.imwrite(debug_path, debug)
    with open(scale_path, "w", encoding="utf-8") as f:
        f.write(f"{depth_scale}\n")
    print("调试图:", debug_path)


def move_flange(target_point, approach_height, speed_percent, post_capture):
    robot = create_robot()
    robot.connect()
    initial_joints = None
    enabled = False
    try:
        initial_joints = get_message(robot, "get_joint_angles")
        initial_pose = get_message(robot, "get_flange_pose")
        target_pose = [
            float(target_point[0]),
            float(target_point[1]),
            float(target_point[2] + approach_height),
            *initial_pose[3:],
        ]
        check_workspace(target_pose)
        print("启动法兰位姿:", initial_pose)
        print("目标法兰位姿:", target_pose)
        answer = input("确认工作区安全后，输入 y 执行 move_p；其他输入取消: ").strip().lower()
        if answer != "y":
            print("已取消机械臂移动。")
            return

        enabled = True
        wait_until_enabled(robot)
        robot.set_speed_percent(speed_percent)
        time.sleep(0.2)
        robot.move_p(target_pose)
        wait_motion_done(robot, "移动到目标上方", target_pose, initial_pose)
        actual = get_message(robot, "get_flange_pose")
        error = np.asarray(actual[:3]) - np.asarray(target_pose[:3])
        print("到位法兰反馈:", actual)
        print("法兰 xyz 误差(actual-target), m:", error.tolist())

        if post_capture is not None:
            post_capture()

        print("准备使用启动关节角回位...")
        return_start_pose = get_message(robot, "get_flange_pose")
        robot.move_j(initial_joints)
        wait_motion_done(robot, "回到启动关节角", initial_pose, return_start_pose)
    finally:
        if enabled:
            robot.disable()
        robot.disconnect()
        print("机械臂已断开。")


def main():
    parser = argparse.ArgumentParser(description="用 RealSense 点击目标点并验证 Piper 法兰坐标。")
    parser.add_argument("--calib-dir", default="./collect_data_test")
    parser.add_argument("--out-dir", default="./verify_output")
    parser.add_argument("--pixel", type=parse_pixel, help="跳过点击，直接指定像素 u,v")
    parser.add_argument("--depth-window", type=int, default=9)
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--approach-height", type=float, default=0.05)
    parser.add_argument("--speed-percent", type=int, default=10)
    parser.add_argument("--move", action="store_true", help="实际移动法兰；默认仅计算坐标")
    args = parser.parse_args()

    K, D, transform = load_calibration(args.calib_dir)
    color, depth, scale = capture_aligned_rgbd(
        args.width, args.height, args.fps, args.warmup_frames
    )
    pixel = args.pixel if args.pixel is not None else select_pixel(color)
    depth_m = depth_at_pixel(depth, pixel, scale, args.depth_window)
    point_camera = pixel_to_camera(pixel, depth_m, K, D)
    point_base = camera_to_base(point_camera, transform)

    print("目标像素 u,v:", pixel.tolist())
    print("目标深度, m:", depth_m)
    print("相机坐标 xyz, m:", point_camera.tolist())
    print("机械臂基坐标 xyz, m:", point_base.tolist())
    save_debug(args.out_dir, color, depth, scale, pixel, point_base, "before_move")

    if not args.move:
        print("当前为只计算模式；确认坐标合理后再添加 --move。")
        return

    def capture_after_move():
        print("法兰已到位，正在保存移动后图像...")
        post_color, post_depth, post_scale = capture_aligned_rgbd(
            args.width, args.height, args.fps, 10
        )
        save_debug(
            args.out_dir,
            post_color,
            post_depth,
            post_scale,
            pixel,
            point_base,
            "after_move",
        )


    move_flange(
        point_base,
        args.approach_height,
        args.speed_percent,
        capture_after_move,
    )


if __name__ == "__main__":
    main()
