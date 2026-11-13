"""Bottle RGB-D 抓取姿态估计 demo（v3f，模块化入口）。"""

import argparse
import cv2
import json
import os
import time

import numpy as np

from bottle_grasp_pipeline import BottleGraspPipeline, InferenceConfig, YoloSegmenter
from grasp_pose_estimator import GeometryConfig
from grasp_visualization import PointCloudVisualizer, draw_frame_result
from hand_eye_transform import DEFAULT_HAND_EYE_PATH, HandEyeTransform
from rgbd_camera import create_camera

# 与 bottle_grasp_executor 的 robot_base_frame 参数保持一致
BASE_FRAME_NAME = "base_link"


def parse_args():
    parser = argparse.ArgumentParser(description="YOLO RGB-D 瓶体抓取姿态估计")
    parser.add_argument(
        "--camera", choices=("realsense", "orbbec"), default="orbbec"
    )
    parser.add_argument(
        "--model", default="best_train_multi",
        help="模型基础文件名；程序按 amd64/arm64 自动选择 .pt/.rknn",
    )
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument(
        "--image-size",
        type=int,
        default=640,
        help="YOLO 推理输入边长 imgsz，默认 640",
    )
    parser.add_argument("--class-id", type=int, action="append", dest="classes")
    parser.add_argument(
        "--tracker",
        default="iou",
        help="跟踪器：iou、bytetrack、botsort、ocsort 或 Ultralytics tracker YAML 路径；RKNN 固定回退 iou",
    )
    parser.add_argument("--no-point-cloud", action="store_true")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="无图形界面运行（禁用 Open3D 和 OpenCV 窗口，适用于 RK3588）",
    )
    parser.add_argument("--timing", action="store_true", default=True)
    parser.add_argument(
        "--json-out",
        metavar="PATH",
        default=None,
        help="将最新抓取候选写入 JSON 文件 (原子替换), 供 ROS 抓取执行器轮询",
    )
    parser.add_argument(
        "--hand-eye",
        metavar="PATH",
        default=DEFAULT_HAND_EYE_PATH,
        help="手眼标定目录或 hand_eye_extrinsics.yaml 路径, 用于输出基座坐标系"
        f" (默认 {DEFAULT_HAND_EYE_PATH}); 传 none 可关闭",
    )
    parser.add_argument(
        "--offset",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="基座坐标系下的固定偏置 (米), 叠加在转换后的抓取点上; "
        "如 --offset 0 0 0.02 表示抬高 2cm。需要 --hand-eye 生效",
    )
    parser.add_argument(
        "--cylinder-axis",
        action="store_true",
        help="拟合瓶子半径并把抓取点从可见表面质心移到瓶轴上; "
        "拟合失败时自动退回表面质心",
    )
    return parser.parse_args()


def load_cylinder_axis(enabled):
    """圆柱轴拟合开关; 打印实际生效状态, 免得漏加旗标时表现为"功能没生效"。"""
    if enabled:
        print("圆柱轴拟合: 开启 (抓取点是拟合出的瓶轴, 不再贴瓶壁)")
    else:
        print("圆柱轴拟合: 关闭 (抓取点 = 可见表面质心); 加 --cylinder-axis 启用")
    return GeometryConfig(use_cylinder_axis=enabled)


class CylinderFitReporter:
    """拟合状态翻转时才打印, 避免逐帧刷屏。"""

    def __init__(self, enabled):
        self.enabled = enabled
        self.last_ok = None

    def report(self, candidate):
        if not self.enabled:
            return
        ok = candidate.pose.radius is not None
        if ok == self.last_ok:
            return
        if ok:
            stage = "已生效" if self.last_ok is None else "恢复"
            print(
                f"圆柱轴拟合: {stage} r={candidate.pose.radius * 1000:.1f}mm"
            )
        else:
            print(
                "圆柱轴拟合: 失败, 抓取点退回可见表面质心 "
                f"({candidate.pose.fit_reason})"
            )
        self.last_ok = ok


def load_offset(values, hand_eye):
    """构造基座系偏置向量; 没有手眼变换时偏置无处可加, 打印原因后忽略。"""
    offset = np.asarray(values, dtype=np.float64)
    if offset.shape != (3,):
        raise ValueError(f"--offset 需要 3 个数, 实际得到 {values}")
    if not np.any(offset):
        return offset
    if hand_eye is None:
        print("offset: 未启用手眼变换, 基座系偏置无法应用, 已忽略")
        return np.zeros(3)
    print(
        "offset(基座系, m):",
        " ".join(f"{value:+.4f}" for value in offset),
    )
    return offset


def load_hand_eye(path):
    """加载手眼外参; 关闭或文件缺失时返回 None 并打印原因, 不中断运行。"""
    if path is None or str(path).lower() in ("none", "off", ""):
        print("hand-eye: 已关闭, 仅输出相机光学坐标系")
        return None
    try:
        transform = HandEyeTransform.from_yaml(path)
    except (OSError, ValueError) as error:
        print(f"hand-eye: 加载失败, 退回相机光学坐标系 ({error})")
        return None
    print(f"hand-eye: {transform.source}")
    print(
        "hand-eye 平移(m):",
        " ".join(f"{value:+.4f}" for value in transform.translation),
        f"-> {BASE_FRAME_NAME}",
    )
    return transform


def target_base_position(candidate, hand_eye, offset):
    """相机系抓取点 -> 基座系, 再叠加基座系固定偏置。"""
    return hand_eye.point_camera_to_base(candidate.pose.position) + offset


def print_candidate(candidate, hand_eye=None, offset=None):
    x, y, z = candidate.pose.position
    normal = candidate.pose.rotation[:, 2]
    line = (
        f"ID={candidate.track_id} {candidate.class_name} "
        f"{candidate.confidence:.2f} XYZ=({x:.3f},{y:.3f},{z:.3f})m "
        f"pose={candidate.pose.mode} "
        f"axis-table={candidate.pose.axis_table_angle_deg:.1f}deg "
        f"normal=({normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f})"
    )
    if candidate.pose.radius is not None:
        line += f" r={candidate.pose.radius * 1000:.1f}mm"
    if hand_eye is not None:
        if offset is None:
            offset = np.zeros(3)
        bx, by, bz = target_base_position(candidate, hand_eye, offset)
        line += f" base=({bx:.3f},{by:.3f},{bz:.3f})m"
    print(line)


def candidate_payload(candidate, max_point_count, hand_eye=None, offset=None):
    """单个候选的 JSON 字段；position 保持相机光学坐标系不变。"""
    payload = {
        "track_id": int(candidate.track_id),
        "class_name": str(candidate.class_name),
        "confidence": float(candidate.confidence),
        "grasp_mode": str(candidate.pose.mode),
        "position": [float(v) for v in candidate.pose.position],
        "quality": float(
            candidate.confidence
            * min(1.0, len(candidate.point_cloud) / float(max_point_count))
        ),
    }
    if candidate.pose.surface_position is not None:
        payload["position_surface"] = [
            float(v) for v in candidate.pose.surface_position
        ]
    cylinder = candidate.pose.cylinder
    if cylinder is not None:
        payload["axis_fit"] = {
            "ok": bool(cylinder.ok),
            "radius": (
                None if cylinder.radius is None else float(cylinder.radius)
            ),
            "inlier_ratio": float(cylinder.inlier_ratio),
            "arc_deg": (
                None if cylinder.arc_deg is None else float(cylinder.arc_deg)
            ),
            "reject_reason": str(cylinder.reject_reason),
        }
    if hand_eye is not None:
        if offset is None:
            offset = np.zeros(3)
        # 新增字段, 不影响 executor 对 position 的既有解析
        payload["position_base"] = [
            float(v) for v in target_base_position(candidate, hand_eye, offset)
        ]
        payload["orientation_base_xyzw"] = [
            float(v)
            for v in hand_eye.quaternion_camera_to_base(candidate.pose.rotation)
        ]
    return payload


def write_candidates_json(
    path, candidates, max_point_count, hand_eye=None, offset=None
):
    """将本帧候选写入 JSON 文件, 原子替换避免读到半截文件。

    ``position`` 始终是相机光学坐标系; 附加 ``position_base`` /
    ``orientation_base_xyzw`` 时它们的参考系为 ``base_frame``,
    且 ``position_base`` 已包含 ``offset``。
    """
    if offset is None:
        offset = np.zeros(3)
    payload = {
        "stamp": time.time(),
        "base_frame": BASE_FRAME_NAME if hand_eye is not None else None,
        "hand_eye": hand_eye.source if hand_eye is not None else None,
        "offset_base": [float(v) for v in offset],
        "candidates": [
            candidate_payload(candidate, max_point_count, hand_eye, offset)
            for candidate in candidates
        ],
    }
    tmp_path = path + ".tmp"
    with open(tmp_path, "w") as handle:
        json.dump(payload, handle)
    os.replace(tmp_path, path)


def main():
    args = parse_args()
    camera = None
    viewer = None
    try:
        segmenter = YoloSegmenter(
            InferenceConfig(
                model_path=args.model,
                confidence=args.confidence,
                image_size=args.image_size,
                classes=args.classes,
                tracker=args.tracker,
            )
        )
        geometry = load_cylinder_axis(args.cylinder_axis)
        pipeline = BottleGraspPipeline(segmenter, geometry)
        print(
            f"backend: {segmenter.backend}, model: {segmenter.model_path}, "
            f"tracker: requested={segmenter.requested_tracker}, "
            f"effective={segmenter.effective_tracker}"
        )
        hand_eye = load_hand_eye(args.hand_eye)
        offset = load_offset(args.offset, hand_eye)
        fit_reporter = CylinderFitReporter(args.cylinder_axis)
        max_point_count = pipeline.estimator.config.max_point_count
        camera = create_camera(args.camera)

        if not args.headless and not args.no_point_cloud:
            viewer = PointCloudVisualizer()
        print(f"camera: {args.camera}")
        print("intr:", *camera.intrinsics)

        while True:
            color, depth = camera.read()
            if color is None or depth is None:
                continue
            frame_result = pipeline.process(color, depth, camera.intrinsics)
            if args.timing:
                print("TIMING", frame_result.timings.format_ms())
            for candidate in frame_result.candidates:
                fit_reporter.report(candidate)
                print_candidate(candidate, hand_eye, offset)
            if args.json_out:
                write_candidates_json(
                    args.json_out,
                    frame_result.candidates,
                    max_point_count,
                    hand_eye,
                    offset,
                )
            if not args.headless:
                if viewer is not None and not viewer.update(frame_result):
                    break
                debug_image = draw_frame_result(
                    color, frame_result, camera.intrinsics
                )
                cv2.imshow(
                    f"YOLO RGBD Bottle Grasp ({args.camera})", debug_image
                )
                if cv2.waitKey(1) == 27:
                    break
    finally:
        if viewer is not None:
            viewer.close()
        if camera is not None:
            camera.stop()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
