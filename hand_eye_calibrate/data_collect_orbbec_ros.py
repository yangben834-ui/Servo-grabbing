#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Collect calibration images from an Orbbec ROS image topic.

This script keeps the same output format as data_collect.py:
  - numbered images: 0.jpg, 1.jpg, ...
  - poses.txt lines: x,y,z,rx,ry,rz

The robot pose is the AgileX Piper flange pose (already in meters and radians).
"""

import argparse
import os
import sys
import time
from platform import system

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# 将当前目录加入 sys.path，以便导入本地模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# 机械臂适配层：单位换算、自检、Piper/H3001 切换都在里面，见该模块头部说明。
import robot_arm
# 边采边查：标定板是否还在法兰原位（旋转不变量，与坐标系无关）。
import calib_drift


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="通过 Orbbec ROS 图像话题采集手眼标定图片和法兰位姿。"
    )
    parser.add_argument(
        "--mode",
        choices=["eye_in_hand", "eye_to_hand", "head_camera"],
        default="eye_to_hand",
        help="标定模式；当前会记录法兰位姿，主要用于 eye_to_hand / eye_in_hand。",
    )
    parser.add_argument(
        "--data-dir",
        default="./collect_data_orbbec",
        help="保存标定图片和 poses.txt 的目录。",
    )
    parser.add_argument(
        "--color-topic",
        default="/camera/color/image_raw",
        help="Orbbec 彩色图像 ROS 话题。",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/camera/color/camera_info",
        help="彩色相机 CameraInfo 话题；留空则不订阅。",
    )
    parser.add_argument(
        "--spin-timeout",
        type=float,
        default=0.1,
        help="每次 ROS spin_once 的超时时间。",
    )
    parser.add_argument(
        "--pose-retries",
        type=int,
        default=30,
        help="读取法兰位姿的重试次数。",
    )
    parser.add_argument(
        "--robot",
        choices=list(robot_arm.ROBOT_KINDS),
        default="h3001",
        help="机械臂类型。h3001(默认) 或 piper(松灵 AgileX)。",
    )
    parser.add_argument(
        "--no-can-fd",
        action="store_true",
        help="仅 H3001：用经典 CAN 而非 CAN-FD（经典 CAN 适配器时加此项）。",
    )
    parser.add_argument(
        "--no-save-camera-info",
        action="store_true",
        help="不额外保存 color_camera_info.yaml。",
    )

    gate = parser.add_argument_group(
        "自洽性门控（边采边查标定板有没有移位）",
        "标定板刚性固定在法兰上时，任意两张图之间的「法兰相对转角」必须等于"
        "「板在相机系里的相对转角」。板一旦动了这个等式就崩，H3003 实测正常样本"
        "中位 0.1~1.9°，板动过的样本中位 27~133°，区分度极大。",
    )
    gate.add_argument(
        "--no-consistency-gate",
        action="store_true",
        help="关闭自洽性门控（默认开启）。",
    )
    gate.add_argument(
        "--gate-every",
        type=int,
        default=5,
        help="每采这么多张打印一次当前段的均值统计（口径同 "
             "check_calib_consistency.py，但用增量方式算，不重算内参）。",
    )
    gate.add_argument(
        "--gate-tol",
        type=float,
        default=2.0,
        help="判据阈值（度）：段内配对差异的均值超过它就报警。默认 2.0。",
    )
    gate.add_argument(
        "--gate-action",
        choices=["delete", "abort", "warn"],
        default="delete",
        help="门控判定'标定板动过'时怎么办（默认 delete）："
             "delete=自动删掉这一张（图 + poses.txt 最后一行）、编号不前进、"
             "原地重采；abort=停止整轮采集；warn=只提示，照常收下这一张。",
    )
    gate.add_argument(
        "--gate-max-retry",
        type=int,
        default=5,
        help="连续这么多张都被判为板动过，就提示可能夹不回原状态了（默认 5）。",
    )
    gate.add_argument(
        "--intrinsics",
        default=None,
        help="内参 yaml 路径。默认优先数据目录里的 camera_intrinsics.yaml，"
             "否则用相机节点写的 color_camera_info.yaml。",
    )

    settle = parser.add_argument_group(
        "位姿停稳检查",
        "state.pose 是三条 CAN 反馈帧拼起来的（0x2A2->X,Y；0x2A3->Z,RX；0x2A4->RY,RZ）。"
        "机械臂还在动时三帧来自不同时刻，拼出来的位姿内部就是错的（实测能差 17°）。"
        "因此默认会连读几次、确认彼此一致才存图；不合格的这一张直接不保存。",
    )
    settle.add_argument(
        "--no-pose-settle-check",
        action="store_true",
        help="关闭停稳检查（只读一次位姿就存，不推荐）。",
    )
    settle.add_argument(
        "--pose-settle-timeout",
        type=float,
        default=5.0,
        help="等机械臂停稳的最长时间（秒），超时就放弃这一张。默认 5。",
    )
    return parser.parse_args()


def create_robot(kind, fd=True):
    """创建机械臂连接（Linux 下走 SocketCAN）。

    kind 取 "h3001" 或 "piper"。H3001 默认用 h3001_sdk 的 CAN-FD 配置
    （1 Mbps / 5 Mbps）；经典 CAN 适配器需要 fd=False，并先把总线起好：
        sudo ip link set can0 down
        sudo ip link set can0 type can bitrate 1000000
        sudo ip link set can0 up
    """
    return robot_arm.create_robot(kind, fd=fd)


def get_flange_pose(arm, retries):
    """读取法兰位姿，返回 [x, y, z, rx, ry, rz]，单位米和弧度。

    单位换算与合理性检查都在 robot_arm 里完成（H3001 的 SDK 原始输出是 mm 和度）。
    欧拉角为 R = Rz*Ry*Rx，与 hand_eye_calibrate.py 一致。
    """
    return robot_arm.read_flange_pose(arm, retries)


def next_image_index(data_dir):
    indices = []
    if not os.path.isdir(data_dir):
        return 0
    for name in os.listdir(data_dir):
        base, ext = os.path.splitext(name)
        if ext.lower() in {".jpg", ".jpeg", ".png"} and base.isdigit():
            indices.append(int(base))
    return max(indices) + 1 if indices else 0


def discard_last_sample(image_file, pose_file, pose, count):
    """删掉刚存下的这一张：图片文件 + poses.txt 的最后一行。

    用于门控判定"标定板动过"时原地重采 —— 编号不前进，重按 s 会覆盖同一个编号。

    poses.txt 是"追加一行"写的，所以最后一行就是这一张。但删之前**核对一遍内容**：
    只有确认最后一行确实是这次写的位姿才动它，否则宁可不动、报出来让人手工核对。
    返回 (ok, message)。
    """
    # 1) 先动 poses.txt：核对最后一行，然后整文件重写（原子替换）
    try:
        with open(pose_file, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except FileNotFoundError:
        lines = []
    if lines:
        expected = ",".join(str(v) for v in pose)
        if lines[-1].strip() != expected:
            return False, (f"poses.txt 最后一行不是第 {count} 张的位姿，未做任何删除。"
                           f"请手工核对 {pose_file}")
        tmp = pose_file + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                f.writelines(lines[:-1])
            os.replace(tmp, pose_file)
        except OSError as exc:
            return False, f"改写 {pose_file} 失败: {exc}"

    # 2) 再删图片
    if os.path.exists(image_file):
        try:
            os.remove(image_file)
        except OSError as exc:
            return False, f"删除 {image_file} 失败: {exc}（poses.txt 已回退，请手工删图）"
    return True, f"已删除 {image_file} 与 poses.txt 最后一行"


class OrbbecImageNode(Node):
    def __init__(self, color_topic, camera_info_topic):
        super().__init__("hand_eye_orbbec_collector")
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_color_stamp = None
        self.camera_info = None

        self.create_subscription(Image, color_topic, self._on_color, 10)
        if camera_info_topic:
            self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, 10)

    def _on_color(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_color_stamp = msg.header.stamp
        except Exception as exc:  # noqa: BLE001 - show ROS conversion failures directly.
            self.get_logger().error(f"彩色图像转换失败: {exc}")

    def _on_camera_info(self, msg):
        if self.camera_info is None:
            self.camera_info = msg


def save_camera_info_yaml(data_dir, msg):
    if msg is None:
        return
    path = os.path.join(data_dir, "color_camera_info.yaml")
    if os.path.exists(path):
        return
    data = {
        "image_width": int(msg.width),
        "image_height": int(msg.height),
        "distortion_model": msg.distortion_model,
        "K": np.asarray(msg.k, dtype=np.float64).reshape(3, 3).tolist(),
        "D": [float(v) for v in msg.d],
        "R": np.asarray(msg.r, dtype=np.float64).reshape(3, 3).tolist(),
        "P": np.asarray(msg.p, dtype=np.float64).reshape(3, 4).tolist(),
        "metadata": {
            "source": "ROS CameraInfo",
            "note": "hand_eye_calibrate.py will compute camera_intrinsics.yaml from chessboard images.",
        },
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
    print(f"已保存彩色相机 CameraInfo: {path}")


def collect(args):
    os.makedirs(args.data_dir, exist_ok=True)
    count = next_image_index(args.data_dir)

    robot = create_robot(args.robot, fd=not args.no_can_fd)
    print(f"{args.robot} 机械臂已连接，读取的是法兰位姿（单位 m 和 rad）。")
    if not robot_arm.wait_for_arm_ready(robot):
        print("警告: 5 秒内未收到反馈，请检查 CAN 接线与波特率配置。")
    print("当前法兰位姿:", robot_arm.describe_pose(get_flange_pose(robot, args.pose_retries)))
    print("请先对照示教器确认上方位姿读数正确，再开始采集。")

    rclpy.init()
    node = OrbbecImageNode(args.color_topic, args.camera_info_topic)
    print("正在等待 Orbbec 彩色图像话题...")
    print(f"  color_topic: {args.color_topic}")
    if args.camera_info_topic:
        print(f"  camera_info_topic: {args.camera_info_topic}")

    # ---------- 自洽性门控 ----------
    # 采集阶段还没有 camera_intrinsics.yaml，用相机节点写的 color_camera_info.yaml
    # 兜底；那份文件要等第一张存盘时才写出，所以这里允许延迟初始化。
    gate_state = {"monitor": None, "tried": False, "warned": False}

    def try_init_gate():
        if args.no_consistency_gate or gate_state["monitor"] is not None:
            return gate_state["monitor"]
        gate_state["tried"] = True
        try:
            mon = calib_drift.DriftMonitor.from_data_dir(
                args.data_dir, args.intrinsics, tol_deg=args.gate_tol)
            warmed = mon.warm_up(args.data_dir)
            gate_state["monitor"] = mon
            if warmed:
                s = mon.segment_summary()
                print(f"[门控] 已载入已有 {warmed} 张作为参考；"
                      f"当前段 {s['n']} 对，均值 "
                      f"{'%.2f' % s['mean'] if s['mean'] is not None else 'N/A'}°")
            else:
                print(f"[门控] 已启用（阈值 mean<{args.gate_tol}°）。")
        except Exception as exc:  # noqa: BLE001 - 门控不应阻断采集
            if not gate_state["warned"]:
                gate_state["warned"] = True
                print(f"[门控] 暂不可用: {exc}")
                print("        采集继续，但这一轮不会检查标定板是否移位。")
        return gate_state["monitor"]

    if not args.no_consistency_gate:
        try_init_gate()

    window_name = "orbbec calibration collect"
    cv2.namedWindow(
        window_name,
        flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED,
    )
    print(f"采集将从编号 {count} 开始。机械臂停稳后按 s 保存，按 q 退出。")
    printed_first_frame = False
    rejects = 0  # 连续被判"板动过"的次数（原地重采时会累加，成功一张就清零）

    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=args.spin_timeout)
            except (ExternalShutdownException, KeyboardInterrupt):
                print("收到退出信号，正在关闭采集脚本。")
                break
            if node.latest_color is None:
                continue

            image = node.latest_color.copy()
            if not printed_first_frame:
                print(f"已收到彩色图像: {image.shape[1]}x{image.shape[0]}")
                printed_first_frame = True
            cv2.imshow(window_name, image)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                capture_image = image.copy()

                # 位姿必须在存图之前拿到，并且要求机械臂确实停稳了 ——
                # state.pose 是三条 CAN 帧拼的，臂还在动时拼出来的位姿内部就是错的，
                # 实测能差 17°。所以宁可不存这一张，也不要存一张错配的数据。
                if args.no_pose_settle_check:
                    pose = get_flange_pose(robot, args.pose_retries)
                else:
                    try:
                        pose = robot_arm.read_settled_flange_pose(
                            robot, args.pose_retries,
                            timeout_s=args.pose_settle_timeout)
                    except RuntimeError as exc:
                        print(f"  [跳过] {exc}")
                        continue

                image_file = os.path.join(args.data_dir, f"{count}.jpg")
                pose_file = os.path.join(args.data_dir, "poses.txt")

                cv2.imwrite(image_file, capture_image)
                with open(pose_file, "a+", encoding="utf-8") as f:
                    f.write(",".join(str(v) for v in pose) + "\n")

                if not args.no_save_camera_info:
                    save_camera_info_yaml(args.data_dir, node.camera_info)

                print(f"已采集第 {count} 组")
                print(f"  图像: {image_file}")
                print(f"  法兰pose: {pose}")

                # ---- 自洽性门控：逐张与已有样本比对 ----
                monitor = try_init_gate()
                abort = False
                accepted = True
                if monitor is not None:
                    # 先快照：若判定这张要丢弃，回滚监测器状态，
                    # 免得被判"动过"的这张成为新段的参考点污染后续比对。
                    snap = monitor.save_state()
                    try:
                        v = monitor.check_image(count, image_file, pose)
                    except Exception as exc:  # noqa: BLE001 - 门控不应阻断采集
                        v = {"status": "error", "text": f"检查失败: {exc}"}

                    # 分为两类：判明有问题（不该留在数据集里）／仅供参考
                    problem = None
                    if v["status"] == "moved":
                        problem = {
                            "title": "标定板相对法兰动过（或被重新装夹）",
                            "detail": v["text"] + (
                                f"；第 {count} 张与之前的样本对不上，标定板很可能被碰到/"
                                "翻到支架另一面。"),
                            "fix": "请把标定板夹回**前几张**的安装状态"
                                   "（不是它现在歪着的样子）",
                        }
                    elif v["status"] == "no_board":
                        problem = {
                            "title": "这张图里检不出棋盘",
                            "detail": "标定时这张会被跳过，留着只会占编号。",
                            "fix": "请确认标定板完整在画面内（必要时先把机械臂挪一下）",
                        }

                    if problem is not None:
                        print("\n" + "!" * 72)
                        print(f"!! {problem['title']} —— {problem['detail']}")
                        if args.gate_action == "delete":
                            ok, msg = discard_last_sample(image_file, pose_file, pose, count)
                            if ok:
                                monitor.restore_state(snap)
                                accepted = False
                                rejects += 1
                                print(f"!! {msg}；编号不前进。")
                                print(f"!! {problem['fix']}，然后重新按 s 采第 {count} 张。")
                                if args.gate_max_retry and rejects >= args.gate_max_retry:
                                    print(f"!! 已连续 {rejects} 张被判有问题："
                                          "要么清空数据目录、以当前安装状态从头采，"
                                          "要么加 --gate-action warn 先把它收下再看。")
                            else:
                                print(f"!! 自动删除失败：{msg}")
                                print("!! 为免把坏数据混进去，停止采集。")
                                abort = True
                        elif args.gate_action == "abort":
                            print("!! 停止整轮采集（--gate-action abort）。")
                            abort = True
                        else:
                            print("!! --gate-action warn：照常收下这一张，"
                                  "但它会让标定结果不可信。")
                        print("!" * 72 + "\n")
                    elif v["status"] == "suspect":
                        print(f"  [门控] 可疑：{v['text']}")
                    elif v["status"] == "ok":
                        print(f"  [门控] 与已有 {v['n_pairs']} 对配对一致（中位 "
                              f"{v['median']:.2f}°）")
                    else:
                        print(f"  [门控] {v.get('text', v['status'])}")

                    # 每 gate_every 张报一次当前段的均值（口径同 check_calib_consistency.py）
                    n_accepted = count + 1
                    if args.gate_every > 0 and n_accepted % args.gate_every == 0:
                        s = monitor.segment_summary()
                        if s["mean"] is not None:
                            print(f"  [门控] 当前段 {s['n']} 对配对：均值 "
                                  f"{s['mean']:.2f}° 中位 {s['median']:.2f}° "
                                  f"p90 {s['p90']:.2f}° 最大 {s['max']:.2f}°"
                                  f"（判据 mean<{args.gate_tol}°）")
                            if s["mean"] > args.gate_tol:
                                print(f"  [门控] ⚠ 均值 {s['mean']:.2f}° 已超过 "
                                      f"{args.gate_tol}°，段内数据不够自洽，请检查采集环节"
                                      "（角点是否稳、机械臂是否停稳、板是否松动）。")

                if accepted:
                    count += 1
                    rejects = 0
                if abort:
                    print("已按自洽性门控停止采集"
                          "（要改成原地重采用默认的 --gate-action delete，"
                          "要照常收下用 --gate-action warn）。")
                    break
            elif key == ord("q"):
                print("采集结束，已退出。")
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        robot_arm.disconnect_robot(robot)
        print("机械臂已断开。")


def main():
    args = parse_arguments()
    collect(args)


if __name__ == "__main__":
    main()
