"""采集相机的照片和机械臂的位姿并保存成文件。
这里以intel realsense 相机为例， 其他相机数据读取可能需要对应修改。
机械臂使用 H3001（通过 CAN 总线，SocketCAN 接口）。
"""

import argparse
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import sys
import time
from platform import system

# 将当前目录加入 sys.path，以便导入本地模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# 机械臂适配层：单位换算、自检、Piper/H3001 切换都在里面，见该模块头部说明。
import robot_arm

arm = None
count = 0

pipeline = rs.pipeline()    # 创建管道对象
config = rs.config()
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)  # 配置流参数：分辨率、像素格式、帧率
pipeline.start(config)  # 启动相机

image_save_path = "./collect_data_test/"
calibration_mode = "eye_to_hand"  # "eye_in_hand"、"eye_to_hand" 或 "head_camera"

def parse_arguments():
    parser = argparse.ArgumentParser(description="采集手眼标定图片和机器人位姿。")
    parser.add_argument("--mode", 
                        choices=["eye_in_hand", "eye_to_hand", "head_camera"], 
                        default=calibration_mode,
                        help="标定模式。eye_in_hand:相机在末端; eye_to_hand:相机在外; head_camera:相机在头部")
    parser.add_argument("--data-dir", default=image_save_path,
                        help="保存标定图片和pose文件的目录。")
    parser.add_argument("--robot", choices=list(robot_arm.ROBOT_KINDS), default="h3001",
                        help="机械臂类型。h3001(默认) 或 piper(松灵 AgileX)。")
    parser.add_argument("--no-can-fd", action="store_true",
                        help="仅 H3001：用经典 CAN 而非 CAN-FD（经典 CAN 适配器时加此项）。")
    return parser.parse_args()

# 确保保存目录存在
os.makedirs(image_save_path, exist_ok=True)


def next_image_index(data_dir):
    """根据已有数字命名图片，返回下一组采集编号。"""
    indices = []
    for name in os.listdir(data_dir):
        base, ext = os.path.splitext(name)
        if ext.lower() in {".jpg", ".jpeg", ".png"} and base.isdigit():
            indices.append(int(base))
    return max(indices) + 1 if indices else 0


def data_collect():
    """实时采集相机图像并保存，同时记录当前机械臂或头部相关位姿。

    对于 eye_in_hand：相机固定在机械臂末端，按下 's' 记录当前图片和机械臂末端位姿。
    对于 eye_to_hand：相机固定在外部，机械臂移动后按下 's' 记录当前图片和机械臂位姿。
    对于 head_camera：相机安装在机器人头部，按下 's' 记录当前图片以及头部/机械臂当前姿态信息。
    当前位姿需要根据实际机器人控制系统获取并填入 pose 列表。
    """
    global count
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()  # 阻塞等待数据
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        cv2.namedWindow('detection', flags=cv2.WINDOW_NORMAL |
                                           cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("detection", color_image)  # 窗口显示，显示名为 Capture_Video

        k = cv2.waitKey(1) & 0xFF  # 每帧数据延时 1ms，延时不能为 0，否则读取的结果会是静态帧
        if k == ord('s'):  # 键盘按一下s, 保存当前照片和机械臂位姿
            print(f"采集第{count}组数据...")
            if calibration_mode == "eye_in_hand":
                print("当前模式: eye_in_hand（相机在机械臂末端）")
            elif calibration_mode == "eye_to_hand":
                print("当前模式: eye_to_hand（相机固定在外部，机械臂移动）")
            else:
                print("当前模式: head_camera（相机安装在机器人头部）")

            # read_flange_pose 已完成单位换算（m, rad）与合理性检查
            pose = robot_arm.read_flange_pose(arm)
            print(f"机械臂末端法兰pose (m, rad):{pose}")

            pose_file = os.path.join(image_save_path, "poses.txt")
            image_file = os.path.join(image_save_path, f"{count}.jpg")

            with open(pose_file, 'a+', encoding="utf-8") as f:
                # 将列表中的元素用空格连接成一行
                pose_ = [str(i) for i in pose]
                new_line = f'{",".join(pose_)}\n'
                # 将新行附加到文件的末尾
                f.write(new_line)

            cv2.imwrite(image_file, color_image)
            count += 1
            print(f"图像已保存到: {image_file}")
        elif k == ord('q'):
            print('采集结束！！！，已退出。')
            break


if __name__ == "__main__":
    args = parse_arguments()
    calibration_mode = args.mode
    image_save_path = args.data_dir
    os.makedirs(image_save_path, exist_ok=True)
    count = next_image_index(image_save_path)

    arm = robot_arm.create_robot(args.robot, fd=not args.no_can_fd)
    print(f"{args.robot} 机械臂已连接")
    if not robot_arm.wait_for_arm_ready(arm):
        print("警告: 5 秒内未收到反馈，请检查 CAN 接线与波特率配置。")
    print("提示: 请先对照示教器确认下方位姿读数正确，再开始采集。")
    print("  当前法兰位姿:", robot_arm.describe_pose(robot_arm.read_flange_pose(arm)))

    print(f"采集将从编号 {count} 开始。请手动移动机械臂，停稳后按 s 保存。")
    data_collect()
