from setuptools import setup

package_name = "bottle_grasp_executor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/bottle_grasp_executor.yaml"]),
        ("share/" + package_name + "/launch", ["launch/bottle_grasp_executor.launch.py"]),
        # 夹爪脚本: 节点用 get_package_share_directory 找到它再 spawn (见 _load_gripper_config)。
        # 放 share 而不是放进 python 包, 是为了与 config/launch 保持一致 ——
        # 它不是节点的模块, 是一个可以单独在终端里跑的脚本。
        ("share/" + package_name + "/scripts", ["scripts/gripper_grip_hold.py"]),
        # 放置位姿单点验证: 单独在终端跑, 只发一条 move_p 看固件收不收 (2026-09-21)。
        # 与上面那个脚本同理: 它不是节点的模块, 是排障时手边的一个工具。
        ("share/" + package_name + "/scripts", ["scripts/check_place_pose.py"]),
        # 按关节角下发 J 模式指令 (2026-09-24)。同样是个手边工具: 想让机械臂摆到某个
        # 姿势时, 关节角比 XYZ+RPY 直接, 而且不经过 IK, 不存在「固件说不可达」。
        ("share/" + package_name + "/scripts", ["scripts/move_joints.py"]),
        # 上位机求逆解 + 轨迹检查 (2026-09-28)。**只算不发**, 是 move_joints.py 的上游:
        # 它把位姿/直线解成关节序列、检查限位与分支连续性, 判定通过后打印
        # move_joints.py 的命令行。依赖 ik_h3001.py 与 URDF —— 两者都不在本包里
        # (默认从工作区 urdf/ 或 --ik-src 的旁边找), 所以它是**从源码目录跑的台架
        # 工具**, 装到 share/ 只是跟着一起带走。
        ("share/" + package_name + "/scripts", ["scripts/ik_plan.py"]),
        # ── 求逆解的三件套 (2026-09-28) ──
        # 执行节点自己也要求逆解了 (运动指令改走 /control/move_j, 见 README §5.15)。
        # 三个文件都是**从上游拷来的 vendored 副本**: ik_h3001.py 原样拷贝 (文件头写清
        # 来源与更新方式), URDF 只拷 xml —— **21MB 的 STL 一个都不要**,
        # pin.buildModelFromUrdf 不加载网格 (实测); CSV 拷过来时改成了 ASCII 名
        # (原名带中文/括号/空格, 无中文 locale 的机器上 colcon 拷贝会出问题)。
        #
        # 为什么不依赖上游路径: /home/zdl/pinocchio-devel/SDK_linux 是开发目录,
        # 部署到别的机器上不一定在。拷进 share/ 之后节点按
        # share/<pkg>/ik/ → 包根/ik/ 的次序解析 (同 _resolve_gripper_script 的套路)。
        ("share/" + package_name + "/ik", [
            "ik/ik_h3001.py",
            "ik/L4002-0100.urdf",
            "ik/L4002-0100-cad-limits.csv",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="qin",
    maintainer_email="2745146351@qq.com",
    description="瓶子抓取执行节点: 感知候选 + 手眼变换 + H3001 move_p",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bottle_grasp_executor = bottle_grasp_executor.bottle_grasp_executor_node:main",
        ],
    },
)
