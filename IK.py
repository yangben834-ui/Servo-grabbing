# ik_solver.py
# 依赖: pip install pybullet numpy

import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R


class IKSolver:
    def __init__(self, urdf_path, base_link=None, tip_link=None,
                 use_fixed_base=True, verbose=True):
        """
        urdf_path : URDF 文件路径
        base_link : 基坐标系对应的 link 名。None 表示用 URDF 根节点
        tip_link  : 末端 link 名（IK 求解目标所在 link）
        """
        self.verbose = verbose
        self.client = p.connect(p.DIRECT)  # 无 GUI；要可视化改 p.GUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = p.loadURDF(urdf_path, useFixedBase=use_fixed_base)

        self.num_joints = p.getNumJoints(self.robot)
        self.joint_infos = [p.getJointInfo(self.robot, j)
                            for j in range(self.num_joints)]

        # 建立 name -> index 映射
        self.link_name_to_index = {}
        for j, info in enumerate(self.joint_infos):
            link_name = info[12].decode("utf-8")
            self.link_name_to_index[link_name] = j
        # 根 link（base link）不在 getJointInfo 里，用 -1 表示
        root_link_name = p.getBodyInfo(self.robot)[0].decode("utf-8")
        self.link_name_to_index[root_link_name] = -1

        if base_link is not None and base_link not in self.link_name_to_index:
            raise ValueError(f"base_link '{base_link}' 不存在。可用: "
                             f"{list(self.link_name_to_index.keys())}")
        if tip_link not in self.link_name_to_index:
            raise ValueError(f"tip_link '{tip_link}' 不存在。可用: "
                             f"{list(self.link_name_to_index.keys())}")

        self.base_link = base_link
        self.base_index = (self.link_name_to_index[base_link]
                           if base_link is not None else -1)
        self.tip_index = self.link_name_to_index[tip_link]
        self.tip_link = tip_link

        # 可动关节列表（排除 fixed）
        self.movable_joints = [
            j for j, info in enumerate(self.joint_infos)
            if info[2] != p.JOINT_FIXED
        ]
        self.movable_names = [
            self.joint_infos[j][1].decode("utf-8") for j in self.movable_joints
        ]

        # 关节限位 / 范围 / 初始位姿，供 IK 使用
        self.lower = []
        self.upper = []
        self.ranges = []
        self.rest = []
        for j in self.movable_joints:
            info = self.joint_infos[j]
            lo, up = info[8], info[9]
            if lo > up:   # 连续关节 (revolute with no limit)
                lo, up = -np.pi, np.pi
            self.lower.append(lo)
            self.upper.append(up)
            self.ranges.append(up - lo)
            self.rest.append(0.0)

        if verbose:
            print(f"[IK] 加载 URDF: {urdf_path}")
            print(f"[IK] 根 link: {root_link_name}")
            print(f"[IK] base_link: {base_link} (index={self.base_index})")
            print(f"[IK] tip_link : {tip_link} (index={self.tip_index})")
            print(f"[IK] 可动关节 : {self.movable_names}")

    # ---------------- 坐标变换 ----------------
    def _link_pose_world(self, link_index):
        """返回 link 在世界坐标系的 (pos, quat)，quat=[x,y,z,w]"""
        if link_index == -1:
            pos, quat = p.getBasePositionAndOrientation(self.robot)
        else:
            state = p.getLinkState(self.robot, link_index,
                                   computeForwardKinematics=True)
            pos, quat = state[4], state[5]  # world frame
        return np.array(pos), np.array(quat)

    def _world_from_base(self):
        """T_world_base 的 4x4 齐次矩阵"""
        pos, quat = self._link_pose_world(self.base_index)
        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = pos
        return T

    # ---------------- 正运动学 ----------------
    def forward_kinematics(self, q):
        """给定可动关节角，返回 base 到 tip 的 4x4 变换"""
        q_full = self._expand_q(q)
        for i, j in enumerate(self.movable_joints):
            p.resetJointState(self.robot, j, q_full[j])
        tip_pos, tip_quat = self._link_pose_world(self.tip_index)
        T_world_tip = np.eye(4)
        T_world_tip[:3, :3] = R.from_quat(tip_quat).as_matrix()
        T_world_tip[:3, 3] = tip_pos
        T_world_base = self._world_from_base()
        return np.linalg.inv(T_world_base) @ T_world_tip

    # ---------------- 逆运动学 ----------------
    def inverse_kinematics(self, target_pos, target_quat,
                           seed=None, max_iter=500, tol=1e-8):
        """
        target_pos  : [x, y, z]，base_link 坐标系下
        target_quat : [qx, qy, qz, qw]，base_link 坐标系下
        seed        : 可动关节初值列表；None 用 rest
        返回: (q, success, err_pos, err_rot)
        """
        if seed is None:
            seed = self.rest
        q_full_seed = self._expand_q(seed)

        # 把 base 坐标系下的目标转到世界坐标系
        T_world_base = self._world_from_base()
        T_base_target = np.eye(4)
        T_base_target[:3, :3] = R.from_quat(target_quat).as_matrix()
        T_base_target[:3, 3] = target_pos
        T_world_target = T_world_base @ T_base_target

        pos_w = T_world_target[:3, 3]
        quat_w = R.from_matrix(T_world_target[:3, :3]).as_quat()  # x,y,z,w

        q_sol = p.calculateInverseKinematics(
            self.robot,
            self.tip_index,
            targetPosition=pos_w.tolist(),
            targetOrientation=quat_w.tolist(),
            lowerLimits=self.lower,
            upperLimits=self.upper,
            jointRanges=self.ranges,
            restPoses=self.rest,
            jointDamping=[0.05] * len(self.movable_joints),
            maxNumIterations=max_iter,
            residualThreshold=tol,
        )

        # q_sol 长度 == num_joints（含有 fixed），提取可动的
        q_movable = self._extract_movable(q_sol)

        # 验证
        T_achieved = self.forward_kinematics(q_movable)
        err_pos = np.linalg.norm(T_achieved[:3, 3] - target_pos)
        R_err = T_achieved[:3, :3].T @ T_base_target[:3, :3]
        err_rot = np.linalg.norm(R.from_matrix(R_err).as_rotvec())

        success = (err_pos < 1e-3) and (err_rot < 1e-2)
        return q_movable, success, err_pos, err_rot

    # ---------------- 工具函数 ----------------
    def _expand_q(self, q_movable):
        """把可动关节角度展开成长度 num_joints 的数组"""
        q_full = [0.0] * self.num_joints
        for i, j in enumerate(self.movable_joints):
            q_full[j] = q_movable[i]
        return q_full

    def _extract_movable(self, q_full):
        return [q_full[j] for j in self.movable_joints]

    def random_seed(self):
        return [np.random.uniform(lo, up)
                for lo, up in zip(self.lower, self.upper)]

    def solve_multi(self, target_pos, target_quat, num_seeds=20, **kw):
        """多初值求 IK，返回所有成功的解（去重后）"""
        solutions = []
        seeds = [self.rest] + [self.random_seed() for _ in range(num_seeds - 1)]
        for s in seeds:
            q, ok, ep, er = self.inverse_kinematics(
                target_pos, target_quat, seed=s, **kw)
            if ok:
                if not any(np.allclose(q, qs, atol=1e-3) for qs in solutions):
                    solutions.append(np.array(q))
        return solutions

    def close(self):
        p.disconnect(self.client)


# ============================ 使用示例 ============================
if __name__ == "__main__":
    URDF = "your_robot.urdf"   # ← 改成你的 URDF 路径
    BASE = "base_link"          # ← 基坐标系 link 名
    TIP  = "tool0"              # ← 末端 link 名

    solver = IKSolver(URDF, base_link=BASE, tip_link=TIP)

    # 目标：base_link 下的位置 (m) 和姿态（四元数 x,y,z,w）
    target_pos  = np.array([0.3, 0.1, 0.4])
    target_quat = np.array([0.0, 1.0, 0.0, 0.0])   # 例：绕 X 转 180°

    # ---- 单次求解 ----
    q, ok, ep, er = solver.inverse_kinematics(target_pos, target_quat)
    print("\n[单次 IK]")
    print("success :", ok)
    print("pos_err :", ep, "m")
    print("rot_err :", er, "rad")
    for name, ang in zip(solver.movable_names, q):
        print(f"  {name:20s} = {ang: .6f} rad ({np.degrees(ang): .3f} deg)")

    # ---- 多初值求解，拿到所有解 ----
    sols = solver.solve_multi(target_pos, target_quat, num_seeds=30)
    print(f"\n[多初值 IK] 找到 {len(sols)} 个不同解")
    for i, qs in enumerate(sols):
        print(f"  解 {i}: {np.round(np.degrees(qs), 3)} deg")

    solver.close()