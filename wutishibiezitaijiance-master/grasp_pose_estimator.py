"""与相机、YOLO 和 GUI 无关的瓶体抓取姿态几何算法。"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


Intrinsics = Tuple[float, float, float, float]


@dataclass
class GeometryConfig:
    """点云过滤、桌面拟合和姿态分类参数。"""

    min_point_count: int = 100
    max_point_count: int = 6000
    depth_outlier_threshold: float = 0.05
    environment_point_step: int = 4
    environment_max_depth: float = 1.0
    table_ransac_iterations: int = 80
    table_distance_threshold: float = 0.015
    table_min_inliers: int = 500
    table_max_sample_points: int = 8000
    table_normal_smoothing: float = 0.2
    upright_angle_threshold_deg: float = 50.0

    # 圆柱轴拟合: 相机只能看到瓶子朝向自己的半壳, 抓取点默认落在可见表面质心上,
    # 比瓶轴偏向相机约 2r/pi。开启后改为拟合圆柱轴并把抓取点放到轴上。
    use_cylinder_axis: bool = False
    cylinder_ransac_iterations: int = 300
    cylinder_max_sample_points: int = 1500
    cylinder_distance_threshold: float = 0.004
    cylinder_min_inlier_ratio: float = 0.35
    cylinder_min_radius: float = 0.010
    cylinder_max_radius: float = 0.060
    cylinder_axial_trim: float = 0.25

    random_seed: Optional[int] = None


@dataclass
class CylinderFit:
    """可见半壳的圆柱拟合结果; ok 为 False 时给出拒绝原因。"""

    ok: bool
    axis_point: Optional[np.ndarray]
    radius: Optional[float]
    inlier_ratio: float
    arc_deg: Optional[float]
    reject_reason: str = ""


@dataclass
class GraspPose:
    """相机光学坐标系中的几何抓取位姿。"""

    position: np.ndarray
    rotation: np.ndarray
    eigenvalues: np.ndarray
    mode: str
    axis_table_angle_deg: float
    # 圆柱拟合结果; None 表示没开开关或拟合失败(此时 position 就是可见表面质心)
    cylinder: Optional[CylinderFit] = None
    # 可见表面质心, 用于对照检查抓取点有没有被推向瓶轴
    surface_position: Optional[np.ndarray] = None

    @property
    def radius(self) -> Optional[float]:
        """拟合出的瓶半径; 没开开关或拟合失败时为 None。"""
        if self.cylinder is not None and self.cylinder.ok:
            return self.cylinder.radius
        return None

    @property
    def fit_reason(self) -> str:
        """拟合失败的原因; 没开开关时为空串。"""
        if self.cylinder is None:
            return ""
        return self.cylinder.reject_reason


def mask_to_point_cloud(
    depth_image: np.ndarray,
    mask: np.ndarray,
    intrinsics: Intrinsics,
    config: GeometryConfig,
) -> Optional[np.ndarray]:
    """将 mask 内的有效米制深度反投影为相机坐标系点云。"""
    fx, fy, cx, cy = intrinsics
    rows, cols = np.nonzero(
        mask & np.isfinite(depth_image) & (depth_image > 0)
    )
    if rows.size < config.min_point_count:
        return None

    depths = depth_image[rows, cols]
    median_depth = np.median(depths)
    inliers = (
        np.abs(depths - median_depth) < config.depth_outlier_threshold
    )
    rows, cols, depths = rows[inliers], cols[inliers], depths[inliers]
    if depths.size < config.min_point_count:
        return None

    if depths.size > config.max_point_count:
        indices = np.linspace(
            0, depths.size - 1, config.max_point_count, dtype=np.int32
        )
        rows, cols, depths = rows[indices], cols[indices], depths[indices]

    x = (cols.astype(np.float32) - cx) * depths / fx
    y = (rows.astype(np.float32) - cy) * depths / fy
    return np.column_stack((x, y, depths)).astype(np.float32)


def depth_to_environment_cloud(
    depth_image: np.ndarray,
    object_mask: np.ndarray,
    intrinsics: Intrinsics,
    config: GeometryConfig,
) -> Optional[np.ndarray]:
    """将目标区域外的有效深度反投影为降采样环境点云。"""
    fx, fy, cx, cy = intrinsics
    step = config.environment_point_step
    sampled_depth = depth_image[::step, ::step]
    sampled_mask = object_mask[::step, ::step]
    valid = (
        ~sampled_mask
        & np.isfinite(sampled_depth)
        & (sampled_depth > 0)
        & (sampled_depth <= config.environment_max_depth)
    )
    sampled_rows, sampled_cols = np.nonzero(valid)
    if sampled_rows.size == 0:
        return None

    depths = sampled_depth[sampled_rows, sampled_cols]
    rows = sampled_rows * step
    cols = sampled_cols * step
    x = (cols.astype(np.float32) - cx) * depths / fx
    y = (rows.astype(np.float32) - cy) * depths / fy
    return np.column_stack((x, y, depths)).astype(np.float32)


def estimate_table_normal(
    environment_points: Optional[np.ndarray],
    config: GeometryConfig,
) -> Optional[np.ndarray]:
    """通过 RANSAC 主平面拟合估计朝向相机的桌面法向。"""
    if (
        environment_points is None
        or len(environment_points) < config.table_min_inliers
    ):
        return None

    if len(environment_points) > config.table_max_sample_points:
        indices = np.linspace(
            0,
            len(environment_points) - 1,
            config.table_max_sample_points,
            dtype=np.int32,
        )
        points = environment_points[indices]
    else:
        points = environment_points

    rng = np.random.default_rng(config.random_seed)
    best_inliers = None
    best_count = 0
    for _ in range(config.table_ransac_iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        distances = np.abs((points - sample[0]) @ normal)
        inliers = distances < config.table_distance_threshold
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < config.table_min_inliers:
        return None

    plane_points = points[best_inliers]
    plane_center = np.mean(plane_points, axis=0)
    _, _, vh = np.linalg.svd(plane_points - plane_center, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] > 0:
        normal = -normal
    return normal.astype(np.float32)


# Kåsa 精修轮数; 内点集基本不变, 3 轮足够收敛。
_CIRCLE_REFINE_ROUNDS = 3


def _circumcircle(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
    """三点定圆的闭式解; 三点共线时返回 None。"""
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    determinant = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(determinant) < 1e-12:
        return None
    a_sq = ax * ax + ay * ay
    b_sq = bx * bx + by * by
    c_sq = cx * cx + cy * cy
    ux = (a_sq * (by - cy) + b_sq * (cy - ay) + c_sq * (ay - by)) / determinant
    uy = (a_sq * (cx - bx) + b_sq * (ax - cx) + c_sq * (bx - ax)) / determinant
    center = np.array([ux, uy])
    return center, float(np.hypot(ax - ux, ay - uy))


def _kasa_circle(uv: np.ndarray):
    """Kåsa 代数圆拟合: 对 [x, y, 1] 最小二乘拟合 x^2 + y^2。"""
    x = uv[:, 0]
    y = uv[:, 1]
    matrix = np.column_stack((x, y, np.ones_like(x)))
    target = x * x + y * y
    solution, *_ = np.linalg.lstsq(matrix, target, rcond=None)
    center = np.array([solution[0] * 0.5, solution[1] * 0.5])
    squared = solution[2] + center[0] * center[0] + center[1] * center[1]
    if squared <= 0:
        return None
    return center, float(np.sqrt(squared))


def _circle_inliers(
    uv: np.ndarray, center: np.ndarray, radius: float, config: GeometryConfig
) -> np.ndarray:
    """到圆周的径向距离小于阈值的点。"""
    distances = np.abs(
        np.hypot(uv[:, 0] - center[0], uv[:, 1] - center[1]) - radius
    )
    return distances < config.cylinder_distance_threshold


def fit_circle_ransac(
    uv: np.ndarray, config: GeometryConfig
) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
    """RANSAC 三点定圆并做内点精修, 返回 (圆心, 半径, 内点掩码)。"""
    if uv is None or len(uv) < 3:
        return None

    rng = np.random.default_rng(config.random_seed)
    best_count = 0
    best = None
    for _ in range(config.cylinder_ransac_iterations):
        sample = uv[rng.choice(len(uv), 3, replace=False)]
        circle = _circumcircle(sample[0], sample[1], sample[2])
        if circle is None:
            continue
        center, radius = circle
        # 半径先验: 桌面/方盒/直线退化会给出荒谬半径, 直接丢弃该假设
        if not (
            config.cylinder_min_radius < radius < config.cylinder_max_radius
        ):
            continue
        inliers = _circle_inliers(uv, center, radius, config)
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best = (center, radius)
    if best is None:
        return None

    center, radius = best
    for _ in range(_CIRCLE_REFINE_ROUNDS):
        inliers = _circle_inliers(uv, center, radius, config)
        if np.count_nonzero(inliers) < 3:
            break
        refined = _kasa_circle(uv[inliers])
        if refined is None:
            break
        center, radius = refined

    inliers = _circle_inliers(uv, center, radius, config)
    if np.count_nonzero(inliers) < 3:
        return None
    return center, radius, inliers


def _cylinder_reject(
    reason: str,
    inlier_ratio: float = 0.0,
    arc_deg: Optional[float] = None,
) -> CylinderFit:
    return CylinderFit(
        ok=False,
        axis_point=None,
        radius=None,
        inlier_ratio=inlier_ratio,
        arc_deg=arc_deg,
        reject_reason=reason,
    )


def _arc_coverage_deg(vectors: np.ndarray) -> float:
    """点相对圆心的方位角里最长空档之外的部分, 即可见弧张角。

    只作诊断, 不作判据: 恰好落在内点圆环带里的背景点会补上空档, 把半壳的
    180deg 报成 330deg+。要当判据就得先按方位角分箱、要求每箱有足够多的点,
    但在真机上还没有量过这条分布的分离度, 所以先只打印出来。
    """
    azimuth = np.degrees(np.arctan2(vectors[:, 1], vectors[:, 0]))
    azimuth.sort()
    gaps = np.diff(np.concatenate((azimuth, [azimuth[0] + 360.0])))
    return float(360.0 - gaps.max())


def estimate_cylinder_axis(
    points: np.ndarray, axis: np.ndarray, config: GeometryConfig
) -> CylinderFit:
    """在垂直于长轴的平面里拟合可见半壳的圆, 得到瓶轴与半径。

    相机只看得到瓶子朝向自己的那半个圆柱壳, 它在 ⊥ 轴平面里的投影是一段圆弧;
    拟合圆心即瓶轴, 半径即瓶半径。背景点 (桌面) 靠 RANSAC 排除, 所以轴心不受
    表面质心被背景拖拽的影响。任一判据不过就返回 ok=False, 由调用方退回表面质心。
    """
    if points is None or len(points) < config.min_point_count:
        return _cylinder_reject("点云过少")

    axis = np.asarray(axis, dtype=np.float64)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-8:
        return _cylinder_reject("长轴退化为零向量")
    axis = axis / axis_norm

    reference = np.median(points, axis=0).astype(np.float64)
    centered = points.astype(np.float64) - reference

    # ⊥ 轴平面内的两根基
    e1 = np.cross(axis, (0.0, 0.0, 1.0))
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(axis, (0.0, 1.0, 0.0))
    e1_norm = np.linalg.norm(e1)
    if e1_norm < 1e-6:
        return _cylinder_reject("无法构造垂直于长轴的基")
    e1 = e1 / e1_norm
    e2 = np.cross(axis, e1)
    e2 = e2 / np.linalg.norm(e2)

    axial = centered @ axis
    radial = np.column_stack((centered @ e1, centered @ e2))

    # 轴向裁剪: 去掉瓶盖/颈肩以及瓶子与桌面交界处的点, 只留圆柱段
    trim = float(np.clip(config.cylinder_axial_trim, 0.0, 0.45))
    if trim > 0.0:
        low, high = np.quantile(axial, (trim, 1.0 - trim))
        keep = (axial >= low) & (axial <= high)
        if np.count_nonzero(keep) < config.min_point_count:
            return _cylinder_reject("轴向裁剪后点云过少")
        axial = axial[keep]
        radial = radial[keep]

    if len(radial) > config.cylinder_max_sample_points:
        indices = np.linspace(
            0,
            len(radial) - 1,
            config.cylinder_max_sample_points,
            dtype=np.int32,
        )
        axial = axial[indices]
        radial = radial[indices]

    circle = fit_circle_ransac(radial, config)
    if circle is None:
        return _cylinder_reject("未找到满足半径先验的圆")
    center, radius, inliers = circle

    if not (
        config.cylinder_min_radius <= radius <= config.cylinder_max_radius
    ):
        return _cylinder_reject(f"半径 {radius * 1000:.1f}mm 超出范围")

    inlier_ratio = float(np.count_nonzero(inliers)) / float(len(radial))
    if inlier_ratio < config.cylinder_min_inlier_ratio:
        return _cylinder_reject(
            f"内点率 {inlier_ratio:.2f} 过低", inlier_ratio=inlier_ratio
        )

    # 轴心点与可见表面质心取同一轴向高度, 使两者之差是纯径向位移
    inlier_axial = axial[inliers]
    height = float(np.median(inlier_axial))
    surface_2d = radial[inliers].mean(axis=0)
    surface_point = (
        reference + e1 * surface_2d[0] + e2 * surface_2d[1] + axis * height
    )

    to_camera = -surface_point
    to_camera -= np.dot(to_camera, axis) * axis
    to_camera_norm = np.linalg.norm(to_camera)
    if to_camera_norm < 1e-8:
        return _cylinder_reject(
            "视线方向退化", inlier_ratio=inlier_ratio
        )
    to_camera = to_camera / to_camera_norm
    to_camera_2d = np.array([np.dot(to_camera, e1), np.dot(to_camera, e2)])

    # 瓶轴必然在可见面之后(远离相机); 反号说明拟合跑到了瓶子背面
    if float(np.dot(center - surface_2d, to_camera_2d)) >= 0.0:
        return _cylinder_reject(
            "拟合圆心不在可见面之后", inlier_ratio=inlier_ratio
        )

    # 内点可见弧张角: 只作诊断量输出, 不作判据。原因见 _arc_coverage_deg 的说明。
    arc_deg = _arc_coverage_deg(radial[inliers] - center)

    axis_point = reference + e1 * center[0] + e2 * center[1] + axis * height
    return CylinderFit(
        ok=True,
        axis_point=axis_point,
        radius=radius,
        inlier_ratio=inlier_ratio,
        arc_deg=arc_deg,
    )


def estimate_grasp_pose(
    points: Optional[np.ndarray],
    table_normal: Optional[np.ndarray],
    config: GeometryConfig,
) -> Optional[GraspPose]:
    """由目标长轴与桌面法向确定平行夹爪的抓取位姿。"""
    if points is None or len(points) < config.min_point_count:
        return None
    if table_normal is None or not np.all(np.isfinite(table_normal)):
        return None

    center = np.median(points, axis=0)
    centered = points - center
    covariance = centered.T @ centered / (len(centered) - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[2] <= 0:
        return None

    table_normal = table_normal.astype(np.float64)
    table_normal /= np.linalg.norm(table_normal)
    long_axis = eigenvectors[:, 2]
    long_axis /= np.linalg.norm(long_axis)

    # 抓取点默认是可见表面质心; 开启圆柱拟合后改到拟合出的瓶轴上。
    # 与抓取模式无关, 所以在分支之前完成。
    cylinder = None
    grasp_point = center
    if config.use_cylinder_axis:
        cylinder = estimate_cylinder_axis(points, long_axis, config)
        if cylinder.ok:
            grasp_point = cylinder.axis_point

    axis_normal_cosine = abs(np.dot(long_axis, table_normal))
    upright_cosine = np.cos(
        np.deg2rad(config.upright_angle_threshold_deg)
    )
    is_upright = axis_normal_cosine >= upright_cosine

    if is_upright:
        if np.dot(long_axis, table_normal) < 0:
            long_axis = -long_axis
        approach = -np.asarray(grasp_point, dtype=np.float64)
        approach -= np.dot(approach, table_normal) * table_normal
        approach_norm = np.linalg.norm(approach)
        if approach_norm < 1e-6:
            approach = np.asarray((1.0, 0.0, 0.0))
            approach -= np.dot(approach, table_normal) * table_normal
            approach_norm = np.linalg.norm(approach)
        if approach_norm < 1e-6:
            return None
        approach /= approach_norm
        lateral = table_normal.copy()
        closing = np.cross(lateral, approach)
        mode = "side"
    else:
        approach = table_normal.copy()
        lateral = long_axis - np.dot(long_axis, approach) * approach
        lateral_norm = np.linalg.norm(lateral)
        if lateral_norm < 1e-6:
            return None
        lateral /= lateral_norm
        if lateral[0] < 0:
            lateral = -lateral
        closing = np.cross(lateral, approach)
        mode = "top"

    closing /= np.linalg.norm(closing)
    rotation = np.column_stack((closing, lateral, approach))
    angle = np.degrees(
        np.arccos(np.clip(axis_normal_cosine, 0.0, 1.0))
    )
    return GraspPose(
        position=grasp_point,
        rotation=rotation,
        eigenvalues=eigenvalues,
        mode=mode,
        axis_table_angle_deg=float(angle),
        cylinder=cylinder,
        surface_position=center,
    )


class GraspPoseEstimator:
    """保存桌面法向时序状态的单帧几何估计器。"""

    def __init__(self, config: Optional[GeometryConfig] = None):
        self.config = config or GeometryConfig()
        self.smoothed_table_normal = None

    def reset(self) -> None:
        self.smoothed_table_normal = None

    def update_table_normal(
        self, environment_points: Optional[np.ndarray]
    ) -> Optional[np.ndarray]:
        normal = estimate_table_normal(environment_points, self.config)
        if normal is None:
            # 不复用过期平面作为本帧有效结果；内部状态仅用于下一次平滑。
            return None
        if self.smoothed_table_normal is None:
            self.smoothed_table_normal = normal
        else:
            alpha = self.config.table_normal_smoothing
            self.smoothed_table_normal = (
                (1.0 - alpha) * self.smoothed_table_normal + alpha * normal
            )
            self.smoothed_table_normal /= np.linalg.norm(
                self.smoothed_table_normal
            )
        return self.smoothed_table_normal
