#!/usr/bin/env python3
"""基于当前 STM32 AGX 协议的 H3001 PC 到 MCU SDK。

SDK 只与 MCU 主 CAN 口通信，电机总线通信由 MCU 固件内部完成。
"""

from __future__ import annotations

import os
import platform
import threading
import time
import gc
import math
import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable


def _add_libusb_dll_dir() -> None:
    """让 PyUSB 能够找到 libusb 软件包安装的 DLL。"""

    try:
        import libusb
    except ModuleNotFoundError:
        return

    arch = "x86_64"
    machine = platform.machine().lower()
    if machine in {"x86", "i386", "i686"}:
        arch = "x86"
    elif "arm" in machine and "64" in machine:
        arch = "arm64"

    dll_dir = os.path.join(os.path.dirname(libusb.__file__), "_platform", "windows", arch)
    if os.path.isdir(dll_dir):
        os.environ["PATH"] = dll_dir + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(dll_dir)


_add_libusb_dll_dir()

try:
    import can
    # 显式加载广成/Agilex CANDO 后端，避免从 bat 文件或嵌入式 Python 启动时
    # 依赖 python-can 的入口点自动发现机制。
    # Linux 默认使用 python-can 的 socketcan；广成后端按需加载。
    try:
        import agx_cando  # noqa: F401
    except ModuleNotFoundError:
        agx_cando = None
except ModuleNotFoundError as exc:
    if exc.name != "can":
        raise
    raise SystemExit(
        "缺少 CAN 依赖 python-can。请在 Linux 环境执行：\n"
        "  python3 -m pip install python-can"
    ) from exc


# PC -> MCU 控制命令 ID。帧类型为 Classic CAN，11 位标准 ID，最长 8 字节。
AGX_ID_ESTOP = 0x150
AGX_ID_MODE = 0x151
AGX_ID_POSE_XY = 0x152
AGX_ID_POSE_Z_RX = 0x153
AGX_ID_POSE_RY_RZ = 0x154
AGX_ID_ARC_POINT = 0x158
AGX_ID_JOINT_12 = 0x155
AGX_ID_JOINT_34 = 0x156
AGX_ID_JOINT_56 = 0x157
AGX_ID_MOTOR_ENABLE = 0x471
AGX_ID_ZERO_TORQUE = 0x520
AGX_ID_REPLAY_CONTROL = 0x521
AGX_ID_TEACH_LIGHT = 0x522
AGX_ID_ZERO_FORCE_MODE = 0x531
AGX_ID_TORQUE_14 = 0x523
AGX_ID_TORQUE_56 = 0x524
AGX_ID_DEXTEROUS_POSITION = 0x525
AGX_ID_RETURN_ZERO = 0x526
AGX_ID_ORIGIN_POSITION = 0x527
AGX_ID_DIRECT_POSITION = 0x528
AGX_ID_DIRECT_POSITION_34 = 0x529
AGX_ID_DIRECT_POSITION_56 = 0x52A
AGX_ID_ENCODER_ZERO = 0x52E
AGX_ID_MOTOR_CURRENT_QUERY = 0x472
AGX_ID_GRAVITY_PARAM_WRITE = 0x533
AGX_ID_GRAVITY_PARAM_READ = 0x534

# MCU -> PC 反馈 ID。电机总线 PDO 反馈在下面单独列出。
AGX_FB_STATE = 0x2A1
AGX_FB_POSE_XY = 0x2A2
AGX_FB_POSE_Z_RX = 0x2A3
AGX_FB_POSE_RY_RZ = 0x2A4
AGX_FB_JOINT_12 = 0x2A5
AGX_FB_JOINT_34 = 0x2A6
AGX_FB_JOINT_56 = 0x2A7
AGX_FB_GRIPPER = 0x2A8
AGX_FB_BUS_VOLTAGE = 0x2AD
# 重力补偿计算目标扭矩：每帧包含三个大端 int16，单位为电机协议原始值。
AGX_FB_GRAVITY_TORQUE_123 = 0x2AE
AGX_FB_GRAVITY_TORQUE_456 = 0x2AF
AGX_FB_DRIVER_STATUS_BASE = 0x261
AGX_FB_DRIVER_STATUS_EXT_BASE = 0x2BC
H3001_FB_IK_STATE = 0x5A9
H3001_FB_IK_STATE_LEGACY = 0x2A9
AGX_FB_MOTOR_RAW_BASE = 0x2B0
AGX_FB_STATUSWORD_RAW_BASE = 0x2B6
AGX_FB_MOTOR_ERROR_BASE = 0x2C2
AGX_FB_MOTOR_CURRENT_BASE = 0x2D0
AGX_FB_MODE_RAW_BASE = 0x2C8
AGX_FB_GRAVITY_WRITE = 0x5B3
AGX_FB_GRAVITY_READ = 0x5B4

# 电机总线直接反馈：两个 32 字节 CAN-FD PDO，每个节点包含三个轴。
# 这些帧由 MCU 从电机总线转发接收，SDK 不会直接发送它们。
MOTOR_PDO_RX_BASE = 0x101
MOTOR_PDO_NODE_COUNT = 2
MOTOR_PDO_AXIS_COUNT = 3
MOTOR_PDO_DATA_LEN = 32
MOTOR_MODE_CSP = 0x08
MOTOR_MODE_CST = 0x0A
MOTOR_CW_ENABLE = 0x00FD
MOTOR_CW_DISABLE = 0x0002

FEEDBACK_IDS = {
    AGX_FB_STATE,
    AGX_FB_POSE_XY,
    AGX_FB_POSE_Z_RX,
    AGX_FB_POSE_RY_RZ,
    AGX_FB_JOINT_12,
    AGX_FB_JOINT_34,
    AGX_FB_JOINT_56,
    AGX_FB_GRIPPER,
    AGX_FB_BUS_VOLTAGE,
    AGX_FB_GRAVITY_TORQUE_123,
    AGX_FB_GRAVITY_TORQUE_456,
    AGX_FB_GRAVITY_WRITE,
    AGX_FB_GRAVITY_READ,
    H3001_FB_IK_STATE,
    H3001_FB_IK_STATE_LEGACY,
    *range(AGX_FB_DRIVER_STATUS_BASE, AGX_FB_DRIVER_STATUS_BASE + 6),
    *range(AGX_FB_DRIVER_STATUS_EXT_BASE, AGX_FB_DRIVER_STATUS_EXT_BASE + 6),
    *range(AGX_FB_MOTOR_RAW_BASE, AGX_FB_MOTOR_RAW_BASE + 6),
    *range(AGX_FB_STATUSWORD_RAW_BASE, AGX_FB_STATUSWORD_RAW_BASE + 6),
    *range(AGX_FB_MODE_RAW_BASE, AGX_FB_MODE_RAW_BASE + MOTOR_PDO_NODE_COUNT),
    *range(AGX_FB_MOTOR_ERROR_BASE, AGX_FB_MOTOR_ERROR_BASE + 6),
    *range(AGX_FB_MOTOR_CURRENT_BASE, AGX_FB_MOTOR_CURRENT_BASE + 6),
    *range(MOTOR_PDO_RX_BASE, MOTOR_PDO_RX_BASE + MOTOR_PDO_NODE_COUNT),
}


class MoveMode(IntEnum):
    P = 0x00
    J = 0x01
    L = 0x02
    C = 0x03


class MotorMode(IntEnum):
    POSITION = 1
    TORQUE = 2


class UnsupportedByCurrentFirmwareError(RuntimeError):
    """当请求的功能当前没有对应的 PC 到 MCU 命令时抛出。"""


class GloriaGripper:
    def __init__(self, interface="socketcan", channel="can1", bitrate=1_000_000, motor_id=0x0A):
        self.interface, self.channel, self.bitrate = interface, str(channel), int(bitrate)
        self.motor_id, self.feedback_id = int(motor_id), 0x100 + int(motor_id)
        self.bus = None
        self.enabled = False
        self.mode = "stop"
        self.command_torque = 0.8
        self.position = self.velocity = self.torque = 0.0
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.RLock()

    def connect(self):
        kwargs = {"interface": self.interface, "channel": self.channel, "bitrate": self.bitrate}
        if self.interface == "agx_cando":
            kwargs.update(local_loopback=False, receive_own_messages=False)
        self.bus = can.Bus(**kwargs)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def _send(self, data):
        if self.bus is not None:
            self.bus.send(can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False))

    def _set_mit_mode(self):
        if self.bus is None:
            return
        # Gloria-M 协议：广播参数写入，RID 10 表示 MIT（1）。
        self.bus.send(can.Message(
            arbitration_id=0x7FF,
            data=bytes([self.motor_id & 0xFF, (self.motor_id >> 8) & 0xFF,
                        0x55, 10, 1, 0, 0, 0]),
            is_extended_id=False,
        ))
        time.sleep(0.05)

    def set_command(self, mode, torque=0.8):
        with self._lock:
            if mode == "enable":
                self._set_mit_mode()
                self._send(bytes([0xFF] * 7 + [0xFC]))
                self.enabled = True
                self.mode = "stop"
            elif mode == "disable":
                self._send(bytes([0xFF] * 7 + [0xFD])); self.enabled = False; self.mode = "stop"
            elif mode in ("clamp", "open", "stop"):
                self.mode, self.command_torque = mode, abs(float(torque))
            else:
                raise ValueError("invalid gripper mode")

    def snapshot(self):
        return {"connected": self.bus is not None, "enabled": self.enabled, "mode": self.mode,
                "command_torque": self.command_torque, "position": self.position,
                "velocity": self.velocity, "torque": self.torque,
                "can_id": self.motor_id, "feedback_id": self.feedback_id}

    @staticmethod
    def _uint(value, low, high, bits):
        return round((max(low, min(high, float(value))) - low) / (high - low) * ((1 << bits) - 1))

    def _mit_data(self, torque):
        q = self._uint(self.position, -3.14, 3.14, 16)
        dq = self._uint(0.0, -10.0, 10.0, 12)
        tau = self._uint(torque, -12.0, 12.0, 12)
        return bytes([q >> 8, q & 255, dq >> 4, (dq & 15) << 4,
                      0, 0, tau >> 8, tau & 255])

    def _loop(self):
        while not self._stop.wait(0.01):
            with self._lock:
                if self.bus is None: return
                try:
                    while True:
                        msg = self.bus.recv(timeout=0.0)
                        if msg is None: break
                        if msg.arbitration_id == self.feedback_id and len(msg.data) >= 6:
                            d = bytes(msg.data)
                            self.position = ((d[1] << 8) | d[2]) / 65535.0 * 6.28 - 3.14
                            self.velocity = (((d[3] << 4) | (d[4] >> 4)) / 4095.0) * 20.0 - 10.0
                            self.torque = (((d[4] & 15) << 8) | d[5]) / 4095.0 * 24.0 - 12.0
                    if self.enabled:
                        value = -self.command_torque if self.mode == "clamp" else self.command_torque if self.mode == "open" else 0.0
                        self._send(self._mit_data(value))
                except Exception:
                    return


@dataclass
class ArmState:
    """最近一次解析得到的 MCU 反馈快照。

    所有列表均按 J1~J6 排列。``*_raw`` 字段保留协议原始单位，
    ``joint_deg``、``motor_current`` 等便捷字段已经由 SDK 完成单位转换。
    """
    control_mode: int = 0
    arm_state: int = 0
    move_mode: int = 0
    teach_state: int = 0
    motion_state: int = 0
    trajectory_index: int = 0
    motor_mode: int = 0
    motor_status: int = 0
    joint_limit_fault: int = 0
    joint_comm_fault: int = 0
    pose: list[float] = field(default_factory=lambda: [0.0] * 6)
    joint_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    joint_raw: list[int] = field(default_factory=lambda: [0] * 6)
    motor_position_raw: list[int] = field(default_factory=lambda: [0] * 6)
    motor_torque_raw: list[int] = field(default_factory=lambda: [0] * 6)
    # 重力补偿计算出的目标扭矩，按 J1~J6 排列。
    gravity_torque_raw: list[int] = field(default_factory=lambda: [0] * 6)
    motor_current_raw: list[int] = field(default_factory=lambda: [0] * 6)
    motor_current: list[float] = field(default_factory=lambda: [0.0] * 6)
    motor_velocity_raw: list[int] = field(default_factory=lambda: [0] * 6)
    bus_voltage: float = 0.0
    motor_feedback_seen: list[bool] = field(default_factory=lambda: [False] * 6)
    motor_feedback_count: list[int] = field(default_factory=lambda: [0] * 6)
    motor_statusword_raw: list[list[int]] = field(default_factory=lambda: [[0] * 8 for _ in range(6)])
    motor_statusword_seen: list[int] = field(default_factory=lambda: [0] * 6)
    motor_mode_raw: list[int] = field(default_factory=lambda: [0] * 6)
    ik_status: int = 0
    ik_status_valid: int = 0
    ik_validation_ok: int = 0
    ik_motion_state: int = 0
    ik_pose_pending: int = 0
    ik_joint_limit_fault: int = 0
    gripper_raw: bytes = b""
    feedback_count: int = 0
    updated_at: float = 0.0
    motor_enable: list[int] = field(default_factory=lambda: [0] * 6)
    motor_fault: list[int] = field(default_factory=lambda: [0] * 6)
    motor_error_code: list[int] = field(default_factory=lambda: [0] * 6)
    motor_error_category: list[int] = field(default_factory=lambda: [0] * 6)
    motor_error_flags: list[int] = field(default_factory=lambda: [0] * 6)
    motor_error_abort: list[int] = field(default_factory=lambda: [0] * 6)


@dataclass(frozen=True)
class GravityParams:
    """按轴保存在 MCU E2PROM 中的运行时重力参数。

    连杆长度由 MCU 模型固定定义，不属于此结构体，也不属于重力 CAN
    读写协议。
    """
    m: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0


def _be_i32(value: int) -> bytes:
    if value < -2147483648 or value > 2147483647:
        raise ValueError(f"int32 out of range: {value}")
    return int(value).to_bytes(4, byteorder="big", signed=True)


def _be_i16(value: int) -> bytes:
    if value < -32768 or value > 32767:
        raise ValueError(f"int16 out of range: {value}")
    return int(value).to_bytes(2, byteorder="big", signed=True)


def _read_be_i32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], byteorder="big", signed=True)


def _read_be_i16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="big", signed=True)


def _read_le_i32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], byteorder="little", signed=True)


def _read_le_i16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="little", signed=True)


def _deg_to_mdeg(value: float) -> int:
    return round(float(value) * 1000.0)


def _clamp_speed_percent(speed_percent: int) -> int:
    return max(0, min(100, int(speed_percent)))


def _le_i32(value: int) -> bytes:
    if value < -2147483648 or value > 2147483647:
        raise ValueError(f"int32 out of range: {value}")
    return int(value).to_bytes(4, byteorder="little", signed=True)


def _le_i16(value: int) -> bytes:
    if value < -32768 or value > 32767:
        raise ValueError(f"int16 out of range: {value}")
    return int(value).to_bytes(2, byteorder="little", signed=True)


def _le_u16(value: int) -> bytes:
    return int(value).to_bytes(2, byteorder="little", signed=False)


def _crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass
class H3001Arm:
    interface: str = "socketcan"
    channel: str = "can0"
    bitrate: int = 1_000_000
    data_bitrate: int = 5_000_000
    tty_baudrate: int = 2_000_000
    fd: bool = True

    bus: can.BusABC | None = field(default=None, init=False)
    state: ArmState = field(default_factory=ArmState, init=False)

    _rx_stop: threading.Event = field(default_factory=threading.Event, init=False)
    _rx_thread: threading.Thread | None = field(default=None, init=False)
    _lock: threading.RLock = field(default_factory=threading.RLock, init=False)
    rx_log: list[dict[str, object]] = field(default_factory=list, init=False)
    _rx_sequence: int = field(default=0, init=False)
    _feedback_timestamps: list[float] = field(default_factory=list, init=False)
    _gravity_condition: threading.Condition = field(default_factory=threading.Condition, init=False)
    _gravity_responses: dict[tuple[int, int], tuple[int, float]] = field(default_factory=dict, init=False)

    def connect(self, *, start_listener: bool = True, wait_feedback: bool = False, timeout: float = 2.0) -> None:
        kwargs: dict[str, object] = {
            "interface": self.interface,
            "channel": self.channel,
            "bitrate": self.bitrate,
        }
        if self.interface == "agx_cando":
            # agx_cando 默认启用本地回环；机械臂必须使用物理 CAN 总线，
            # 不能把已发送的控制命令再次作为接收帧反馈回来。
            kwargs["local_loopback"] = False
            kwargs["receive_own_messages"] = False
        if self.interface == "slcan":
            kwargs["tty_baudrate"] = self.tty_baudrate
        if self.fd:
            kwargs["fd"] = True
            kwargs["data_bitrate"] = self.data_bitrate

        try:
            bus = can.Bus(**kwargs)
        except Exception as exc:
            if self.interface == "gs_usb" and "access" in str(exc).lower():
                self._reset_gs_usb_devices()
                time.sleep(1.2)
                try:
                    bus = can.Bus(**kwargs)
                except Exception as retry_exc:
                    raise self._connect_error(retry_exc) from retry_exc
            else:
                raise self._connect_error(exc) from exc
        self.bus = bus
        if start_listener:
            self._rx_stop.clear()
            self._rx_thread = threading.Thread(target=self._rx_loop, args=(bus,), daemon=True)
            self._rx_thread.start()
        if wait_feedback:
            self.wait_for_feedback(timeout=timeout)

    def close(self) -> None:
        old_bus = self.bus
        self.bus = None
        self._rx_stop.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=0.6)
        if old_bus is not None:
            self._shutdown_bus(old_bus)
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=0.6)
            self._rx_thread = None
        gc.collect()

    def _connect_error(self, exc: Exception) -> RuntimeError:
        return RuntimeError(
            f"failed to open CAN bus interface={self.interface!r} channel={self.channel!r}. "
            "Make sure the CAN adapter is connected and not already opened by another program; "
            "for agx_cando use channel 0 and 1 Mbps. Original error: " + str(exc)
        )

    @staticmethod
    def _shutdown_bus(bus: can.BusABC) -> bool:
        gs_usb = getattr(bus, "gs_usb", None)
        usb_device = getattr(gs_usb, "gs_usb", None)
        if gs_usb is not None and usb_device is not None:
            try:
                if hasattr(bus, "stop_all_periodic_tasks"):
                    bus.stop_all_periodic_tasks(remove_tasks=True)
            except Exception:
                pass
            try:
                gs_usb.stop()
            except Exception:
                pass
            try:
                import usb.util

                try:
                    usb.util.release_interface(usb_device, 0)
                except Exception:
                    pass
                try:
                    usb_device.reset()
                    time.sleep(0.8)
                    usb_reset = True
                except Exception:
                    usb_reset = False
                usb.util.dispose_resources(usb_device)
            except Exception:
                usb_reset = False
            try:
                setattr(bus, "_is_shutdown", True)
            except Exception:
                pass
            return usb_reset

        try:
            bus.shutdown()
        except Exception:
            pass
        return False

    @staticmethod
    def _reset_gs_usb_devices() -> None:
        try:
            from gs_usb.gs_usb import GsUsb
            import usb.util
        except Exception:
            return

        for device in GsUsb.scan():
            usb_device = getattr(device, "gs_usb", None)
            if usb_device is None:
                continue
            try:
                usb_device.reset()
                time.sleep(0.8)
            except Exception:
                pass
            try:
                usb.util.dispose_resources(usb_device)
            except Exception:
                pass
        gc.collect()

    def __enter__(self) -> H3001Arm:
        self.connect()
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()

    def _require_bus(self) -> can.BusABC:
        if self.bus is None:
            raise RuntimeError("H3001Arm is not connected. Call connect() first.")
        return self.bus

    def _send(self, can_id: int, data: bytes | bytearray | Iterable[int]) -> None:
        payload = bytes(data)
        if len(payload) > 8:
            raise ValueError("current MCU AGX host protocol uses classic CAN frames, max 8 bytes")
        msg = can.Message(
            arbitration_id=can_id,
            data=payload,
            is_extended_id=False,
            is_fd=False,
            bitrate_switch=False,
        )
        self._require_bus().send(msg, timeout=1.0)

    def _send_motor_fd(self, can_id: int, data: bytes | bytearray) -> None:
        if not self.fd:
            raise RuntimeError("CAN-FD motor PDO requires H3001Arm(fd=True)")
        payload = bytes(data)
        if not payload or len(payload) > 64:
            raise ValueError("motor CAN-FD frame payload must contain 1~64 bytes")
        msg = can.Message(
            arbitration_id=can_id,
            data=payload,
            is_extended_id=False,
            is_fd=True,
            bitrate_switch=True,
        )
        self._require_bus().send(msg, timeout=1.0)

    def _send_motor_pdo(self, can_id: int, data: bytes | bytearray) -> None:
        if len(data) != MOTOR_PDO_DATA_LEN:
            raise ValueError("motor CAN-FD PDO must contain exactly 32 bytes")
        self._send_motor_fd(can_id, data)

    def _rx_loop(self, bus: can.BusABC) -> None:
        while not self._rx_stop.is_set():
            try:
                msg = bus.recv(timeout=0.2)
            except Exception:
                return
            if msg is not None:
                self.handle_message(msg)

    def handle_message(self, msg: can.Message) -> bool:
        """解析一帧 MCU 反馈并更新缓存状态。

        重力参数响应也在这里与等待 ``_gravity_exchange`` 的线程匹配。
        只有识别成功的反馈帧才返回 ``True``。
        """

        data = bytes(msg.data)
        can_id = msg.arbitration_id
        if can_id not in FEEDBACK_IDS:
            return False

        now = time.monotonic()
        with self._lock:
            self._rx_sequence += 1
            self.rx_log.append({
                "seq": self._rx_sequence,
                "time": time.strftime("%H:%M:%S"),
                "can_id": f"0x{can_id:03X}",
                "length": len(data),
                "data": data.hex(" ").upper(),
            })
            del self.rx_log[:-200]
            if can_id in {AGX_FB_GRAVITY_WRITE, AGX_FB_GRAVITY_READ} and len(data) >= 7:
                axis, field_id, status = data[0], data[1], data[2]
                value = struct.unpack("<f", data[3:7])[0]
                with self._gravity_condition:
                    self._gravity_responses[(axis, field_id)] = (status, value)
                    self._gravity_condition.notify_all()
                return True
            if MOTOR_PDO_RX_BASE <= can_id < MOTOR_PDO_RX_BASE + MOTOR_PDO_NODE_COUNT and len(data) >= 30:
                node_index = can_id - MOTOR_PDO_RX_BASE
                for axis in range(MOTOR_PDO_AXIS_COUNT):
                    index = node_index * MOTOR_PDO_AXIS_COUNT + axis
                    offset = axis * 10
                    self.state.motor_velocity_raw[index] = _read_le_i32(data, offset)
                    self.state.motor_position_raw[index] = _read_le_i32(data, offset + 4)
                    self.state.motor_torque_raw[index] = _read_le_i16(data, offset + 8)
                    self.state.motor_feedback_seen[index] = True
                    self.state.motor_feedback_count[index] += 1
                self.state.feedback_count += 1
                self.state.updated_at = now
                self._record_feedback_timestamp(now)
                return True
            if can_id == AGX_FB_STATE and len(data) >= 8:
                self.state.control_mode = data[0]
                self.state.arm_state = data[1]
                self.state.move_mode = data[2]
                self.state.teach_state = data[3]
                self.state.motion_state = data[4]
                self.state.trajectory_index = data[5]
                self.state.joint_limit_fault = data[6]
                self.state.joint_comm_fault = data[7]
            elif can_id == AGX_FB_POSE_XY and len(data) >= 8:
                self.state.pose[0] = _read_be_i32(data, 0) / 1000.0
                self.state.pose[1] = _read_be_i32(data, 4) / 1000.0
            elif can_id == AGX_FB_POSE_Z_RX and len(data) >= 8:
                self.state.pose[2] = _read_be_i32(data, 0) / 1000.0
                self.state.pose[3] = _read_be_i32(data, 4) / 1000.0
            elif can_id == AGX_FB_POSE_RY_RZ and len(data) >= 8:
                self.state.pose[4] = _read_be_i32(data, 0) / 1000.0
                self.state.pose[5] = _read_be_i32(data, 4) / 1000.0
            elif can_id == AGX_FB_JOINT_12 and len(data) >= 8:
                self._update_joint_pair(0, data)
            elif can_id == AGX_FB_JOINT_34 and len(data) >= 8:
                self._update_joint_pair(2, data)
            elif can_id == AGX_FB_JOINT_56 and len(data) >= 8:
                self._update_joint_pair(4, data)
            elif can_id == AGX_FB_GRIPPER:
                self.state.gripper_raw = data
            elif can_id == AGX_FB_BUS_VOLTAGE and len(data) >= 2:
                self.state.bus_voltage = int.from_bytes(data[0:2], "big") / 1000.0
            # 0x2AE/0x2AF 携带重力补偿计算目标扭矩，每帧三个大端有符号 int16 值。
            elif can_id in (AGX_FB_GRAVITY_TORQUE_123, AGX_FB_GRAVITY_TORQUE_456) and len(data) >= 6:
                first = 0 if can_id == AGX_FB_GRAVITY_TORQUE_123 else 3
                for offset in range(0, 6, 2):
                    self.state.gravity_torque_raw[first + offset // 2] = _read_be_i16(data, offset)
            elif AGX_FB_DRIVER_STATUS_BASE <= can_id < AGX_FB_DRIVER_STATUS_BASE + 6 and len(data) >= 6:
                index = can_id - AGX_FB_DRIVER_STATUS_BASE
                self.state.motor_enable[index] = (data[5] >> 6) & 0x01
                self.state.motor_fault[index] = (data[5] >> 5) & 0x01
            elif AGX_FB_DRIVER_STATUS_EXT_BASE <= can_id < AGX_FB_DRIVER_STATUS_EXT_BASE + 6 and len(data) >= 6:
                index = can_id - AGX_FB_DRIVER_STATUS_EXT_BASE
                self.state.motor_enable[index] = (data[5] >> 6) & 0x01
                self.state.motor_fault[index] = (data[5] >> 5) & 0x01
            elif can_id in (H3001_FB_IK_STATE, H3001_FB_IK_STATE_LEGACY) and len(data) >= 2:
                self.state.ik_status = data[1]
                self.state.ik_status_valid = 1
                self.state.ik_validation_ok = data[2] if len(data) >= 3 else 0
                self.state.ik_motion_state = data[3] if len(data) >= 4 else 0
                self.state.ik_pose_pending = data[4] if len(data) >= 5 else 0
                self.state.ik_joint_limit_fault = data[5] if len(data) >= 6 else 0
            elif AGX_FB_MOTOR_RAW_BASE <= can_id < AGX_FB_MOTOR_RAW_BASE + 6 and len(data) >= 8:
                self._update_motor_raw_feedback(can_id - AGX_FB_MOTOR_RAW_BASE, data)
            elif AGX_FB_STATUSWORD_RAW_BASE <= can_id < AGX_FB_STATUSWORD_RAW_BASE + 6:
                index = can_id - AGX_FB_STATUSWORD_RAW_BASE
                self.state.motor_statusword_raw[index] = list(data[:8]) + [0] * max(0, 8 - len(data))
            elif AGX_FB_MODE_RAW_BASE <= can_id < AGX_FB_MODE_RAW_BASE + MOTOR_PDO_NODE_COUNT and len(data) >= MOTOR_PDO_AXIS_COUNT:
                node = can_id - AGX_FB_MODE_RAW_BASE
                for axis in range(MOTOR_PDO_AXIS_COUNT):
                    self.state.motor_mode_raw[node * MOTOR_PDO_AXIS_COUNT + axis] = data[axis]
            elif AGX_FB_MOTOR_ERROR_BASE <= can_id < AGX_FB_MOTOR_ERROR_BASE + 6 and len(data) >= 8:
                index = can_id - AGX_FB_MOTOR_ERROR_BASE
                self.state.motor_error_code[index] = int.from_bytes(data[0:2], "little")
                self.state.motor_error_category[index] = data[2]
                self.state.motor_error_flags[index] = data[3]
                self.state.motor_error_abort[index] = int.from_bytes(data[4:8], "little")
                self.state.motor_statusword_seen[index] = 1
            elif AGX_FB_MOTOR_CURRENT_BASE <= can_id < AGX_FB_MOTOR_CURRENT_BASE + 6 and len(data) >= 4:
                index = can_id - AGX_FB_MOTOR_CURRENT_BASE
                current_raw = _read_be_i16(data, 0)
                self.state.motor_current_raw[index] = current_raw
                # 电机返回值按实际电流的 100 倍编码，SDK 对外提供安培值。
                self.state.motor_current[index] = current_raw / 100.0

            self.state.feedback_count += 1
            self.state.updated_at = now
            self._record_feedback_timestamp(now)
        return True

    def _record_feedback_timestamp(self, timestamp: float) -> None:
        self._feedback_timestamps.append(timestamp)
        cutoff = timestamp - 2.0
        self._feedback_timestamps[:] = [
            item for item in self._feedback_timestamps if item >= cutoff
        ]

    def _update_joint_pair(self, first_index: int, data: bytes) -> None:
        raw_a = _read_be_i32(data, 0)
        raw_b = _read_be_i32(data, 4)
        self.state.joint_raw[first_index] = raw_a
        self.state.joint_raw[first_index + 1] = raw_b
        self.state.joint_deg[first_index] = raw_a / 1000.0
        self.state.joint_deg[first_index + 1] = raw_b / 1000.0

    def _update_motor_raw_feedback(self, index: int, data: bytes) -> None:
        self.state.motor_position_raw[index] = _read_be_i32(data, 0)
        self.state.motor_torque_raw[index] = _read_be_i16(data, 4)
        self.state.motor_velocity_raw[index] = _read_be_i16(data, 6)
        self.state.motor_feedback_seen[index] = True
        self.state.motor_feedback_count[index] += 1

    def get_state(self) -> ArmState:
        with self._lock:
            return ArmState(
                control_mode=self.state.control_mode,
                arm_state=self.state.arm_state,
                move_mode=self.state.move_mode,
                teach_state=self.state.teach_state,
                motion_state=self.state.motion_state,
                trajectory_index=self.state.trajectory_index,
                motor_mode=self.state.motor_mode,
                motor_status=self.state.motor_status,
                pose=list(self.state.pose),
                joint_deg=list(self.state.joint_deg),
                joint_raw=list(self.state.joint_raw),
                motor_position_raw=list(self.state.motor_position_raw),
                motor_torque_raw=list(self.state.motor_torque_raw),
                gravity_torque_raw=list(self.state.gravity_torque_raw),
                motor_current_raw=list(self.state.motor_current_raw),
                motor_current=list(self.state.motor_current),
                motor_velocity_raw=list(self.state.motor_velocity_raw),
                bus_voltage=self.state.bus_voltage,
                motor_feedback_seen=list(self.state.motor_feedback_seen),
                motor_feedback_count=list(self.state.motor_feedback_count),
                motor_statusword_raw=[list(item) for item in self.state.motor_statusword_raw],
                motor_statusword_seen=list(self.state.motor_statusword_seen),
                motor_mode_raw=list(self.state.motor_mode_raw),
                ik_status=self.state.ik_status,
                gripper_raw=bytes(self.state.gripper_raw),
                feedback_count=self.state.feedback_count,
                updated_at=self.state.updated_at,
                motor_enable=list(self.state.motor_enable),
                motor_fault=list(self.state.motor_fault),
                motor_error_code=list(self.state.motor_error_code),
                motor_error_category=list(self.state.motor_error_category),
                motor_error_flags=list(self.state.motor_error_flags),
                motor_error_abort=list(self.state.motor_error_abort),
                joint_limit_fault=self.state.joint_limit_fault,
                joint_comm_fault=self.state.joint_comm_fault,
            )

    def wait_for_feedback(self, timeout: float = 2.0, *, min_count: int = 1) -> ArmState:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self.state.feedback_count >= min_count:
                    return self.get_state()
            time.sleep(0.02)
        raise TimeoutError("timed out waiting for MCU AGX feedback")

    # AGXArm 兼容的读取/状态接口。这些方法是别名，或是对 H3001 MCU
    # 已提供反馈数据的结构化视图。
    def is_connected(self) -> bool:
        return self.bus is not None

    def is_ok(self, *, feedback_timeout: float = 2.0) -> bool:
        if not self.is_connected():
            return False
        state = self.get_state()
        if state.updated_at <= 0.0 or time.monotonic() - state.updated_at > feedback_timeout:
            return False
        return not (
            any(state.motor_fault)
            or any(state.motor_error_code)
            or state.joint_limit_fault
            or state.joint_comm_fault
            or state.ik_joint_limit_fault
        )

    def get_fps(self) -> float:
        with self._lock:
            if len(self._feedback_timestamps) < 2:
                return 0.0
            elapsed = self._feedback_timestamps[-1] - self._feedback_timestamps[0]
            return (len(self._feedback_timestamps) - 1) / elapsed if elapsed > 0 else 0.0

    def get_joint_angles(self) -> list[float]:
        return self.get_joint_positions_deg()

    def get_flange_pose(self) -> list[float]:
        return self.get_pose()

    def get_arm_status(self) -> ArmState:
        return self.get_state()

    def get_driver_states(self) -> list[dict[str, int]]:
        state = self.get_state()
        return [
            {
                "enable": state.motor_enable[index],
                "fault": state.motor_fault[index],
                "error_code": state.motor_error_code[index],
                "error_category": state.motor_error_category[index],
                "error_flags": state.motor_error_flags[index],
                "error_abort": state.motor_error_abort[index],
            }
            for index in range(6)
        ]

    def get_motor_states(self) -> list[dict[str, int | float]]:
        state = self.get_state()
        return [
            {
                "position": state.motor_position_raw[index],
                "velocity": state.motor_velocity_raw[index],
                "torque": state.motor_torque_raw[index],
                "current": state.motor_current[index],
                "current_raw": state.motor_current_raw[index],
                "mode": state.motor_mode_raw[index],
                "statusword": state.motor_statusword_raw[index][0]
                if state.motor_statusword_raw[index]
                else 0,
            }
            for index in range(6)
        ]

    def query_motor_current(self, enabled: bool = True) -> None:
        """启动或停止 MCU 侧主动电机电流 SDO 轮询。"""
        self._send(AGX_ID_MOTOR_CURRENT_QUERY, bytes([1 if enabled else 0, 0, 0, 0, 0, 0, 0, 0]))

    def get_joint_positions_deg(self) -> list[float]:
        return self.get_state().joint_deg

    def get_joint_position_codes(self) -> list[int]:
        return self.get_state().joint_raw

    def get_pose(self) -> list[float]:
        return self.get_state().pose

    def send_target_pose(self, pose: Iterable[float]) -> None:
        values = [round(float(v)) for v in pose]
        if len(values) != 6:
            raise ValueError("target pose requires X,Y,Z,RX,RY,RZ")
        self._send(AGX_ID_POSE_XY, _be_i32(values[0]) + _be_i32(values[1]))
        time.sleep(0.02)
        self._send(AGX_ID_POSE_Z_RX, _be_i32(values[2]) + _be_i32(values[3]))
        time.sleep(0.02)
        self._send(AGX_ID_POSE_RY_RZ, _be_i32(values[4]) + _be_i32(values[5]))

    def send_linear_pose(self, pose: Iterable[float], *, speed_percent: int = 20) -> None:
        """发送目标位姿并启动 MOVE_L 笛卡尔直线运动。"""
        self.send_target_pose(pose)
        time.sleep(0.02)
        self.enter_can_mode(move_mode=MoveMode.L, speed_percent=speed_percent)

    def move_linear(self, pose: Iterable[float], *, speed_percent: int = 20) -> None:
        self.send_linear_pose(pose, speed_percent=speed_percent)

    def send_circular_poses(self, poses: Iterable[Iterable[float]], *, speed_percent: int = 20) -> None:
        """发送起点、中间点和终点位姿，并启动 MOVE_C 圆弧运动。"""
        points = [list(point) for point in poses]
        if len(points) != 3 or any(len(point) != 6 for point in points):
            raise ValueError("MOVE_C requires three [X,Y,Z,RX,RY,RZ] poses")
        # 上传圆弧点之前先选择 MOVE_C，否则 MCU 可能仍处于 MOVE_P，
        # 从而把收到的第一个完整位姿误触发为点到点运动。
        self.enter_can_mode(move_mode=MoveMode.C, speed_percent=speed_percent)
        time.sleep(0.02)
        for index, point in enumerate(points, start=1):
            self.send_target_pose([float(value) for value in point])
            time.sleep(0.02)
            self._send(AGX_ID_ARC_POINT, bytes([index]))
            time.sleep(0.02)

    def move_circular(self, poses: Iterable[Iterable[float]], *, speed_percent: int = 20) -> None:
        self.send_circular_poses(poses, speed_percent=speed_percent)

    def enter_can_mode(self, *, move_mode: MoveMode = MoveMode.P, speed_percent: int = 0) -> None:
        self._send(AGX_ID_MODE, bytes([0x01, int(move_mode), _clamp_speed_percent(speed_percent), 0, 0, 0, 0, 0]))

    def emergency_stop(self) -> None:
        self._send(AGX_ID_ESTOP, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]))

    def recover(self) -> None:
        self._send(AGX_ID_ESTOP, bytes([0x02, 0, 0, 0, 0, 0, 0, 0]))

    def enable_motor(self, motor: int) -> None:
        self._send(AGX_ID_MOTOR_ENABLE, bytes([self._validate_motor_or_all(motor), 0x02, 0, 0, 0, 0, 0, 0]))

    def disable_motor(self, motor: int) -> None:
        self._send(AGX_ID_MOTOR_ENABLE, bytes([self._validate_motor_or_all(motor), 0x01, 0, 0, 0, 0, 0, 0]))

    def enable_all_motors(self) -> None:
        self.enable_motor(7)

    def disable_all_motors(self) -> None:
        self.disable_motor(7)

    # AGXArm 兼容的控制别名。
    def enable(self) -> None:
        self.enable_all_motors()

    def disable(self) -> None:
        self.disable_all_motors()

    def reset(self) -> None:
        self.recover()

    def electronic_emergency_stop(self) -> None:
        self.emergency_stop()

    def switch_to_position_mode(self, joints_deg: Iterable[float] | None = None, *, speed_percent: int = 20) -> None:
        """使用固件当前的位置命令路径进行模式切换。

        MCU 接收到关节目标后会应用 MOTORMOD_POSITION，随后发送带有
        CAN + MOVE_J 的 AGX_ID_MODE 命令。
        """

        if joints_deg is None:
            joints_deg = self.get_joint_positions_deg()
        self.send_position_targets(joints_deg, speed_percent=speed_percent)

    def send_position_targets(self, joints_deg: Iterable[float], *, speed_percent: int = 20) -> None:
        values = [float(value) for value in joints_deg]
        if len(values) != 6:
            raise ValueError("send_position_targets expects 6 joint angles in degrees")
        self._send(AGX_ID_JOINT_12, _be_i32(_deg_to_mdeg(values[0])) + _be_i32(_deg_to_mdeg(values[1])))
        time.sleep(0.02)
        self._send(AGX_ID_JOINT_34, _be_i32(_deg_to_mdeg(values[2])) + _be_i32(_deg_to_mdeg(values[3])))
        time.sleep(0.02)
        self._send(AGX_ID_JOINT_56, _be_i32(_deg_to_mdeg(values[4])) + _be_i32(_deg_to_mdeg(values[5])))
        time.sleep(0.02)
        self.enter_can_mode(move_mode=MoveMode.J, speed_percent=speed_percent)

    def switch_to_torque_mode(self, torques: Iterable[float] | None = None) -> None:
        """切换到力矩模式并发送六个原始 int16 力矩目标值。"""

        if torques is None:
            self.zero_torque_all()
            return
        values = [round(float(value)) for value in torques]
        if len(values) != 6:
            raise ValueError("switch_to_torque_mode expects 6 raw torque targets")
        if all(value == 0 for value in values):
            self.zero_torque_all()
            return
        self.send_torque_targets(values)

    def send_torque_targets(self, torques: Iterable[float]) -> None:
        values = [round(float(value)) for value in torques]
        if len(values) != 6:
            raise ValueError("send_torque_targets expects 6 raw torque targets")
        self._send(
            AGX_ID_TORQUE_14,
            _be_i16(values[0]) + _be_i16(values[1]) + _be_i16(values[2]) + _be_i16(values[3]),
        )
        time.sleep(0.02)
        self._send(
            AGX_ID_TORQUE_56,
            _be_i16(values[4]) + _be_i16(values[5]) + bytes([0, 0, 0, 0]),
        )

    @staticmethod
    def _build_motor_pdo(
        axis_values: Iterable[tuple[int, int, int]],
        *,
        led: int = 0,
    ) -> bytes:
        values = list(axis_values)
        if len(values) != MOTOR_PDO_AXIS_COUNT:
            raise ValueError("motor PDO requires three (velocity, position, torque) tuples")
        data = bytearray(MOTOR_PDO_DATA_LEN)
        for axis, (velocity, position, torque) in enumerate(values):
            offset = axis * 10
            data[offset : offset + 4] = _le_i32(int(velocity))
            data[offset + 4 : offset + 8] = _le_i32(int(position))
            data[offset + 8 : offset + 10] = _le_i16(int(torque))
        data[30] = max(0, min(255, int(led)))
        data[31] = _crc8(bytes(data[:31]))
        return bytes(data)

    def zero_torque_all(self) -> None:
        self._send(AGX_ID_ZERO_TORQUE, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]))

    def set_zero_force_mode(self, enabled: bool) -> None:
        """启用或关闭 MCU 侧零力重力补偿。"""
        self._send(AGX_ID_ZERO_FORCE_MODE,
                   bytes([1 if enabled else 0, 0, 0, 0, 0, 0, 0, 0]))

    def _gravity_exchange(self, axis: int, field_id: int, *, value: float | None = None,
                          timeout: float = 2.0) -> float:
        """读写一个由 E2PROM 保存的重力参数字段。

        字段编号为 1=m、2=Gx、3=Gy、4=Gz。CAN 数据中的参数值为小端
        IEEE-754 单精度浮点数。
        """
        axis = int(axis)
        field_id = int(field_id)
        if axis < 1 or axis > 6:
            raise ValueError("gravity axis must be 1~6")
        if field_id < 1 or field_id > 4:
            raise ValueError("gravity field must be 1(m), 2(gx), 3(gy), or 4(gz)")
        if value is not None and not math.isfinite(float(value)):
            raise ValueError("gravity parameter must be finite")
        key = (axis, field_id)
        with self._gravity_condition:
            self._gravity_responses.pop(key, None)
        payload = bytearray(8)
        payload[0] = axis
        payload[1] = field_id
        if value is None:
            self._send(AGX_ID_GRAVITY_PARAM_READ, payload)
        else:
            payload[2:6] = struct.pack("<f", float(value))
            self._send(AGX_ID_GRAVITY_PARAM_WRITE, payload)
        deadline = time.monotonic() + timeout
        with self._gravity_condition:
            while key not in self._gravity_responses:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"timeout waiting for gravity parameter axis={axis}, field={field_id}")
                self._gravity_condition.wait(remaining)
            status, result = self._gravity_responses.pop(key)
        if status != 0:
            raise RuntimeError(f"MCU rejected gravity parameter axis={axis}, field={field_id}, status={status}")
        return result

    def set_gravity_params(self, axis: int, m: float = 0.0,
                           gx: float = 0.0, gy: float = 0.0, gz: float = 0.0,
                           *, timeout: float = 2.0) -> GravityParams:
        """写入一个轴后自动回读一次，确认保存的参数值。

        E2PROM 写入成功后，MCU 会同步更新 RAM 缓存，因此新参数无需重启
        即可立即用于重力补偿计算。
        """
        values = (m, gx, gy, gz)
        for field_id, value in enumerate(values, 1):
            self._gravity_exchange(axis, field_id, value=float(value), timeout=timeout)
        return self.get_gravity_params(axis, timeout=timeout)

    def get_gravity_params(self, axis: int, *, timeout: float = 2.0) -> GravityParams:
        """读取一个轴的 m/Gx/Gy/Gz 运行时参数。"""
        values = [self._gravity_exchange(axis, field_id, timeout=timeout) for field_id in range(1, 5)]
        return GravityParams(*values)

    def get_all_gravity_params(self, *, timeout: float = 2.0) -> dict[int, GravityParams]:
        """读取六个轴，每个轴只返回 m/Gx/Gy/Gz。"""
        return {axis: self.get_gravity_params(axis, timeout=timeout) for axis in range(1, 7)}

    def zero_encoder(self, motor: int = 7) -> None:
        """将单个编码器或六个编码器全部置零（motor=7 表示全部）。"""
        motor = int(motor)
        if motor < 1 or motor > 7:
            raise ValueError("zero_encoder motor must be 1~6, or 7 for all")
        self._send(AGX_ID_ENCODER_ZERO, bytes([motor, 0, 0, 0, 0, 0, 0, 0]))

    def return_zero(self) -> None:
        """执行一次逆解并移动到固件定义的回零位。"""
        self._send(AGX_ID_RETURN_ZERO, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]))

    def origin_position(self) -> None:
        """执行一次逆解并移动到固件定义的原点位置。"""
        self._send(AGX_ID_ORIGIN_POSITION, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]))

    def dexterous_position(self) -> None:
        """移动到固件定义的原始电机计数灵巧位置。"""
        self._send(AGX_ID_DEXTEROUS_POSITION, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]))

    def send_direct_position(self, joints_deg: Iterable[float]) -> None:
        """直接发送六个关节目标值，MCU 会按关节限位进行钳位。"""
        values = [float(value) for value in joints_deg]
        if len(values) != 6:
            raise ValueError("send_direct_position expects 6 joint angles in degrees")
        payload = [_be_i32(_deg_to_mdeg(value)) for value in values]
        self._send(AGX_ID_DIRECT_POSITION, payload[0] + payload[1])
        time.sleep(0.02)
        self._send(AGX_ID_DIRECT_POSITION_34, payload[2] + payload[3])
        time.sleep(0.02)
        self._send(AGX_ID_DIRECT_POSITION_56, payload[4] + payload[5])

    def set_mode(self, mode: MotorMode, parameters: Iterable[float] | None = None, *, speed_percent: int = 20) -> None:
        mode = MotorMode(mode)
        if mode == MotorMode.POSITION:
            self.switch_to_position_mode(parameters, speed_percent=speed_percent)
            return
        if mode == MotorMode.TORQUE:
            self.switch_to_torque_mode(parameters)
            return

    def set_teach_light(self, node: int, color: int) -> None:
        if node not in (0, 1, 2, 7):
            raise ValueError("teach light node must be 1, 2, or 7/0 for all")
        if color < 0 or color > 9:
            raise ValueError("teach light color must be 0~9")
        self._send(AGX_ID_TEACH_LIGHT, bytes([node & 0xFF, color & 0xFF, 0, 0, 0, 0, 0, 0]))

    def start_recording(self) -> None:
        self._send(AGX_ID_REPLAY_CONTROL, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]))

    def stop_recording(self) -> None:
        self._send(AGX_ID_REPLAY_CONTROL, bytes([0x02, 0, 0, 0, 0, 0, 0, 0]))

    def start_replay(self) -> None:
        self._send(AGX_ID_REPLAY_CONTROL, bytes([0x03, 0, 0, 0, 0, 0, 0, 0]))

    def stop_replay(self) -> None:
        self._send(AGX_ID_REPLAY_CONTROL, bytes([0x04, 0, 0, 0, 0, 0, 0, 0]))

    @staticmethod
    def _validate_motor(motor: int) -> int:
        motor = int(motor)
        if motor < 1 or motor > 6:
            raise ValueError("motor must be 1~6")
        return motor

    @staticmethod
    def _validate_motor_or_all(motor: int) -> int:
        motor = int(motor)
        if motor == 7:
            return motor
        return H3001Arm._validate_motor(motor)
