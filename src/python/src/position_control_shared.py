# =========================
# FILE 2: position_control_shared.py
# SINGLE CAN connection + SINGLE control loop for all motors
#
# IMPORTANT:
# - This file assumes you already have your RobStride CAN protocol code somewhere.
# - You MUST NOT create 3 RobstrideBus connections or 3 control-loop threads.
#
# Integration points are clearly marked below.
# =========================
import time
import threading
from types import SimpleNamespace
from dataclasses import dataclass

@dataclass
class MotorCalib:
    direction: int = 1         # +1 or -1
    homing_offset_deg: float = 0.0


# position_control_shared.py
from types import SimpleNamespace

class SharedCanTransport:
    def __init__(self, channel="can0"):
        self.channel = channel
        self._lock = threading.Lock()
        self._connected = False
        self._bus = None

        try:
            from robstride_dynamics.bus import RobstrideBus
            import inspect
            sig = inspect.signature(RobstrideBus.__init__)

            if "motors" in sig.parameters:
                # RobstrideBus expects a dict-like mapping where values have .id
                motors = {
                    1: SimpleNamespace(id=1),
                    2: SimpleNamespace(id=2),
                    3: SimpleNamespace(id=3),
                }
                self._bus = RobstrideBus(channel, motors=motors)
            else:
                self._bus = RobstrideBus(channel)

        except Exception as e:
            self._bus = None
            raise RuntimeError(f"RobstrideBus import/init failed: {e}")

    def connect(self):
        if self._connected:
            return
        print(f"🔍 Connecting to CAN channel {self.channel}...")
        if hasattr(self._bus, "connect"):
            self._bus.connect()
        self._connected = True
        print("✅ CAN connected (single shared transport).")

    def close(self):
        # IMPORTANT: call disconnect() explicitly so __del__ won't be relied on
        if not self._connected:
            return
        try:
            if hasattr(self._bus, "disconnect"):
                self._bus.disconnect()
            elif hasattr(self._bus, "close"):
                self._bus.close()
        finally:
            self._connected = False
            print("🛑 CAN closed cleanly.")


class DeltaMotorGroupController:
    """
    One controller that commands motor 1,2,3 with ONE timing loop.
    """

    def __init__(self, transport: SharedCanTransport, calib: dict[int, MotorCalib] | None = None, hz: float = 50.0):
        self.transport = transport
        self.hz = hz
        self.dt = 1.0 / hz
        self._running = False
        self._thread = None

        self.calib = calib or {
            1: MotorCalib(direction=1, homing_offset_deg=0.0),
            2: MotorCalib(direction=1, homing_offset_deg=0.0),
            3: MotorCalib(direction=1, homing_offset_deg=0.0),
        }

        # latest targets
        self._targets_deg = {1: 0.0, 2: 0.0, 3: 0.0}
        self._tlock = threading.Lock()

    def start(self):
        self.transport.connect()
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"🔄 Shared control loop started @ {self.hz:.1f} Hz (ONE loop for 3 motors)")

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.transport.close()

    def set_targets_deg(self, t1: float, t2: float, t3: float):
        with self._tlock:
            self._targets_deg[1] = t1
            self._targets_deg[2] = t2
            self._targets_deg[3] = t3

    def _apply_calib(self, motor_id: int, deg: float) -> float:
        c = self.calib[motor_id]
        return (deg + c.homing_offset_deg) * c.direction

    def _loop(self):
        while self._running:
            with self._tlock:
                t = dict(self._targets_deg)

            # Send 3 commands back-to-back INSIDE ONE loop tick
            for mid in (1, 2, 3):
                cmd_deg = self._apply_calib(mid, t[mid])
                self.transport.send_position_deg(mid, cmd_deg)

            time.sleep(self.dt)
