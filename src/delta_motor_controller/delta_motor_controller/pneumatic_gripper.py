#!/usr/bin/env python3
"""
pneumatic_gripper.py — Pneumatic solenoid gripper for delta pick-and-place.

Sends CAN frames directly via python-can (socketcan) — no can_driver_node needed.

CAN frame format (reverse-engineered from can_driver.py):
    arbitration_id = can_id (4)
    data[0] = 0x40
    data[1] = digital bitmask (unused here, always 0)
    data[2] = solenoid bitmask (bit 0 = solenoid1)
    is_extended_id = False

Hardware convention (verified with testing_solenoid.py):
    Board uses active-low outputs: data[2] bit=0 → solenoid energized (ON)
    solenoid1 bit=0 (False in data) →  GRIP   (energized)
    solenoid1 bit=1 (True  in data) →  RELEASE (de-energized)
"""

import time
import can


class PneumaticGripper:
    """
    Gripper controller for delta robot pick-and-place.
    Sends CAN frames directly — no ROS topic or can_driver_node required.

    Parameters
    ----------
    can_channel   : str   — SocketCAN channel (default 'can0')
    can_id        : int   — CAN ID of solenoid board (default 4)
    grip_settle_s : float — wait after grip command   (default 0.5 s)
    open_settle_s : float — wait after release command (default 0.3 s)
    """

    def __init__(self, can_channel: str = 'can0', can_id: int = 4,
                 grip_settle_s: float = 0.5,
                 open_settle_s: float = 0.3):
        self._can_channel = can_channel
        self._can_id = can_id
        self._grip_settle_s = grip_settle_s
        self._open_settle_s = open_settle_s
        self._bus = None

    # ── lifecycle ──────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open CAN bus and send initial release command (safe state)."""
        self._bus = can.interface.Bus(
            interface='socketcan',
            channel=self._can_channel,
            bitrate=1000000,
        )
        self.release(wait=False)

    def disconnect(self) -> None:
        """Release gripper and close CAN bus."""
        if self._bus is not None:
            self.release(wait=False)
            self._bus.shutdown()
            self._bus = None

    # ── control ────────────────────────────────────────────────────────────────

    def grip(self, wait: bool = True) -> None:
        """Energize solenoid1 to grip (active-low: bit=0 → ON)."""
        self._send(solenoid1=False)
        if wait:
            time.sleep(self._grip_settle_s)

    def release(self, wait: bool = True) -> None:
        """De-energize solenoid1 to release (active-low: bit=1 → OFF)."""
        self._send(solenoid1=True)
        if wait:
            time.sleep(self._open_settle_s)

    # ── backward-compatible aliases ────────────────────────────────────────────

    def close(self, wait: bool = True) -> None:
        self.grip(wait=wait)

    def open(self, wait: bool = True) -> None:
        self.release(wait=wait)

    # ── internal ───────────────────────────────────────────────────────────────

    def _send(self, solenoid1: bool = True, solenoid2: bool = True) -> None:
        # Active-low board: bit=0 → solenoid energized (ON), bit=1 → de-energized (OFF).
        # Defaults are True (OFF) so unused solenoids 2-6 stay de-energized.
        if self._bus is None:
            print('PneumaticGripper: send called before connect()')
            return
        data = [0] * 8
        data[0] = 0x40
        data[1] = 0
        data[2] = (int(solenoid1)       |   # bit 0
                   int(solenoid2) << 1  |   # bit 1
                   0b00111100)              # bits 2-5 always 1 (OFF)
        msg = can.Message(
            arbitration_id=self._can_id,
            data=data,
            is_extended_id=False,
        )
        try:
            self._bus.send(msg)
        except can.CanError as e:
            print(f'PneumaticGripper: CAN send failed: {e}')
