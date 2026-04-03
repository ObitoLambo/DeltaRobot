#!/usr/bin/env python3
import time
from robstride_dynamics import RobstrideBus, Motor


# Define motor(s)
motors = {
    2: Motor(id=2, model="rs-00")
}

# Initialize CAN bus
bus = RobstrideBus('can0', motors)

motor_id = 2

print("Reading REAL RobStride00 motor position...")

while True:
    frame = bus.read_frame(motor_id)

    if frame:
        position = frame["position"]     # radians
        velocity = frame["velocity"]     # rad/s
        torque   = frame["torque"]       # Nm

        print(
            f"Pos: {position:.6f} rad | "
            f"Vel: {velocity:.6f} rad/s | "
            f"Torque: {torque:.6f} Nm"
        )

    time.sleep(0.01)
