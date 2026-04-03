import robstride.client
import can
import time
import numpy as np

CAN_PORT = 'can0'
MOTOR_IDS = [1, 2, 3]  # Motor IDs to read from
FULL_ROTATION = 2 * 3.14159  # Full rotation in radians

with can.interface.Bus(interface='socketcan', channel=CAN_PORT) as bus:
    client = robstride.client.Client(bus)
    
    try:
        # Enable motors in current control mode (allows manual rotation)
        print("Enabling motors in current control mode...")
        for motor_id in MOTOR_IDS:
            success = client.write_param(motor_id, 'run_mode', robstride.client.RunMode.Position)
            if success:
                print(f"Motor {motor_id} enabled successfully.")
            else:
                print(f"Failed to enable Motor {motor_id}")
            client.enable(motor_id)
        
        print("\nRotate the motors by hand. Press Ctrl+C to stop.")
        
        for motor_id in MOTOR_IDS:
            try:
                # Read position of the motor
                current_angle = client.read_param(motor_id, 'loc_ref')
                #current_angle = client.write_param(motor_id, 'loc_ref',0.5)
                if current_angle is not None:
                    degrees = (current_angle * 360.0) / FULL_ROTATION  # Convert radians to degrees
                    print(f"Motor {motor_id} - Current: {degrees:6.1f}°")
                else:
                    print(f"Motor {motor_id} - No valid position feedback received.")
            except Exception as e:
                print(f"Failed to read position for Motor {motor_id}: {e}")
              
            time.sleep(0.1)  # Short delay between each motor read to prevent too rapid polling.

    finally:
        print("\nDisabling motors...")
        for motor_id in MOTOR_IDS:
            client.disable(motor_id)
