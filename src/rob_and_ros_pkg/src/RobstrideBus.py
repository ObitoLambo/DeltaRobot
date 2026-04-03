from protocol import CommunicationType


class RobstrideBus:
    # Other methods and properties

    def write_id(self, motor: str, new_id: int):
        """
        Change motor CAN ID using the SET_DEVICE_ID communication type.
        """
        old_id = self.motors[motor].id  # Get the current motor ID
        print(f"⚡ Trying to change motor CAN ID from {old_id} → {new_id}...")

        # SEND command: using SET_DEVICE_ID (correct communication type)
        self.transmit(
            CommunicationType.SET_DEVICE_ID,  # Correct communication type for CAN ID change
            new_id,                           # New CAN ID to set
            old_id                            # Old CAN ID (target motor)
        )

        # RECEIVE response
        response = self.receive(timeout=0.2)
        if not response:
            raise RuntimeError("❌ No response while setting CAN ID")

        comm_type, extra_data, device_id, uuid = response
        print(f"✅ Motor responded | new_id={device_id}, uuid={uuid.hex()}")

        # 🔥 IMPORTANT: update local mapping
        self.motors[motor].id = device_id

        return device_id, uuid
