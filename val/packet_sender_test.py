"""
Author: Dean Rowlett
Purpose: Send commands to the lead-screw lifter over COM4 and print the
         response(s), handling every response shape the protocol uses.

Message structure:
    Byte 0 = Start byte (0xAA)
    Byte 1 = Command byte
    Byte 2 = Sequence
    Byte 3-6 = Value (signed 32-bit integer, little endian)
    Byte 7 = Flags
    Byte 8 = CRC-8 checksum
    Byte 9 = End byte (0x55)

Commands:
    Home          = 0x01,  -> ACK, then later HomeComplete/Fault
    MoveAbsolute  = 0x02,  -> ACK, then later MoveComplete/Fault
    MoveRelative  = 0x03,  -> ACK, then later MoveComplete/Fault
    StartTracking = 0x04,  -> ACK/NACK only
    StopTracking  = 0x05,  -> ACK/NACK only
    Stop          = 0x06,  -> ACK/NACK only
    GetStatus     = 0x07,  -> Status directly, no ACK stage
    ClearFault    = 0x08,  -> ACK/NACK only
    SetStandoff   = 0x09,  -> ACK/NACK only
    Ping          = 0x10   -> ACK/NACK only
"""

import serial
import time


START_BYTE = 0xAA
END_BYTE = 0x55

PACKET_SIZE = 10

CMD_HOME = 0x01
CMD_MOVE_ABSOLUTE = 0x02
CMD_MOVE_RELATIVE = 0x03
CMD_START_TRACKING = 0x04
CMD_STOP_TRACKING = 0x05
CMD_STOP = 0x06
CMD_GET_STATUS = 0x07
CMD_CLEAR_FAULT = 0x08
CMD_SET_STANDOFF = 0x09
CMD_PING = 0x10

RESPONSE_ACK = 0x80
RESPONSE_NACK = 0x81
RESPONSE_STATUS = 0x82
RESPONSE_HOME_COMPLETE = 0x83
RESPONSE_MOVE_COMPLETE = 0x84
RESPONSE_FAULT = 0x85

NACK_BAD_CRC = 0x01
NACK_UNKNOWN_COMMAND = 0x02
NACK_INVALID_STATE = 0x03
NACK_NOT_HOMED = 0x04
NACK_OUT_OF_RANGE = 0x05
NACK_BUSY = 0x06

NACK_REASONS = {
    NACK_BAD_CRC: "Bad CRC.",
    NACK_UNKNOWN_COMMAND: "Unknown command.",
    NACK_INVALID_STATE: "Invalid state.",
    NACK_NOT_HOMED: "Lifter is not homed.",
    NACK_OUT_OF_RANGE: "Requested value is out of range.",
    NACK_BUSY: "Lifter is busy.",
}

# Home / MoveAbsolute / MoveRelative send an ACK immediately, then a
# separate HomeComplete / MoveComplete / Fault packet once the motion
# finishes. Every other command's ACK/NACK *is* the whole response.
COMPLETION_COMMANDS = {CMD_HOME, CMD_MOVE_ABSOLUTE, CMD_MOVE_RELATIVE}

# GetStatus skips the ACK stage entirely and replies with Status directly.
DIRECT_RESPONSE_COMMANDS = {CMD_GET_STATUS}


class PacketSender:
    def __init__(self, port="COM4", baudrate=115200, timeout=20.0):
        self.port = port
        self.baudrate = baudrate
        self.serial_timeout = timeout
        self.serial_port = None

    def send_packet(self, msg):
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")

        if len(msg) != PACKET_SIZE:
            raise ValueError(
                f"Packet must be {PACKET_SIZE} bytes long. Got {len(msg)}"
            )

        bytes_written = self.serial_port.write(msg)
        self.serial_port.flush()

        if bytes_written != PACKET_SIZE:
            raise RuntimeError(
                f"Only sent {bytes_written} of {PACKET_SIZE} bytes."
            )

    def receive_packet(self):
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")

        msg = self.serial_port.read(PACKET_SIZE)

        if len(msg) != PACKET_SIZE:
            raise TimeoutError(
                f"Expected {PACKET_SIZE} bytes but received {len(msg)}."
            )

        return msg

    def construct_packet(self, command, sequence, value, flags):
        packet = bytearray()

        packet.append(START_BYTE)
        packet.append(command)
        packet.append(sequence)
        packet.extend(value.to_bytes(4, byteorder="little", signed=True))
        packet.append(flags & 0xFF)
        packet.append(self.crc8(packet[1:8]))
        packet.append(END_BYTE)

        return packet

    def validate_packet(self, packet):
        if len(packet) != PACKET_SIZE:
            return False

        if packet[0] != START_BYTE:
            print(f"Invalid start byte: 0x{packet[0]:02X}")
            return False

        if packet[9] != END_BYTE:
            print(f"Invalid end byte: 0x{packet[9]:02X}")
            return False

        calculated_crc = self.crc8(packet[1:8])

        if packet[8] != calculated_crc:
            print(
                f"Invalid response CRC. Received 0x{packet[8]:02X}, "
                f"calculated 0x{calculated_crc:02X}"
            )
            return False

        return True

    def decode_packet(self, packet):
        command = packet[1]
        sequence = packet[2]

        value = int.from_bytes(
            packet[3:7], byteorder="little", signed=True
        )

        flags = packet[7]
        crc = packet[8]

        return command, sequence, value, flags, crc

    def connect(self):
        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout,
            )

            time.sleep(2.0)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            print(f"Connected to serial port {self.port} at {self.baudrate}.")

            return True

        except serial.SerialException as error:
            self.serial_port = None
            print(f"Failed to connect to serial port {self.port}: {error}")
            return False

    def close(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")

    def crc8(self, data):
        crc = 0

        for byte in data:
            crc ^= byte

            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1

                crc &= 0xFF

        return crc


def describe_nack(flags):
    return NACK_REASONS.get(flags, f"Unknown NACK reason 0x{flags:02X}.")


def print_packet_fields(label, command, sequence, value, flags, crc):
    print(f"{label} command: 0x{command:02X}")
    print(f"Sequence: {sequence}")
    print(f"Value: {value}")
    print(f"Flags: 0x{flags:02X}")
    print(f"CRC: 0x{crc:02X}")


def send_command(sender, command, sequence, value=0, flags=0):
    """
    Send one command packet and handle whatever response shape it uses:
    direct Status, one-shot ACK/NACK, or ACK followed later by a
    completion/fault packet.
    """
    packet = sender.construct_packet(command, sequence, value, flags)
    sender.send_packet(packet)
    print(f"Packet sent successfully: {packet.hex(' ')}")

    response = sender.receive_packet()
    print(f"Initial response received: {response.hex(' ')}")

    if not sender.validate_packet(response):
        raise ValueError("Initial response packet validation failed.")

    resp_command, resp_sequence, resp_value, resp_flags, resp_crc = (
        sender.decode_packet(response)
    )

    print_packet_fields(
        "Response", resp_command, resp_sequence, resp_value, resp_flags, resp_crc
    )

    if resp_sequence != sequence:
        raise ValueError(
            f"Sequence mismatch. Sent {sequence}, received {resp_sequence}."
        )

    if command in DIRECT_RESPONSE_COMMANDS:
        if resp_command == RESPONSE_STATUS:
            print("Status received.")
        elif resp_command == RESPONSE_NACK:
            print(f"NACK received. Reason: {describe_nack(resp_flags)}")
        else:
            print(f"Unexpected response: 0x{resp_command:02X}")
        return

    if resp_command == RESPONSE_NACK:
        print(f"NACK received. Reason: {describe_nack(resp_flags)}")
        return

    if resp_command != RESPONSE_ACK:
        print(f"Unexpected initial response command: 0x{resp_command:02X}")
        return

    print("ACK received.")

    if command not in COMPLETION_COMMANDS:
        return

    print("Waiting for completion or fault...")

    completion = sender.receive_packet()
    print(f"Completion response received: {completion.hex(' ')}")

    if not sender.validate_packet(completion):
        raise ValueError("Completion packet validation failed.")

    comp_command, comp_sequence, comp_value, comp_flags, comp_crc = (
        sender.decode_packet(completion)
    )

    print_packet_fields(
        "Completion", comp_command, comp_sequence, comp_value, comp_flags, comp_crc
    )

    if comp_sequence != sequence:
        raise ValueError(
            f"Completion sequence mismatch. Expected {sequence}, "
            f"received {comp_sequence}."
        )

    if comp_command == RESPONSE_HOME_COMPLETE:
        print("Homing completed successfully.")
        print(f"Final encoder position: {comp_value}")

    elif comp_command == RESPONSE_MOVE_COMPLETE:
        print("Move completed successfully.")
        print(f"Final encoder position: {comp_value}")

    elif comp_command == RESPONSE_FAULT:
        print("Command failed.")
        print(f"Encoder position at fault: {comp_value}")
        print(f"Fault code: 0x{comp_flags:02X}")

    else:
        print(f"Unexpected completion response: 0x{comp_command:02X}")


if __name__ == "__main__":
    print("Starting communication test")

    sender = PacketSender()

    if not sender.connect():
        raise SystemExit("Could not open serial port.")

    try:
        # ------------------------------------------------------------
        # Edit these to choose what to send.
        # ------------------------------------------------------------
        # command = CMD_MOVE_RELATIVE
        # command = CMD_HOME
        # command = CMD_MOVE_RELATIVE
        # command = CMD_PING
        # command = CMD_START_TRACKING
        command = CMD_STOP_TRACKING
        # command = CMD_STOP

        value = -500
        flags = 0x01
        # ------------------------------------------------------------

        send_command(sender, command, sequence=1, value=value, flags=flags)

    except (
        RuntimeError,
        ValueError,
        TimeoutError,
        serial.SerialException,
    ) as error:
        print(f"Communication test failed: {error}")

    finally:
        sender.close()