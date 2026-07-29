"""
Author: Dean Rowlett
Date: 2024-06-15
Purpose: Test sending multibyte commands to a serial device on COM4.

Message structure:
    Byte 0 = Start byte (0xAA)
    Byte 1 = Command byte
    Byte 2 = Sequence
    Byte 3-6 = Value (signed 32-bit integer, little endian)
    Byte 7 = Flags
    Byte 8 = CRC-8 checksum
    Byte 9 = End byte (0x55)

Example Message:
    [0xAA][CMD][SEQ][VAL0][VAL1][VAL2][VAL3][FLAGS][CRC8][0x55]

Commands:
    Home          = 0x01,
    MoveAbsolute  = 0x02,
    MoveRelative  = 0x03,
    StartTracking = 0x04,
    StopTracking  = 0x05,
    Stop          = 0x06,
    GetStatus     = 0x07,
    ClearFault    = 0x08,
    SetStandoff   = 0x09,
    Ping          = 0x10
"""

import serial
import time


START_BYTE = 0xAA
END_BYTE = 0x55

PACKET_SIZE = 10

CMD_HOME = 0x01
CMD_MOVE_ABSOLUTE = 0x02
CMD_MOVE_RELATIVE = 0x03
CMD_START_TRACKING = 0X04
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


class packetSender:
    def __init__(self, serial_port):
        self.serial_port = serial_port

        self.port = "COM4"
        self.baudrate = 115200
        self.serial_timeout = 20.0


    def send_packet(self, msg):
        """
        Send a binary packet to a serial device.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")

        if len(msg) != PACKET_SIZE:
            raise ValueError(
                f"Packet must be {PACKET_SIZE} bytes long. "
                f"Got {len(msg)}"
            )

        bytes_written = self.serial_port.write(msg)
        self.serial_port.flush()

        if bytes_written != PACKET_SIZE:
            raise RuntimeError(
                f"Only sent {bytes_written} of "
                f"{PACKET_SIZE} bytes."
            )


    def receive_packet(self):
        """
        Receive one binary packet from the serial device.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")

        msg = self.serial_port.read(PACKET_SIZE)

        if len(msg) != PACKET_SIZE:
            raise TimeoutError(
                f"Expected {PACKET_SIZE} bytes but "
                f"received {len(msg)}."
            )

        return msg


    def construct_packet(self, command, sequence, value, flags):
        """
        Construct a packet with the given command, sequence,
        value and flags.
        """
        start_byte = START_BYTE
        end_byte = END_BYTE

        value_bytes = value.to_bytes(
            4,
            byteorder="little",
            signed=True
        )

        flags_byte = flags.to_bytes(
            1,
            byteorder="little"
        )

        packet = bytearray()

        packet.append(start_byte)
        packet.append(command)
        packet.append(sequence)
        packet.extend(value_bytes)
        packet.append(flags_byte[0])
        packet.append(self.crc8(packet[1:8]))
        packet.append(end_byte)

        return packet


    def validate_packet(self, packet):
        """
        Check the packet start byte, end byte and CRC.
        """
        if len(packet) != PACKET_SIZE:
            return False

        if packet[0] != START_BYTE:
            print(
                f"Invalid start byte: "
                f"0x{packet[0]:02X}"
            )
            return False

        if packet[9] != END_BYTE:
            print(
                f"Invalid end byte: "
                f"0x{packet[9]:02X}"
            )
            return False

        calculated_crc = self.crc8(packet[1:8])

        if packet[8] != calculated_crc:
            print(
                f"Invalid response CRC. "
                f"Received 0x{packet[8]:02X}, "
                f"calculated 0x{calculated_crc:02X}"
            )
            return False

        return True


    def decode_packet(self, packet):
        """
        Decode the fields from a received packet.
        """
        command = packet[1]
        sequence = packet[2]

        value = int.from_bytes(
            packet[3:7],
            byteorder="little",
            signed=True
        )

        flags = packet[7]
        crc = packet[8]

        return command, sequence, value, flags, crc


    def connect_serial(self):
        """
        Connect to the serial port.
        """
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

            print(
                f"Connected to serial port {self.port} "
                f"at {self.baudrate}."
            )

            return True

        except serial.SerialException as error:
            self.serial_port = None

            print(
                f"Failed to connect to serial port "
                f"{self.port}: {error}"
            )

            return False


    def close_serial(self):
        """
        Close the serial port.
        """
        if self.serial_port is not None:
            if self.serial_port.is_open:
                self.serial_port.close()
                print("Serial port closed.")


    def crc8(self, data):
        """
        Calculate a CRC-8 checksum for the given data.
        """
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


print("Starting communication test")

sender = packetSender("COM4")

if not sender.connect_serial():
    raise SystemExit("Could not open serial port.")

sequence = 1

packet = sender.construct_packet(
    CMD_HOME,
    sequence,
    123456789,
    0x01
)

# Uncomment to deliberately corrupt one payload byte.
# The CRC is not recalculated after this change.
# packet[3] ^= 0x01

try:
    sender.send_packet(packet)

    print(
        f"Packet sent successfully: "
        f"{packet.hex(' ')}"
    )

    # ============================================================
    # RECEIVE IMMEDIATE ACK OR NACK
    # ============================================================

    response = sender.receive_packet()

    print(
        f"Initial response received: "
        f"{response.hex(' ')}"
    )

    if not sender.validate_packet(response):
        raise ValueError(
            "Initial response packet validation failed."
        )

    command, received_sequence, value, flags, crc = (
        sender.decode_packet(response)
    )

    print(f"Response command: 0x{command:02X}")
    print(f"Sequence: {received_sequence}")
    print(f"Value: {value}")
    print(f"Flags: 0x{flags:02X}")
    print(f"CRC: 0x{crc:02X}")

    if received_sequence != sequence:
        raise ValueError(
            f"Sequence mismatch. Sent {sequence}, "
            f"received {received_sequence}."
        )

    if command == RESPONSE_NACK:
        print("NACK received. Command rejected.")

        if flags == NACK_BAD_CRC:
            print("Reason: Bad CRC.")

        elif flags == NACK_UNKNOWN_COMMAND:
            print("Reason: Unknown command.")

        elif flags == NACK_INVALID_STATE:
            print("Reason: Invalid state.")

        elif flags == NACK_NOT_HOMED:
            print("Reason: Lifter is not homed.")

        elif flags == NACK_OUT_OF_RANGE:
            print("Reason: Requested value is out of range.")

        elif flags == NACK_BUSY:
            print("Reason: Lifter is busy.")

        else:
            print(
                f"Reason: Unknown NACK reason "
                f"0x{flags:02X}."
            )

    elif command == RESPONSE_ACK:
        print("ACK received. Homing command accepted.")
        print("Waiting for HomeComplete or Fault...")

        # ========================================================
        # WAIT FOR THE HOMING OPERATION TO FINISH
        # ========================================================

        completion_response = sender.receive_packet()

        print(
            f"Completion response received: "
            f"{completion_response.hex(' ')}"
        )

        if not sender.validate_packet(completion_response):
            raise ValueError(
                "Completion response packet validation failed."
            )

        (
            completion_command,
            completion_sequence,
            completion_value,
            completion_flags,
            completion_crc
        ) = sender.decode_packet(completion_response)

        print(
            f"Completion command: "
            f"0x{completion_command:02X}"
        )
        print(f"Sequence: {completion_sequence}")
        print(f"Value: {completion_value}")
        print(
            f"Flags: "
            f"0x{completion_flags:02X}"
        )
        print(
            f"CRC: "
            f"0x{completion_crc:02X}"
        )

        if completion_sequence != sequence:
            raise ValueError(
                f"Completion sequence mismatch. "
                f"Expected {sequence}, "
                f"received {completion_sequence}."
            )

        if completion_command == RESPONSE_HOME_COMPLETE:
            print("Homing completed successfully.")
            print(
                f"Final encoder position: "
                f"{completion_value}"
            )

        elif completion_command == RESPONSE_FAULT:
            print("Homing failed.")
            print(
                f"Encoder position at fault: "
                f"{completion_value}"
            )
            print(
                f"Fault code: "
                f"0x{completion_flags:02X}"
            )

        else:
            print(
                f"Unexpected completion response: "
                f"0x{completion_command:02X}"
            )

    else:
        print(
            f"Unexpected initial response command: "
            f"0x{command:02X}"
        )

except (
    RuntimeError,
    ValueError,
    TimeoutError,
    serial.SerialException
) as error:
    print(f"Communication test failed: {error}")

finally:
    sender.close_serial()