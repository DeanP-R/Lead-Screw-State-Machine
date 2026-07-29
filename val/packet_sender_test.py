"""
Author: Dean Rowlett
Date: 2024-06-15
Purpose: Test sending multibyte commands to a serial device on com4. 
Message structure: 
    Byte 0 = Start byte (0xAA)
    Byte 1 = Command byte
    Byte 2 = Sequence 
    Byte 3-6 = Value (signed 32 bit integer, little endian)
    Byte 7 = Flags
    Byte 8 = CRC-8 checksum (Calculated over COMMAND, SEQUENCE, VALUE and FLAGS)
    Byte 9 = End byte (0x55)
Example Message:
    [0xAA][CMD][SEQ][VAL0][VAL1][VAL2][VAL3][FLAGS][CRC8][0x55]
"""
import serial
import time

START_BYTE = 0xAA
END_BYTE = 0x55

PACKET_SIZE = 10

CMD_PING = 0x10

class packetSender:
    def __init__(self, serial_port):
        self.serial_port = serial_port

        self.port = "COM4"
        self.baudrate = 115200
        self.serial_timeout = 2.0


    def send_packet(self, msg):
        """
        Send a binary packet to a serial device.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")
        if len(msg) != PACKET_SIZE:
            raise ValueError(f"Packet must be {PACKET_SIZE} bytes long. got {len(msg)}")
        self.serial_port.write(msg)




    def construct_packet(self, command, sequence, value, flags):
        """
        Construct a packet with the given command, sequence, value, and flags.
        """
        start_byte = START_BYTE
        end_byte = END_BYTE
        value_bytes = value.to_bytes(4, byteorder='little', signed=True)
        flags_byte = flags.to_bytes(1, byteorder='little')

        packet = bytearray()
        packet.append(start_byte)
        packet.append(command)
        packet.append(sequence)
        packet.extend(value_bytes)
        packet.append(flags_byte[0])
        packet.append(self.crc8(packet[1:8])) 
        packet.append(end_byte)

        return packet


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
            print(f"Connected to serial port {self.port} at {self.baudrate}.")
            return True
        except serial.SerialException as e:
            self.serial_port = None
            print(f"Failed to connect to serial port {self.port}: {e}")
            return False


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

print("starting packet construction test")

sender = packetSender("COM4")
sender.connect_serial()

# print("attempting to construct a packet [0xAA][0x10][0x01][0x15][0xCD][0x5B][0x07][0x01][CRC8][0x55]")
# print("expected packet: aa100115cd5b0701d555")
packet = sender.construct_packet(CMD_PING, 1, 123456789, 0x01)
# print(f"resulted packet: {packet.hex()}")
try:
    sender.send_packet(packet)
    print(f"Packet sent successfully: {packet.hex()}")

except (RuntimeError, ValueError, serial.SerialException) as error:
    print(f"Something went wrong: {error}")