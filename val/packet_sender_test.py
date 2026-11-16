import threading
import struct
import time
import sys
import serial
import argparse
START = 0xAA
END = 0x55
LENGTH = 10




COMMANDS = {
    'home': 0x01,
    'track': 0x04,
    'untrack': 0x05,
    'stop': 0x06,
    'status': 0x07,
    'clear': 0x08,
    'ping': 0x10,
    'telemetry on': 0x0A,
    'telemetry off': 0x0B,
}

STATES = {
    0: 'IDLE',
    1: 'HOMING',
    2: 'MOVING',
    3: 'TRACKING',
    4: 'STOPPING',
    5: 'FAULT',
}

NACKS = {
    1: 'Bad CRC',
    2: 'Unknown command',
    3: 'Invalid state',
    4: 'Not homed',
    5: 'Out of range',
    6: 'Busy',
}

def crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def packet(cmd, seq, value=0, flags=0):
    body = bytes([cmd, seq]) + struct.pack('<i', value) + bytes([flags])
    return bytes([START]) + body + bytes([crc8(body), END])


def decode(frame):
    if len(frame) != LENGTH or frame[0] != START or frame[-1] != END or crc8(frame[1:8]) != frame[8]:
        return None
    return frame[1], frame[2], struct.unpack('<i', frame[3:7])[0], frame[7]


def reader(port, stop_event):
    buffer = bytearray()
    last_distance = None
    while not stop_event.is_set():
        try:
            chunk = port.read(port.in_waiting or 1)
        except serial.SerialException as exc:
            print(f'\nSerial read failed: {exc}', flush=True)
            stop_event.set()
            return
        if not chunk:
            continue
        buffer.extend(chunk)
        while len(buffer) >= LENGTH:
            if buffer[0] != START:
                del buffer[0]
                continue
            parsed = decode(buffer[:LENGTH])
            if parsed is None:
                del buffer[0]
                continue
            del buffer[:LENGTH]
            cmd, seq, value, flags = parsed
            if cmd == 0x80:
                print(f'\nACK seq={seq}', flush=True)
            elif cmd == 0x81:
                print(f'\nNACK seq={seq}: {NACKS.get(flags, f"reason {flags}")}', flush=True)
            elif cmd == 0x82:
                print(f'\nSTATUS seq={seq}: encoder={value}, state={STATES.get(flags, flags)}', flush=True)
            elif cmd == 0x83:
                print(f'\nHOME COMPLETE seq={seq}: encoder={value}', flush=True)
            elif cmd == 0x84:
                print(f'\nMOVE COMPLETE seq={seq}: encoder={value}', flush=True)
            elif cmd == 0x85:
                print(f'\nFAULT seq={seq}: value={value}, flags=0x{flags:02X}', flush=True)
            elif cmd == 0x86:
                last_distance = value if flags & 0x80 else None
                print(f'\nDistance: {last_distance if last_distance is not None else "INVALID"} mm | state={STATES.get(flags & 0x7F, flags & 0x7F)}', flush=True)
            elif cmd == 0x87:
                print(f'Encoder: {value} counts | state={STATES.get(flags & 0x7F, flags & 0x7F)}', flush=True)
            elif cmd == 0x88:
                # Diagnostic bit packing is firmware-specific. Preserve raw fields
                # rather than incorrectly assigning meanings without the new source.
                print(f'Sensor diagnostic seq={seq}: raw_value={value} (0x{value & 0xFFFFFFFF:08X}), flags=0x{flags:02X}', flush=True)
            else:
                print(f'\nRX 0x{cmd:02X}: seq={seq}, value={value}, flags=0x{flags:02X}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    try:
        port = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1)
    except serial.SerialException as exc:
        sys.exit(f'Unable to open {args.port}: {exc}')
    stop_event = threading.Event()
    thread = threading.Thread(target=reader, args=(port, stop_event), daemon=True)
    sequence = 0

    def send(command, value=0):
        nonlocal sequence
        sequence = (sequence + 1) & 0xFF
        try:
            port.write(packet(command, sequence, value))
            print(f'Sent 0x{command:02X}, seq={sequence}, value={value}')
        except serial.SerialException as exc:
            print(f'Serial write failed: {exc}')
            stop_event.set()

    print(f'Connected to {args.port} at {args.baud}. Opening serial may reset Nano.')
    time.sleep(2)
    port.reset_input_buffer()
    thread.start()
    print('Commands: fixed, telemetry on/off, status, ping, track, untrack, stop, home, relative N, clear, exit')
    print('Start with fixed. Do not use track until sensor and direction are verified.')
    try:
        while not stop_event.is_set():
            try:
                line = input('lead-screw> ').strip().lower()
            except EOFError:
                break
            if line in ('exit', 'quit'):
                break
            if line == 'fixed':
                send(0x06)  # Stop motion first.
                send(0x0A)  # Enable telemetry.
            elif line.startswith('relative '):
                try:
                    send(0x03, int(line.split()[1]))
                except (ValueError, IndexError):
                    print('Usage: relative INTEGER_COUNTS')
            elif line in COMMANDS:
                send(COMMANDS[line])
            elif line:
                print('Unknown command. Commands: fixed, telemetry on/off, status, ping, track, untrack, stop, home, relative N, clear, exit')
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        if port.is_open:
            try:
                send(0x06)
                send(0x0B)
                time.sleep(0.2)
            except Exception:
                pass
        stop_event.set()
        thread.join(timeout=0.5)
        port.close()
        print('Serial port closed.')


if __name__ == '__main__':
    main()

