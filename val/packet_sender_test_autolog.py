#!/usr/bin/env python3
"""Interactive lead-screw serial monitor with automatic fixed-mode CSV logging and Ctrl+C cleanup."""
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import statistics
import struct
import sys
import threading
import time

import serial

START, END, LENGTH = 0xAA, 0x55, 10
COMMANDS = {'home': 0x01, 'track': 0x04, 'untrack': 0x05,
            'stop': 0x06, 'status': 0x07, 'clear': 0x08, 'ping': 0x10,
            'telemetry on': 0x0A, 'telemetry off': 0x0B}
STATES = {0: 'IDLE', 1: 'HOMING', 2: 'MOVING', 3: 'TRACKING',
          4: 'STOPPING', 5: 'FAULT'}
NACKS = {1: 'Bad CRC', 2: 'Unknown command', 3: 'Invalid state',
         4: 'Not homed', 5: 'Out of range', 6: 'Busy'}
FIELDS = ['timestamp_utc', 'elapsed_s', 'sample_seq', 'distance_mm',
          'valid', 'state', 'encoder_counts', 'diagnostic_raw', 'diagnostic_flags']


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xff if crc & 0x80 else (crc << 1) & 0xff
    return crc


def packet(cmd, seq, value=0, flags=0):
    body = bytes([cmd, seq]) + struct.pack('<i', value) + bytes([flags])
    return bytes([START]) + body + bytes([crc8(body), END])


def decode(frame):
    if len(frame) != LENGTH or frame[0] != START or frame[-1] != END or crc8(frame[1:8]) != frame[8]:
        return None
    return frame[1], frame[2], struct.unpack('<i', frame[3:7])[0], frame[7]


class Monitor:
    def __init__(self, port, baud, directory):
        self.port = serial.Serial(port, baud, timeout=0.1, write_timeout=1)
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.sequence = 0
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.pending = {}
        self.data_lock = threading.RLock()
        self.start_time = time.monotonic()
        self.rows = []
        self.logfile = None
        self.writer = None
        self.log_path = None
        self.directory = Path(directory)
        self.current_encoder = None
        self.count_packets = 0

    def send(self, cmd, value=0):
        with self.write_lock:
            self.sequence = (self.sequence + 1) & 0xff
            seq = self.sequence
            self.port.write(packet(cmd, seq, value))
            self.port.flush()
        print(f'Sent 0x{cmd:02X}, seq={seq}, value={value}', flush=True)

    def start_log(self):
        with self.data_lock:
            self._start_log()

    def _start_log(self):
        if self.logfile:
            print('Logging is already active.')
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        self.log_path = self.directory / f'lead_screw_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.logfile = self.log_path.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.logfile, fieldnames=FIELDS)
        self.writer.writeheader()
        self.rows = []
        self.start_time = time.monotonic()
        print(f'Recording to {self.log_path}', flush=True)

    def stop_log(self):
        with self.data_lock:
            self._stop_log()

    def _stop_log(self):
        if not self.logfile:
            return
        for row in self.pending.values():
            self.writer.writerow(row)
        self.pending.clear()
        self.logfile.flush()
        self.logfile.close()
        self.logfile = None
        self.writer = None
        rows = self.rows
        valid = [r['distance_mm'] for r in rows if r['valid']]
        invalid_count = len(rows) - len(valid)
        print(f'\nCSV saved: {self.log_path}')
        print(f'Samples: {len(rows)} | Valid: {len(valid)} | Invalid: {invalid_count}')
        if len(rows) > 1:
            elapsed = rows[-1]['elapsed_s'] - rows[0]['elapsed_s']
            if elapsed > 0:
                print(f'Observed sample rate: {(len(rows) - 1) / elapsed:.2f} Hz')
        if valid:
            print(f'Mean: {statistics.mean(valid):.3f} mm')
            if len(valid) > 1:
                print(f'Sample standard deviation: {statistics.stdev(valid):.3f} mm')
            print(f'Min / Max: {min(valid)} / {max(valid)} mm')
        encoders = [r['encoder_counts'] for r in rows if r['encoder_counts'] is not None]
        if encoders:
            print(f'Encoder first / last: {encoders[0]} / {encoders[-1]} counts')
        print(flush=True)

    def record(self, seq, value, flags):
        with self.data_lock:
            self._record(seq, value, flags)

    def _record(self, seq, value, flags):
        if not self.logfile:
            return
        now = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        row = {'timestamp_utc': now, 'elapsed_s': round(time.monotonic() - self.start_time, 4),
               'sample_seq': seq, 'distance_mm': value if flags & 0x80 else None,
               'valid': bool(flags & 0x80), 'state': STATES.get(flags & 0x7f, str(flags & 0x7f)),
               'encoder_counts': None, 'diagnostic_raw': None, 'diagnostic_flags': None}
        self.rows.append(row)
        # The firmware sends distance, encoder and diagnostics with matching sequence.
        # Finalise older samples after their companion packets have arrived.
        for old_seq in list(self.pending):
            self.writer.writerow(self.pending.pop(old_seq))
        self.logfile.flush()
        self.pending[seq] = row

    def reader(self):
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                chunk = self.port.read(self.port.in_waiting or 1)
            except (serial.SerialException, OSError) as exc:
                print(f'\nSerial read failed: {exc}', flush=True)
                self.stop_event.set()
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
                    self.record(seq, value, flags)
                    display = f'{value} mm' if flags & 0x80 else 'INVALID'
                    print(f'\nDistance: {display} | state={STATES.get(flags & 0x7F, flags & 0x7F)}', flush=True)
                elif cmd == 0x87:
                    self.current_encoder = value
                    with self.data_lock:
                        if seq in self.pending:
                            self.pending[seq]['encoder_counts'] = value
                    print(f'Encoder: {value} counts', flush=True)
                elif cmd == 0x88:
                    with self.data_lock:
                        if seq in self.pending:
                            self.pending[seq]['diagnostic_raw'] = value
                            self.pending[seq]['diagnostic_flags'] = flags
                    print(f'Sensor diagnostic seq={seq}: raw_value={value} (0x{value & 0xFFFFFFFF:08X}), flags=0x{flags:02X}', flush=True)
                else:
                    print(f'\nRX 0x{cmd:02X}: seq={seq}, value={value}, flags=0x{flags:02X}', flush=True)

    def close(self):
        print('\nStopping motor and disabling telemetry...', flush=True)
        if self.port.is_open:
            try:
                self.send(0x06)
                self.send(0x0B)
                time.sleep(0.3)
            except (serial.SerialException, OSError) as exc:
                print(f'Shutdown commands could not be confirmed: {exc}', flush=True)
        self.stop_event.set()
        self.thread.join(timeout=1)
        self.stop_log()
        if self.port.is_open:
            self.port.close()
        print('Serial port closed.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--log-dir', default='val/logs')
    args = parser.parse_args()
    try:
        monitor = Monitor(args.port, args.baud, args.log_dir)
    except (serial.SerialException, OSError) as exc:
        sys.exit(f'Unable to open {args.port}: {exc}')
    print(f'Connected to {args.port} at {args.baud}. Opening serial may reset Nano.')
    try:
        time.sleep(2)
        monitor.port.reset_input_buffer()
        monitor.thread.start()
        print('Commands: fixed (auto-logs), log start, log stop, track, untrack, telemetry on/off, status, ping, stop, home, relative N, clear, exit')
        print('Ctrl+C or exit saves CSV and sends STOP + TELEMETRY OFF.')
        while not monitor.stop_event.is_set():
            try:
                line = input('lead-screw> ').strip().lower()
            except EOFError:
                break
            if line in ('exit', 'quit'):
                break
            if line == 'fixed':
                monitor.send(0x06)
                monitor.start_log()  # Start recording before telemetry is enabled.
                monitor.send(0x0A)
            elif line == 'log start':
                monitor.start_log()
            elif line == 'log stop':
                monitor.stop_log()
            elif line.startswith('relative '):
                try:
                    monitor.send(0x03, int(line.split()[1]))
                except (ValueError, IndexError):
                    print('Usage: relative INTEGER_COUNTS')
            elif line in COMMANDS:
                monitor.send(COMMANDS[line])
            elif line:
                print('Unknown command.')
    except KeyboardInterrupt:
        print('\nInterrupted; saving log.', flush=True)
    finally:
        monitor.close()


if __name__ == '__main__':
    main()
