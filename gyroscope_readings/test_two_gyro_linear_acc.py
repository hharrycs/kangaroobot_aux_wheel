import serial
import struct
import time
import os
import csv

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

ser = serial.Serial(port='COM6', baudrate=9600, timeout=1)
t_run = 5

linear_acc_command1 = bytes([0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84])
linear_acc_command2 = bytes([0x51, 0x03, 0x00, 0x34, 0x00, 0x03, 0x48, 0x55])

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def read_gyro_linear_acc(t_count):
    try:
        # ask sensor #1
        t_record = time.perf_counter()
        ser.write(linear_acc_command1)
        frame1 = ser.read(11)

        # ask sensor #2
        ser.write(linear_acc_command2)
        frame2 = ser.read(11)
        t_record = time.perf_counter()-t_record
        print(f'Freq: {1/t_record:.2f}Hz')

        # parse both frames
        X_acc1, Y_acc1, Z_acc1 = _parse_linear_acc_frame(frame1)
        X_acc2, Y_acc2, Z_acc2 = _parse_linear_acc_frame(frame2)

        print(f'{t_count:.2f}s: {X_acc1:.2f}, {Y_acc1:.2f}, {Z_acc1:.2f}, {X_acc2:.2f}, {Y_acc2:.2f}, {Z_acc2:.2f} (*9.81 m/s^2)')
        return X_acc1, Y_acc1, Z_acc1, X_acc2, Y_acc2, Z_acc2

    except Exception as e:
        # any problem (timeout, CRC, struct error, etc.) ends up here
        print(f"Gyro read error: {e}")
        return None

def _parse_linear_acc_frame(frame: bytes):
    if len(frame) != 11:
        raise ValueError("incomplete frame")
    if frame[0] not in (0x50, 0x51):
        raise ValueError("bad start-byte / slave ID")
    if frame[1] != 0x03 or frame[2] != 0x06:
        raise ValueError("unexpected function-code or byte-count")

    expected_crc = calculate_crc(frame[:-2])
    if frame[-2:] != expected_crc:
        raise ValueError("CRC mismatch")

    # signed big-endian 16-bit integers
    X_acc_raw, Y_acc_raw, Z_acc_raw = struct.unpack(">hhh", frame[3:9])

    # scale to degrees
    scale = 16/ 32768
    return (
        X_acc_raw  * scale,
        Y_acc_raw * scale,
        Z_acc_raw   * scale,
    )

def main_script():
    t_start = time.perf_counter(); t_count = 0
    results = []
    while t_count < t_run:
        X_acc1, Y_acc1, Z_acc1, X_acc2, Y_acc2, Z_acc2 = read_gyro_linear_acc(t_count)
        t_count = time.perf_counter() - t_start
    ser.close()
    return

main_script()