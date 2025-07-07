import serial
import struct
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import os
import csv

ser = serial.Serial(port='COM6', baudrate=9600, timeout=1)
t_run = 15
filename="two_gyro_overlap sim_on_arm.csv"

angle_command1 = bytes([0x50, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x99, 0x86])
angle_command2 = bytes([0x51, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x98, 0x57])
linear_acc_command1 = bytes([0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84])
# linear_acc_command2 = bytes([0x51, 0x03, 0x00, 0x34, 0x00, 0x03, 0x48, 0x55])

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

def read_gyroscope(t_count):
    try:

        t_record = time.perf_counter()
        # ask sensor angles #1
        ser.write(angle_command1)
        angle_frame1 = ser.read(11)

        # ask sensor angles #2
        ser.write(angle_command2)
        angle_frame2 = ser.read(11)

        # ask sensor angles #2
        ser.write(linear_acc_command1)
        linear_acc_frame1 = ser.read(11)

        t_record = time.perf_counter()-t_record
        print(f'Freq: {1/t_record:.2f} Hz')

        # parse frames
        roll1, pitch1, yaw1 = _parse_angle_frame(angle_frame1)
        roll2, pitch2, yaw2 = _parse_angle_frame(angle_frame2)
        X_acc1, Y_acc1, Z_acc1 = _parse_linear_acc_frame(linear_acc_frame1)

        print(f'{t_count:.2f}s:\t{roll1:.2f}, {pitch1:.2f}, {yaw1:.2f}, {roll2:.2f}, {pitch2:.2f}, {yaw2:.2f} (°); {X_acc1:.2f}, {Y_acc1:.2f}, {Z_acc1:.2f} (*9.81 m/s^2)')
        return roll1, pitch1, yaw1, roll2, pitch2, yaw2, X_acc1, Y_acc1, Z_acc1

    except Exception as e:
        # any problem (timeout, CRC, struct error, etc.) ends up here
        print(f"Gyro read error: {e}")
        return None

def _parse_angle_frame(frame: bytes):
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
    roll_raw, pitch_raw, yaw_raw = struct.unpack(">hhh", frame[3:9])

    # scale to degrees
    scale = 180.0 / 32768
    return (
        roll_raw  * scale,
        pitch_raw * scale,
        yaw_raw   * scale,
    )

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

def process_gyroscope(t_count, angleX_50, angleY_50, angleZ_50, 
                      angleX_51, angleY_51, angleZ_51, 
                      linear_accX_50, linear_accY_50, linear_accZ_50,
                      record):

    temp_record = [t_count]

    temp_record.extend([angleX_50, angleY_50, angleZ_50])
    tilting_angle50 = np.arccos( np.cos(angleX_50/180*np.pi) * np.cos(angleY_50/180*np.pi)) # in rad
    temp_record.extend([angleX_51, angleY_51, angleZ_51])
    tilting_angle51 = np.arccos( np.cos(angleX_51/180*np.pi) * np.cos(angleY_51/180*np.pi))
    temp_record.extend([tilting_angle50*180/np.pi, tilting_angle51*180/np.pi])

    tilting_diff = -tilting_angle50 + tilting_angle51 # *-1: depends on X rotation direction 
    Z_diff = angleZ_50 - angleZ_51 # in deg
    temp_record.extend([tilting_diff*180/np.pi, Z_diff])

    temp_record.extend([linear_accX_50, linear_accY_50, linear_accZ_50])

    record.append(temp_record)
    return record

def printcsv(filename, results):
    header = ['Time (s)']
    for gyro_num in range(2):
        header.extend([
            f'Roll/X rotation of gyro {gyro_num+50} (°)',
            f'Pitch/Y rotation of gyro {gyro_num+50} (°)',
            f'Yaw/Z rotation of gyro {gyro_num+50} (°)'
            ])
        gyro_num += 1
    for gyro_num in range(2):
        header.extend([
            f'Resultant tilting angle of gyro {gyro_num+50} from ground (°)'
            ])
        gyro_num += 1
    header.extend(["Difference in tilting (°)"])
    header.extend(["Difference in Z rotations (°)"])
    
    header.extend([
        f'X acclereation of gyro 50 (*9.81m/s^2)',
        f'Y acclereation of gyro 50 (*9.81m/s^2)',
        f'Z acclereation of gyro 50 (*9.81m/s^2)'
        ])

    timestamp = time.strftime("%m%d_%H%M%S")
    outputfile = "{}_{}".format(timestamp, filename)
    folder_path = os.path.join('gyro_records')
    os.makedirs(folder_path, exist_ok=True)
    output_file = os.path.join(folder_path, outputfile)

    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(results)
    return header

def plot_graphs(results, header):
    results = np.asarray(results)       
    timestamp = results[:, 0]    

    fig = plt.figure(figsize=(12, 8))
    outer = gridspec.GridSpec(
        nrows=1, ncols=2, width_ratios=[1, 2], wspace=0.30   # more space between cols
    )

    left = outer[0].subgridspec(3, 1, hspace=0.45) 
    left_pairs = [(1, 4), (2, 5), (3, 6)] 
    for row, (c1, c2) in enumerate(left_pairs):
        ax = fig.add_subplot(left[row, 0])
        ax.plot(timestamp, results[:, c1], label=header[c1])
        ax.plot(timestamp, results[:, c2], label=header[c2])
        ax.set_ylabel("Value")
        if row == 2:                       # bottom left plot gets the time label
            ax.set_xlabel("Time")
        ax.legend(fontsize="small")

    right        = outer[1].subgridspec(3, 1, hspace=0.45)
    right_groups = [(7, 8), (9, 10), (11, 12, 13)]   # 0-based indices

    for row, cols in enumerate(right_groups):
        ax = fig.add_subplot(right[row, 0])
        for c in cols:                               # any number of curves
            ax.plot(timestamp, results[:, c], label=header[c])
        ax.set_ylabel("Value")
        if row == 2:
            ax.set_xlabel("Time")
        ax.legend(fontsize="small")

    fig.suptitle("Gyroscope data vs Time", fontsize=14, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # keep room for the suptitle
    plt.show()
    return

def log_to_csv(t_run, t_buffer):
    t_count = 0
    t_start = time.perf_counter()
    while t_count <= t_run:
        angles = read_gyroscope()
        t_current = time.perf_counter()
        t_count = t_current - t_start
        if angles:
            roll, pitch, yaw = angles
            timestamp = time.strftime("%m%d_%H%M%S")
            filename="single_gyro_angles.csv"
            outputfile = "{}_{}".format(timestamp, filename)

            folder_path = os.path.join('..', 'gyro_records')
            os.makedirs(folder_path, exist_ok=True)
            output_file = os.path.join(folder_path, outputfile)
            # print(f"Writing to: {os.path.abspath(output_file)}")

            with open(outputfile, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write header if file is empty
                if file.tell() == 0:
                    writer.writerow(["Timestamp", "Roll (°)", "Pitch (°)", "Yaw (°)"])
                writer.writerow([t_count, f"{roll:.6f}", f"{pitch:.6f}", f"{yaw:.6f}"])
            print(f"Logged {t_count:.2f}s\t Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            time.sleep(0)


def main_script():
    t_start = time.perf_counter(); t_count = 0
    results = []
    while t_count < t_run:
        angleX_50, angleY_50, angleZ_50, \
            angleX_51, angleY_51, angleZ_51, \
                linear_accX_50, linear_accY_50, linear_accZ_50 = read_gyroscope(t_count)
        results = process_gyroscope(t_count, 
                                    angleX_50, angleY_50, angleZ_50, 
                                    angleX_51, angleY_51, angleZ_51,
                                    linear_accX_50, linear_accY_50, linear_accZ_50,
                                    results)
        # log_to_csv(t_run, t_buffer)
        t_count = time.perf_counter() - t_start
    ser.close()
    header = printcsv(filename, results)
    plot_graphs(results, header)
    return

main_script()