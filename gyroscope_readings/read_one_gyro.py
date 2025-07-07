import serial
import struct
import csv
import time
import numpy as np

import os

t_run = 60
filename="TITA_standing single_gyro_angles.csv"

ser = serial.Serial(
    port='COM12', 
    baudrate=9600,  
    timeout=1
)

# Modbus command to read Roll, Pitch, Yaw 
command1 = bytes([0x50, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x99, 0x86])
command2 = bytes([0x51, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x98, 0x57])

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

def read_gyroscope():
    try:
        t1 = time.perf_counter()
        # Send command
        ser.write(command1)
        response = ser.read(11)
        
        # response = ser.read(22) # Read response (expecting 3 + 6 + 2 = 11 bytes: ID, func, count, 6 data bytes, 2 CRC)
        t2 = time.perf_counter() - t1
        # print("read time:", t2)                           
        if len(response) != 11:
            print("Error in Gyro1: Incomplete response")
            return None
        
        # Verify response format
        if response[0] not in [0x50, 0x51]:
            print("Error: Invalid response format")
            return None
        if response[1] != 0x03 or response[2] != 0x06:
            print("Error: Invalid response format")
            return None
        
        # Verify CRC
        expected_crc = calculate_crc(response[:-2])
        if response[-2:] != expected_crc:
            print("Error: CRC mismatch")
            return None
        
        # Extract Roll, Pitch, Yaw
        roll = struct.unpack('>h', response[3:5])[0]  # RollH << 8 | RollL
        pitch = struct.unpack('>h', response[5:7])[0]  # PitchH << 8 | PitchL
        yaw = struct.unpack('>h', response[7:9])[0]  # YawH << 8 | YawL
        
        # Convert to degrees
        roll_deg = roll / 32768 * 180
        pitch_deg = pitch / 32768 * 180
        yaw_deg = yaw / 32768 * 180
        
        return roll_deg, pitch_deg, yaw_deg
    
    except Exception as e:
        print(f"Error: {e}")
        return None

def set_gyro_xy_zero():
    ser.write(bytes([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1]))
    time.sleep(100e-3)
    ser.write(bytes([0x50, 0x06, 0x00, 0x01, 0x00, 0x08, 0xD4, 0x4D]))
    time.sleep(100e-3)
    ser.write(bytes([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B]))
    time.sleep(100e-3)

    print("Set gyroscope x-y reference angles succeeded!\n")
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
            if roll >= 0:
                sign = +1
            else: 
                sign = -1
            tilting_angle = np.degrees(np.arccos( np.cos(roll/180*np.pi) * np.cos(pitch/180*np.pi))) *sign 
            timestamp = time.strftime("%m%d_%H%M%S")
            outputfile = "{}_{}".format(timestamp, filename)

            folder_path = os.path.join('gyro_records')
            os.makedirs(folder_path, exist_ok=True)
            output_file = os.path.join(folder_path, outputfile)

            with open(filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write header if file is empty
                if file.tell() == 0:
                    writer.writerow(["Timestamp", "Roll (°)", "Pitch (°)", "Yaw (°)", "Resultant tilting (°)"])
                writer.writerow([t_count, f"{roll:.6f}", f"{pitch:.6f}", f"{yaw:.6f}", f"{tilting_angle:.6f}"])
            print(f"Logged {t_count:.2f}s\tRoll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°\t Resultant tilting {tilting_angle:.2f}")
            time.sleep(0)

try:
    t_buffer = 1/100
    set_gyro_xy_zero()
    ser.reset_input_buffer()
    time.sleep(3)
    log_to_csv(t_run, t_buffer)
finally:
    ser.close()