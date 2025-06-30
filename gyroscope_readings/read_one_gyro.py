import serial
import struct
import csv
import time

import os

ser = serial.Serial(
    port='COM6', 
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
        print("read time:", t2)
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
            filename="CHANGE_NAME single_gyro_angles.csv"
            outputfile = "{}_{}".format(timestamp, filename)

            folder_path = os.path.join('..', 'gyro_records')
            os.makedirs(folder_path, exist_ok=True)
            output_file = os.path.join(folder_path, outputfile)
            # print(f"Writing to: {os.path.abspath(output_file)}")

            with open(filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write header if file is empty
                if file.tell() == 0:
                    writer.writerow(["Timestamp", "Roll (°)", "Pitch (°)", "Yaw (°)"])
                writer.writerow([t_count, f"{roll:.6f}", f"{pitch:.6f}", f"{yaw:.6f}"])
            print(f"Logged {t_count:.2f}s\t Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            time.sleep(0)

try:
    t_run = 5
    t_buffer = 1/100
    log_to_csv(t_run, t_buffer)
finally:
    ser.close()