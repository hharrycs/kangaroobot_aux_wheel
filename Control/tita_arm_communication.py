# position control and adjustment
# communicate with gyroscope: tune mech arm based on tilting angle on TITA

import time
import serial
import numpy as np
import struct
import csv
import os

from motor_control_def import Controller
from motor_process_responses import Responses

filename="gyro_on_TITA control from_ground.csv"

main_CAN = 253
motor_CAN = [11, 41, 51, 31, 21] 
angle_command1 = bytes([0x50, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x99, 0x86]) # gyro angle reading command

ser1 = serial.Serial('COM11', baudrate=921600, timeout=1)
ser2 = serial.Serial('COM8', baudrate=921600, timeout=1)
ser3 = serial.Serial(port='COM6', baudrate=9600, timeout=1)
controller = Controller(ser1, ser2, None)
processor = Responses(motor_CAN)

t_MAX = 60
t_delay = 1/1000 # between sending commands
t_response = 1/200 # before reading responses
record = []

# Joint 2 tilting
# tilting_limit = np.array([-10*np.pi/180, -(np.pi/2 - np.arccos(0.4925/0.5494))]) 
tilting_limit = np.array([-10, -70]) # in degree
lowest_limit, highest_limit = np.sort(tilting_limit)

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
        ser3.reset_input_buffer()
        # t_record = time.perf_counter()
        ser3.write(angle_command1)
        angle_frame1 = ser3.read(11)

        # t_record = time.perf_counter()-t_record
        # print(f'Freq: {1/t_record:.2f} Hz')

        roll1, pitch1, yaw1 = _parse_angle_frame(angle_frame1)
        if roll1 >= 0:
            sign = +1
        else: 
            sign = -1
        tilting_angle = np.degrees(np.arccos( np.cos(roll1/180*np.pi) * np.cos(pitch1/180*np.pi))) *sign # in deg

        # print(f'{t_count:.2f}s:\t{roll1:.2f}, {pitch1:.2f}, {yaw1:.2f} (deg)')
        return roll1, pitch1, yaw1, tilting_angle

    except Exception as e:
        # any problem (timeout, CRC, struct error, etc.) ends up here
        print(f"Gyro read error: {e}")
        return None

def _parse_angle_frame(frame: bytes):
    # print(frame.hex())
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

def initialise_RSmotors(zero_pos_motor_offset):
    for num in range(len(motor_CAN)):
        print("Initialise motor CAN {} in main CAN {}, position control.".format(motor_CAN[num], main_CAN))
        controller.position_mode_initialise(main_CAN, motor_CAN[num])
    time.sleep(t_response)
    zero_pos_responses = []; motor_initial_pos = []; count = 0
    zero_pos_responses.append(ser1.read(ser1.in_waiting))
    time.sleep(t_response)
    zero_pos_responses.append(ser2.read(ser2.in_waiting))
    data = processor.process_responses(b''.join(zero_pos_responses)) 
    for sublist in data:
        motor_initial_pos.append(sublist[1] + zero_pos_motor_offset[count])
        count += 1
    joint_initial_pos = motor_to_joint_position(motor_initial_pos)
    print(f'\nInitial motor position: {np.array(motor_initial_pos)/np.pi*180}')
    print(f'Initial joint position: {np.array(joint_initial_pos)/np.pi*180}')

    return np.array(joint_initial_pos)[1]/np.pi*180 # in degree

def stop_RSmotors():
    for i in range(len(motor_CAN)):
        controller.stop_command(main_CAN, motor_CAN[i]) 
    return

def final_pos_on_table(): # only apply when testing on table
    final_joint_pos = np.array([0, -90, 0, 55, 0])*np.pi/180 - zero_pos_joint_offset

    final_motor_pos = np.hstack((desired_joint_position[0], 
                                 computed_differential_position(final_joint_pos[1:])))
    
    speed_commands, position_commands = controller.write_position_commands(main_CAN, motor_CAN, 
                                                                        desired_motor_speed, 
                                                                        final_motor_pos)
    _ = controller.send_position_commands(motor_CAN, t_delay, t_response, 
                                                speed_commands, position_commands)
    print("\nGo to final position (Test on optical table). Final motors pos in degree: ", np.degrees(final_motor_pos + zero_pos_motor_offset), "\n")
    time.sleep(5)

    return

def set_gyro_xy_zero():
    ser3.write(bytes([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1]))
    time.sleep(100e-3)
    ser3.write(bytes([0x50, 0x06, 0x00, 0x01, 0x00, 0x08, 0xD4, 0x4D]))
    time.sleep(100e-3)
    ser3.write(bytes([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B]))
    time.sleep(100e-3)

    print("Set gyroscope x-y reference angles succeeded!\n")
    return

def motor_to_joint_position(motor_positions):
    joint_positions = np.array([motor_positions[0], 
                            1/2*(motor_positions[1] - motor_positions[2]), 
                            1/2*(-motor_positions[1] - motor_positions[2]), 
                            1/2*(-motor_positions[3] + motor_positions[4]), 
                            1/2*(motor_positions[3] + motor_positions[4])])
    return joint_positions

def computed_differential_position(thetalist):
    ''' Input joint positions, 
        output motor position in differential gear system.'''
    motor1_theta = thetalist[0] - thetalist[1]
    motor2_theta = -thetalist[0] - thetalist[1]
    motor3_theta = -thetalist[2] + thetalist[3]
    motor4_theta = thetalist[2] + thetalist[3]

    differential_theta = [motor1_theta, motor2_theta, motor3_theta, motor4_theta] # motor 2,3,4,5
    return differential_theta

def computed_differential_speed(dthetalist):
    ''' Input joint speed, 
        output motor speed.'''
    motor1_dtheta = dthetalist[0] - dthetalist[1] 
    motor2_dtheta = -dthetalist[0] - dthetalist[1] 
    motor3_dtheta = -dthetalist[2] + dthetalist[3] 
    motor4_dtheta = dthetalist[2] + dthetalist[3] 

    differential_dtheta = [motor1_dtheta, motor2_dtheta, motor3_dtheta, motor4_dtheta] # motor 2,3,4,5
    return differential_dtheta

def printcsv(filename, results):
    header = ['Time (s)']
    header.extend([
        f'Resultant gyro tilting angle on TITA (°)'
        ])
    for motor_num in range(len(motor_CAN)):
        motor_num+=1
        header.extend([
            f'Desired joint {motor_num} position (deg)',
            f'Desired joint {motor_num} speed (deg/s)',
            f'Desired motor {motor_num} position (deg)',
            f'Desired motor {motor_num} speed (deg/s)',
            f'Detected motor {motor_num} position (deg)',
            f'Detected motor {motor_num} speed (deg/s)',
            f'Detected motor {motor_num} torque (Nm)' 
        ])
    header.extend(["Detected current sum (Arms)"])
    header.extend(["Responses of RS motors"])

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

TORQUE_CONSTANT_R01 = 1.22  
TORQUE_CONSTANT_R03 = 2.36 
torque_constants = np.where(np.array(motor_CAN) < 40, TORQUE_CONSTANT_R01, TORQUE_CONSTANT_R03)

speed_name = "speed_45degPerSec"
dtheta = np.pi/4
desired_joint_speed = np.ones(5)*dtheta
desired_motor_speed = desired_joint_speed # not important, desired joint position**

zero_pos_joint_offset = np.array([0, np.pi*102.5/180, 0, np.pi*146.25/180, 0])
zero_pos_joint_offset2to5 = computed_differential_position(zero_pos_joint_offset[1:])
zero_pos_motor_offset = np.hstack((zero_pos_joint_offset[0], zero_pos_joint_offset2to5))


''' Make a while-loop to continuously tune Joint 2 angle '''

'''
 0. Set gyroscope x-y zero.
 1. Detect tilting angle (resultant x-y rotation) of gyroscope on top of TITA 
 1a. In case gyro suddenly not detectable, still use previous gyro reading to maintain control, show error, keep extracting reading.
 2a. Check if the adjusted angle added to prev angle excceed the possible range of joint2
        Yes: Maintain Joint 2 angle, show error
        No: proceed
 2b. Update desired Joint 2 position: send 5R position commands and read their positions
 3. Return Joint 2 position as previous position

 Introduce a button that return to standby position after pressing
'''


prev_joint2_angle = initialise_RSmotors(zero_pos_motor_offset) # in deg
prev_desired_joint2_angle = -65
set_gyro_xy_zero()
time.sleep(5)

t_start = time.perf_counter(); t_count = 0
while t_count <= t_MAX:
    roll, pitch, yaw, tilting_angle = read_gyroscope(t_count) # in deg
    # tilting_angle = 0.5
    ''' +ve tilting: more -ve joint2, -ve tilting: more +ve joint 2 '''

    if abs(tilting_angle) < 1: # 1deg
        new_joint2_angle = prev_desired_joint2_angle
    else:
        new_joint2_angle = prev_joint2_angle + tilting_angle
    desired_joint2_angle = float(
        np.clip(new_joint2_angle,
                tilting_limit.min(), tilting_limit.max())
    )
    within_limit = lowest_limit <= new_joint2_angle <= highest_limit
    t_count = time.perf_counter() - t_start
    if within_limit != True:
        print(f'{t_count:.2f}s: Error! Desired Joint 2 is outside limits! Desired joint2 now: {desired_joint2_angle:.2f}')

    desired_joint_position = np.array([0, desired_joint2_angle, 0, 155, 0]) *np.pi/180 - zero_pos_joint_offset
    # desired_joint_position = np.array([0, -20, 0, 155, 0]) *np.pi/180 - zero_pos_joint_offset
    desired_motor_position = np.hstack((desired_joint_position[0], 
                                        computed_differential_position(desired_joint_position[1:])))
    # print(desired_motor_speed, np.degrees(desired_motor_position))
    speed_commands, position_commands = controller.write_position_commands(main_CAN, motor_CAN, 
                                                                        desired_motor_speed, 
                                                                        desired_motor_position)
    responses = controller.send_position_commands(motor_CAN, t_delay, t_response, 
                                                speed_commands, position_commands)
    data = processor.process_responses(responses) 

    t_process = time.perf_counter() - t_count - t_start
    temp_record = [t_count]
    temp_record.extend([tilting_angle])
    count_num = 0; joint2_cal = 0; detected_current_sum = 0
    for sublist in data:
        temp_record.extend([np.degrees(desired_joint_position[count_num] + zero_pos_joint_offset[count_num])]) # for joints in macro view
        temp_record.extend([np.degrees(desired_joint_speed[count_num])])
        temp_record.extend([np.degrees(desired_motor_position[count_num] + zero_pos_motor_offset[count_num])]) 
        temp_record.extend([np.degrees(desired_motor_speed[count_num])])
        temp_record.extend([np.degrees(sublist[1] + zero_pos_motor_offset[count_num])]) # motor detected position + zero_position_offset 
        temp_record.extend([np.degrees(sublist[2])]) # motor detected speed 
        temp_record.extend([sublist[3]]) # motor detected torque
        detected_current_sum += sublist[3]/torque_constants[count_num]
        if count_num == 1:
            motor2_angle = sublist[1] + zero_pos_motor_offset[count_num]
            joint2_cal += 1
        if count_num == 2:
            motor3_angle = sublist[1] + zero_pos_motor_offset[count_num]
            joint2_cal += 1
        count_num+=1
    if joint2_cal != 2:
        print("Error!!! Cannot detect joint 2 position.")
    # print(f'{t_count:.2f}s:\t\tProcess time: {t_process:.2f}s\t{1/t_process:.2f}Hz')

    temp_record.extend([detected_current_sum]) # detected current sum
    temp_record.extend([responses.hex()]) 

    prev_joint2_angle = np.degrees(1/2*(motor2_angle - motor3_angle)) # in deg
    print(f'{t_count:.2f}s Joint 2 angle now: {prev_joint2_angle:.2f}\tbuffer length = {len(responses)} bytes')
    temp_record.extend([prev_joint2_angle])
    record.append(temp_record)

    prev_desired_joint2_angle = desired_joint2_angle

final_pos_on_table() # only apply when testing on table

stop_RSmotors()
header = printcsv(filename, record)
# plot_graphs(record, header)