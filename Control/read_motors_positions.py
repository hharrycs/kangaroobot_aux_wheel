import serial
import numpy as np

from motor_control_def import Controller
from motor_process_responses import Responses

main_CAN = 253
motor_CAN = [11, 41, 51, 31, 21] # 2 R03 motors, 2 R01 motors
# motor_CAN = [1] 

# ser = serial.Serial('COM11', baudrate=921600, timeout=1)
# ser1 = serial.Serial('COM8', baudrate=921600, timeout=1)
ser = serial.Serial('COM15', baudrate=921600, timeout=1)
ser1 = serial.Serial('COM16', baudrate=921600, timeout=1)
bus = None
controller = Controller(ser, ser1, bus)
processor = Responses(motor_CAN)

t_delay = 1/1000 # between sending commands
t_response = 1/200 # before reading responses
current_value = 0
sent_current = np.ones(len(motor_CAN))*current_value
print(sent_current) 

detected_positions = []

# read positions
for num in range(len(motor_CAN)):
    controller.current_mode_initialise(main_CAN, motor_CAN[num]) 
responses = controller.send_current_command(main_CAN, motor_CAN, t_delay, t_response, sent_current)
data = processor.process_responses(responses)
print(data)
for sublist in data:
    detected_positions.append(sublist[1])

for i in range(len(motor_CAN)):
    controller.stop_command(main_CAN, motor_CAN[i]) 
print("")

for motor_ID, detected_position in zip(motor_CAN, detected_positions):
    print(f'motor {motor_ID} detected position: {detected_position/np.pi*180} deg')

zero_joint_offsets = np.array([0, 102.5, 0, 146.25, 0]) # unit degree
zero_motor_offsets = np.array([zero_joint_offsets[0], 
                               zero_joint_offsets[1] - zero_joint_offsets[2], 
                               -zero_joint_offsets[1] - zero_joint_offsets[2], 
                               -zero_joint_offsets[3] + zero_joint_offsets[4],  
                               zero_joint_offsets[3] + zero_joint_offsets[4]])
motor_angles = []

print("\nConsider offsets:")
for motor_ID, detected_position, offset in zip(motor_CAN, detected_positions, zero_motor_offsets):
    angle = detected_position/np.pi*180 + offset
    print(f'motor {motor_ID} detected position: {angle} deg')
    motor_angles.append(angle)

joint_angles = np.array([motor_angles[0], 
                         1/2*(motor_angles[1] - motor_angles[2]), 
                         1/2*(-motor_angles[1] - motor_angles[2]), 
                         1/2*(-motor_angles[3] + motor_angles[4]), 
                         1/2*(motor_angles[3] + motor_angles[4])])
print()
print(f'Detected joint positions: {joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}, {joint_angles[3]}, {joint_angles[4]}')
joint_angles = joint_angles/180*np.pi
print(f'Detected joint positions in rad: {joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}, {joint_angles[3]}, {joint_angles[4]}')
print()