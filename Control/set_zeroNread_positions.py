import serial
import numpy as np

from motor_control_def import Controller
from motor_process_responses import Responses

main_CAN = 253
motor_CAN = [11, 41, 51, 31, 21] # 2 R03 motors, 2 R01 motors
# motor_CAN = [41, 51] # 2 R01 motors
# motor_CAN = [11, 21, 31] # 2 R01 motors

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

for num in range(len(motor_CAN)):
    print("Set motor CAN {} as zero position.".format(motor_CAN[num]))
    controller.set_zero_position(main_CAN, motor_CAN[num])

# read positions
for num in range(len(motor_CAN)):
    controller.current_mode_initialise(main_CAN, motor_CAN[num]) 
responses = controller.send_current_command(main_CAN, motor_CAN, t_delay, t_response, sent_current)
data = processor.process_responses(responses)
for sublist in data:
    detected_positions.append(sublist[1])

for i in range(len(motor_CAN)):
    controller.stop_command(main_CAN, motor_CAN[i]) 
print("")

for motor_ID, detected_position in zip(motor_CAN, detected_positions):
    print(f'motor {motor_ID} detected position: {detected_position}')

print("Set zeros complete.")