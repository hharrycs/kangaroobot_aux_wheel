import math
import struct

class Responses:
    def __init__(self, motor_CAN):
        self.motor_CAN = motor_CAN

    @staticmethod
    def linearinterpolation(min_y,max_y,steps,data_x):
        m = (max_y-min_y)/steps
        data_y = m*data_x + min_y
        return data_y

    def process_responses(self, response): 
        response = response.hex()
        if len(response) % 34 != 0: # Check if signals has 34*n digits
            signal_len = len(response)
            print("Wrong response length (l={}), something goes wrong!".format(signal_len))
            multi_motor_reading = []
            return multi_motor_reading
        else:
            signal = []; multi_motor_reading = []
            num_response = int(len(response) / 34)
            for num_iter in range(num_response):
                start_index = num_iter * 34
                end_index = start_index + 34  # ending index
                current_response = response[start_index:end_index]  
                # extended2real, response type
                extended_frame = current_response[4:12]
                parts = [extended_frame[i:i+2] for i in range(0, len(extended_frame), 2)]
                binary_parts = ''.join(format(int(part, 16), '08b') for part in parts)
                shifted_binary_parts = [binary_parts[0:5], binary_parts[5:13], \
                                        binary_parts[13:21], binary_parts[21:29]]
                decimal_values = [int(b, 2) for b in shifted_binary_parts]
                # motor feedback data
                data_frame = current_response[14:30]
                signal.append([decimal_values[0], decimal_values[2], data_frame]) # decimal_values[0]: communication_type; decimal_values[2]: detected_ID

            last_rows = [[2, motor, '0'] for motor in self.motor_CAN] # only read one set of data per motor every iteration
            for entry in signal:
                motor_value = entry[1]
                for i, motor in enumerate(self.motor_CAN):
                    if motor_value == motor:
                        last_rows[i] = entry
            for row in last_rows:
                motor_ID = row[1]
                data_frame = row[2]
                if data_frame != '0': 
                    parts = [data_frame[i:i+4] for i in range(0, len(data_frame), 4)]
                    binary_parts = [format(int(part, 16), '08b') for part in parts]
                    decimal_values = [int(b, 2) for b in binary_parts]
                    
                    angle = self.linearinterpolation(-4 * math.pi, 4 * math.pi, 65535, decimal_values[0])
                    temperature = decimal_values[3] / 10
                    if motor_ID < 40: # self-defined R01 ID range 
                        speed = self.linearinterpolation(-44, 44, 65535, decimal_values[1])
                        torque = self.linearinterpolation(-17, 17, 65535, decimal_values[2])
                    else: # R03 ID
                        speed = self.linearinterpolation(-20, 20, 65535, decimal_values[1])
                        torque = self.linearinterpolation(-60, 60, 65535, decimal_values[2])
                    single_motor_reading = [motor_ID, angle, speed, torque, temperature]
                else: 
                    single_motor_reading = [motor_ID, 0.0, 0.0, 0.0, 0.0]
                multi_motor_reading.append(single_motor_reading)
            return multi_motor_reading
    
    @staticmethod
    def process_gripper_responses(gripper_responses):
        # MG4010 response
        if gripper_responses and gripper_responses.arbitration_id == 0x148:
            temperature = struct.unpack("b", bytes([gripper_responses.data[1]]))[0]
            current = struct.unpack("<h", bytes([gripper_responses.data[2], 
                                                 gripper_responses.data[3]]))[0] * 66/4096
            torque = current*0.07
            speed = struct.unpack("<h", bytes([gripper_responses.data[4], 
                                               gripper_responses.data[5]]))[0] 
            encoder_position = struct.unpack("<H", bytes([gripper_responses.data[6], 
                                                          gripper_responses.data[7]]))[0] /65535 * 360
            motor_4010_reading = [encoder_position, speed, torque, current, temperature]
            return motor_4010_reading
        return [0, 0, 0, 0, 0]