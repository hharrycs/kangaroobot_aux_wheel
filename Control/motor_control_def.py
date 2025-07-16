import time
import struct
import can

class Controller:
    def __init__(self, ser, ser1, bus):
        self.ser = ser
        self.ser1 = ser1
        self.bus = bus

    def write_command(self, extended_frame, data_frame): # construct comments
        frame_header = b'\x41\x54'
        num_data_bits = b'\x08'
        end_frame = b'\x0d\x0a'
        command = frame_header + extended_frame + num_data_bits + data_frame + end_frame
        return command

    def stop_command(self, main_CAN, motor_CAN):
        extended_frame = self.main_motorCAN_extended(4, main_CAN, motor_CAN)
        data_frame = bytes.fromhex('00' * 8)  
        stop_command = self.write_command(extended_frame, data_frame)
        # print("\nSending stop command:", stop_command.hex())
        ser_num = self.ser if motor_CAN > 40 else self.ser1 # select ports
        ser_num.write(stop_command)
        time.sleep(0.1)
        response = ser_num.read(ser_num.in_waiting)
        print("Response received when stop:", response.hex()) 

    def main_motorCAN_extended(self, com_type, main_CAN, motor_CAN): # type 0(host), 3, 6, 17, 18
        com_binary = bin(com_type)[2:].zfill(5)
        main_CAN_binary = bin(main_CAN)[2:].zfill(16)
        motor_CAN_binary = bin(motor_CAN)[2:].zfill(8)
        extended_binary = com_binary + main_CAN_binary + motor_CAN_binary + '100'
        extended_hex = hex(int(extended_binary, 2))[2:].upper()
        # print(extended_hex)
        return bytes.fromhex(extended_hex)

    def set_zero_position(self, main_CAN, motor_CAN):
        extended_frame = self.main_motorCAN_extended(6, main_CAN, motor_CAN)
        data_frame = b'\x01\x01\xcd\x00\x00\x00\x00\x00'
        zero_position_command = self.write_command(extended_frame,data_frame)
        # print("Set zero position:", zero_position_command.hex())
        ser_num = self.ser if motor_CAN > 40 else self.ser1 # select ports
        ser_num.write(zero_position_command)
        time.sleep(0.01)
        # response = ser_num.read(ser_num.in_waiting)
        # print("Response received from setting zero position mode:", response.hex())

    def type18_dataframe(self, index, parameter): 
        index_hex = f"{index:X}".zfill(4) # byte0~1
        index_bytes_list = [index_hex[i:i+2] for i in range(0, len(index_hex), 2)]
        reversed_index = index_bytes_list[::-1]  # Reverse byte order
        index_hex = ''.join(reversed_index)

        parameter_hex = hex(parameter)[2:].zfill(8) # byte4~7, first convert to hex
        parameter_bytes_list = [parameter_hex[i:i+2] for i in range(0, len(parameter_hex), 2)]
        reversed_parameter = parameter_bytes_list[::-1]
        parameter_hex = ''.join(reversed_parameter)
        data_frame = index_hex + '0000' + parameter_hex
        return bytes.fromhex(data_frame)

    def run_mode(self, main_CAN, motor_CAN, mode_type): # type 18
        extended_frame = self.main_motorCAN_extended(18, main_CAN, motor_CAN)
        data_frame = self.type18_dataframe(0x7005, mode_type)
        run_mode_command = self.write_command(extended_frame, data_frame)
        # print("Send run command:", run_mode_command.hex())

        ser_num = self.ser if motor_CAN > 40 else self.ser1 # select ports
        ser_num.write(run_mode_command)
        time.sleep(0.01)
        response = ser_num.read(ser_num.in_waiting)
        # print("Response received from run mode:", response.hex())

    def start_mode(self, main_CAN, motor_CAN): # type 3: enable motor operation
        extended_frame = self.main_motorCAN_extended(3, main_CAN, motor_CAN)
        data_frame = bytes.fromhex('00' * 8)
        # extended_frame = b'\x18\x07\xe8\x5c'
        # data_frame = b'\x00\x00\x00\x00\x00\x00\x00\x00'
        start_command = self.write_command(extended_frame, data_frame)
        # print("Start mode command:", start_command.hex())

        ser_num = self.ser if motor_CAN > 40 else self.ser1 # select ports
        ser_num.write(start_command)  
        time.sleep(0.01)
        response = ser_num.read(ser_num.in_waiting)
        # print("Response received from start mode:", response.hex())

    def position_mode_initialise(self, main_CAN, motor_CAN):
        self.run_mode(main_CAN, motor_CAN, 1)
        self.start_mode(main_CAN, motor_CAN)

    def current_mode_initialise(self, main_CAN, motor_CAN): # call run and start mode
        self.run_mode(main_CAN, motor_CAN, 3)
        self.start_mode(main_CAN, motor_CAN)

    @staticmethod
    def float2hex(value): # for low byte first high byte last 
        packed_value = struct.pack('<f', float(value))  # Pack the float value as a little-endian float ('<f')
        hex_string = packed_value.hex()  # Convert the packed bytes to a hexadecimal string
        return bytes.fromhex(hex_string) # Convert hex string to bytes

    def hex2float(hex_input):
        # If input is a hex string, convert to bytes; if already bytes, use directly
        if isinstance(hex_input, str):
            byte_data = bytes.fromhex(hex_input)
        elif isinstance(hex_input, bytes):
            byte_data = hex_input
        else:
            raise ValueError("Input must be a hex string or bytes object")
        # Unpack the 4-byte little-endian float
        float_value = struct.unpack('<f', byte_data)[0]
        return float_value

    def obtain_ID(self): # communication type 0 
        extended_frame = b'\x00\x00' + b'\x00\x5c' 
        data_frame = b'\x00\x00\x00\x00\x00\x00\x00\x00'
        obtainID_command = self.write_command(extended_frame, data_frame)
        print("Obtain ID command:", obtainID_command.hex())
        self.ser.write(obtainID_command)

        time.sleep(1)
        response = self.ser.read(self.ser.in_waiting)
        print("Response received:", response.hex(), "\n")
        return obtainID_command

    def send_current_command(self, main_CAN, motor_CAN, t_delay, t_response, current):
        commands = [] ; responses = []
        for motor_ID, curr in zip(motor_CAN, current):
            current_byte = self.float2hex(curr)  # value to bytes
            extended_frame = self.main_motorCAN_extended(18, main_CAN, motor_ID) 
            dataframe_current = b'\x06\x70\x00\x00' + current_byte 
            current_command = self.write_command(extended_frame, dataframe_current)
            commands.append(current_command)
            time.sleep(t_delay) 

            ser_num = self.ser if motor_ID > 40 else self.ser1 # select ports
            ser_num.write(current_command) 
        # print("Send current command:", current_command.hex())
        time.sleep(t_response) 
        responses.append(self.ser.read(self.ser.in_waiting))
        responses.append(self.ser1.read(self.ser1.in_waiting))

        return b''.join(responses) # response
        # return "415410000fec087fff7f4e7fff01320d0a"

    def send_current_pos_commands(self, main_CAN, motor_CAN, t_delay, t_response, current, desired_theta, desired_dtheta, expected_theta):
        commands = [] ; responses = []
        for motor_ID, curr, position, speed in zip(motor_CAN, current, desired_theta, desired_dtheta):
            ser_num = self.ser if motor_ID > 40 else self.ser1 # select ports

            current_byte = self.float2hex(curr)  # value to bytes
            extended_frame = self.main_motorCAN_extended(18, main_CAN, motor_ID) 
            if motor_ID == 11:  # motor 1 position control
                v_byte = self.float2hex(1.5)
                pos_byte = self.float2hex(position)
                dataframe_v = b'\x17\x70\x00\x00' + v_byte
                dataframe_pos = b'\x16\x70\x00\x00' + pos_byte
                time.sleep(t_delay) 
                ser_num.write(self.write_command(extended_frame, dataframe_v)) 
                time.sleep(t_delay) 
                ser_num.write(self.write_command(extended_frame, dataframe_pos)) 
            else:  # other motors control with current
                dataframe = b'\x06\x70\x00\x00' + current_byte 
                current_command = self.write_command(extended_frame, dataframe)
                commands.append(current_command)
                time.sleep(t_delay) 
                ser_num.write(current_command) 

        time.sleep(t_response) 
        responses.append(self.ser.read(self.ser.in_waiting))
        responses.append(self.ser1.read(self.ser1.in_waiting))

        return b''.join(responses) # response

    def write_position_commands(self, main_CAN, motor_CAN, speed, position):
        speed_commands, position_commands = [[] for _ in range(2)]
        for motor_ID, v, pos in zip(motor_CAN, speed, position):
            extended_frame = self.main_motorCAN_extended(18, main_CAN, motor_ID)
            v_byte = self.float2hex(v)
            pos_byte = self.float2hex(pos)
            dataframe_v = b'\x17\x70\x00\x00' + v_byte
            dataframe_pos = b'\x16\x70\x00\x00' + pos_byte

            speed_commands.append(self.write_command(extended_frame, dataframe_v))
            position_commands.append(self.write_command(extended_frame, dataframe_pos))
        
        return speed_commands, position_commands

    def send_position_commands(self, motor_CAN, t_delay, t_response, 
                               speed_commands, position_commands):
        self.ser.reset_input_buffer()  # clear buffer
        self.ser1.reset_input_buffer() 
        responses = []
        for motor_ID, speed_command, position_command in zip(motor_CAN, 
                                                             speed_commands, position_commands):
            ser_num = self.ser if motor_ID > 40 else self.ser1 # select ports
            time.sleep(t_delay) 
            ser_num.write(speed_command) 
            time.sleep(t_delay) 
            ser_num.write(position_command) 
        time.sleep(t_response) 
        responses.append(self.ser.read(self.ser.in_waiting))
        responses.append(self.ser1.read(self.ser1.in_waiting))

        return b''.join(responses)

    # gripper control
    # @staticmethod
    # def get_CAN_bus():
    #     return can.interface.Bus(
    #         channel='PCAN_USBBUS1',
    #         interface= 'pcan',
    #         bitrate=1000000
    #     )
    
    @staticmethod
    def write_CAN_command(data_frame):
        return can.Message(
            arbitration_id=0x148,
            data=data_frame,
            is_extended_id=False,
            dlc=8
        )
    
    def gripper4010_on(self):
        data_frame = [0x88] + [0x00] * 7
        bus = self.bus
        msg = self.write_CAN_command(data_frame)
        bus.send(msg)
    
    def gripper4010_off(self):
        bus = self.bus
        data_frame_stop = [0x81] + [0x00] * 7
        msg_stop = self.write_CAN_command(data_frame_stop)
        bus.send(msg_stop)
        time.sleep(1/200)
        data_frame_off = [0x80] + [0x00] * 7
        msg_off = self.write_CAN_command(data_frame_off)
        bus.send(msg_off)
        bus.shutdown()
    
    @staticmethod
    def position_to_CAN(angle_deg):
        value = int(angle_deg * 1000) 
        hex_str = format(value, '08X') 
        byte_array = bytes.fromhex(hex_str)
        little_endian = byte_array[::-1].hex().upper()
        return [int(little_endian[i:i + 2], 16) for i in range(0, 8, 2)]
    
    @staticmethod
    def speed_to_CAN(speed):
        value = int(speed)
        hex_str = format(value, '04X')
        byte_array = bytes.fromhex(hex_str)
        return list(byte_array[::-1])

    def write_4010command(self, gripper_position, gripper_speed):
        position_bytes = self.position_to_CAN(gripper_position)
        speed_bytes = self.speed_to_CAN(gripper_speed)
        direction = 0x00 if gripper_position>50 else 0x01
        data_frame = [0xA6, direction] + speed_bytes + position_bytes
        return data_frame

    def send_4010command(self, gripper_position, gripper_speed):
        data_frame = self.write_4010command(gripper_position, gripper_speed)
        # t_start = time.time()
        bus = self.bus
        # print(f'Used time: {float(time.time() - t_start):.2f}s')
        msg = self.write_CAN_command(data_frame)
        bus.send(msg)
        response_4010 = bus.recv(timeout=1/500)
        return response_4010