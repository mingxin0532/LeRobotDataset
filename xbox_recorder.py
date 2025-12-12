#!/usr/bin/env python3
import serial
import time
import sys
import pygame
import rospy  # <--- [新增] 引入 ROS
from std_msgs.msg import Float32  # <--- [新增] 引入消息类型

# --- GLOBAL CONFIGURATION ---
# Gripper Slave ID (Default is 0x09)
SLAVE_ID = 0x09
# Modbus Control Register Start Address (1000)
START_ADDRESS = 0x03E8

# --- HARDCODED COMMANDS ---
# Activation Request: 09 10 03 E8 00 03 06 00 00 00 00 00 00 73 30
gripper_activation = b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30"

# [注意] 请确保这里是您真实的端口，之前我们用的是 ttyUSB0，您的脚本里是 ttyUSB1
# 如果报错，请改回 /dev/ttyUSB0
SERIAL_PORT = "/dev/ttyUSB1"

# Initialize Serial Connection
try:
    ser = serial.Serial(port=SERIAL_PORT, baudrate=115200, timeout=0.05,  # timeout稍微改小一点提高频率
                        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
except Exception as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    sys.exit(1)


# ----------------------------------------------------------------------
## Modbus RTU CRC-16 Checksum Function (您的原函数)
# ----------------------------------------------------------------------

def calculate_modbus_crc(data: bytes) -> bytes:
    """Calculates the Modbus RTU CRC-16 checksum."""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    # Modbus RTU uses LSB (Little Endian) format
    return crc.to_bytes(2, byteorder='little')


# ----------------------------------------------------------------------
## Dynamic Modbus RTU Frame Generation
# ----------------------------------------------------------------------

def build_position_command(position_value: int, speed: int, force: int) -> bytes:
    """Constructs the write command."""
    # Core data body (excluding CRC)
    data_body = bytes([
        SLAVE_ID, 0x10,  # 1. ID 09, Function 10 (Write Multiple Registers)
        (START_ADDRESS >> 8) & 0xFF, START_ADDRESS & 0xFF,  # 2. Start Address 03 E8
        0x00, 0x03, 0x06,  # 3. Register Count 00 03, Byte Count 06
        0x09, 0x00,  # 4. Register 1: 09 00 (Activate and Execute - rACT/rGTO)
        0x00, position_value,  # 5. Register 2: Target Position (rPOS)
        speed, force  # 6. Register 3: Speed and Force (rSPE/rFOR)
    ])
    crc_value = calculate_modbus_crc(data_body)
    return data_body + crc_value


# [新增] 读取夹爪真实位置的功能
def read_gripper_position() -> float:
    """
    Reads the actual position (gPO) from the gripper.
    Target Register: 0x07D2 (2002). gPO is the High Byte.
    Modbus Function: 03 (Read Holding Registers)
    """
    # 构造读取指令: ID 03 07 D2 00 01
    data_body = bytes([SLAVE_ID, 0x03, 0x07, 0xD2, 0x00, 0x01])
    crc = calculate_modbus_crc(data_body)
    request = data_body + crc

    # 清空缓冲区防止读到旧数据
    ser.reset_input_buffer()

    ser.write(request)
    # 响应应该是 7 个字节: ID, Func, Bytes, DataHi, DataLo, CRC_L, CRC_H
    response = ser.read(7)

    if len(response) == 7:
        # Robotiq 寄存器 2002 的高字节是 gPO (真实位置)
        # response[3] 是 DataHi
        gpo = response[3]
        # 归一化 0-255 -> 0.0-1.0
        return gpo / 255.0
    else:
        return None  # 读取失败


# ----------------------------------------------------------------------
## Gripper Operation Functions
# ----------------------------------------------------------------------

def activate_gripper():
    """Sends the activation command to the gripper."""
    ser.write(gripper_activation)
    ser.read(10)  # Read the response
    time.sleep(1)  # Wait for initialization


def control_gripper_linear(position: float):
    """Controls the gripper position using a float (0.0 to 1.0)."""
    target_pos_int = int(position * 255)
    speed_int = 0xFF
    force_int = 0xFF

    command = build_position_command(target_pos_int, speed_int, force_int)
    ser.write(command)
    # 读取一下回音，防止缓冲区堆积
    ser.read(ser.in_waiting or 1)


def joystick_linear_control():
    # --- [新增] ROS 初始化 ---
    rospy.init_node('xbox_gripper_control', anonymous=True)
    # 这个 topic 名字必须和您的 my_robot_config.yaml 里的 gripper_state 保持一致
    pub = rospy.Publisher('/robotiq/position', Float32, queue_size=10)

    # 使用 ROS 的 Rate 来控制循环频率 (30Hz 配合 LeRobot)
    rate = rospy.Rate(30)

    # --- Pygame Initialization ---
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        rospy.logerr("No joystick detected!")
        # 即使没有手柄，我们通常也希望脚本继续跑(作为纯读取器)，但这里为了安全先退出
        # sys.exit(1)

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        rospy.loginfo(f"Joystick Connected: {joystick.get_name()}")
    except:
        rospy.logwarn("Joystick init failed or no joystick.")

    # Activate Gripper
    rospy.loginfo("Activating Gripper...")
    activate_gripper()
    rospy.loginfo("Gripper Activated. Publishing to ROS /robotiq/position")

    while not rospy.is_shutdown():  # 使用 ROS 的循环条件
        pygame.event.pump()  # Update joystick state

        # 1. [关键] 先读取并发布当前真实位置
        current_pos = read_gripper_position()
        if current_pos is not None:
            msg = Float32()
            msg.data = current_pos
            pub.publish(msg)

        # 2. 处理手柄控制逻辑
        if pygame.joystick.get_count() > 0:
            try:
                if joystick.get_numaxes() > 5:
                    # Axis 5 is Right Trigger
                    right_trigger_val = joystick.get_axis(5)
                    # Normalize -1.0~1.0 -> 0.0~1.0
                    normalized_val = (right_trigger_val + 1) / 2.0

                    if normalized_val > 0.05:
                        position_target_0_to_1 = normalized_val
                        control_gripper_linear(position_target_0_to_1)

                # Buttons
                if joystick.get_button(0):  # A Button
                    control_gripper_linear(0.0)
                if joystick.get_button(1):  # B Button
                    control_gripper_linear(1.0)

            except IndexError:
                pass

        # 3. 维持 30Hz 频率
        rate.sleep()


if __name__ == "__main__":
    try:
        joystick_linear_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")
        pygame.quit()