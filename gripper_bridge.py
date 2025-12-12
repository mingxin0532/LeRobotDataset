#!/usr/bin/env python3
import time
import rospy
import sys
from std_msgs.msg import Float32
from pymodbus.client import ModbusSerialClient

# ================= 配置区域 =================
# 请确保这与您 ls -l /dev/ttyUSB* 看到的一致
USB_PORT = '/dev/ttyUSB1'
#
# Robotiq 默认 Slave ID 是 9
SLAVE_ID = 9


# ===========================================

def connect_gripper():
    # 连接 Modbus RTU
    client = ModbusSerialClient(
        port=USB_PORT,
        baudrate=115200,
        stopbits=1,
        bytesize=8,
        parity='N',
        timeout=0.1
    )
    if not client.connect():
        rospy.logerr(f"无法连接到夹爪端口: {USB_PORT}")
        sys.exit(1)
    return client


def activate_gripper(client):
    # 检查并激活夹爪
    try:
        # 读取状态寄存器 0x07D0
        result = client.read_holding_registers(address=0x07D0, count=1, slave=SLAVE_ID)
        if not result.isError():
            status_byte = (result.registers[0] >> 8) & 0xFF
            if status_byte == 0x3B:  # 已激活
                rospy.loginfo("夹爪已就绪。")
                return
    except:
        pass

    rospy.loginfo("正在激活夹爪...")
    # 写入激活命令 (rACT=1, rGTO=1) -> 0xFF00 到 0x03E8
    client.write_register(address=0x03E8, value=0xFF00, slave=SLAVE_ID)
    time.sleep(2.0)


def get_position(client):
    try:
        # 读取位置寄存器 0x07D2 (包含 gPO)
        result = client.read_holding_registers(address=0x07D2, count=1, slave=SLAVE_ID)
        if not result.isError():
            gpo = (result.registers[0] >> 8) & 0xFF
            # 归一化 0-255 -> 0.0-1.0
            return gpo / 255.0
    except:
        pass
    return None


def main():
    rospy.init_node('robotiq_bridge')
    # 发布到 ROS Topic
    pub = rospy.Publisher('/robotiq/position', Float32, queue_size=10)

    client = connect_gripper()
    activate_gripper(client)

    rospy.loginfo("开始发布夹爪数据到 /robotiq/position (30Hz)")
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        pos = get_position(client)
        if pos is not None:
            msg = Float32()
            msg.data = pos
            pub.publish(msg)
        rate.sleep()

    client.close()


if __name__ == '__main__':
    main()