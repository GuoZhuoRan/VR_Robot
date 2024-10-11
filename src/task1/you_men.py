import serial
import time

# 串口配置
port = 'COM4'  # 串口名称，例如COM3、COM4等
baudrate = 9600  # 波特率
timeout = 1  # 超时设置

# 创建串口连接
ser = serial.Serial(port, baudrate, timeout=timeout)

# 检查串口是否打开
if ser.isOpen():
    print(f"串口{port}已打开")

try:
    while True:
        hex_data_list = []  # 初始化一个空列表用于存储十六进制数据
        while len(hex_data_list) < 4 and ser.in_waiting > 0:
            data = ser.read(size=1)  # 读取一个字节
            hex_data_list.append(hex(data[0]))  # 将字节转换为十六进制并添加到列表中
        if len(hex_data_list) == 4:  # 如果列表中有四个字节
            # 检查第一位是否为0xaa
            if hex_data_list[0] == '0xaa':
                # 计算校验和（只考虑最低8位）
                checksum = sum(int(hex_data_list[i], 16) for i in range(3)) % 0x100
                # 检查第四位是否为校验和（只考虑最低8位）
                if checksum == int(hex_data_list[3], 16):
                    # print("true")  # 校验通过，打印true
                    print(hex_data_list)
                    high_byte = int(hex_data_list[1], 16)
                    low_byte = int(hex_data_list[2], 16)
                    decimal_value = (high_byte << 8) | low_byte
                    print(decimal_value/292)  # 打印合并后的十进制数
                else:
                    print("false")
                    print(hex_data_list)
            # print(hex_data_list)
            ser.flushInput()  # 清空串口接收缓冲区

except KeyboardInterrupt:
    print("中断串口读取")

finally:
    ser.close()  # 关闭串口
    print(f"串口{port}已关闭")
