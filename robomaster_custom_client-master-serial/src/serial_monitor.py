#!/usr/bin/env python3
"""
监听工具用于查看串口接收到的原始数据
"""

import serial
import time
import sys

COM_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

def main():
    print(f"串口监听工具")
    print(f"端口: {COM_PORT}")
    print(f"波特率: {BAUD_RATE}")
    print("-" * 50)
    print("按 Ctrl+C 退出\n")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"✓ 串口打开成功")
        print("等待接收数据...\n")
        
        byte_count = 0
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                byte_count += len(data)
                
                # 显示16进制
                hex_str = ' '.join([f'{b:02X}' for b in data])
                print(f"[{byte_count:06d}] ({len(data):3d} bytes) {hex_str}")
                
                # 尝试查找0xA5帧头
                for i, b in enumerate(data):
                    if b == 0xA5:
                        print(f"           ^^^ 找到帧头 0xA5 at position {i}")
            
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"✗ 串口打开失败: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n程序退出")
        if ser and ser.is_open:
            ser.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
