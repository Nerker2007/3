"""
ESP32 三轴步进滑台 键盘点动控制
按键映射：
  W/S - Z轴 +/-
  A/Q - X轴 +/-
  D/E - Y轴 +/-
每次移动5mm，速度F100
ESC - 退出
H - 归零
? - 查询状态
"""

import serial
import serial.tools.list_ports
import sys
import time

# Windows下使用msvcrt，Linux/Mac使用termios
if sys.platform == 'win32':
    import msvcrt
else:
    import tty
    import termios

# === 配置 ===
BAUD_RATE = 115200
STEP_MM = 5.0
FEED_RATE = 100  # mm/min


def find_esp32_port():
    """自动查找ESP32串口"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        if 'cp210' in desc or 'ch340' in desc or 'usb' in desc or 'serial' in desc:
            return port.device
    # 如果没找到，列出所有端口让用户选择
    if ports:
        print("可用串口:")
        for i, port in enumerate(ports):
            print(f"  {i}: {port.device} - {port.description}")
        idx = input("选择串口编号: ")
        return ports[int(idx)].device
    return None


def get_key():
    """跨平台获取按键"""
    if sys.platform == 'win32':
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\xe0' or key == b'\x00':
                msvcrt.getch()  # 跳过功能键第二字节
                return None
            return key.decode('utf-8', errors='ignore').lower()
        return None
    else:
        import select
        if select.select([sys.stdin], [], [], 0.05)[0]:
            return sys.stdin.read(1).lower()
        return None


def send_gcode(ser, cmd):
    """发送G-code并等待响应"""
    ser.write((cmd + '\n').encode())
    ser.flush()
    time.sleep(0.05)
    response = ""
    while ser.in_waiting:
        response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        time.sleep(0.01)
    return response.strip()


def print_help():
    print("\n=== 三轴步进滑台 键盘点动 ===")
    print(f"步距: {STEP_MM}mm | 速度: F{FEED_RATE}")
    print("─────────────────────────────")
    print("  W - Z轴 上移(+)")
    print("  S - Z轴 下移(-)")
    print("  Q - X轴 负方向(-)")
    print("  A - X轴 正方向(+)")
    print("  E - Y轴 负方向(-)")
    print("  D - Y轴 正方向(+)")
    print("─────────────────────────────")
    print("  H - 归零($H)")
    print("  ? - 查询状态")
    print("  0 - 移动到原点(0,0,95)")
    print("  ESC/Ctrl+C - 退出")
    print("═════════════════════════════\n")


def main():
    # 查找串口
    port = find_esp32_port()
    if not port:
        print("错误: 未找到串口设备")
        sys.exit(1)

    print(f"连接到: {port}")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # 等待ESP32重启

        # 读取启动信息
        while ser.in_waiting:
            print(ser.read(ser.in_waiting).decode('utf-8', errors='ignore'), end='')
            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"串口打开失败: {e}")
        sys.exit(1)

    # 设置相对坐标模式
    resp = send_gcode(ser, "G91")
    print(f"设置相对模式: {resp}")

    print_help()

    # Linux/Mac 设置终端为raw模式
    old_settings = None
    if sys.platform != 'win32':
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    try:
        while True:
            key = get_key()

            if key is None:
                # 检查串口是否有数据
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    if data.strip():
                        print(f"\r< {data.strip()}")
                time.sleep(0.02)
                continue

            cmd = None
            desc = ""

            if key == 'w':
                cmd = f"G1 Z{STEP_MM} F{FEED_RATE}"
                desc = f"Z+ {STEP_MM}mm"
            elif key == 's':
                cmd = f"G1 Z-{STEP_MM} F{FEED_RATE}"
                desc = f"Z- {STEP_MM}mm"
            elif key == 'a':
                cmd = f"G1 X{STEP_MM} F{FEED_RATE}"
                desc = f"X+ {STEP_MM}mm"
            elif key == 'q':
                cmd = f"G1 X-{STEP_MM} F{FEED_RATE}"
                desc = f"X- {STEP_MM}mm"
            elif key == 'd':
                cmd = f"G1 Y{STEP_MM} F{FEED_RATE}"
                desc = f"Y+ {STEP_MM}mm"
            elif key == 'e':
                cmd = f"G1 Y-{STEP_MM} F{FEED_RATE}"
                desc = f"Y- {STEP_MM}mm"
            elif key == 'h':
                print("\r> 归零中...")
                # 切回绝对模式归零，再切回相对
                resp = send_gcode(ser, "$H")
                print(f"\r< {resp}")
                time.sleep(0.5)
                send_gcode(ser, "G91")
                continue
            elif key == '?':
                resp = send_gcode(ser, "?")
                print(f"\r< {resp}")
                continue
            elif key == '0':
                # 移动到原点
                send_gcode(ser, "G90")
                resp = send_gcode(ser, f"G0 X0 Y0 Z95")
                print(f"\r> 移动到原点: {resp}")
                send_gcode(ser, "G91")
                continue
            elif key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
                print("\r退出...")
                break
            else:
                continue

            if cmd:
                print(f"\r> {desc}: {cmd}")
                resp = send_gcode(ser, cmd)
                if resp:
                    print(f"\r< {resp}")

    except KeyboardInterrupt:
        print("\r退出...")
    finally:
        # 恢复终端设置
        if old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()
