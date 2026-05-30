"""
液滴自动夹取 - 总控程序(骨架版)

串起三个子系统:
  1. 三轴滑台  : ESP32, G-code, 自动检测串口
  2. 夹爪      : PGEA 50-40-O, COM3, Modbus RTU
  3. 视觉(只读): C:\\opencv_water\\...\\droplet_log.csv (液滴棋盘格mm坐标)

流程(每次开机执行一次):
  第一段(自动): 归零 -> X132 -> Z170 -> Y3 (让位, 不遮挡相机)
  第二段(人工): 在视觉程序里 识别棋盘格/采背景/滴液滴 -> CSV记录坐标
  第三段(自动): 回车触发 -> 读CSV -> 变换 -> 取液滴 -> 抬起 -> 放到(132,114)

依赖: pip install pyserial minimalmodbus
"""

import csv
import sys
import time

import serial
import minimalmodbus


# ============================================================
#                         配  置  区
# ============================================================

# ---- 运行模式 ----
# True = 首次验证: 只归零+摆位+打印换算坐标, 不移动取放(确认方向安全后改 False)
DRY_RUN = True

# ---- 滑台(ESP32) ----
SLIDER_PORT = "COM4"
SLIDER_BAUD = 115200

# ---- 夹爪(PGEA 50-40-O, Modbus RTU) ----
GRIPPER_PORT = "COM3"
GRIPPER_SLAVE_ID = 1
GRIPPER_BAUD = 115200
REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_TARGET_POSITION = 0x0103
REG_SPEED = 0x0104
WRITE_SINGLE_REGISTER = 6

GRIP_FORCE = 10            # 夹取力(同手动按一下方向键)
GRIP_SPEED = 10
# 夹取动作分三个 position(position 越小越闭合, 越大越张开):
GRIP_PRE_OPEN = 385       # 套液滴前张开一点点(标定值)
GRIP_CLOSE_POS = 310      # 套住后闭合一点点夹住(标定值, < GRIP_PRE_OPEN)
GRIP_FULL_OPEN = 1000     # 释放: 完全打开到最大(固定 1000)

# ---- 视觉输出 CSV(只读, 列: 序号,时间,X_mm,Y_mm) ----
CSV_PATH = r"C:\opencv_water\droplet_deploy\droplet_deploy\droplet_log.csv"

# ---- 坐标变换: 棋盘格坐标(CSV的mm) -> 滑台坐标(mm) ----
# 棋盘格原点对应的滑台坐标
BOARD_ORIGIN_X = 154.345
BOARD_ORIGIN_Y = 139.003
# 轴向: 棋盘格X+ -> 滑台+Y ; 棋盘格Y+ -> 滑台-X
def board_to_slider(bx, by):
    """棋盘格(bx,by) mm -> 滑台(sx,sy) mm。方向: 见上方注释。"""
    sx = BOARD_ORIGIN_X - by
    sy = BOARD_ORIGIN_Y + bx
    return sx, sy

# ---- 位姿 / 高度(mm) ----
PARK_X = 192.0           # 让位位姿 X
PARK_Y = 154.0           # 让位位姿 Y
PARK_Z = 150.0           # 让位位姿 Z(独立于移动安全高度)
SAFE_Z = 159.0           # 移动XY时Z保持在此的高度(取后抬起/平移用; Z小=高)
PICK_Z = 163.9           # 夹取下降高度
PLACE_Z = 163.0          # 放置下降高度
PLACE_X = 132.0          # 放置点 X
PLACE_Y = 114.0          # 放置点 Y

# Plan B 定点测试的释放点(滑台坐标, 高位释放不下降)
PLANB_RELEASE_X = 145.0
PLANB_RELEASE_Y = 129.0

# ---- 滑台行程(用于安全检查, 防止变换结果越界撞机) ----
X_MAX_TRAVEL = 195.0
Y_MAX_TRAVEL = 195.0

# 进给速度(mm/min) - 固件上限1000。
FEED = 500


# ============================================================
#                      滑  台  通  信
# ============================================================

def _wait_slider_ready(ser, timeout):
    """在 timeout 秒内等待 ESP32 的 'Ready' 启动信号。"""
    start = time.time()
    buf = ""
    while time.time() - start < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
            if "Ready" in buf:
                return True
        time.sleep(0.1)
    return False


def connect_slider():
    """连接 ESP32 滑台(固定串口 SLIDER_PORT, 禁用 DTR/RTS 防止重启)。"""
    print(f"[滑台] 连接 {SLIDER_PORT}")
    ser = serial.Serial(SLIDER_PORT, SLIDER_BAUD, timeout=1, dsrdtr=False, rtscts=False)

    # 硬复位 ESP32 到"运行模式": 全程 DTR=False 保证 GPIO0 为高(不进下载模式),
    # 仅脉冲 RTS 复位 EN; 释放后 ESP32 带着 GPIO0=高 正常启动应用。
    # 修复: 打开串口瞬间的 DTR/RTS 过渡偶发把 ESP32 带入下载模式,
    #       表现为 'invalid header' 反复复位、收不到 Ready。
    ser.dtr = False
    ser.rts = True       # EN 拉低, 复位
    time.sleep(0.1)
    ser.rts = False      # 释放 -> 带 GPIO0=高 启动应用
    ser.reset_input_buffer()

    # 等待 "Ready"; 超时则提示手动按 EN 再等一轮
    if not _wait_slider_ready(ser, 15):
        print("[滑台] ⚠️ 未收到 Ready, 请按一下 ESP32 板上的 EN/RST 键...")
        ser.reset_input_buffer()
        if not _wait_slider_ready(ser, 20):
            raise RuntimeError(
                "滑台未就绪: ESP32 没启动到应用。检查固件是否烧录、"
                "COM 口是否被其他程序占用, 或手动按 EN 复位后重试。")
    print("[滑台] ESP32 已就绪")
    return ser


def slider_send(ser, cmd, timeout=120):
    """发送一条 G-code, 阻塞等待固件返回 'ok'(固件移动是阻塞执行的)。
    返回固件的完整响应文本; 遇到 Error 抛异常, 遇到 ALARM 打印警告。"""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()

    resp = ""
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            resp += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
            if "ALARM" in resp:
                print(f"  ⚠️ [滑台] 触发限位: {resp.strip()}")
            # 固件每条指令完成后会输出含 'ok' 的行
            if "ok" in resp:
                return resp.strip()
            if "Error" in resp:
                raise RuntimeError(f"滑台报错: {resp.strip()}")
        time.sleep(0.02)
    raise TimeoutError(f"滑台指令超时({timeout}s): {cmd}  收到: {resp.strip()}")


def slider_query(ser):
    """发送 '?' 查询固件当前坐标, 返回形如 <Idle|MPos:x,y,z|...> 的状态行。"""
    ser.reset_input_buffer()
    ser.write(b"?\n")
    ser.flush()
    resp = ""
    start = time.time()
    while time.time() - start < 3:
        if ser.in_waiting:
            resp += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
            if ">" in resp:
                return resp.strip()
        time.sleep(0.02)
    return resp.strip()


# ============================================================
#                      夹  爪  通  信
# ============================================================

def connect_gripper():
    """连接夹爪 Modbus RTU 并初始化使能。"""
    print(f"[夹爪] 连接 {GRIPPER_PORT}")
    g = minimalmodbus.Instrument(GRIPPER_PORT, GRIPPER_SLAVE_ID)
    g.serial.baudrate = GRIPPER_BAUD
    g.serial.bytesize = 8
    g.serial.parity = serial.PARITY_NONE
    g.serial.stopbits = 1
    g.serial.timeout = 0.2
    g.mode = minimalmodbus.MODE_RTU
    g.clear_buffers_before_each_transaction = True
    g.close_port_after_each_call = False

    print("[夹爪] 初始化中...")
    g.write_register(REG_INIT, 1, 0, WRITE_SINGLE_REGISTER)
    time.sleep(2)
    print("[夹爪] 初始化完成")
    return g


def gripper_move(g, position, speed=GRIP_SPEED, force=GRIP_FORCE):
    """移动夹爪到指定位置(0~1000)。顺序: 力 -> 速度 -> 位置。"""
    position = max(0, min(1000, int(position)))
    g.write_register(REG_FORCE, force, 0, WRITE_SINGLE_REGISTER)
    g.write_register(REG_SPEED, speed, 0, WRITE_SINGLE_REGISTER)
    g.write_register(REG_TARGET_POSITION, position, 0, WRITE_SINGLE_REGISTER)
    print(f"[夹爪] -> 位置 {position} (力{force} 速{speed})")
    time.sleep(1.0)  # 等夹爪动作到位


# ============================================================
#                      视  觉  CSV
# ============================================================

def read_csv_first_point():
    """读 droplet_log.csv 第一条数据(第2行), 返回 (X_mm, Y_mm) 棋盘格坐标。"""
    with open(CSV_PATH, "r", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader, None)          # 跳过表头 序号,时间,X_mm,Y_mm
        row = next(reader, None)             # 第一条数据
        if not row or len(row) < 4:
            raise RuntimeError("CSV 里没有液滴坐标(第一条数据为空)")
        return float(row[2]), float(row[3])


# ============================================================
#                      动  作  序  列
# ============================================================

def move_xy(ser, x, y):
    """绝对移动 XY(Z 不变, 受控速度防丢步)。"""
    slider_send(ser, f"G1 X{x:.3f} Y{y:.3f} F{FEED}")

def move_z(ser, z):
    """绝对移动 Z。"""
    slider_send(ser, f"G1 Z{z:.3f} F{FEED}")


def stage1_home_and_park(ser):
    """第一段: 归零 + 摆让位位姿 X192 -> Z153 -> Y114。"""
    print("\n=== 第一段: 归零 + 让位 ===")
    slider_send(ser, "$H", timeout=90)              # 归零(耗时长)
    slider_send(ser, "G90")                         # 绝对坐标
    slider_send(ser, f"G1 X{PARK_X:.3f} F{FEED}")   # 先 X(受控速度防丢步)
    move_z(ser, PARK_Z)                             # 再抬 Z 到让位高度
    slider_send(ser, f"G1 Y{PARK_Y:.3f} F{FEED}")   # 最后 Y
    print(f"目标让位位姿:   X{PARK_X} Y{PARK_Y} Z{PARK_Z}")
    print(f"固件实际坐标: {slider_query(ser)}")       # 诊断: 对比固件坐标 vs 物理位置


def stage3_pick_and_place(ser, g):
    """第三段: 读CSV -> 变换 -> 取液滴 -> 抬起 -> 放到(132,114)。"""
    print("\n=== 第三段: 取液滴并放置 ===")

    # 1) 读坐标 + 变换
    bx, by = read_csv_first_point()
    sx, sy = board_to_slider(bx, by)
    print(f"液滴棋盘格坐标 ({bx:.2f}, {by:.2f}) -> 滑台坐标 ({sx:.2f}, {sy:.2f})")

    # 2) 安全检查: 越界则中止, 防撞机
    if not (0 <= sx <= X_MAX_TRAVEL and 0 <= sy <= Y_MAX_TRAVEL):
        raise RuntimeError(
            f"换算后的滑台坐标 ({sx:.2f},{sy:.2f}) 超出行程 [0~{X_MAX_TRAVEL}]! "
            f"请检查坐标变换/原点/轴向是否正确, 已中止防止撞机。")

    # DRY_RUN: 首次验证方向, 打印完坐标就停, 不动滑台/夹爪
    if DRY_RUN:
        print("【DRY_RUN】仅验证: 已打印换算坐标, 不执行移动/夹取。\n"
              "  确认 ①摆位正确 ②上面滑台坐标落在小平台合理位置, "
              "再把配置区 DRY_RUN 改成 False 正式跑。")
        return

    # 3) 检查夹取 position 是否已标定
    if GRIP_PRE_OPEN is None or GRIP_CLOSE_POS is None:
        raise RuntimeError("GRIP_PRE_OPEN / GRIP_CLOSE_POS 未标定! 请先手动试出再运行。")

    # 4) 取液滴: 张开一点点 -> 套住 -> 闭合一点点 -> 抬起
    gripper_move(g, GRIP_PRE_OPEN)           # 先张开一点点(刚好能套住液滴)
    move_xy(ser, sx, sy)                     # 移到液滴上方(Z仍在SAFE_Z)
    move_z(ser, PICK_Z)                      # 下降套住液滴
    gripper_move(g, GRIP_CLOSE_POS)          # 闭合一点点夹住
    move_z(ser, SAFE_Z)                      # 抬起

    # 5) 放置: 到位 -> 下降 -> 完全打开释放 -> 抬起
    move_xy(ser, PLACE_X, PLACE_Y)           # 移到放置点
    move_z(ser, PLACE_Z)                     # 下降
    gripper_move(g, GRIP_FULL_OPEN)          # 完全打开(1000)释放液滴
    move_z(ser, SAFE_Z)                      # 收尾抬起

    print("=== 完成一次取放 ===")


def wait_trigger():
    """等待触发键: 返回 'b'(Plan B 定点测试) 或 'enter'(视觉 CSV 流程)。"""
    if sys.platform == 'win32':
        import msvcrt
        while True:
            ch = msvcrt.getch()
            if ch in (b'b', b'B'):
                return 'b'
            if ch in (b'\r', b'\n'):
                return 'enter'
    else:
        return 'b' if input().strip().lower() == 'b' else 'enter'


def stage_planb(ser, g):
    """Plan B 定点取放测试: 写死棋盘格原点(0,0), 完全不读视觉。
    移到原点上空 -> 下降夹取 -> 抬起 -> 移到释放点 -> 完全张开释放, 验证整条机械链路。
    注意: Z 越大越往下, 故行进/抬起用 PARK_Z(高位), 夹取用 PICK_Z(低位)。"""
    print("\n=== Plan B: 棋盘格原点定点取放测试 ===")
    print("⚠️ 物理测试(忽略 DRY_RUN), 即将真实移动! 手放急停旁, 异常按 Ctrl+C。")

    sx, sy = board_to_slider(0.0, 0.0)       # 棋盘格原点 -> 滑台(固件)坐标
    print(f"棋盘格原点 (0,0) -> 滑台坐标 ({sx:.2f}, {sy:.2f})")

    # 安全检查: 越界中止防撞机
    if not (0 <= sx <= X_MAX_TRAVEL and 0 <= sy <= Y_MAX_TRAVEL):
        raise RuntimeError(f"原点换算坐标 ({sx:.2f},{sy:.2f}) 越界, 已中止。")

    # 取: 张开一点点 -> (确保高位)移到原点上方 -> 下降套住 -> 闭合夹住 -> 抬起
    gripper_move(g, GRIP_PRE_OPEN)           # 张开一点点
    move_xy(ser, sx, sy)                     # 不降高度, 直接在让位高度150平移到原点正上方
    move_z(ser, PICK_Z)                      # 下降套住
    gripper_move(g, GRIP_CLOSE_POS)          # 闭合一点点夹住
    move_z(ser, SAFE_Z)                      # 取后上升到XY行进高度(159)

    # 释放: 移到释放点(在行进高度159不下降) -> 完全张开
    move_xy(ser, PLANB_RELEASE_X, PLANB_RELEASE_Y)   # 移到释放点
    gripper_move(g, GRIP_FULL_OPEN)          # 完全打开(1000)释放
    print("=== Plan B 完成 ===")


# ============================================================
#                          主  程  序
# ============================================================

def main():
    ser = None
    g = None
    try:
        ser = connect_slider()
        g = connect_gripper()

        # 第一段: 归零 + 让位
        stage1_home_and_park(ser)

        # 选择下一步: B=Plan B 定点测试 / 回车=视觉流程
        print("\n>>> 选择下一步:")
        print("    [B]    = Plan B 定点取放测试(写死棋盘格原点, 不用视觉)")
        print("    [回车] = 视觉流程(先在视觉程序里 识别c/采背景b/滴液滴/STILL记录, 再回车)")
        if wait_trigger() == 'b':
            stage_planb(ser, g)
        else:
            stage3_pick_and_place(ser, g)

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n❌ 出错: {e}")
    finally:
        if g is not None:
            try:
                g.serial.close()
            except Exception:
                pass
        if ser is not None:
            ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()
