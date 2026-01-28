import serial
import struct
import threading
import time
import logging
import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
import robomaster_pb2 as rm_pb
import socket

# ----------------------
# 配置项（与MQTT版对齐，可修改）
# ----------------------
LOG_FILE = "serialclient.log"
UDP_VIDEO_PORT = 3334  # 与MQTT版一致的视频接收端口
COM_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
RECONNECT_DELAY = 3  # 与MQTT版对齐的重连延迟（串口暂未用，保留）

# 日志设置（与MQTT版格式完全一致）
logging.basicConfig(filename=LOG_FILE, level=logging.INFO,
                    format='%(asctime)s - %(message)s')

# ----------------------
# 全局变量（与MQTT版完全一致）
# ----------------------
game_stage = "Unknown"
red_score = 0
event_log = []  # 事件列表，最多保留10条
video_frame = None  # 当前视频帧
seq = 0  # 串口帧序列号
ser = None  # 串口对象，全局化方便重连

# ----------------------
# CRC8/CRC16表及函数（串口通信核心，保留原逻辑）
# ----------------------
CRC8_INIT = 0xff
CRC8_TABLE = [0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
              0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
              0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
              0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
              0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
              0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
              0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
              0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
              0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
              0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
              0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9]

CRC16_INIT = 0xffff
CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]

def get_crc8_check_sum(message, length, init_crc=CRC8_INIT):
    crc = init_crc
    for i in range(length):
        crc = CRC8_TABLE[(crc ^ message[i]) & 0xff]
    return crc

def append_crc8_check_sum(message, length):
    crc = get_crc8_check_sum(message, length - 1, CRC8_INIT)
    message[length - 1] = crc

def get_crc16_check_sum(message, length, init_crc=CRC16_INIT):
    crc = init_crc
    for i in range(length):
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ message[i]) & 0xff]
    return crc

def append_crc16_check_sum(message, length):
    crc = get_crc16_check_sum(message, length - 2, CRC16_INIT)
    message[length - 2] = crc & 0xff
    message[length - 1] = (crc >> 8) & 0xff

def verify_crc8_check_sum(message, length):
    expected = get_crc8_check_sum(message, length - 1, CRC8_INIT)
    return expected == message[length - 1]

def verify_crc16_check_sum(message, length):
    expected = get_crc16_check_sum(message, length - 2, CRC16_INIT)
    return (expected & 0xff) == message[length - 2] and ((expected >> 8) & 0xff) == message[length - 1]

# ----------------------
# 串口核心函数：帧构建+串口初始化
# ----------------------
def build_frame(cmd_id, data):
    """构建串口通信帧（与原逻辑一致）"""
    global seq
    data_bytes = data.SerializeToString()
    data_len = len(data_bytes)
    frame_header = bytearray(struct.pack('<BHB', 0xA5, data_len, seq))
    frame_header += b'\x00'
    append_crc8_check_sum(frame_header, 5)
    frame = frame_header + struct.pack('<H', cmd_id) + data_bytes
    frame += b'\x00\x00'
    append_crc16_check_sum(frame, len(frame))
    seq = (seq + 1) % 256
    return frame

def init_serial():
    """初始化串口，全局化方便调用"""
    global ser
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        if ser.is_open:
            print(f"Serial port {COM_PORT} opened successfully (Baud: {BAUD_RATE})")
            logging.info(f"Serial port {COM_PORT} opened successfully (Baud: {BAUD_RATE})")
            return True
    except Exception as e:
        print(f"Serial port open failed: {e}")
        logging.error(f"Serial port open failed: {e}")
    return False

# ----------------------
# 发送函数（与MQTT版完全对齐，函数名/参数一致）
# ----------------------
def send_remote_control(mouse_x=100, left_down=True):
    """发送RemoteControl指令（对应MQTT版，按钮点击触发）"""
    if not ser or not ser.is_open:
        logging.warning("Serial port not open, skip send RemoteControl")
        return
    try:
        rc = rm_pb.RemoteControl()
        rc.mouse_x = mouse_x
        rc.left_button_down = left_down
        frame = build_frame(0x0301, rc)
        ser.write(frame)
        logging.info("Sent RemoteControl")
        print("Sent RemoteControl")
    except Exception as e:
        logging.error(f"Send RemoteControl error: {e}")

def send_map_target(target_x=500, target_y=300):
    """发送MapCommand指令（对应MQTT版，自定义X/Y）"""
    if not ser or not ser.is_open:
        logging.warning("Serial port not open, skip send MapTarget")
        return
    try:
        map_target = rm_pb.MapCommand()
        map_target.target_x = target_x
        map_target.target_y = target_y
        frame = build_frame(0x0302, map_target)
        ser.write(frame)
        logging.info(f"Sent Map Target: ({target_x}, {target_y})")
        print(f"Sent Map Target: ({target_x}, {target_y})")
    except Exception as e:
        logging.error(f"Send MapTarget error: {e}")

# ----------------------
# 自定义圆角按钮（与MQTT版完全一致，无修改）
# ----------------------
class ModernButton(tk.Canvas):
    """自定义圆角按钮"""
    def __init__(self, parent, text="", command=None, width=100, height=30,
                 bg="#3498db", fg="#ffffff", hover_bg="#2980b9", radius=8):
        super().__init__(parent, width=width, height=height, highlightthickness=0, bg=parent.cget('bg'))
        self.command = command
        self.bg = bg
        self.fg = fg
        self.hover_bg = hover_bg
        self.radius = radius
        self.width = width
        self.height = height
        self.text = text
        self.draw_button(self.bg)
        self.bind("<Button-1>", self.on_click)
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)
        self.text_id = self.create_text(width//2, height//2, text=text, fill=fg,
                                        font=('Segoe UI', 10))

    def draw_button(self, color):
        self.delete("all")
        self.create_rounded_rectangle(0, 0, self.width, self.height,
                                      radius=self.radius, fill=color, outline="")
        shadow_color = "#aaaaaa"
        self.create_rounded_rectangle(2, 2, self.width+2, self.height+2,
                                      radius=self.radius, fill=shadow_color, outline="", tags="shadow")
        self.lower("shadow")
        self.text_id = self.create_text(self.width//2, self.height//2, text=self.text, fill=self.fg,
                                        font=('Segoe UI', 10))

    def create_rounded_rectangle(self, x1, y1, x2, y2, radius=25, **kwargs):
        points = []
        points.extend([x1+radius, y1, x1+radius, y1, x2-radius, y1, x2-radius, y1, x2, y1,
                       x2, y1+radius, x2, y1+radius, x2, y2-radius, x2, y2-radius, x2, y2,
                       x2-radius, y2, x2-radius, y2, x1+radius, y2, x1+radius, y2,
                       x1, y2, x1, y2-radius, x1, y2-radius, x1, y1+radius, x1, y1+radius, x1, y1])
        return self.create_polygon(points, smooth=True, **kwargs)

    def on_click(self, event):
        if self.command:
            self.command()

    def on_enter(self, event):
        self.draw_button(self.hover_bg)

    def on_leave(self, event):
        self.draw_button(self.bg)

# ----------------------
# UI函数（与MQTT版完全复刻，布局/样式/交互无差异）
# ----------------------
def update_ui():
    """UI创建与刷新，与MQTT版完全一致"""
    root = tk.Tk()
    root.title("RoboMaster Client UI")
    root.geometry("900x700")
    root.configure(bg="#f0f0f0")
    root.minsize(800, 600)
    root.attributes('-alpha', 0.95)

    # 主容器框架
    main_frame = tk.Frame(root, bg="#f0f0f0")
    main_frame.pack(expand=True, fill='both', padx=20, pady=1)

    # 左侧状态面板
    status_frame = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    status_frame.pack(side=tk.LEFT, padx=(0, 15), fill='y')
    shadow_frame = tk.Frame(main_frame, bg="#dcdde1", bd=0)
    shadow_frame.place(in_=status_frame, relx=0.02, rely=0.02, relwidth=1, relheight=1)
    status_frame.lift()

    # 标题
    title_label = tk.Label(status_frame, text="ROBOMASTER DASHBOARD",
                           bg="#ffffff", fg="#2c3e50", font=('Segoe UI', 12, 'bold'))
    title_label.pack(pady=(25, 25))

    # 游戏阶段
    tk.Label(status_frame, text="Game Stage:", bg="#ffffff", fg="#333333",
             font=('Segoe UI', 14, 'bold')).pack(anchor='w', padx=30, pady=(20, 0))
    stage_label = tk.Label(status_frame, text=game_stage, bg="#ffffff", fg="#e74c3c",
                           font=('Segoe UI', 24, 'bold'))
    stage_label.pack(anchor='w', padx=30, pady=(10, 20))

    # 红方分数
    tk.Label(status_frame, text="Red Score:", bg="#ffffff", fg="#333333",
             font=('Segoe UI', 14, 'bold')).pack(anchor='w', padx=30, pady=(20, 0))
    score_label = tk.Label(status_frame, text=red_score, bg="#ffffff", fg="#e74c3c",
                           font=('Segoe UI', 24, 'bold'))
    score_label.pack(anchor='w', padx=30, pady=(10, 25))

    # 分隔线
    separator = tk.Frame(status_frame, bg="#ecf0f1", height=1)
    separator.pack(fill='x', padx=25, pady=(20, 25))

    # 视频显示区域
    video_container = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    video_container.pack(side=tk.TOP, fill='x', expand=True, pady=(0, 1))
    video_shadow = tk.Frame(main_frame, bg="#dcdde1", bd=0)
    video_shadow.place(in_=video_container, relx=0.02, rely=0.02, relwidth=1, relheight=1)
    video_container.lift()

    # 视频标题
    tk.Label(video_container, text="LIVE VIDEO FEED", bg="#ffffff", fg="#2c3e50",
             font=('Segoe UI', 16, 'bold')).pack(pady=(25, 12))

    # 视频画布
    video_canvas = tk.Canvas(video_container, width=640, height=360,
                             highlightthickness=0, bg="#ecf0f1", relief='flat')
    video_canvas.pack(pady=12, padx=30)

    # 右侧交互面板
    interact_frame = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    interact_frame.pack(side=tk.BOTTOM, fill='x')
    interact_shadow = tk.Frame(main_frame, bg="#dcdde1", bd=0)
    interact_shadow.place(in_=interact_frame, relx=0.02, rely=0.02, relwidth=1, relheight=1)
    interact_frame.lift()

    # 事件日志标题
    tk.Label(interact_frame, text="EVENT LOG", bg="#ffffff", fg="#2c3e50",
             font=('Segoe UI', 16, 'bold')).pack(pady=(10, 8))

    # 事件日志滚动区域
    log_frame_container = tk.Frame(interact_frame, bg="#ecf0f1", relief='flat', bd=0)
    log_frame_container.pack(fill='both', expand=True, padx=25, pady=(0, 10))
    log_frame = tk.Frame(log_frame_container, bg="#ffffff")
    log_frame.pack(fill='both', expand=True, padx=12, pady=12)
    log_scrollbar = tk.Scrollbar(log_frame)
    log_scrollbar.pack(side=tk.RIGHT, fill='y')
    event_listbox = tk.Listbox(log_frame, height=6, width=50,
                               yscrollcommand=log_scrollbar.set,
                               bg="#ffffff", fg="#333333",
                               selectbackground="#3498db",
                               selectforeground="#ffffff",
                               font=('Segoe UI', 10),
                               borderwidth=0, relief='flat',
                               highlightthickness=0)
    event_listbox.pack(side=tk.LEFT, fill='both', expand=True)
    log_scrollbar.config(command=event_listbox.yview)

    # 控制区域
    control_frame = tk.Frame(interact_frame, bg="#ffffff")
    control_frame.pack(fill='x', padx=25, pady=(0, 8))

    # 地图目标控制标题
    tk.Label(control_frame, text="MAP TARGET COORDINATES",
             bg="#ffffff", fg="#333333", font=('Segoe UI', 12, 'bold')).pack(anchor='w', pady=(10, 6))

    # X/Y输入框
    coord_frame = tk.Frame(control_frame, bg="#ffffff")
    coord_frame.pack(fill='x', pady=(6, 15))
    tk.Label(coord_frame, text="X:", bg="#ffffff", fg="#333333", font=('Segoe UI', 10)).pack(side=tk.LEFT)
    x_entry = tk.Entry(coord_frame, width=8, relief='flat', bd=1,
                       highlightthickness=1, highlightcolor="#3498db", font=('Segoe UI', 11))
    x_entry.pack(side=tk.LEFT, padx=(5, 15))
    tk.Label(coord_frame, text="Y:", bg="#ffffff", fg="#333333", font=('Segoe UI', 10)).pack(side=tk.LEFT)
    y_entry = tk.Entry(coord_frame, width=8, relief='flat', bd=1,
                       highlightthickness=1, highlightcolor="#3498db", font=('Segoe UI', 11))
    y_entry.pack(side=tk.LEFT, padx=(5, 15))

    # 按钮区域
    button_frame = tk.Frame(control_frame, bg="#ffffff")
    button_frame.pack(fill='x', pady=(15, 0))

    # SEND MAP TARGET 按钮
    def send_map_command():
        try:
            x_val = float(x_entry.get() or 0)
            y_val = float(y_entry.get() or 0)
            send_map_target(x_val, y_val)
        except ValueError:
            pass  # 忽略无效输入，与MQTT版一致

    send_map_btn = ModernButton(button_frame, text="SEND MAP TARGET",
                                command=send_map_command, width=180, height=35,
                                bg="#3498db", fg="#ffffff", hover_bg="#2980b9", radius=8)
    send_map_btn.pack(side=tk.LEFT, padx=(0, 10))

    # SEND REMOTE CONTROL 按钮
    def send_rc_command():
        send_remote_control(100, True)  # 固定参数，与MQTT版一致

    send_rc_btn = ModernButton(button_frame, text="SEND REMOTE CONTROL",
                               command=send_rc_command, width=200, height=35,
                               bg="#e74c3c", fg="#ffffff", hover_bg="#c0392b", radius=8)
    send_rc_btn.pack(side=tk.LEFT, padx=(0, 0))

    # UI刷新函数（每100ms刷新，与MQTT版一致）
    def refresh():
        nonlocal stage_label, score_label, event_listbox, video_canvas
        # 更新游戏状态和分数
        stage_label.config(text=game_stage)
        score_label.config(text=red_score)
        # 更新事件日志，最多10条
        event_listbox.delete(0, tk.END)
        for event in event_log:
            event_listbox.insert(tk.END, event)
        # 更新视频帧
        global video_frame
        if video_frame is not None:
            try:
                img = Image.fromarray(cv2.cvtColor(video_frame, cv2.COLOR_BGR2RGB))
                img = img.resize((640, 360), Image.Resampling.LANCZOS)
                imgtk = ImageTk.PhotoImage(image=img)
                video_canvas.delete("all")
                video_canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
                video_canvas.image = imgtk
            except Exception as e:
                logging.error(f"UI video update error: {e}")
        # 循环刷新
        root.after(100, refresh)

    # 启动UI刷新
    refresh()
    # 启动UI主循环
    root.mainloop()

# ----------------------
# 后台线程：串口接收+UDP视频+每秒日志（与MQTT版对齐）
# ----------------------
def serial_receive_loop():
    """串口接收线程，解析GameStatus/Event，更新全局变量"""
    global game_stage, red_score, event_log
    while True:
        if not ser or not ser.is_open:
            time.sleep(RECONNECT_DELAY)
            init_serial()
            continue
        try:
            header = ser.read(5)
            if len(header) < 5 or header[0] != 0xA5:
                continue
            if not verify_crc8_check_sum(header, 5):
                continue
            data_len = struct.unpack('<H', header[1:3])[0]
            frame_rest = ser.read(2 + data_len + 2)
            if len(frame_rest) < 2 + data_len + 2:
                continue
            full_frame = header + frame_rest
            if not verify_crc16_check_sum(full_frame, len(full_frame)):
                continue
            cmd_id = struct.unpack('<H', frame_rest[0:2])[0]
            data = frame_rest[2:2+data_len]
            # 解析GameStatus（与MQTT版topic GameStatus对齐）
            if cmd_id == 0x0001:
                gs = rm_pb.GameStatus()
                gs.ParseFromString(data)
                game_stage = gs.current_stage
                red_score = gs.red_score
                logging.info(f"GameStatus received: stage {game_stage}, Red score {red_score}")
                print(f"Game stage: {game_stage}, Red score: {red_score}")
            # 解析Event（与MQTT版topic Event对齐）
            elif cmd_id == 0x0002:
                ev = rm_pb.Event()
                ev.ParseFromString(data)
                event_str = f"Event ID: {ev.event_id}, Param: {ev.param}"
                event_log.append(event_str)
                if len(event_log) > 10:
                    event_log.pop(0)
                logging.info(event_str)
                print(event_str)
        except Exception as e:
            logging.error(f"Serial receive error: {e}")
        time.sleep(0.01)

def receive_video():
    """UDP视频接收线程，与MQTT版完全一致，端口3334"""
    global video_frame
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', UDP_VIDEO_PORT))
        print(f"UDP video receiver started on port {UDP_VIDEO_PORT}")
        logging.info(f"UDP video receiver started on port {UDP_VIDEO_PORT}")
        while True:
            data, addr = sock.recvfrom(65535)
            try:
                nparr = np.frombuffer(data, np.uint8)
                video_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            except Exception as e:
                logging.error(f"Video decode error: {e}")
    except Exception as e:
        print(f"UDP video receiver start failed: {e}")
        logging.error(f"UDP video receiver start failed: {e}")
        time.sleep(RECONNECT_DELAY)
        receive_video()  # 重连

def log_per_second():
    """每秒打印日志，复刻MQTT版的持续日志输出"""
    while True:
        try:
            if ser and ser.is_open:
                log_content = f"Serial client running - Port: {COM_PORT}, Stage: {game_stage}, Red score: {red_score}"
            else:
                log_content = f"Serial client running - Port {COM_PORT} closed, Stage: {game_stage}, Red score: {red_score}"
            logging.info(log_content)
            time.sleep(1)  # 每秒执行一次
        except Exception as e:
            logging.error(f"Log per second error: {e}")
            time.sleep(1)

# ----------------------
# 程序入口（主函数）
# ----------------------
if __name__ == "__main__":
    # 1. 初始化串口
    init_serial()
    # 2. 启动后台线程（与MQTT版线程数一致）
    # 串口接收线程
    serial_thread = threading.Thread(target=serial_receive_loop, daemon=True)
    serial_thread.start()
    # UDP视频接收线程
    video_thread = threading.Thread(target=receive_video, daemon=True)
    video_thread.start()
    # 每秒日志线程
    log_thread = threading.Thread(target=log_per_second, daemon=True)
    log_thread.start()
    # 3. 启动UI线程（主交互）
    ui_thread = threading.Thread(target=update_ui)
    ui_thread.start()
    # 4. 主线程保持运行，捕获Ctrl+C退出
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting client...")
        logging.info("Client exited by user (Ctrl+C)")
        # 关闭串口
        if ser and ser.is_open:
            ser.close()
        # 等待线程退出
        ui_thread.join()
        serial_thread.join()
        video_thread.join()
        log_thread.join()

