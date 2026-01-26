import serial
import struct
import threading
import time
import logging
import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
from generated import robomaster_pb2 as rm_pb  # Protobuf类

# ----------------------
# 日志设置
# ----------------------
LOG_FILE = "serialclient.log"
logging.basicConfig(filename=LOG_FILE, level=logging.INFO,
                    format='%(asctime)s - %(message)s')

# ----------------------
# 全局变量
# ----------------------
game_stage = "Unknown"
red_score = 0
event_log = []  # 事件列表
video_frame = None  # 当前视频帧

# ----------------------
# 串口配置
# ----------------------
COM_PORT = '/dev/ttyUSB0'  # 改成你的串口
BAUD_RATE = 115200
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
seq = 0

# 视频串口
VIDEO_COM = '/dev/ttyUSB1'  # 改成你的视频串口
VIDEO_BAUD = 921600

# ----------------------
# CRC8/CRC16表及函数（保持原逻辑）
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
    # 省略中间表，保持原代码
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
# 帧构建
# ----------------------
def build_frame(cmd_id, data):
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

# ----------------------
# 串口发送函数
# ----------------------
def send_remote_control(mouse_x=100, left_down=True):
    rc = rm_pb.RemoteControl()
    rc.mouse_x = mouse_x
    rc.left_button_down = left_down
    frame = build_frame(0x0301, rc)
    ser.write(frame)
    logging.info("Sent RemoteControl")

def send_map_target(target_x=500, target_y=300):
    map_target = rm_pb.MapCommand()
    map_target.target_x = target_x
    map_target.target_y = target_y
    frame = build_frame(0x0302, map_target)
    ser.write(frame)
    logging.info(f"Sent MapTarget ({target_x}, {target_y})")

# ----------------------
# UI组件 (保持 MQTT UI 风格)
# ----------------------
class ModernButton(tk.Canvas):
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
        self.text_id = self.create_text(width//2, height//2, text=text, fill=fg, font=('Segoe UI', 10))

    def draw_button(self, color):
        self.delete("all")
        self.create_rounded_rectangle(0, 0, self.width, self.height, radius=self.radius, fill=color, outline="")
        shadow_color = "#aaaaaa"
        self.create_rounded_rectangle(2, 2, self.width+2, self.height+2, radius=self.radius, fill=shadow_color, outline="", tags="shadow")
        self.lower("shadow")
        self.text_id = self.create_text(self.width//2, self.height//2, text=self.text, fill=self.fg, font=('Segoe UI', 10))

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
# 串口接收线程
# ----------------------
def serial_receive_loop():
    global game_stage, red_score, event_log
    while True:
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
            if cmd_id == 0x0001:
                gs = rm_pb.GameStatus()
                gs.ParseFromString(data)
                game_stage = gs.current_stage
                red_score = gs.red_score
            elif cmd_id == 0x0002:
                ev = rm_pb.Event()
                ev.ParseFromString(data)
                event_log.append(f"Event {ev.event_id}: {ev.param}")
                if len(event_log) > 10:
                    event_log.pop(0)
        except Exception as e:
            logging.error(f"Serial receive error: {e}")
        time.sleep(0.01)

# ----------------------
# 串口视频接收线程
# ----------------------
def receive_video():
    global video_frame
    try:
        ser_video = serial.Serial(VIDEO_COM, VIDEO_BAUD, timeout=0.05)
    except Exception as e:
        print(f"打开视频串口失败: {e}")
        return

    while True:
        try:
            header = ser_video.read(5)
            if len(header) < 5 or header[0] != 0xA5:
                continue
            if not verify_crc8_check_sum(header, 5):
                continue
            data_len = struct.unpack('<H', header[1:3])[0]
            frame_rest = ser_video.read(2 + data_len + 2)
            if len(frame_rest) < 2 + data_len + 2:
                continue
            full_frame = header + frame_rest
            if not verify_crc16_check_sum(full_frame, len(full_frame)):
                continue
            data = frame_rest[2:2+data_len]
            np_arr = np.frombuffer(data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                video_frame = img
        except Exception as e:
            print(f"视频接收错误: {e}")
            time.sleep(0.01)

# ----------------------
# UI刷新函数
# ----------------------
def refresh_ui(root, stage_label, score_label, event_listbox, video_canvas):
    global video_frame
    stage_label.config(text=game_stage)
    score_label.config(text=red_score)
    event_listbox.delete(0, tk.END)
    for e in event_log:
        event_listbox.insert(tk.END, e)

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

    root.after(100, refresh_ui, root, stage_label, score_label, event_listbox, video_canvas)

# ----------------------
# 主函数
# ----------------------
def main():
    root = tk.Tk()
    root.title("Serial Client UI")
    root.geometry("900x700")
    root.configure(bg="#f0f0f0")
    root.minsize(800, 600)

    main_frame = tk.Frame(root, bg="#f0f0f0")
    main_frame.pack(expand=True, fill='both', padx=20, pady=1)

    # 左侧状态面板
    status_frame = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    status_frame.pack(side=tk.LEFT, padx=(0, 15), fill='y')

    title_label = tk.Label(status_frame, text="ROBOMASTER DASHBOARD",
                           bg="#ffffff", fg="#2c3e50", font=('Segoe UI', 12, 'bold'))
    title_label.pack(pady=(25, 25))

    tk.Label(status_frame, text="Game Stage:", bg="#ffffff", fg="#333333",
             font=('Segoe UI', 14, 'bold')).pack(anchor='w', padx=30, pady=(20, 0))
    stage_label = tk.Label(status_frame, text=game_stage, bg="#ffffff", fg="#e74c3c",
                           font=('Segoe UI', 24, 'bold'))
    stage_label.pack(anchor='w', padx=30, pady=(10, 20))

    tk.Label(status_frame, text="Red Score:", bg="#ffffff", fg="#333333",
             font=('Segoe UI', 14, 'bold')).pack(anchor='w', padx=30, pady=(20, 0))
    score_label = tk.Label(status_frame, text=red_score, bg="#ffffff", fg="#e74c3c",
                           font=('Segoe UI', 24, 'bold'))
    score_label.pack(anchor='w', padx=30, pady=(10, 25))

    # 视频显示
    video_container = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    video_container.pack(side=tk.TOP, fill='x', expand=True, pady=(0, 1))
    tk.Label(video_container, text="LIVE VIDEO FEED", bg="#ffffff", fg="#2c3e50",
             font=('Segoe UI', 16, 'bold')).pack(pady=(25, 12))
    video_canvas = tk.Canvas(video_container, width=640, height=360, highlightthickness=0, bg="#ecf0f1")
    video_canvas.pack(pady=12, padx=30)

    # 事件日志
    interact_frame = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    interact_frame.pack(side=tk.BOTTOM, fill='x')
    tk.Label(interact_frame, text="EVENT LOG", bg="#ffffff", fg="#2c3e50",
             font=('Segoe UI', 16, 'bold')).pack(pady=(10, 8))
    log_frame_container = tk.Frame(interact_frame, bg="#ecf0f1", relief='flat', bd=0)
    log_frame_container.pack(fill='both', expand=True, padx=25, pady=(0, 10))
    log_frame = tk.Frame(log_frame_container, bg="#ffffff")
    log_frame.pack(fill='both', expand=True, padx=12, pady=12)
    log_scrollbar = tk.Scrollbar(log_frame)
    log_scrollbar.pack(side=tk.RIGHT, fill='y')
    event_listbox = tk.Listbox(log_frame, height=6, width=50, yscrollcommand=log_scrollbar.set,
                               bg="#ffffff", fg="#333333", selectbackground="#3498db",
                               selectforeground="#ffffff", font=('Segoe UI', 10),
                               borderwidth=0, relief='flat', highlightthickness=0)
    event_listbox.pack(side=tk.LEFT, fill='both', expand=True)
    log_scrollbar.config(command=event_listbox.yview)

    # 控制按钮
    control_frame = tk.Frame(interact_frame, bg="#ffffff")
    control_frame.pack(fill='x', padx=25, pady=(0, 8))
    coord_frame = tk.Frame(control_frame, bg="#ffffff")
    coord_frame.pack(fill='x', pady=(6, 15))
    tk.Label(coord_frame, text="X:", bg="#ffffff", fg="#333333", font=('Segoe UI', 10)).pack(side=tk.LEFT)
    x_entry = tk
