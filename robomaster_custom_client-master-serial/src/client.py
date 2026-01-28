import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
import robomaster_pb2 as rm_pb
from config import SERVER_IP, MQTT_PORT, CLIENT_ID, RECONNECT_DELAY, LOG_FILE
import logging
import time
import cv2
import socket
import threading
import tkinter as tk
from tkinter import ttk
import numpy as np
from PIL import Image, ImageTk
import tkinter.font as tkFont

# 日志设置
logging.basicConfig(filename=LOG_FILE, level=logging.INFO, format='%(asctime)s - %(message)s')

# 全局变量
game_stage = "Unknown"
red_score = 0
event_log = []  # 事件列表
video_frame = None  # 当前视频帧


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

        # 绘制圆角矩形按钮
        self.draw_button(self.bg)

        # 绑定事件
        self.bind("<Button-1>", self.on_click)
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

        # 添加文字
        self.text_id = self.create_text(width//2, height//2, text=text, fill=fg,
                                        font=('Segoe UI', 10))

    def draw_button(self, color):
        # 清除之前的绘制
        self.delete("all")

        # 绘制圆角矩形
        self.create_rounded_rectangle(0, 0, self.width, self.height,
                                      radius=self.radius, fill=color, outline="")

        # 添加轻微阴影效果
        shadow_color = "#aaaaaa"
        self.create_rounded_rectangle(2, 2, self.width+2, self.height+2,
                                      radius=self.radius, fill=shadow_color, outline="", tags="shadow")
        self.lower("shadow")  # 将阴影置于底层

        # 重新添加文字
        self.text_id = self.create_text(self.width//2, self.height//2, text=self.text, fill=self.fg,
                                        font=('Segoe UI', 10))

    def create_rounded_rectangle(self, x1, y1, x2, y2, radius=25, **kwargs):
        points = []
        # 计算圆角点
        points.extend([x1+radius, y1,
                       x1+radius, y1,
                       x2-radius, y1,
                       x2-radius, y1,
                       x2, y1,
                       x2, y1+radius,
                       x2, y1+radius,
                       x2, y2-radius,
                       x2, y2-radius,
                       x2, y2,
                       x2-radius, y2,
                       x2-radius, y2,
                       x1+radius, y2,
                       x1+radius, y2,
                       x1, y2,
                       x1, y2-radius,
                       x1, y2-radius,
                       x1, y1+radius,
                       x1, y1+radius,
                       x1, y1])

        return self.create_polygon(points, smooth=True, **kwargs)

    def on_click(self, event):
        if self.command:
            self.command()

    def on_enter(self, event):
        self.draw_button(self.hover_bg)

    def on_leave(self, event):
        self.draw_button(self.bg)


# UI函数
def update_ui():
    root = tk.Tk()
    root.title("RoboMaster Client UI")
    root.geometry("900x700")
    root.configure(bg="#f0f0f0")

    # 设置最小窗口尺寸
    root.minsize(800, 600)

    # 设置窗口圆角（通过创建圆角背景）
    root.attributes('-alpha', 0.95)  # 设置透明度以便看到圆角效果

    # 主容器框架
    main_frame = tk.Frame(root, bg="#f0f0f0")
    main_frame.pack(expand=True, fill='both', padx=20, pady=1)  # 减少顶部padding

    # 左侧状态面板 - 极致圆角效果
    status_frame = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    status_frame.pack(side=tk.LEFT, padx=(0, 15), fill='y')

    # 添加阴影效果
    shadow_frame = tk.Frame(main_frame, bg="#dcdde1", bd=0)
    shadow_frame.place(in_=status_frame, relx=0.02, rely=0.02, relwidth=1, relheight=1)
    status_frame.lift()

    # 标题
    title_label = tk.Label(status_frame, text="ROBOMASTER DASHBOARD",
                           bg="#ffffff", fg="#2c3e50",
                           font=('Segoe UI', 12, 'bold'))
    title_label.pack(pady=(25, 25))

    # 游戏阶段
    tk.Label(status_frame, text="Game Stage:",
             bg="#ffffff", fg="#333333",
             font=('Segoe UI', 14, 'bold')).pack(anchor='w', padx=30, pady=(20, 0))
    stage_label = tk.Label(status_frame, text=game_stage,
                           bg="#ffffff", fg="#e74c3c",
                           font=('Segoe UI', 24, 'bold'))
    stage_label.pack(anchor='w', padx=30, pady=(10, 20))

    # 红方分数
    tk.Label(status_frame, text="Red Score:",
             bg="#ffffff", fg="#333333",
             font=('Segoe UI', 14, 'bold')).pack(anchor='w', padx=30, pady=(20, 0))
    score_label = tk.Label(status_frame, text=red_score,
                           bg="#ffffff", fg="#e74c3c",
                           font=('Segoe UI', 24, 'bold'))
    score_label.pack(anchor='w', padx=30, pady=(10, 25))

    # 分隔线
    separator = tk.Frame(status_frame, bg="#ecf0f1", height=1)
    separator.pack(fill='x', padx=25, pady=(20, 25))

    # 视频显示区域 - 极致圆角效果
    video_container = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    video_container.pack(side=tk.TOP, fill='x', expand=True, pady=(0, 1))  # 减少padding

    # 视频区域阴影
    video_shadow = tk.Frame(main_frame, bg="#dcdde1", bd=0)
    video_shadow.place(in_=video_container, relx=0.02, rely=0.02, relwidth=1, relheight=1)
    video_container.lift()

    # 视频标题 - 字体增大
    tk.Label(video_container, text="LIVE VIDEO FEED",
             bg="#ffffff", fg="#2c3e50",
             font=('Segoe UI', 16, 'bold')).pack(pady=(25, 12))

    # 视频画布
    video_canvas = tk.Canvas(video_container, width=640, height=360,
                             highlightthickness=0, bg="#ecf0f1", relief='flat')
    video_canvas.pack(pady=12, padx=30)

    # 右侧交互面板 - 极致圆角效果
    interact_frame = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    interact_frame.pack(side=tk.BOTTOM, fill='x')

    # 交互面板阴影
    interact_shadow = tk.Frame(main_frame, bg="#dcdde1", bd=0)
    interact_shadow.place(in_=interact_frame, relx=0.02, rely=0.02, relwidth=1, relheight=1)
    interact_frame.lift()

    # 事件日志标题 - 字体增大
    tk.Label(interact_frame, text="EVENT LOG",
             bg="#ffffff", fg="#2c3e50",
             font=('Segoe UI', 16, 'bold')).pack(pady=(10, 8))  # 减少padding

    # 事件日志滚动区域 - 添加深色背景区域
    log_frame_container = tk.Frame(interact_frame, bg="#ecf0f1", relief='flat', bd=0)  # 深色背景
    log_frame_container.pack(fill='both', expand=True, padx=25, pady=(0, 10))  # 减少padding

    # 内部框架
    log_frame = tk.Frame(log_frame_container, bg="#ffffff")  # 白色内部
    log_frame.pack(fill='both', expand=True, padx=12, pady=12)  # 内边距形成边框效果

    # 创建滚动条和列表框
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

    # 控制区域 - 大幅减少padding使其更靠上
    control_frame = tk.Frame(interact_frame, bg="#ffffff")
    control_frame.pack(fill='x', padx=25, pady=(0, 8))  # 减少上下padding

    # 地图目标控制
    tk.Label(control_frame, text="MAP TARGET COORDINATES",
             bg="#ffffff", fg="#333333",
             font=('Segoe UI', 12, 'bold')).pack(anchor='w', pady=(10, 6))

    coord_frame = tk.Frame(control_frame, bg="#ffffff")
    coord_frame.pack(fill='x', pady=(6, 15))  # 减少padding

    tk.Label(coord_frame, text="X:",
             bg="#ffffff", fg="#333333",
             font=('Segoe UI', 10)).pack(side=tk.LEFT)
    x_entry = tk.Entry(coord_frame, width=8, relief='flat', bd=1,
                       highlightthickness=1, highlightcolor="#3498db",
                       font=('Segoe UI', 11))
    x_entry.pack(side=tk.LEFT, padx=(5, 15))

    tk.Label(coord_frame, text="Y:",
             bg="#ffffff", fg="#333333",
             font=('Segoe UI', 10)).pack(side=tk.LEFT)
    y_entry = tk.Entry(coord_frame, width=8, relief='flat', bd=1,
                       highlightthickness=1, highlightcolor="#3498db",
                       font=('Segoe UI', 11))
    y_entry.pack(side=tk.LEFT, padx=(5, 15))

    # 按钮区域 - 在XY坐标下方
    button_frame = tk.Frame(control_frame, bg="#ffffff")
    button_frame.pack(fill='x', pady=(15, 0))  # 在坐标下方

    # Send Map Target 按钮 (圆角设计) - 加宽
    def send_map_command():
        try:
            x_val = float(x_entry.get() or 0)
            y_val = float(y_entry.get() or 0)
            send_map_target(x_val, y_val)
        except ValueError:
            pass  # 忽略无效输入

    send_map_btn = ModernButton(button_frame, text="SEND MAP TARGET",
                                command=send_map_command,
                                width=180, height=35,  # 加宽
                                bg="#3498db", fg="#ffffff", hover_bg="#2980b9",
                                radius=8)  # 按钮圆角保持不变
    send_map_btn.pack(side=tk.LEFT, padx=(0, 10))

    # 远程控制按钮 (圆角设计) - 放在同一行，加宽
    def send_rc_command():
        send_remote_control(100, True)

    send_rc_btn = ModernButton(button_frame, text="SEND REMOTE CONTROL",
                               command=send_rc_command,
                               width=200, height=35,  # 加宽
                               bg="#e74c3c", fg="#ffffff", hover_bg="#c0392b",
                               radius=8)  # 按钮圆角保持不变
    send_rc_btn.pack(side=tk.LEFT, padx=(0, 0))

    def refresh():
        stage_label.config(text=game_stage)
        score_label.config(text=red_score)
        event_listbox.delete(0, tk.END)
        for event in event_log:
            event_listbox.insert(tk.END, event)

        # 更新视频
        global video_frame
        if video_frame is not None:
            try:
                img = Image.fromarray(cv2.cvtColor(video_frame, cv2.COLOR_BGR2RGB))
                img = img.resize((640, 360), Image.Resampling.LANCZOS)  # 调整大小以适应画布
                imgtk = ImageTk.PhotoImage(image=img)
                video_canvas.delete("all")  # 清除之前的图像
                video_canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
                video_canvas.image = imgtk  # 保持引用防止垃圾回收
            except Exception as e:
                logging.error(f"UI video update error: {e}")

        root.after(100, refresh)  # 每100ms刷新

    refresh()
    root.mainloop()


# MQTT回调
def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("Connected successfully")
        logging.info("Connected to MQTT server")
        client.subscribe("GameStatus")
        client.subscribe("Event")
        client.subscribe("GlobalUnitStatus")
    else:
        print(f"Connection failed with code {reason_code}")
        logging.error(f"Connection failed with code {reason_code}")


def on_message(client, userdata, msg):
    global game_stage, red_score, event_log
    try:
        if msg.topic == "GameStatus":
            game_status = rm_pb.GameStatus()
            game_status.ParseFromString(msg.payload)
            game_stage = game_status.current_stage
            red_score = game_status.red_score
            print(f"Game stage: {game_stage}, Red score: {red_score}")
            logging.info(f"GameStatus received: stage {game_stage}")
        elif msg.topic == "Event":
            event = rm_pb.Event()
            event.ParseFromString(msg.payload)
            event_str = f"Event ID: {event.event_id}, Param: {event.param}"
            event_log.append(event_str)
            if len(event_log) > 10:
                event_log.pop(0)
            print(event_str)
            logging.info(event_str)
        # 添加其他处理
    except Exception as e:
        print(f"Parse error: {e}")
        logging.error(f"Parse error on topic {msg.topic}: {e}")


# （适配新版API，5个参数）


def on_disconnect(client, userdata, disconnect_flags, rc, properties):
    print("Disconnected, trying to reconnect...")
    logging.warning("Disconnected, reconnecting...")
    time.sleep(RECONNECT_DELAY)
    client.reconnect()


# UDP视频接收（HEVC解码简化，实际需ffmpeg集成）
def receive_video():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', 3334))
    while True:
        data, addr = sock.recvfrom(65535)
        try:
            global video_frame
            nparr = np.frombuffer(data, np.uint8)
            video_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            logging.info("Video frame received")
        except Exception as e:
            logging.error(f"Video decode error: {e}")


# 发送示例函数
def send_remote_control(mouse_x=100, left_down=True):
    rc = rm_pb.RemoteControl()
    rc.mouse_x = mouse_x
    rc.left_button_down = left_down
    payload = rc.SerializeToString()
    client.publish("RemoteControl", payload)
    logging.info("Sent RemoteControl")


def send_map_target(target_x=500, target_y=300):
    map_target = rm_pb.MapCommand()  # 假设proto有MapCommand，根据文档调整
    map_target.target_x = target_x
    map_target.target_y = target_y
    payload = map_target.SerializeToString()
    client.publish("MapCommand", payload)
    logging.info(f"Sent Map Target: ({target_x}, {target_y})")


client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION2, client_id=CLIENT_ID)
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

try:
    client.connect(SERVER_IP, MQTT_PORT, 60)
except Exception as e:
    print("Connection failed: ", e)
    logging.error(f"Connection failed: {e}")

client.loop_start()

# 启动线程
ui_thread = threading.Thread(target=update_ui)
ui_thread.start()
video_thread = threading.Thread(target=receive_video)
video_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    client.disconnect()
    ui_thread.join()
    video_thread.join()
