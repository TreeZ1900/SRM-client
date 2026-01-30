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
# 配置项
# ----------------------
LOG_FILE = "serialclient_simple.log"
UDP_VIDEO_PORT = 3334
COM_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
RECONNECT_DELAY = 3

# 日志设置
logging.basicConfig(filename=LOG_FILE, level=logging.INFO,
                    format='%(asctime)s - %(message)s')

# ----------------------
# 全局变量
# ----------------------
game_stage = "Unknown"
red_score = 0
event_log = []
video_frame = None
seq = 0
ser = None

# ----------------------
# 串口初始化
# ----------------------
def init_serial():
    """初始化串口"""
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
# 发送函数（保持不变，以防万一需要发送）
# ----------------------
def send_remote_control(mouse_x=100, left_down=True):
    """发送RemoteControl指令"""
    if not ser or not ser.is_open:
        logging.warning("Serial port not open, skip send RemoteControl")
        return
    try:
        rc = rm_pb.RemoteControl()
        rc.mouse_x = mouse_x
        rc.left_button_down = left_down
        # 简化格式：[长度2字节][cmd_id 2字节][数据]
        data_bytes = rc.SerializeToString()
        data_len = len(data_bytes)
        frame = struct.pack('<HH', data_len, 0x0301) + data_bytes
        ser.write(frame)
        logging.info("Sent RemoteControl (simplified protocol)")
        print("Sent RemoteControl")
    except Exception as e:
        logging.error(f"Send RemoteControl error: {e}")

def send_map_target(target_x=500, target_y=300):
    """发送MapCommand指令"""
    if not ser or not ser.is_open:
        logging.warning("Serial port not open, skip send MapTarget")
        return
    try:
        map_target = rm_pb.MapCommand()
        map_target.target_x = target_x
        map_target.target_y = target_y
        data_bytes = map_target.SerializeToString()
        data_len = len(data_bytes)
        frame = struct.pack('<HH', data_len, 0x0302) + data_bytes
        ser.write(frame)
        logging.info(f"Sent Map Target: ({target_x}, {target_y}) (simplified protocol)")
        print(f"Sent Map Target: ({target_x}, {target_y})")
    except Exception as e:
        logging.error(f"Send MapTarget error: {e}")

# ----------------------
# 自定义圆角按钮
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

    def draw_button(self, bg_color):
        self.delete("all")
        r = self.radius
        self.create_arc((0, 0, 2*r, 2*r), start=90, extent=90, fill=bg_color, outline="")
        self.create_arc((self.width-2*r, 0, self.width, 2*r), start=0, extent=90, fill=bg_color, outline="")
        self.create_arc((0, self.height-2*r, 2*r, self.height), start=180, extent=90, fill=bg_color, outline="")
        self.create_arc((self.width-2*r, self.height-2*r, self.width, self.height), start=270, extent=90, fill=bg_color, outline="")
        self.create_rectangle(r, 0, self.width-r, self.height, fill=bg_color, outline="")
        self.create_rectangle(0, r, self.width, self.height-r, fill=bg_color, outline="")
        self.create_text(self.width/2, self.height/2, text=self.text, fill=self.fg, font=('Segoe UI', 10, 'bold'))

    def on_enter(self, event):
        self.draw_button(self.hover_bg)

    def on_leave(self, event):
        self.draw_button(self.bg)

    def on_click(self, event):
        if self.command:
            self.command()

# ----------------------
# UI更新函数
# ----------------------
def update_ui():
    """UI主界面"""
    root = tk.Tk()
    root.title("RoboMaster Client UI (Simplified Protocol)")
    root.geometry("800x700")
    root.configure(bg="#f5f5f5")

    # 主标题区
    title_frame = tk.Frame(root, bg="#ffffff", height=70)
    title_frame.pack(fill='x', pady=(0, 10))
    title_label = tk.Label(title_frame, text="ROBOMASTER DASHBOARD",
                           bg="#ffffff", fg="#333333", font=('Segoe UI', 20, 'bold'))
    title_label.pack(pady=15)

    # 主容器
    main_frame = tk.Frame(root, bg="#f5f5f5")
    main_frame.pack(fill='both', expand=True, padx=20, pady=(0, 20))

    # 左侧：视频显示
    video_frame = tk.Frame(main_frame, bg="#ffffff", relief='flat', bd=0)
    video_frame.pack(side=tk.LEFT, fill='both', expand=True, padx=(0, 10))

    video_title = tk.Label(video_frame, text="LIVE VIDEO FEED",
                           bg="#ffffff", fg="#333333", font=('Segoe UI', 12, 'bold'))
    video_title.pack(anchor='w', padx=15, pady=(15, 10))

    video_canvas = tk.Canvas(video_frame, width=640, height=360, bg="#1a1a1a", highlightthickness=0)
    video_canvas.pack(padx=15, pady=(0, 15))

    # 右侧：状态+交互
    interact_frame = tk.Frame(main_frame, bg="#ffffff", width=350, relief='flat', bd=0)
    interact_frame.pack(side=tk.RIGHT, fill='y')
    interact_frame.pack_propagate(False)

    # Game Stage 显示
    stage_frame = tk.Frame(interact_frame, bg="#ffffff")
    stage_frame.pack(fill='x', padx=25, pady=(20, 8))
    tk.Label(stage_frame, text="Game Stage:", bg="#ffffff", fg="#555555", font=('Segoe UI', 11)).pack(anchor='w')
    stage_label = tk.Label(stage_frame, text="Unknown", bg="#ffffff", fg="#ff5733",
                           font=('Segoe UI', 18, 'bold'))
    stage_label.pack(anchor='w', pady=(3, 0))

    # Red Score 显示
    score_frame = tk.Frame(interact_frame, bg="#ffffff")
    score_frame.pack(fill='x', padx=25, pady=(8, 15))
    tk.Label(score_frame, text="Red Score:", bg="#ffffff", fg="#555555", font=('Segoe UI', 11)).pack(anchor='w')
    score_label = tk.Label(score_frame, text="0", bg="#ffffff", fg="#e74c3c",
                           font=('Segoe UI', 18, 'bold'))
    score_label.pack(anchor='w', pady=(3, 0))

    # Event Log 显示
    log_frame = tk.Frame(interact_frame, bg="#ffffff")
    log_frame.pack(fill='both', expand=True, padx=25, pady=(15, 8))
    tk.Label(log_frame, text="EVENT LOG", bg="#ffffff", fg="#333333",
             font=('Segoe UI', 12, 'bold')).pack(anchor='w', pady=(0, 8))

    log_container = tk.Frame(log_frame, bg="#ffffff")
    log_container.pack(fill='both', expand=True)
    log_scrollbar = tk.Scrollbar(log_container, orient='vertical', bg="#e0e0e0", troughcolor="#f0f0f0")
    log_scrollbar.pack(side=tk.RIGHT, fill='y')
    event_listbox = tk.Listbox(log_container, yscrollcommand=log_scrollbar.set,
                               bg="#f9f9f9", fg="#333333",
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
            pass

    send_map_btn = ModernButton(button_frame, text="SEND MAP TARGET",
                                command=send_map_command, width=180, height=35,
                                bg="#3498db", fg="#ffffff", hover_bg="#2980b9", radius=8)
    send_map_btn.pack(side=tk.LEFT, padx=(0, 10))

    # SEND REMOTE CONTROL 按钮
    def send_rc_command():
        send_remote_control(100, True)

    send_rc_btn = ModernButton(button_frame, text="SEND REMOTE CONTROL",
                               command=send_rc_command, width=200, height=35,
                               bg="#e74c3c", fg="#ffffff", hover_bg="#c0392b", radius=8)
    send_rc_btn.pack(side=tk.LEFT, padx=(0, 0))

    # UI刷新函数
    def refresh():
        nonlocal stage_label, score_label, event_listbox, video_canvas
        stage_label.config(text=game_stage)
        score_label.config(text=red_score)
        event_listbox.delete(0, tk.END)
        for event in event_log:
            event_listbox.insert(tk.END, event)
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
        root.after(100, refresh)

    refresh()
    root.mainloop()

# ----------------------
# 简化协议的串口接收
# ----------------------
def serial_receive_loop_simple():
    """
    简化协议的串口接收：
    格式：[数据长度2字节][命令ID 2字节][protobuf数据]
    无帧头，无CRC校验
    """
    global game_stage, red_score, event_log
    buffer = bytearray()
    
    while True:
        if not ser or not ser.is_open:
            time.sleep(RECONNECT_DELAY)
            init_serial()
            continue
        
        try:
            # 读取所有可用数据
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting)
                buffer.extend(new_data)
                
                # 调试日志
                hex_str = ' '.join([f'{b:02X}' for b in new_data[:50]])  # 只显示前50字节
                logging.debug(f"Received {len(new_data)} bytes: {hex_str}...")
            
            # 尝试解析数据包
            while len(buffer) >= 4:  # 至少需要4字节（长度2 + cmd_id 2）
                # 读取数据长度
                data_len = struct.unpack('<H', buffer[0:2])[0]
                
                # 合理性检查：数据长度应该在1-1000之间
                if data_len < 1 or data_len > 1000:
                    # 可能不是有效的包头，丢弃第一个字节继续查找
                    buffer.pop(0)
                    continue
                
                # 检查是否有完整的数据包
                packet_len = 4 + data_len  # 长度(2) + cmd_id(2) + 数据(data_len)
                if len(buffer) < packet_len:
                    break  # 等待更多数据
                
                # 提取完整的数据包
                cmd_id = struct.unpack('<H', buffer[2:4])[0]
                data = bytes(buffer[4:4+data_len])
                
                logging.info(f"Packet: cmd_id=0x{cmd_id:04X}, data_len={data_len}")
                
                # 解析GameStatus (cmd_id=0x0001)
                if cmd_id == 0x0001:
                    try:
                        gs = rm_pb.GameStatus()
                        gs.ParseFromString(data)
                        game_stage = str(gs.current_stage)
                        red_score = gs.red_score
                        logging.info(f"✓ GameStatus: stage={game_stage}, red_score={red_score}")
                        print(f"✓ Game stage: {game_stage}, Red score: {red_score}")
                    except Exception as e:
                        logging.error(f"Failed to parse GameStatus: {e}")
                
                # 解析Event (cmd_id=0x0002)
                elif cmd_id == 0x0002:
                    try:
                        ev = rm_pb.Event()
                        ev.ParseFromString(data)
                        event_str = f"Event ID: {ev.event_id}, Param: {ev.param}"
                        event_log.append(event_str)
                        if len(event_log) > 10:
                            event_log.pop(0)
                        logging.info(f"✓ {event_str}")
                        print(f"✓ {event_str}")
                    except Exception as e:
                        logging.error(f"Failed to parse Event: {e}")
                
                else:
                    logging.warning(f"Unknown cmd_id: 0x{cmd_id:04X}")
                
                # 从缓冲区移除已处理的数据包
                buffer = buffer[packet_len:]
                
        except Exception as e:
            logging.error(f"Serial receive error: {e}", exc_info=True)
        
        time.sleep(0.01)

def receive_video():
    """UDP视频接收线程"""
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
        receive_video()

def log_per_second():
    """每秒打印日志"""
    while True:
        try:
            if ser and ser.is_open:
                log_content = f"Client running - Stage: {game_stage}, Red score: {red_score}"
            else:
                log_content = f"Client running - Port closed, Stage: {game_stage}, Red score: {red_score}"
            logging.info(log_content)
            time.sleep(1)
        except Exception as e:
            logging.error(f"Log per second error: {e}")
            time.sleep(1)

# ----------------------
# 程序入口
# ----------------------
if __name__ == "__main__":
    print("=" * 60)
    print("RoboMaster Client - Simplified Protocol (No 0xA5 header)")
    print("=" * 60)
    
    # 1. 初始化串口
    init_serial()
    
    # 2. 启动后台线程
    serial_thread = threading.Thread(target=serial_receive_loop_simple, daemon=True)
    serial_thread.start()
    
    video_thread = threading.Thread(target=receive_video, daemon=True)
    video_thread.start()
    
    log_thread = threading.Thread(target=log_per_second, daemon=True)
    log_thread.start()
    
    # 3. 启动UI线程
    ui_thread = threading.Thread(target=update_ui)
    ui_thread.start()
    
    # 4. 主线程保持运行
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting client...")
        logging.info("Client exited by user (Ctrl+C)")
        if ser and ser.is_open:
            ser.close()
        ui_thread.join()
        serial_thread.join()
        video_thread.join()
        log_thread.join()
