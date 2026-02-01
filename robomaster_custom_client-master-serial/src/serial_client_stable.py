import serial, struct, threading, logging, time, socket
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import robomaster_pb2 as pb

# -----------------------------
# 配置项
# -----------------------------
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
READ_TIMEOUT = 0.1
MAX_PAYLOAD_LEN = 4096
HEALTH_MAX = 1000
REFRESH_INTERVAL = 100  # UI刷新间隔(ms)
SYNC_DROP_THRESHOLD = 50

# 日志配置
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# -----------------------------
# 全局变量
# -----------------------------
client = None
cmd_panels = {}
event_log = []

# -----------------------------
# CMD 映射（修复：0x0002是Event，不是GlobalUnitStatus）
# -----------------------------
RECV_CMD_MAP = {
    0x0001: pb.GameStatus,
    0x0002: pb.Event,  # 修复：0x0002是Event
    0x0003: pb.GlobalUnitStatus,  # GlobalUnitStatus可能是0x0003
    0x0004: pb.GlobalLogisticsStatus,
    0x0102: pb.RobotDynamicStatus,
    0x0101: pb.RobotStaticStatus,
    0x0103: pb.RobotModuleStatus,
}

SEND_CMD_MAP = {
    0x0201: pb.RemoteControl,
    0x0202: pb.MapCommand,
}

# -----------------------------
# 串口客户端类
# -----------------------------
class StudentEngineSerialClient:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUDRATE):
        self.ser = None
        self.running = True
        self.buffer = bytearray()
        self.latest_msgs = {}
        self.sync_drop_count = 0
        self.parse_error_count = 0  # 统计解析错误
        self.parse_success_count = 0  # 统计解析成功
        self._init_default_msgs()

        # 串口连接
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baudrate, timeout=READ_TIMEOUT,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS
            )
            if self.ser.is_open:
                self.thread = threading.Thread(target=self._recv_loop, daemon=True)
                self.thread.start()
                logging.info(f"✓ 串口连接成功: {port} @ {baudrate}")
                self._add_event(f"✓ 串口连接成功: {port} @ {baudrate}")
            else:
                raise Exception("串口打开失败")
        except Exception as e:
            logging.error(f"✗ 串口连接失败: {e}")
            self._add_event(f"✗ 串口连接失败: {e}")

    def _init_default_msgs(self):
        """初始化默认消息，避免UI显示空数据"""
        # GameStatus
        default_gs = pb.GameStatus()
        default_gs.current_stage = 0
        default_gs.current_round = 0
        default_gs.total_rounds = 0
        default_gs.red_score = 0
        default_gs.blue_score = 0
        default_gs.stage_countdown_sec = 0
        self.latest_msgs[0x0001] = default_gs

        # GlobalUnitStatus
        default_gus = pb.GlobalUnitStatus()
        default_gus.base_health = HEALTH_MAX
        default_gus.outpost_health = HEALTH_MAX
        for _ in range(5):
            default_gus.robot_health.append(HEALTH_MAX)
        self.latest_msgs[0x0003] = default_gus

        # RobotDynamicStatus
        default_rds = pb.RobotDynamicStatus()
        default_rds.current_health = HEALTH_MAX
        default_rds.current_heat = 0.0
        default_rds.remaining_ammo = 0
        self.latest_msgs[0x0102] = default_rds

    def send(self, cmd_id, msg):
        """发送指令"""
        if not self.ser or not self.ser.is_open:
            self._add_event("✗ 串口未打开，无法发送")
            return
        if cmd_id not in SEND_CMD_MAP:
            self._add_event(f"✗ 未知指令ID: {hex(cmd_id)}")
            return
        try:
            payload = msg.SerializeToString()
            if len(payload) > MAX_PAYLOAD_LEN:
                self._add_event(f"✗ 载荷过长: {len(payload)}字节")
                return
            
            frame_head = struct.pack("<HH", len(payload), cmd_id)
            frame = frame_head + payload
            self.ser.write(frame)
            self.ser.flush()
            
            self._add_event(f"✓ 发送成功: {hex(cmd_id)} ({len(frame)}字节)")
            logging.info(f"发送指令 {hex(cmd_id)}, 帧长度: {len(frame)}字节")
        except Exception as e:
            err_msg = f"✗ 发送失败: {hex(cmd_id)} - {str(e)[:30]}"
            self._add_event(err_msg)
            logging.error(err_msg)

    def _recv_loop(self):
        """接收循环"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    self.buffer.extend(self.ser.read(self.ser.in_waiting))
                    self.sync_drop_count = 0
                self._parse_buffer()
                time.sleep(0.005)
            except Exception as e:
                logging.error(f"接收异常: {str(e)[:50]}")
                time.sleep(0.05)

    def _parse_buffer(self):
        """解析缓冲区"""
        while len(self.buffer) >= 4:
            # 读取数据长度和命令ID
            try:
                data_len, cmd_id = struct.unpack("<HH", self.buffer[:4])
            except:
                self.buffer.pop(0)
                continue
            
            # 数据长度合理性检查
            if not (0 < data_len <= MAX_PAYLOAD_LEN):
                if self.sync_drop_count < SYNC_DROP_THRESHOLD:
                    self.buffer.pop(0)
                    self.sync_drop_count += 1
                else:
                    # 连续错误太多，清空缓冲区
                    self.buffer.clear()
                    self.sync_drop_count = 0
                    logging.warning("帧同步失败次数过多，清空缓冲区")
                continue
            
            # 检查是否有完整数据包
            if len(self.buffer) < 4 + data_len:
                return  # 等待更多数据
            
            # 提取数据包
            payload = bytes(self.buffer[4:4+data_len])
            self.buffer = self.buffer[4+data_len:]
            self.sync_drop_count = 0
            
            # 处理数据包
            self._handle_packet(cmd_id, payload)

    def _handle_packet(self, cmd_id, payload):
        """处理数据包 - 增强错误处理"""
        proto_cls = RECV_CMD_MAP.get(cmd_id)
        if not proto_cls:
            # 未知命令ID，不算错误
            logging.debug(f"未知cmd_id: {hex(cmd_id)}")
            return
        
        try:
            msg = proto_cls()
            msg.ParseFromString(payload)
            
            # 更新缓存
            self.latest_msgs[cmd_id] = msg
            self.parse_success_count += 1
            
            # 记录重要消息
            if cmd_id == 0x0001:
                logging.info(f"GameStatus | 阶段:{msg.current_stage} 红:{msg.red_score} 蓝:{msg.blue_score} 倒计时:{msg.stage_countdown_sec}s")
                self._add_event(f"游戏状态 | 阶段{msg.current_stage} | {msg.red_score}:{msg.blue_score}")
            elif cmd_id == 0x0002:
                # Event
                logging.info(f"Event | ID:{msg.event_id} 参数:{msg.param}")
                self._add_event(f"事件 | ID:{msg.event_id} 参数:{msg.param}")
            elif cmd_id == 0x0003:
                # GlobalUnitStatus
                logging.info(f"单位状态 | 基地:{msg.base_health} 前哨:{msg.outpost_health}")
                self._add_event(f"血量 | 基地:{msg.base_health} 前哨:{msg.outpost_health}")
            elif cmd_id == 0x0102:
                # RobotDynamicStatus
                self._add_event(f"本机 | HP:{msg.current_health} 热量:{msg.current_heat:.1f}")
            
            # 更新UI面板
            self.on_message(cmd_id, msg)
            
        except Exception as e:
            self.parse_error_count += 1
            error_msg = str(e)[:50]
            
            # 只记录非GameStatus的错误，减少日志噪音
            if cmd_id != 0x0001:
                logging.warning(f"解析{hex(cmd_id)}失败: {error_msg}")
            else:
                # GameStatus解析失败时，每10次才记录一次
                if self.parse_error_count % 10 == 0:
                    logging.warning(f"GameStatus解析失败(已忽略{self.parse_error_count}次)")
            
            # 解析失败时不更新缓存，使用之前的有效数据

    def on_message(self, cmd_id, msg):
        """更新UI面板"""
        panel = cmd_panels.get(cmd_id)
        if panel:
            try:
                panel.config(state=tk.NORMAL)
                panel.delete("1.0", tk.END)
                panel.insert(tk.END, str(msg))
                panel.config(state=tk.DISABLED)
            except:
                pass

    def _add_event(self, msg):
        """添加事件日志"""
        global event_log
        timestamp = time.strftime('%H:%M:%S')
        event_log.append(f"[{timestamp}] {msg}")
        if len(event_log) > 100:
            event_log.pop(0)

    def close(self):
        """关闭串口"""
        self.running = False
        if hasattr(self, 'thread') and self.thread:
            self.thread.join(timeout=1)
        if self.ser and self.ser.is_open:
            self.ser.close()
        logging.info("串口已关闭")

# -----------------------------
# 自定义按钮
# -----------------------------
class ModernButton(tk.Canvas):
    def __init__(self, parent, text="", command=None, width=120, height=35,
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

    def draw_button(self, color):
        self.delete("all")
        self.create_rounded_rectangle(2, 2, self.width+2, self.height+2, radius=self.radius, fill="#aaaaaa")
        self.create_rounded_rectangle(0, 0, self.width, self.height, radius=self.radius, fill=color)
        self.create_text(self.width//2, self.height//2, text=self.text, fill=self.fg, font=('Arial', 11, 'bold'))

    def create_rounded_rectangle(self, x1, y1, x2, y2, radius=25, **kwargs):
        points = [x1+radius, y1, x2-radius, y1, x2, y1, x2, y1+radius, x2, y2-radius, x2, y2, 
                  x2-radius, y2, x1+radius, y2, x1, y2, x1, y2-radius, x1, y1+radius, x1, y1]
        return self.create_polygon(points, smooth=True, **kwargs)

    def on_click(self, event):
        if self.command:
            self.command()

    def on_enter(self, event):
        self.draw_button(self.hover_bg)

    def on_leave(self, event):
        self.draw_button(self.bg)

# -----------------------------
# UI界面
# -----------------------------
def create_ui(recv_callback, send_callback):
    root = tk.Tk()
    root.title("RoboMaster 仪表盘 - 稳定版")
    root.geometry("1000x700")
    root.configure(bg="#f5f5f5")
    root.resizable(True, True)

    # 定义ttk进度条样式
    style = ttk.Style(root)
    style.layout('Green.TProgressbar', style.layout('Horizontal.TProgressbar'))
    style.layout('Blue.TProgressbar', style.layout('Horizontal.TProgressbar'))
    style.layout('Yellow.TProgressbar', style.layout('Horizontal.TProgressbar'))
    style.layout('Red.TProgressbar', style.layout('Horizontal.TProgressbar'))
    style.configure('Green.TProgressbar', background='#2e7d32')
    style.configure('Blue.TProgressbar', background='#1976d2')
    style.configure('Yellow.TProgressbar', background='#f9a825')
    style.configure('Red.TProgressbar', background='#d32f2f')

    # 主标题
    title_frame = tk.Frame(root, bg="#ffffff", height=60)
    title_frame.pack(fill='x', pady=(0, 10))
    title_frame.pack_propagate(False)
    tk.Label(title_frame, text="ROBOMASTER DASHBOARD", bg="#ffffff", fg="#2c3e50", 
             font=('Arial', 20, 'bold')).pack(pady=15)

    # 主容器
    main_frame = tk.Frame(root, bg="#f5f5f5")
    main_frame.pack(fill='both', expand=True, padx=20, pady=(0, 20))

    # 左侧：状态显示
    left_frame = tk.Frame(main_frame, bg="#ffffff", bd=1, relief=tk.FLAT)
    left_frame.pack(side=tk.LEFT, fill='both', expand=True, padx=(0, 10))

    # GameStatus面板
    gs_frame = tk.Frame(left_frame, bg="#e8f4f8", bd=2, relief=tk.GROOVE)
    gs_frame.pack(fill='x', padx=15, pady=(15, 10))
    tk.Label(gs_frame, text="🎮 游戏状态", bg="#e8f4f8", fg="#00796b", 
             font=('Arial', 14, 'bold')).pack(anchor='w', padx=10, pady=(10, 15))
    
    gs_info_frame = tk.Frame(gs_frame, bg="#e8f4f8")
    gs_info_frame.pack(fill='x', padx=20, pady=(0, 15))
    
    stage_label = tk.Label(gs_info_frame, text="阶段: 0", bg="#e8f4f8", fg="#d32f2f", 
                           font=('Arial', 14, 'bold'))
    stage_label.grid(row=0, column=0, padx=(0, 30), pady=5)
    
    round_label = tk.Label(gs_info_frame, text="回合: 0/0", bg="#e8f4f8", fg="#f57c00", 
                           font=('Arial', 14, 'bold'))
    round_label.grid(row=0, column=1, padx=(0, 30), pady=5)
    
    countdown_label = tk.Label(gs_info_frame, text="倒计时: 0s", bg="#e8f4f8", fg="#1976d2", 
                               font=('Arial', 14, 'bold'))
    countdown_label.grid(row=0, column=2, pady=5)
    
    score_label = tk.Label(gs_info_frame, text="红方 0 : 蓝方 0", bg="#e8f4f8", fg="#c2185b", 
                           font=('Arial', 16, 'bold'))
    score_label.grid(row=1, column=0, columnspan=3, pady=(10, 10))

    # 血量面板
    health_frame = tk.Frame(left_frame, bg="#ffffff", bd=1, relief=tk.FLAT)
    health_frame.pack(fill='x', padx=15, pady=(0, 15))
    tk.Label(health_frame, text="🛡️ 血量状态", bg="#ffffff", fg="#2c3e50", 
             font=('Arial', 12, 'bold')).pack(anchor='w', padx=10, pady=(8, 10))
    
    health_info = tk.Frame(health_frame, bg="#ffffff")
    health_info.pack(fill='x', padx=20, pady=(0, 15))

    # 基地血条
    tk.Label(health_info, text="基地:", bg="#ffffff", fg="#666666", 
             font=('Arial', 11)).grid(row=0, column=0, sticky='w')
    base_health_var = tk.StringVar(value=f"{HEALTH_MAX}/{HEALTH_MAX}")
    base_health_label = tk.Label(health_info, textvariable=base_health_var, bg="#ffffff", 
                                  fg="#2e7d32", font=('Arial', 11), width=10)
    base_health_label.grid(row=0, column=1, sticky='w')
    
    base_bar_frame = tk.Frame(health_info, bg="#e0e0e0", height=18, width=280)
    base_bar_frame.grid(row=0, column=2, padx=(10, 0), sticky='ew')
    base_bar_frame.pack_propagate(False)
    base_health_bar = ttk.Progressbar(base_bar_frame, orient=tk.HORIZONTAL, length=280, 
                                      mode='determinate', maximum=HEALTH_MAX, style='Green.TProgressbar')
    base_health_bar.pack(fill='both', expand=True, padx=2, pady=2)
    base_health_bar['value'] = HEALTH_MAX

    # 前哨血条
    tk.Label(health_info, text="前哨:", bg="#ffffff", fg="#666666", 
             font=('Arial', 11)).grid(row=1, column=0, sticky='w', pady=(8, 0))
    outpost_health_var = tk.StringVar(value=f"{HEALTH_MAX}/{HEALTH_MAX}")
    outpost_health_label = tk.Label(health_info, textvariable=outpost_health_var, bg="#ffffff", 
                                     fg="#2e7d32", font=('Arial', 11), width=10)
    outpost_health_label.grid(row=1, column=1, sticky='w', pady=(8, 0))
    
    outpost_bar_frame = tk.Frame(health_info, bg="#e0e0e0", height=18, width=280)
    outpost_bar_frame.grid(row=1, column=2, padx=(10, 0), sticky='ew', pady=(8, 0))
    outpost_bar_frame.pack_propagate(False)
    outpost_health_bar = ttk.Progressbar(outpost_bar_frame, orient=tk.HORIZONTAL, length=280, 
                                         mode='determinate', maximum=HEALTH_MAX, style='Blue.TProgressbar')
    outpost_health_bar.pack(fill='both', expand=True, padx=2, pady=2)
    outpost_health_bar['value'] = HEALTH_MAX

    # 本机状态
    robot_frame = tk.Frame(left_frame, bg="#f8f9fa", bd=1, relief=tk.FLAT)
    robot_frame.pack(fill='x', padx=15, pady=(0, 15))
    tk.Label(robot_frame, text="🤖 本机状态", bg="#f8f9fa", fg="#2c3e50", 
             font=('Arial', 12, 'bold')).pack(anchor='w', padx=10, pady=(8, 10))
    
    robot_info_frame = tk.Frame(robot_frame, bg="#f8f9fa")
    robot_info_frame.pack(fill='x', padx=20, pady=(0, 15))

    # 本机血条
    tk.Label(robot_info_frame, text="血量:", bg="#f8f9fa", fg="#666666", 
             font=('Arial', 11)).grid(row=0, column=0, sticky='w')
    robot_health_var = tk.StringVar(value=f"{HEALTH_MAX}/{HEALTH_MAX}")
    robot_health_label = tk.Label(robot_info_frame, textvariable=robot_health_var, bg="#f8f9fa", 
                                   fg="#2e7d32", font=('Arial', 11), width=10)
    robot_health_label.grid(row=0, column=1, sticky='w')
    
    robot_bar_frame = tk.Frame(robot_info_frame, bg="#e0e0e0", height=18, width=280)
    robot_bar_frame.grid(row=0, column=2, padx=(10, 0), sticky='ew')
    robot_bar_frame.pack_propagate(False)
    robot_health_bar = ttk.Progressbar(robot_bar_frame, orient=tk.HORIZONTAL, length=280, 
                                       mode='determinate', maximum=HEALTH_MAX, style='Green.TProgressbar')
    robot_health_bar.pack(fill='both', expand=True, padx=2, pady=2)
    robot_health_bar['value'] = HEALTH_MAX

    # 其他状态
    robot_heat_var = tk.StringVar(value="0.0")
    robot_ammo_var = tk.StringVar(value="0")
    tk.Label(robot_info_frame, text="热量:", bg="#f8f9fa", fg="#666666", 
             font=('Arial', 11)).grid(row=1, column=0, sticky='w', pady=(8, 0))
    tk.Label(robot_info_frame, textvariable=robot_heat_var, bg="#f8f9fa", fg="#d32f2f", 
             font=('Arial', 11), width=10).grid(row=1, column=1, sticky='w', pady=(8, 0))
    tk.Label(robot_info_frame, text="弹药:", bg="#f8f9fa", fg="#666666", 
             font=('Arial', 11)).grid(row=1, column=2, sticky='w', padx=(10, 0), pady=(8, 0))
    tk.Label(robot_info_frame, textvariable=robot_ammo_var, bg="#f8f9fa", fg="#f57c00", 
             font=('Arial', 11)).grid(row=1, column=3, sticky='w', pady=(8, 0))

    # 右侧：控制+日志
    right_frame = tk.Frame(main_frame, bg="#ffffff", bd=1, relief=tk.FLAT, width=350)
    right_frame.pack(side=tk.RIGHT, fill='y', padx=(10, 0))
    right_frame.pack_propagate(False)

    # 控制面板
    control_frame = tk.Frame(right_frame, bg="#ffffff")
    control_frame.pack(fill='x', padx=15, pady=(15, 10))
    tk.Label(control_frame, text="📡 指令控制", bg="#ffffff", fg="#2c3e50", 
             font=('Arial', 12, 'bold')).pack(anchor='w', pady=(0, 15))
    
    # 按钮
    btn_frame = tk.Frame(control_frame, bg="#ffffff")
    btn_frame.pack(fill='x', pady=(0, 15))
    ModernButton(btn_frame, text="远程控制-前进", 
                 command=lambda: send_callback(0x0201, 50, 0, True), 
                 bg="#27ae60").pack(fill='x', pady=3)
    ModernButton(btn_frame, text="远程控制-停止", 
                 command=lambda: send_callback(0x0201, 0, 0, False), 
                 bg="#e74c3c").pack(fill='x', pady=3)
    
    # 地图坐标
    map_frame = tk.Frame(control_frame, bg="#ffffff")
    map_frame.pack(fill='x')
    tk.Label(map_frame, text="地图坐标:", bg="#ffffff", fg="#2c3e50", 
             font=('Arial', 10, 'bold')).pack(anchor='w', pady=(0, 8))
    
    xy_frame = tk.Frame(map_frame, bg="#ffffff")
    xy_frame.pack(fill='x', pady=(0, 8))
    tk.Label(xy_frame, text="X:", bg="#ffffff", fg="#666666", width=2).pack(side=tk.LEFT)
    x_entry = tk.Entry(xy_frame, bg="#f8f9fa", bd=1, relief=tk.FLAT, font=('Arial', 10))
    x_entry.pack(side=tk.LEFT, fill='x', expand=True, padx=(3, 8))
    x_entry.insert(0, "100.0")
    tk.Label(xy_frame, text="Y:", bg="#ffffff", fg="#666666", width=2).pack(side=tk.LEFT)
    y_entry = tk.Entry(xy_frame, bg="#f8f9fa", bd=1, relief=tk.FLAT, font=('Arial', 10))
    y_entry.pack(side=tk.LEFT, fill='x', expand=True, padx=(3, 0))
    y_entry.insert(0, "200.0")
    
    def send_map_cmd():
        try:
            x = float(x_entry.get().strip())
            y = float(y_entry.get().strip())
            send_callback(0x0202, x, y)
        except ValueError:
            recv_callback._add_event("✗ 地图坐标格式错误")
    
    ModernButton(map_frame, text="发送地图指令", command=send_map_cmd, 
                 bg="#e67e22").pack(fill='x')

    # 事件日志
    log_frame = tk.Frame(right_frame, bg="#ffffff")
    log_frame.pack(fill='both', expand=True, padx=15, pady=(10, 15))
    tk.Label(log_frame, text="📋 事件日志", bg="#ffffff", fg="#2c3e50", 
             font=('Arial', 12, 'bold')).pack(anchor='w', pady=(0, 8))
    
    log_scroll = tk.Scrollbar(log_frame, orient=tk.VERTICAL)
    log_listbox = tk.Listbox(log_frame, yscrollcommand=log_scroll.set, bg="#f8f9fa", 
                             fg="#2c3e50", font=('Courier', 9), height=28)
    log_scroll.config(command=log_listbox.yview)
    log_scroll.pack(side=tk.RIGHT, fill='y')
    log_listbox.pack(side=tk.LEFT, fill='both', expand=True)

    # 统计信息
    stats_frame = tk.Frame(right_frame, bg="#ffffff")
    stats_frame.pack(fill='x', padx=15, pady=(0, 10))
    stats_label = tk.Label(stats_frame, text="统计: 成功0 失败0", bg="#ffffff", 
                          fg="#7f8c8d", font=('Arial', 8))
    stats_label.pack(anchor='w')

    # UI刷新
    def refresh_ui():
        try:
            # GameStatus
            gs = recv_callback.latest_msgs.get(0x0001)
            if gs:
                stage_label.config(text=f"阶段: {gs.current_stage}")
                round_label.config(text=f"回合: {gs.current_round}/{gs.total_rounds}")
                countdown_label.config(text=f"倒计时: {gs.stage_countdown_sec}s")
                score_label.config(text=f"红方 {gs.red_score} : 蓝方 {gs.blue_score}")

            # 血量
            gu = recv_callback.latest_msgs.get(0x0003)
            if gu:
                # 基地
                base_hp = min(max(gu.base_health, 0), HEALTH_MAX)
                base_health_var.set(f"{base_hp}/{HEALTH_MAX}")
                base_health_bar['value'] = base_hp
                if base_hp > HEALTH_MAX * 0.7:
                    base_health_bar.config(style='Green.TProgressbar')
                    base_health_label.config(fg="#2e7d32")
                elif base_hp > HEALTH_MAX * 0.3:
                    base_health_bar.config(style='Yellow.TProgressbar')
                    base_health_label.config(fg="#f9a825")
                else:
                    base_health_bar.config(style='Red.TProgressbar')
                    base_health_label.config(fg="#d32f2f")

                # 前哨
                outpost_hp = min(max(gu.outpost_health, 0), HEALTH_MAX)
                outpost_health_var.set(f"{outpost_hp}/{HEALTH_MAX}")
                outpost_health_bar['value'] = outpost_hp
                if outpost_hp > HEALTH_MAX * 0.7:
                    outpost_health_bar.config(style='Blue.TProgressbar')
                    outpost_health_label.config(fg="#2e7d32")
                elif outpost_hp > HEALTH_MAX * 0.3:
                    outpost_health_bar.config(style='Yellow.TProgressbar')
                    outpost_health_label.config(fg="#f9a825")
                else:
                    outpost_health_bar.config(style='Red.TProgressbar')
                    outpost_health_label.config(fg="#d32f2f")

            # 本机
            rd = recv_callback.latest_msgs.get(0x0102)
            if rd:
                robot_hp = min(max(rd.current_health, 0), HEALTH_MAX)
                robot_health_var.set(f"{robot_hp}/{HEALTH_MAX}")
                robot_health_bar['value'] = robot_hp
                if robot_hp > HEALTH_MAX * 0.7:
                    robot_health_bar.config(style='Green.TProgressbar')
                    robot_health_label.config(fg="#2e7d32")
                elif robot_hp > HEALTH_MAX * 0.3:
                    robot_health_bar.config(style='Yellow.TProgressbar')
                    robot_health_label.config(fg="#f9a825")
                else:
                    robot_health_bar.config(style='Red.TProgressbar')
                    robot_health_label.config(fg="#d32f2f")
                robot_heat_var.set(f"{rd.current_heat:.1f}")
                robot_ammo_var.set(f"{rd.remaining_ammo}")

            # 统计
            stats_label.config(text=f"统计: 成功{recv_callback.parse_success_count} "
                                   f"失败{recv_callback.parse_error_count}")

            # 日志
            log_listbox.delete(0, tk.END)
            for log in event_log[-30:]:  # 只显示最近30条
                log_listbox.insert(tk.END, log)
            log_listbox.see(tk.END)

        except Exception as e:
            logging.error(f"UI刷新错误: {e}")

        root.after(REFRESH_INTERVAL, refresh_ui)

    refresh_ui()
    root.mainloop()

# -----------------------------
# 主程序
# -----------------------------
if __name__ == "__main__":
    print("=" * 60)
    print("RoboMaster 自定义客户端 - 稳定版")
    print("=" * 60)
    
    # 初始化串口
    client = StudentEngineSerialClient()

    # 发送回调
    def send_callback(cmd_id, *args):
        if cmd_id not in SEND_CMD_MAP:
            client._add_event(f"✗ 未知指令ID: {hex(cmd_id)}")
            return
        
        msg = SEND_CMD_MAP[cmd_id]()
        
        if cmd_id == 0x0201 and len(args) >= 3:  # RemoteControl
            msg.mouse_x = int(args[0])
            msg.mouse_y = int(args[1])
            msg.left_button_down = bool(args[2])
        elif cmd_id == 0x0202 and len(args) >= 2:  # MapCommand
            msg.target_x = float(args[0])
            msg.target_y = float(args[1])
            msg.type = 1
        
        client.send(cmd_id, msg)

    # 启动UI
    try:
        create_ui(client, send_callback)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        client.close()
        print("程序退出")
