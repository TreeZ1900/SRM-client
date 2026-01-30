import serial, struct, threading, logging, time, socket
import cv2, numpy as np
import tkinter as tk
from PIL import Image, ImageTk
import robomaster_pb2 as pb
from tkinter import ttk

# -----------------------------
# 配置项（可根据实际情况修改）
# -----------------------------
SERIAL_PORT = "/dev/ttyACM0"  # Windows可改为"COM3"/"COM4"等
BAUDRATE = 115200
READ_TIMEOUT = 0.1
UDP_VIDEO_PORT = 3334
MAX_PAYLOAD_LEN = 4096
HEALTH_MAX = 1000  # 机器人/基地血量上限，适配RoboMaster实际值
REFRESH_INTERVAL = 100  # UI刷新间隔(ms)

# 日志配置：控制台+简洁格式
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# -----------------------------
# 全局变量（仅定义1次，避免冲突）
# -----------------------------
client = None
video_frame = None
cmd_panels = {}  # CMD_ID -> Tkinter Text 显示
event_log = []    # 全局事件日志，用于UI展示

# -----------------------------
# CMD 映射：Protobuf类与指令ID绑定
# -----------------------------
RECV_CMD_MAP = {
    0x0001: pb.GameStatus,
    0x0002: pb.GlobalUnitStatus,
    0x0003: pb.GlobalLogisticsStatus,
    0x0004: pb.GlobalSpecialMechanism,
    0x0005: pb.Event,
    0x0101: pb.RobotStaticStatus,
    0x0102: pb.RobotDynamicStatus,
    0x0103: pb.RobotModuleStatus,
    0x0104: pb.RobotInjuryStat,
    0x0105: pb.RobotRespawnStatus,
    0x0106: pb.RobotPosition,
    0x0203: pb.Buff,
    0x0204: pb.PenaltyInfo,
    0x0205: pb.RobotPathPlanInfo,
    0x0206: pb.MapClickInfoNotify,
    0x0301: pb.RaderInfoToClient,
    0x0302: pb.CustomByteBlock,
    0x0402: pb.TechCoreMotionStateSync,
    0x0404: pb.RobotPerformanceSelectionSync,
    0x0406: pb.DeployModeStatusSync,
    0x0502: pb.RuneStatusSync,
    0x0503: pb.SentinelStatusSync,
    0x0602: pb.DartSelectTargetStatusSync,
    0x0604: pb.GuardCtrlResult,
    0x0606: pb.AirSupportStatusSync,
}

SEND_CMD_MAP = {
    0x0201: pb.RemoteControl,
    0x0202: pb.MapCommand,
    0x0401: pb.AssemblyCommand,
    0x0403: pb.RobotPerformanceSelectionCommand,
    0x0405: pb.HeroDeployModeEventCommand,
    0x0501: pb.RuneActivateCommand,
    0x0601: pb.DartCommand,
    0x0603: pb.GuardCtrlCommand,
    0x0605: pb.AirSupportCommand,
}

# -----------------------------
# 串口客户端类：封装通信与解析逻辑，增加异常处理
# -----------------------------
class StudentEngineSerialClient:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUDRATE):
        self.ser = None
        self.running = True
        self.buffer = bytearray()
        self.latest_msgs = {}  # 缓存最新收到的每条CMD数据，用于UI刷新
        # 串口连接：增加异常处理，避免端口错误崩溃
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=READ_TIMEOUT)
            self.thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.thread.start()
            logging.info(f"串口连接成功：{port} @ {baudrate}")
            self._add_event(f"串口连接成功：{port} @ {baudrate}")
        except Exception as e:
            logging.error(f"串口连接失败：{e}，请检查端口/设备连接")
            self._add_event(f"串口连接失败：{e}")

    def send(self, cmd_id, msg):
        """发送Protobuf消息：校验指令ID+序列化+打包发送"""
        if not self.ser or not self.ser.is_open:
            logging.warning("串口未打开，无法发送指令")
            self._add_event("串口未打开，发送失败")
            return
        if cmd_id not in SEND_CMD_MAP:
            logging.warning(f"不支持的发送指令：{hex(cmd_id)}")
            self._add_event(f"发送失败：未知指令 {hex(cmd_id)}")
            return
        try:
            payload = msg.SerializeToString()
            frame = struct.pack("<HH", len(payload), cmd_id) + payload  # 小端：长度(2)+CMD(2)+数据
            self.ser.write(frame)
            logging.debug(f"发送指令：{hex(cmd_id)}，数据长度：{len(payload)}字节")
            self._add_event(f"发送成功：{hex(cmd_id)} ({len(payload)}字节)")
        except Exception as e:
            logging.error(f"发送指令失败：{e}")
            self._add_event(f"发送失败：{hex(cmd_id)} - {e}")

    def _recv_loop(self):
        """串口接收循环：持续读取数据，带异常容错"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    self.buffer.extend(self.ser.read(self.ser.in_waiting))
                self._parse_buffer()
            except Exception as e:
                logging.error(f"接收循环异常：{e}")
                self._add_event(f"接收异常：{e}")
                time.sleep(0.05)  # 异常后短暂休眠，避免CPU占用过高

    def _parse_buffer(self):
        """解析缓冲区数据：按协议格式拆包，处理粘包/半包"""
        while len(self.buffer) >= 4:  # 至少包含长度+CMD(4字节)
            data_len, cmd_id = struct.unpack("<HH", self.buffer[:4])
            # 校验数据长度：避免无效数据
            if data_len <= 0 or data_len > MAX_PAYLOAD_LEN:
                self.buffer.pop(0)
                logging.warning(f"无效数据长度：{data_len}，丢弃字节")
                continue
            # 数据未接收完整，等待下一次
            if len(self.buffer) < 4 + data_len:
                return
            # 提取有效载荷并解析Protobuf
            payload = bytes(self.buffer[4:4+data_len])
            self.buffer = self.buffer[4+data_len:]  # 清空已解析数据
            self._handle_packet(cmd_id, payload)

    def _handle_packet(self, cmd_id, payload):
        """处理单包数据：解析Protobuf，更新缓存，触发UI刷新"""
        proto_cls = RECV_CMD_MAP.get(cmd_id)
        if not proto_cls:
            logging.debug(f"未映射的接收指令：{hex(cmd_id)}")
            return
        try:
            msg = proto_cls()
            msg.ParseFromString(payload)
            self.latest_msgs[cmd_id] = msg  # 更新最新消息缓存
            logging.debug(f"解析成功：{hex(cmd_id)}，数据：{msg}")
            # 特殊处理Event指令，写入日志
            if cmd_id == 0x0005:
                self._add_event(f"事件通知：{msg}")
            # 自动刷新CMD面板
            self.on_message(cmd_id, msg)
        except Exception as e:
            logging.error(f"解析指令{hex(cmd_id)}失败：{e}")
            self._add_event(f"解析失败：{hex(cmd_id)} - {e}")

    def on_message(self, cmd_id, msg):
        """回调：刷新CMD对应的Tkinter文本面板"""
        panel = cmd_panels.get(cmd_id)
        if panel:
            panel.config(state=tk.NORMAL)
            panel.delete("1.0", tk.END)
            panel.insert(tk.END, str(msg))
            panel.config(state=tk.DISABLED)

    def _add_event(self, msg):
        """添加事件日志，限制长度避免内存溢出"""
        global event_log
        event_log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        if len(event_log) > 100:  # 仅保留最新100条日志
            event_log.pop(0)

    def close(self):
        """关闭串口与线程，优雅退出"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.ser and self.ser.is_open:
            self.ser.close()
        logging.info("串口客户端已关闭")
        self._add_event("串口客户端已关闭")

# -----------------------------
# UDP视频接收线程：持续接收并解码视频帧
# -----------------------------
def receive_video():
    global video_frame
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', UDP_VIDEO_PORT))
        sock.settimeout(1.0)  # 增加超时，避免阻塞
        logging.info(f"UDP视频接收已启动：端口 {UDP_VIDEO_PORT}")
        client._add_event(f"UDP视频启动：端口 {UDP_VIDEO_PORT}") if client else None
        while True:
            try:
                data, _ = sock.recvfrom(65535)
                nparr = np.frombuffer(data, np.uint8)
                video_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            except socket.timeout:
                continue
            except Exception as e:
                logging.error(f"视频接收异常：{e}")
                if client:
                    client._add_event(f"视频接收异常：{e}")
                time.sleep(0.1)
    except Exception as e:
        logging.error(f"视频服务启动失败：{e}")
        if client:
            client._add_event(f"视频启动失败：{e}")

# -----------------------------
# 自定义圆角按钮：带悬停/点击效果，美化UI
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
        # 绑定鼠标事件
        self.bind("<Button-1>", self.on_click)
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def draw_button(self, color):
        """绘制圆角按钮（阴影+主体+文字）"""
        self.delete("all")
        # 绘制阴影
        self.create_rounded_rectangle(2, 2, self.width+2, self.height+2,
                                      radius=self.radius, fill="#aaaaaa", outline="", tags="shadow")
        self.lower("shadow")
        # 绘制按钮主体
        self.create_rounded_rectangle(0, 0, self.width, self.height,
                                      radius=self.radius, fill=color, outline="")
        # 绘制文字
        self.create_text(self.width//2, self.height//2, text=self.text, fill=self.fg,
                         font=('Microsoft YaHei', 11, 'bold'))

    def create_rounded_rectangle(self, x1, y1, x2, y2, radius=25, **kwargs):
        """生成圆角矩形坐标点"""
        points = [
            x1+radius, y1, x2-radius, y1, x2, y1, x2, y1+radius,
            x2, y2-radius, x2, y2, x2-radius, y2, x1+radius, y2,
            x1, y2, x1, y2-radius, x1, y1+radius, x1, y1
        ]
        return self.create_polygon(points, smooth=True, **kwargs)

    def on_click(self, event):
        """点击触发回调"""
        if self.command:
            self.command()

    def on_enter(self, event):
        """鼠标悬停切换背景"""
        self.draw_button(self.hover_bg)

    def on_leave(self, event):
        """鼠标离开恢复背景"""
        self.draw_button(self.bg)

# -----------------------------
# UI界面：完整可视化（视频+状态+血量条+日志+发送按钮）
# -----------------------------
def create_ui(recv_callback, send_callback):
    root = tk.Tk()
    root.title("RoboMaster 智能仪表盘 V1.0")
    root.geometry("1200x800")
    root.configure(bg="#f5f5f5")
    root.resizable(True, True)  # 允许窗口缩放

    # ---------- 主标题栏 ----------
    title_frame = tk.Frame(root, bg="#ffffff", height=70)
    title_frame.pack(fill='x', pady=(0, 10))
    title_frame.pack_propagate(False)  # 固定高度
    tk.Label(title_frame, text="ROBOMASTER DASHBOARD", bg="#ffffff", fg="#2c3e50",
             font=('Microsoft YaHei', 24, 'bold')).pack(pady=12)

    # ---------- 主容器：左（视频+状态）+ 右（控制+日志） ----------
    main_frame = tk.Frame(root, bg="#f5f5f5")
    main_frame.pack(fill='both', expand=True, padx=20, pady=(0, 20))

    # ---------------------- 左侧：视频流 + 核心状态展示 ----------------------
    left_frame = tk.Frame(main_frame, bg="#ffffff", bd=1, relief=tk.FLAT)
    left_frame.pack(side=tk.LEFT, fill='both', expand=True, padx=(0, 10))

    # 视频显示区域
    video_title = tk.Label(left_frame, text="实时视频流 LIVE VIDEO", bg="#ffffff", fg="#2c3e50",
                           font=('Microsoft YaHei', 12, 'bold')).pack(anchor='w', padx=15, pady=(15, 10))
    video_canvas = tk.Canvas(left_frame, width=640, height=360, bg="#1a1a1a", highlightthickness=0)
    video_canvas.pack(padx=15, pady=(0, 15))

    # 游戏状态面板
    status_frame = tk.Frame(left_frame, bg="#f8f9fa", bd=1, relief=tk.FLAT)
    status_frame.pack(fill='x', padx=15, pady=(0, 15))
    tk.Label(status_frame, text="📌 游戏核心状态", bg="#f8f9fa", fg="#2c3e50",
             font=('Microsoft YaHei', 12, 'bold')).pack(anchor='w', padx=10, pady=(8, 10))

    # 游戏阶段/回合/倒计时
    game_info_frame = tk.Frame(status_frame, bg="#f8f9fa")
    game_info_frame.pack(fill='x', padx=10, pady=(0, 8))
    stage_label = tk.Label(game_info_frame, text="阶段：未初始化", bg="#f8f9fa", fg="#e74c3c",
                           font=('Microsoft YaHei', 14, 'bold'))
    round_label = tk.Label(game_info_frame, text="回合：0/0", bg="#f8f9fa", fg="#34495e",
                           font=('Microsoft YaHei', 12))
    countdown_label = tk.Label(game_info_frame, text="倒计时：0s", bg="#f8f9fa", fg="#34495e",
                               font=('Microsoft YaHei', 12))
    stage_label.pack(side=tk.LEFT, padx=(0, 20))
    round_label.pack(side=tk.LEFT, padx=(0, 20))
    countdown_label.pack(side=tk.LEFT)

    # 双方分数
    score_label = tk.Label(status_frame, text="红方：0  |  蓝方：0", bg="#f8f9fa", fg="#e74c3c",
                           font=('Microsoft YaHei', 14, 'bold'))
    score_label.pack(anchor='w', padx=10, pady=(0, 8))

    # 机器人实时状态（血量/热量/弹药）
    robot_status_frame = tk.Frame(status_frame, bg="#f8f9fa")
    robot_status_frame.pack(fill='x', padx=10, pady=(0, 10))
    tk.Label(robot_status_frame, text="🤖 本机状态", bg="#f8f9fa", fg="#2c3e50",
             font=('Microsoft YaHei', 11, 'bold')).grid(row=0, column=0, sticky='w', columnspan=2)
    # 状态标签
    robot_health_label = tk.Label(robot_status_frame, text="血量：0", bg="#f8f9fa", fg="#27ae60", font=('Microsoft YaHei', 10))
    robot_heat_label = tk.Label(robot_status_frame, text="热量：0.0", bg="#f8f9fa", fg="#e67e22", font=('Microsoft YaHei', 10))
    robot_ammo_label = tk.Label(robot_status_frame, text="剩余弹药：0", bg="#f8f9fa", fg="#3498db", font=('Microsoft YaHei', 10))
    robot_fired_label = tk.Label(robot_status_frame, text="累计发射：0", bg="#f8f9fa", fg="#9b59b6", font=('Microsoft YaHei', 10))
    # 网格布局
    robot_health_label.grid(row=1, column=0, sticky='w', pady=2)
    robot_heat_label.grid(row=1, column=1, sticky='w', padx=20, pady=2)
    robot_ammo_label.grid(row=2, column=0, sticky='w', pady=2)
    robot_fired_label.grid(row=2, column=1, sticky='w', padx=20, pady=2)

    # 单位血量条形进度条（基地/前哨/机器人）
    health_frame = tk.Frame(left_frame, bg="#ffffff")
    health_frame.pack(fill='x', padx=15, pady=(0, 15))
    tk.Label(health_frame, text="❤️ 单位血量状态", bg="#ffffff", fg="#2c3e50",
             font=('Microsoft YaHei', 12, 'bold')).pack(anchor='w', pady=(0, 8))
    # 基地血量
    base_health_label = tk.Label(health_frame, text="基地血量：0", bg="#ffffff", fg="#27ae60", font=('Microsoft YaHei', 10))
    base_health_bar = ttk.Progressbar(health_frame, length=580, maximum=HEALTH_MAX, style="success.Horizontal.TProgressbar")
    base_health_label.pack(anchor='w', pady=2)
    base_health_bar.pack(anchor='w', pady=(0, 5))
    # 前哨站血量
    outpost_health_label = tk.Label(health_frame, text="前哨站血量：0", bg="#ffffff", fg="#27ae60", font=('Microsoft YaHei', 10))
    outpost_health_bar = ttk.Progressbar(health_frame, length=580, maximum=HEALTH_MAX, style="info.Horizontal.TProgressbar")
    outpost_health_label.pack(anchor='w', pady=2)
    outpost_health_bar.pack(anchor='w', pady=(0, 10))
    # 机器人血量（5个）
    robot_health_labels = []
    robot_health_bars = []
    for i in range(5):
        lbl = tk.Label(health_frame, text=f"机器人{i+1}血量：0", bg="#ffffff", fg="#27ae60", font=('Microsoft YaHei', 10))
        bar = ttk.Progressbar(health_frame, length=560, maximum=HEALTH_MAX)
        lbl.pack(anchor='w', pady=2)
        bar.pack(anchor='w', pady=(0, 5))
        robot_health_labels.append(lbl)
        robot_health_bars.append(bar)

    # ---------------------- 右侧：指令发送 + 事件日志 ----------------------
    right_frame = tk.Frame(main_frame, bg="#ffffff", bd=1, relief=tk.FLAT, width=350)
    right_frame.pack(side=tk.RIGHT, fill='y', padx=(10, 0))
    right_frame.pack_propagate(False)  # 固定宽度

    # 指令发送面板
    send_frame = tk.Frame(right_frame, bg="#ffffff")
    send_frame.pack(fill='x', padx=15, pady=(15, 10))
    tk.Label(send_frame, text="📤 指令发送", bg="#ffffff", fg="#2c3e50",
             font=('Microsoft YaHei', 12, 'bold')).pack(anchor='w', pady=(0, 15))
    # 常用发送按钮（RemoteControl/MapCommand）
    btn_frame = tk.Frame(send_frame, bg="#ffffff")
    btn_frame.pack(fill='x')
    # 远程控制按钮（示例：速度10，旋转0，使能True）
    ModernButton(btn_frame, text="远程控制-前进",
                 command=lambda: send_callback(0x0201, 10, 0, True),
                 bg="#27ae60", hover_bg="#219653").pack(fill='x', pady=5)
    # 地图指令按钮（示例：坐标(100,200)）
    ModernButton(btn_frame, text="地图指令-(100,200)",
                 command=lambda: send_callback(0x0202, 100, 200),
                 bg="#e67e22", hover_bg="#d35400").pack(fill='x', pady=5)
    # 停止远程控制按钮
    ModernButton(btn_frame, text="远程控制-停止",
                 command=lambda: send_callback(0x0201, 0, 0, False),
                 bg="#e74c3c", hover_bg="#c0392b").pack(fill='x', pady=5)

    # 事件日志面板
    log_frame = tk.Frame(right_frame, bg="#ffffff")
    log_frame.pack(fill='both', expand=True, padx=15, pady=(10, 0))
    tk.Label(log_frame, text="📜 事件日志", bg="#ffffff", fg="#2c3e50",
             font=('Microsoft YaHei', 12, 'bold')).pack(anchor='w', pady=(0, 8))
    # 日志列表+滚动条
    log_scroll = tk.Scrollbar(log_frame, orient=tk.VERTICAL)
    log_listbox = tk.Listbox(log_frame, yscrollcommand=log_scroll.set, bg="#f8f9fa", fg="#2c3e50",
                             font=('Microsoft YaHei', 9), selectbackground="#3498db", selectforeground="#fff")
    log_scroll.config(command=log_listbox.yview)
    log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
    log_listbox.pack(side=tk.LEFT, fill='both', expand=True)

    # ---------------------- UI定时刷新函数 ----------------------
    def refresh_ui():
        # 1. 更新游戏状态（GameStatus 0x0001）
        gs = recv_callback.latest_msgs.get(0x0001)
        if gs:
            stage_label.config(text=f"阶段：{gs.current_stage}")
            round_label.config(text=f"回合：{gs.current_round}/{gs.total_rounds}")
            countdown_label.config(text=f"倒计时：{gs.stage_countdown_sec}s")
            score_label.config(text=f"红方：{gs.red_score}  |  蓝方：{gs.blue_score}")

        # 2. 更新机器人动态状态（RobotDynamicStatus 0x0102）
        rd = recv_callback.latest_msgs.get(0x0102)
        if rd:
            robot_health_label.config(text=f"血量：{rd.current_health}")
            robot_heat_label.config(text=f"热量：{rd.current_heat:.1f}")
            robot_ammo_label.config(text=f"剩余弹药：{rd.remaining_ammo}")
            robot_fired_label.config(text=f"累计发射：{rd.total_projectiles_fired}")

        # 3. 更新单位血量进度条（GlobalUnitStatus 0x0002）
        gu = recv_callback.latest_msgs.get(0x0002)
        if gu:
            # 基地/前哨站
            base_health = gu.base_health if hasattr(gu, 'base_health') else 0
            outpost_health = gu.outpost_health if hasattr(gu, 'outpost_health') else 0
            base_health_label.config(text=f"基地血量：{base_health}")
            base_health_bar['value'] = base_health
            outpost_health_label.config(text=f"前哨站血量：{outpost_health}")
            outpost_health_bar['value'] = outpost_health
            # 机器人血量
            for i, (lbl, bar) in enumerate(zip(robot_health_labels, robot_health_bars)):
                health = gu.robot_health[i] if i < len(getattr(gu, 'robot_health', [])) else 0
                lbl.config(text=f"机器人{i+1}血量：{health}")
                bar['value'] = health

        # 4. 更新事件日志
        log_listbox.delete(0, tk.END)
        for log in event_log:
            log_listbox.insert(tk.END, log)
        log_listbox.see(tk.END)  # 自动滚动到最新日志

        # 5. 更新视频流
        global video_frame
        if video_frame is not None:
            try:
                # 格式转换：BGR(cv2) -> RGB(PIL) -> ImageTk
                img = Image.fromarray(cv2.cvtColor(video_frame, cv2.COLOR_BGR2RGB))
                img = img.resize((640, 360), Image.Resampling.LANCZOS)  # 高质量缩放
                imgtk = ImageTk.PhotoImage(image=img)
                video_canvas.delete("all")
                video_canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
                video_canvas.image = imgtk  # 保留引用，避免被GC回收
            except Exception as e:
                logging.error(f"视频刷新失败：{e}")

        # 定时刷新：Tkinter原生机制，避免递归死循环
        root.after(REFRESH_INTERVAL, refresh_ui)

    # 启动UI刷新
    refresh_ui()
    # 启动Tkinter主循环
    root.mainloop()

# -----------------------------
# 主程序入口：初始化所有模块并启动
# -----------------------------
if __name__ == "__main__":
    # 1. 初始化串口客户端
    client = StudentEngineSerialClient()

    # 2. 补丁on_message：更新最新消息缓存
    orig_on_message = client.on_message
    def patched_on_message(cmd_id, msg):
        client.latest_msgs[cmd_id] = msg
        orig_on_message(cmd_id, msg)
    client.on_message = patched_on_message

    # 3. 启动UDP视频接收线程
    threading.Thread(target=receive_video, daemon=True).start()

    # 4. 定义发送回调：封装Protobuf消息并发送
    def send_callback(cmd_id, *args):
        if cmd_id not in SEND_CMD_MAP:
            logging.warning(f"未知发送指令：{hex(cmd_id)}")
            return
        # 初始化Protobuf消息
        msg_cls = SEND_CMD_MAP[cmd_id]
        msg = msg_cls()
        # 根据指令类型填充字段（可扩展更多指令）
        if cmd_id == 0x0201 and len(args) >= 3:
            # RemoteControl：speed, rotate, enable
            msg.speed = args[0]
            msg.rotate = args[1]
            msg.enable = args[2]
        elif cmd_id == 0x0202 and len(args) >= 2:
            # MapCommand：x, y
            msg.x = args[0]
            msg.y = args[1]
        # 发送消息
        client.send(cmd_id, msg)

    # 5. 启动UI（传入接收/发送回调）
    create_ui(recv_callback=client, send_callback=send_callback)

    # 6. 程序退出时关闭串口
    client.close()