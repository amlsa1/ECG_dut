import serial  # 导入串口通信库
import serial.tools.list_ports  # 导入串口端口扫描工具
import struct  # 导入结构体解析库，用于解析二进制数据
import numpy as np  # 导入numpy，用于数值计算
import matplotlib  # 导入matplotlib绘图库
matplotlib.use("TkAgg")  # 设置matplotlib后端为TkAgg，支持Tkinter嵌入
import matplotlib.pyplot as plt  # 导入matplotlib绘图模块
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # 导入Tkinter画布接口
from matplotlib.animation import FuncAnimation  # 导入动画更新功能
import tkinter as tk  # 导入Tkinter界面库
from tkinter import ttk, messagebox  # 导入Tkinter主题控件和消息框
from collections import deque  # 导入双端队列，高效存储固定长度数据
import time  # 导入时间库

from matplotlib import rcParams  # 导入rcParams配置
rcParams['font.sans-serif'] = ['Microsoft YaHei']  # 设置默认字体为微软雅黑，支持中文
rcParams['axes.unicode_minus'] = False  # 解决坐标轴负号显示问题

# 常量定义
CES_CMDIF_PKT_START_1 = 0x0A
CES_CMDIF_PKT_START_2 = 0xFA
CES_CMDIF_TYPE_DATA = 0x02
CES_CMDIF_PKT_STOP = 0x0B

class ECGRespirationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("心电与呼吸监控系统")
        self.root.state('zoomed')  # 启动时窗口最大化
        self.root.resizable(True, True)  # 允许调整大小

        # 初始化串口选择变量，默认波特率57600
        self.selected_port = tk.StringVar()
        self.selected_baudrate = tk.IntVar(value=57600)

        # 采样参数
        self.sampling_rate = 125
        self.sample_count = 0

        # 数据缓存
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.ecg_data = deque(maxlen=self.max_points)
        self.resp_data = deque(maxlen=self.max_points)
        self.hr_data = deque(maxlen=self.max_points)
        self.rr_data = deque(maxlen=self.max_points)

        # 计时相关
        self.is_timing = False
        self.timer_start_time = 0
        self.timed_hr = []
        self.timed_rr = []
        self.avg_hr = 0
        self.avg_rr = 0

        # 呼吸率算法变量（保持你原有）
        self.MinThreshold = 0x7FFF
        self.MaxThreshold = -0x8000
        self.MinThresholdNew = 0x7FFF
        self.MaxThresholdNew = -0x8000
        self.AvgThreshold = 0
        self.startCalc = False
        self.PtiveEdgeDetected = False
        self.NtiveEdgeDetected = False
        self.peakCount = 0
        self.PeakCount = [0] * 8
        self.skipCount = 0
        self.SampleCount = 0
        self.SampleCountNtve = 0
        self.TimeCnt = 0
        self.PtiveCnt = 0
        self.NtiveCnt = 0
        self.PrevSample = 0
        self.PrevPrevSample = 0
        self.PrevPrevPrevSample = 0
        self.respiration_rate = 0

        self.serial_port = None

        self.build_gui()
        self.ani = None

        self.refresh_ports()

    def build_gui(self):
        # 主体布局：控制栏+图形显示区
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(side=tk.TOP, fill=tk.X, pady=5)

        # 用grid布局两行，控件间距用padx pady控制间隙
        # 第一行：串口、刷新、波特率
        tk.Label(self.control_frame, text="串口:", font=("宋体", 12)).grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.port_combo = ttk.Combobox(self.control_frame, width=15, textvariable=self.selected_port, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky='w')

        self.refresh_btn = tk.Button(self.control_frame, text="刷新串口", command=self.refresh_ports, width=10)
        self.refresh_btn.grid(row=0, column=2, padx=5, pady=5)

        tk.Label(self.control_frame, text="波特率:", font=("宋体", 12)).grid(row=0, column=3, padx=5, pady=5, sticky='e')
        baud_options = [9600, 19200, 38400, 57600, 115200]
        self.baud_combo = ttk.Combobox(self.control_frame, width=10, textvariable=self.selected_baudrate, state="readonly")
        self.baud_combo['values'] = baud_options
        self.baud_combo.grid(row=0, column=4, padx=5, pady=5, sticky='w')
        self.baud_combo.current(3)  # 默认57600

        # 第二行：连接、断开、计时按钮
        self.connect_btn = tk.Button(self.control_frame, text="连接串口", command=self.connect_serial, width=12)
        self.connect_btn.grid(row=1, column=0, padx=5, pady=5)

        self.disconnect_btn = tk.Button(self.control_frame, text="断开串口", command=self.disconnect_serial, state=tk.DISABLED, width=12)
        self.disconnect_btn.grid(row=1, column=1, padx=5, pady=5)

        self.timer_btn = tk.Button(self.control_frame, text="开始1分钟计时", command=self.toggle_timer, state=tk.DISABLED, width=15)
        self.timer_btn.grid(row=1, column=2, padx=5, pady=5)

        # 填充空白列，让控件靠左，右边留白
        self.control_frame.grid_columnconfigure(5, weight=1)

        # 左侧主图frame
        self.left_frame = tk.Frame(self.root)
        self.left_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 创建matplotlib Figure和Axes
        self.fig = plt.Figure(figsize=(9, 6), dpi=100)
        self.ax_ecg = self.fig.add_subplot(211)
        self.ax_resp = self.fig.add_subplot(212)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.left_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 图表设置
        self.ax_ecg.set_title("心电波形（ECG）")
        self.ax_ecg.set_ylabel("信号幅值")
        self.ax_ecg.grid(True)

        self.ax_resp.set_title("呼吸波形")
        self.ax_resp.set_xlabel("时间 (秒)")
        self.ax_resp.set_ylabel("信号幅值")
        self.ax_resp.grid(True)

        self.line_ecg, = self.ax_ecg.plot([], [], 'b-')
        self.line_resp, = self.ax_resp.plot([], [], 'g-')

        self.hr_text = self.ax_ecg.text(0.02, 0.9, '', transform=self.ax_ecg.transAxes, color='red', fontsize=12)
        self.rr_text = self.ax_resp.text(0.02, 0.9, '', transform=self.ax_resp.transAxes, color='red', fontsize=12)
        self.avg_text = self.ax_ecg.text(0.02, 0.8, '', transform=self.ax_ecg.transAxes, color='darkred', fontsize=12)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if not ports:
            ports = ['无可用串口']
        self.port_combo['values'] = ports
        self.selected_port.set(ports[0])

    def connect_serial(self):
        port = self.selected_port.get()
        baud = self.selected_baudrate.get()
        if port == '无可用串口':
            messagebox.showerror("错误", "无可用串口")
            return
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.5)
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.timer_btn.config(state=tk.NORMAL)
            print(f"已连接串口 {port}，波特率 {baud}")
        except Exception as e:
            messagebox.showerror("错误", f"无法打开串口: {e}")

    def disconnect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.timer_btn.config(state=tk.DISABLED)
            print("串口已断开")

    def toggle_timer(self):
        if not self.is_timing:
            self.is_timing = True
            self.timer_start_time = time.time()
            self.timed_hr.clear()
            self.timed_rr.clear()
            self.timer_btn.config(text="计时中...")
        else:
            self.is_timing = False
            self.timer_btn.config(text="开始1分钟计时")

    def parse_packet(self, packet):
        if len(packet) != 9:
            return None, None, None, None
        ecg = struct.unpack('<h', packet[0:2])[0]
        resp_wave = struct.unpack('<h', packet[2:4])[0]
        resp_rate = struct.unpack('<H', packet[4:6])[0]
        heart_rate = struct.unpack('<H', packet[6:8])[0]
        return ecg, resp_wave, resp_rate, heart_rate

    def read_serial(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        while self.serial_port.in_waiting > 0:
            try:
                byte1 = ord(self.serial_port.read(1))
                if byte1 != CES_CMDIF_PKT_START_1:
                    continue
                byte2 = ord(self.serial_port.read(1))
                if byte2 != CES_CMDIF_PKT_START_2:
                    continue
                data_len = ord(self.serial_port.read(1))
                _zero1 = ord(self.serial_port.read(1))
                data_type = ord(self.serial_port.read(1))

                if data_type == CES_CMDIF_TYPE_DATA and data_len == 9:
                    packet = self.serial_port.read(9)
                    _zero2 = ord(self.serial_port.read(1))
                    stop = ord(self.serial_port.read(1))

                    if stop == CES_CMDIF_PKT_STOP:
                        ecg, resp_wave, resp_rate, heart_rate = self.parse_packet(packet)
                        if ecg is not None:
                            t = self.sample_count / self.sampling_rate
                            self.time_data.append(t)
                            self.ecg_data.append(ecg)
                            self.resp_data.append(resp_wave)
                            self.hr_data.append(heart_rate)
                            calculated_rr = self.process_respiration_wave(resp_wave)
                            self.rr_data.append(calculated_rr)

                            if self.is_timing:
                                self.timed_hr.append(heart_rate)
                                self.timed_rr.append(calculated_rr)

                            self.sample_count += 1
            except Exception as e:
                print("串口读取异常:", e)
                break

    def respiration_rate_detection(self, Resp_wave):

        self.SampleCount += 1
        self.SampleCountNtve += 1
        self.TimeCnt += 1

        if Resp_wave < self.MinThresholdNew:
            self.MinThresholdNew = Resp_wave
        if Resp_wave > self.MaxThresholdNew:
            self.MaxThresholdNew = Resp_wave

        if self.SampleCount > 1000:
            self.SampleCount = 0
        if self.SampleCountNtve > 1000:
            self.SampleCountNtve = 0

        if self.startCalc:
            if self.TimeCnt >= 500:
                self.TimeCnt = 0
                if (self.MaxThresholdNew - self.MinThresholdNew) > 400:
                    self.MaxThreshold = self.MaxThresholdNew
                    self.MinThreshold = self.MinThresholdNew
                    self.AvgThreshold = (self.MaxThreshold + self.MinThreshold) >> 1
                else:
                    self.startCalc = False
                    self.respiration_rate = 0

            self.PrevPrevPrevSample = self.PrevPrevSample
            self.PrevPrevSample = self.PrevSample
            self.PrevSample = Resp_wave

            if self.skipCount == 0:
                if self.PrevPrevPrevSample < self.AvgThreshold and Resp_wave > self.AvgThreshold:
                    if 40 < self.SampleCount < 700:
                        self.PtiveEdgeDetected = True
                        self.PtiveCnt = self.SampleCount
                        self.skipCount = 4
                    self.SampleCount = 0

                if self.PrevPrevPrevSample < self.AvgThreshold and Resp_wave > self.AvgThreshold:
                    if 40 < self.SampleCountNtve < 700:
                        self.NtiveEdgeDetected = True
                        self.NtiveCnt = self.SampleCountNtve
                        self.skipCount = 4
                    self.SampleCountNtve = 0

                if self.PtiveEdgeDetected and self.NtiveEdgeDetected:
                    self.PtiveEdgeDetected = False
                    self.NtiveEdgeDetected = False
                    if abs(self.PtiveCnt - self.NtiveCnt) < 5:
                        self.PeakCount[self.peakCount] = self.PtiveCnt
                        self.peakCount += 1
                        self.PeakCount[self.peakCount] = self.NtiveCnt
                        self.peakCount += 1

                        if self.peakCount == 8:
                            self.peakCount = 0
                            avg_peak = sum(self.PeakCount) >> 3
                            self.respiration_rate = int(6000 / avg_peak)
            else:
                self.skipCount -= 1
        else:
            self.TimeCnt += 1
            if self.TimeCnt >= 500:
                self.TimeCnt = 0
                if (self.MaxThresholdNew - self.MinThresholdNew) > 400:
                    self.startCalc = True
                    self.MaxThreshold = self.MaxThresholdNew
                    self.MinThreshold = self.MinThresholdNew
                    self.AvgThreshold = (self.MaxThreshold + self.MinThreshold) >> 1
                    self.PrevPrevPrevSample = Resp_wave
                    self.PrevPrevSample = Resp_wave
                    self.PrevSample = Resp_wave

    def process_respiration_wave(self, resp_wave):
        self.respiration_rate_detection(resp_wave)
        return self.respiration_rate

    def update_plot(self, frame):
        self.read_serial()

        if len(self.time_data) > 0:
            self.line_ecg.set_data(self.time_data, self.ecg_data)
            self.ax_ecg.relim()
            self.ax_ecg.autoscale_view()

            self.line_resp.set_data(self.time_data, self.resp_data)
            self.ax_resp.relim()
            self.ax_resp.autoscale_view()

            current_hr = self.hr_data[-1] if self.hr_data else 0
            current_rr = self.rr_data[-1] if self.rr_data else 0

            self.hr_text.set_text(f'心率: {current_hr} 次/分')
            self.rr_text.set_text(f'呼吸率: {current_rr} 次/分')

            if self.is_timing:
                elapsed = time.time() - self.timer_start_time
                remaining = max(0, 60 - elapsed)
                self.timer_btn.config(text=f"计时中...剩余 {int(remaining)} 秒")

                if elapsed >= 60:
                    self.is_timing = False
                    self.timer_btn.config(text="开始1分钟计时")
                    self.avg_hr = int(sum(self.timed_hr) / len(self.timed_hr)) if self.timed_hr else 0
                    self.avg_rr = int(sum(self.timed_rr) / len(self.timed_rr)) if self.timed_rr else 0
                    self.avg_text.set_text(f'1分钟平均心率: {self.avg_hr} 次/分\n1分钟平均呼吸率: {self.avg_rr} 次/分')

        return self.line_ecg, self.line_resp

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=40, blit=False)
        self.root.mainloop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = ECGRespirationApp(root)
    app.run()
