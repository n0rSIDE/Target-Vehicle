"""Windows GUI controller for the Sentinel HC-05 chassis protocol."""
from __future__ import annotations

import struct
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

FRAME_HEAD = 0xAA
FRAME_TAIL = 0x55
BAUDRATE = 115200
SEND_INTERVAL_MS = 50


def build_frame(rotate_mode, enabled, vx, wz_flag, vz, rotate_move_time_ms, rotate_move):
    """Return AA + 12-byte payload + 55, matching HC05RxCallback."""
    vx = int(vx)
    vz = int(vz)
    rotate_move_time_ms = int(rotate_move_time_ms)
    wz_flag = int(wz_flag)
    if not -70 <= vx <= 70:
        raise ValueError("vx 必须在 -70 到 70 之间")
    if not -70 <= vz <= 70:
        raise ValueError("vz 必须在 -70 到 70 之间")
    if not 0 <= rotate_move_time_ms <= 0xFFFFFFFF:
        raise ValueError("周期时间必须在 0 到 4294967295 ms 之间")
    if wz_flag not in (0, 1, 2):
        raise ValueError("普通模式旋转方向只能为 0、1 或 2")

    payload = bytearray(12)
    payload[0] = int(bool(rotate_mode))
    payload[1] = int(bool(enabled))
    payload[2:4] = struct.pack(">h", vx)
    payload[4] = wz_flag
    payload[5:7] = struct.pack(">h", vz)
    payload[7:11] = struct.pack(">I", rotate_move_time_ms)
    payload[11] = int(bool(rotate_move))
    return bytes((FRAME_HEAD,)) + bytes(payload) + bytes((FRAME_TAIL,))


class HC05Controller(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Sentinel 底盘 HC-05 控制器")
        self.geometry("740x570")
        self.minsize(700, 530)
        self.serial_port = None
        self.connected = False
        self.w_held = False
        self.s_held = False
        self.a_held = False
        self.d_held = False
        self.toggle_keys_down = set()

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="未连接")
        self.enable_var = tk.BooleanVar(value=True)
        self.rotate_mode_var = tk.BooleanVar(value=False)
        self.rotate_move_var = tk.BooleanVar(value=False)
        self.vx_var = tk.StringVar(value="30")
        self.vz_var = tk.StringVar(value="20")
        self.period_var = tk.StringVar(value="8000")
        self.wz_flag_var = tk.StringVar(value="1（停止）")
        self.vx_direction_var = tk.StringVar(value="未按键：停止")

        self.create_widgets()
        self.refresh_ports()
        self.update_mode_widgets()
        self.bind_all("<KeyPress>", self.on_key_press, add="+")
        self.bind_all("<KeyRelease>", self.on_key_release, add="+")
        self.bind_all("<FocusOut>", self.clear_motion_keys, add="+")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_widgets(self):
        root = ttk.Frame(self, padding=16)
        root.pack(fill=tk.BOTH, expand=True)

        connection = ttk.LabelFrame(root, text="串口连接", padding=10)
        connection.pack(fill=tk.X)
        ttk.Label(connection, text="端口").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(
            connection, textvariable=self.port_var, state="readonly", width=25
        )
        self.port_combo.grid(row=0, column=1, padx=8)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=(0, 12)
        )
        ttk.Label(connection, text="115200 / 8N1").grid(row=0, column=3, sticky=tk.W)
        self.connect_button = ttk.Button(
            connection, text="连接并开始发送", command=self.toggle_connection
        )
        self.connect_button.grid(row=0, column=4, padx=(18, 8))
        ttk.Label(connection, textvariable=self.status_var).grid(row=0, column=5)

        iop = ttk.LabelFrame(root, text="IOP 状态", padding=8)
        iop.pack(fill=tk.X, pady=(10, 0))
        self.iop_labels = {}
        for column, (key, caption) in enumerate(
            (("I", "底盘使能"), ("O", "小陀螺模式"), ("P", "周期移动"))
        ):
            label = tk.Label(
                iop,
                text=f"{key}  {caption}",
                width=16,
                padx=6,
                pady=5,
                relief=tk.GROOVE,
                bd=1,
            )
            label.grid(row=0, column=column, padx=(0 if column == 0 else 8, 0))
            self.iop_labels[key] = label

        mode = ttk.LabelFrame(root, text="模式", padding=10)
        mode.pack(fill=tk.X, pady=(10, 0))
        ttk.Checkbutton(
            mode,
            text="底盘使能",
            variable=self.enable_var,
            command=self.update_iop_indicators,
        ).grid(row=0, column=0, padx=(0, 20), sticky=tk.W)
        ttk.Checkbutton(
            mode,
            text="小陀螺模式",
            variable=self.rotate_mode_var,
            command=self.update_mode_widgets,
        ).grid(row=0, column=1, padx=(0, 20), sticky=tk.W)
        self.rotate_move_check = ttk.Checkbutton(
            mode,
            text="周期前后移动",
            variable=self.rotate_move_var,
            command=self.update_mode_widgets,
        )
        self.rotate_move_check.grid(row=0, column=2, sticky=tk.W)

        controls = ttk.LabelFrame(root, text="控制参数（输入范围）", padding=10)
        controls.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(controls, text="vx 速度幅值（0 ~ 70）").grid(
            row=0, column=0, sticky=tk.W, pady=4
        )
        self.vx_spin = tk.Spinbox(
            controls, from_=0, to=70, textvariable=self.vx_var, width=9, state="readonly"
        )
        self.vx_spin.grid(row=0, column=1, padx=10, sticky=tk.W)
        ttk.Label(controls, text="vz 小陀螺角速度（-70 ~ 70）").grid(
            row=1, column=0, sticky=tk.W, pady=4
        )
        self.vz_spin = tk.Spinbox(
            controls, from_=-70, to=70, textvariable=self.vz_var, width=9, state="readonly"
        )
        self.vz_spin.grid(row=1, column=1, padx=10, sticky=tk.W)
        ttk.Label(controls, text="周期时间（1 ~ 4294967295 ms）").grid(
            row=2, column=0, sticky=tk.W, pady=4
        )
        self.period_spin = tk.Spinbox(
            controls, from_=1, to=0xFFFFFFFF, textvariable=self.period_var, width=14
        )
        self.period_spin.grid(row=2, column=1, padx=10, sticky=tk.W)
        ttk.Label(controls, text="当前前进方向").grid(
            row=0, column=2, padx=(28, 8), sticky=tk.W
        )
        ttk.Label(controls, textvariable=self.vx_direction_var).grid(
            row=0, column=3, sticky=tk.W
        )
        ttk.Label(controls, text="普通模式旋转").grid(
            row=1, column=2, padx=(28, 8), sticky=tk.W
        )
        ttk.Label(controls, textvariable=self.wz_flag_var).grid(
            row=1, column=3, sticky=tk.W
        )

        guide = ttk.LabelFrame(root, text="键盘操作", padding=10)
        guide.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(
            guide,
            text=(
                "I：切换底盘使能    O：切换小陀螺模式    P：切换周期移动（需先开小陀螺）\n"
                "W：前进（+vx）    S：后退（-vx）\n"
                "A：左旋（+wz）    D：右旋（-wz）\n"
                "W/S/A/D 按住运动，松开停止；也可用鼠标点击上方开关。"
            ),
            justify=tk.LEFT,
        ).pack(anchor=tk.W)
        ttk.Label(
            root,
            text="连接后持续发送控制帧；关闭底盘使能或断开连接会触发下位机急停。",
            foreground="#805000",
        ).pack(anchor=tk.W, pady=(10, 0))

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = ()
            self.status_var.set("缺少 pyserial")
            return
        ports = list(list_ports.comports())
        values = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo["values"] = values
        if values:
            if self.port_var.get() not in values:
                self.port_var.set(values[0])
        else:
            self.port_var.set("")
            self.status_var.set("未发现串口")

    def update_mode_widgets(self):
        rotate = self.rotate_mode_var.get()
        if not rotate:
            self.rotate_move_var.set(False)
        auto_move = rotate and self.rotate_move_var.get()
        self.rotate_move_check.configure(state="normal" if rotate else "disabled")
        if rotate:
            self.a_held = False
            self.d_held = False
            self.wz_flag_var.set("小陀螺模式（由 vz 控制）")
        else:
            self.update_wz_direction()
        self.vz_spin.configure(state="readonly" if rotate else "disabled")
        self.period_spin.configure(state="normal" if auto_move else "disabled")
        self.update_iop_indicators()

    def update_iop_indicators(self):
        states = {
            "I": self.enable_var.get(),
            "O": self.rotate_mode_var.get(),
            "P": self.rotate_mode_var.get() and self.rotate_move_var.get(),
        }
        for key, active in states.items():
            self.iop_labels[key].configure(
                bg="#22C55E" if active else "#E5E7EB",
                fg="#FFFFFF" if active else "#4B5563",
            )

    def selected_port(self):
        value = self.port_var.get().strip()
        return value.split(" - ", 1)[0] if value else ""

    def update_vx_direction(self):
        if self.w_held and not self.s_held:
            self.vx_direction_var.set("W 前进（正 vx）")
        elif self.s_held and not self.w_held:
            self.vx_direction_var.set("S 后退（负 vx）")
        elif self.w_held and self.s_held:
            self.vx_direction_var.set("W+S 同按（停止）")
        else:
            self.vx_direction_var.set("未按键：停止")

    def update_wz_direction(self):
        if self.rotate_mode_var.get():
            self.wz_flag_var.set("小陀螺模式（忽略）")
        elif self.a_held and not self.d_held:
            self.wz_flag_var.set("A 正方向（2）")
        elif self.d_held and not self.a_held:
            self.wz_flag_var.set("D 负方向（0）")
        else:
            self.wz_flag_var.set("松开停止（1）")

    def on_key_press(self, event):
        key = event.keysym.lower()

        # I/O/P 是切换键，按住时只触发一次，避免键盘自动重复来回切换。
        if key in ("i", "o", "p"):
            if key in self.toggle_keys_down:
                return "break"
            self.toggle_keys_down.add(key)
            if key == "i":
                self.enable_var.set(not self.enable_var.get())
                self.update_iop_indicators()
            elif key == "o":
                self.rotate_mode_var.set(not self.rotate_mode_var.get())
                self.update_mode_widgets()
            elif self.rotate_mode_var.get():
                self.rotate_move_var.set(not self.rotate_move_var.get())
                self.update_mode_widgets()
            return "break"

        if key == "w":
            self.w_held = True
            self.update_vx_direction()
            return "break"
        if key == "s":
            self.s_held = True
            self.update_vx_direction()
            return "break"
        if key == "a" and not self.rotate_mode_var.get():
            self.a_held = True
            self.update_wz_direction()
            return "break"
        if key == "d" and not self.rotate_mode_var.get():
            self.d_held = True
            self.update_wz_direction()
            return "break"

    def on_key_release(self, event):
        key = event.keysym.lower()
        self.toggle_keys_down.discard(key)
        if key in ("i", "o", "p"):
            return "break"
        if key == "w":
            self.w_held = False
            self.update_vx_direction()
            return "break"
        if key == "s":
            self.s_held = False
            self.update_vx_direction()
            return "break"
        if key == "a":
            self.a_held = False
            self.update_wz_direction()
            return "break"
        if key == "d":
            self.d_held = False
            self.update_wz_direction()
            return "break"

    def clear_motion_keys(self, _event=None):
        self.w_held = False
        self.s_held = False
        self.a_held = False
        self.d_held = False
        self.toggle_keys_down.clear()
        self.update_vx_direction()
        self.update_wz_direction()

    def signed_vx(self):
        magnitude = int(self.vx_var.get())
        if not 0 <= magnitude <= 70:
            raise ValueError("vx 速度大小必须在 0 到 70 之间")
        if self.w_held and not self.s_held:
            return magnitude
        if self.s_held and not self.w_held:
            return -magnitude
        return 0

    def make_current_frame(self):
        rotate = self.rotate_mode_var.get()
        if rotate:
            wz_flag = 1
        elif self.a_held and not self.d_held:
            wz_flag = 2
        elif self.d_held and not self.a_held:
            wz_flag = 0
        else:
            wz_flag = 1
        auto_move = rotate and self.rotate_move_var.get()
        return build_frame(
            rotate_mode=rotate,
            enabled=self.enable_var.get(),
            vx=(int(self.vx_var.get()) if auto_move else self.signed_vx()),
            wz_flag=wz_flag,
            vz=self.vz_var.get(),
            rotate_move_time_ms=self.period_var.get() if auto_move else 0,
            rotate_move=auto_move,
        )

    def toggle_connection(self):
        if self.connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        if serial is None:
            messagebox.showerror(
                "缺少依赖", "安装依赖后再运行：python -m pip install pyserial"
            )
            return
        port = self.selected_port()
        if not port:
            messagebox.showwarning("请选择串口", "请刷新并选择 HC-05 对应的虚拟 COM 口。")
            return
        try:
            self.serial_port = serial.Serial(
                port=port, baudrate=BAUDRATE, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0
            )
            self.make_current_frame()  # 验证参数，避免建立连接后无效发送
        except Exception as exc:
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
            messagebox.showerror("连接失败或参数无效", str(exc))
            return
        self.connected = True
        self.connect_button.configure(text="停止发送并断开")
        self.status_var.set(f"已连接：{port}")
        self.send_once()

    def disconnect_serial(self):
        self.connected = False
        self.w_held = False
        self.s_held = False
        self.a_held = False
        self.d_held = False
        self.update_vx_direction()
        self.update_wz_direction()
        if self.serial_port is not None:
            try:
                # Best-effort stop frame; the firmware watchdog also stops on link timeout.
                self.serial_port.write(build_frame(False, False, 0, 1, 0, 0, False))
            except Exception:
                pass
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.connect_button.configure(text="连接并开始发送")
        self.status_var.set("未连接")

    def send_once(self):
        if not self.connected or self.serial_port is None:
            return
        try:
            frame = self.make_current_frame()
            self.serial_port.write(frame)
        except ValueError as exc:
            self.status_var.set(f"参数错误：{exc}")
        except Exception as exc:
            self.status_var.set(f"串口错误：{exc}")
            self.disconnect_serial()
            return
        if self.connected:
            self.after(SEND_INTERVAL_MS, self.send_once)

    def on_close(self):
        self.disconnect_serial()
        self.destroy()


if __name__ == "__main__":
    HC05Controller().mainloop()



