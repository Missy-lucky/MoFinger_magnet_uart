"""Mosense MLX90393 live trend monitor.

Renders three magnetometers plus four derived difference channels.  To stay
responsive at 921600 baud the UI decouples data ingestion from drawing:
``append`` only stores samples, while a fixed-rate ``render`` timer redraws
each dirty visible chart at most once per cycle.

Storage and viewport are decoupled: every channel keeps a long history
(``MAX_HISTORY`` samples) while a shared *view window* selects which slice is
drawn.  In *follow* mode the window tracks the latest samples; when paused (via
the button or by dragging a chart) the window stays put so earlier trends can
be inspected while collection continues in the background.  Drawing cost is
kept flat by capping the min/max decimation to ``MAX_COLS`` columns.
"""
from __future__ import annotations

import queue, threading, time, tkinter as tk
from tkinter import messagebox, ttk
import serial
from serial.tools import list_ports
from mlx90393_collector import SENSOR_TYPE_NAMES, read_frames

BAUD_RATE = 921600
MAX_HISTORY = 600_000   # samples kept per channel (~20 min at 500 Hz)
MAX_COLS = 360          # decimation cap: max min/max buckets per redraw
AXIS_COLOR = {"x": "#d1495b", "y": "#2d6a4f", "z": "#168aad"}
SENSORS = (("mlx0", "U4"), ("mlx1", "U8"), ("mlx2", "U6"))
EMPTY_STREAM_LABEL = "等待传感器数据"
STREAM_STALE_SECONDS = 1.0
# window-size choices: label -> span in samples (None = whole history)
WINDOWS = (("600", 600), ("1200", 1200), ("2400", 2400), ("6000", 6000), ("全部", None))
# (group, title, data-key, color)
CHART_SPECS = [(g, f"{g} {ax.upper()} ({lbl})", f"{g}_{ax}", AXIS_COLOR[ax])
               for g, lbl in SENSORS for ax in "xyz"] + [
    ("diff", "Bz'  (mlx0_z - mlx1_z)", "bz_diff", "#7b2cbf"),
    ("diff", "Bx'  (mlx0_x - mlx1_x)", "bx_diff", "#f77f00"),
    ("diff", "By'  (mlx0_y - mlx1_y)", "by_diff", "#4361ee"),
    ("diff", "Bz'' (mlx0_z - mlx2_z)", "bz2_diff", "#bc4749")]


class Chart(tk.Frame):
    def __init__(self, master, group, title, key, color, on_focus, on_pan):
        super().__init__(master, bd=1, relief="solid", bg="white")
        self.group, self.title, self.key, self.color = group, title, key, color
        self.data = []                     # full history: list of (tick_ms, value)
        self.data_dirty = self.hover_dirty = self.visible = False
        self.hover_x = None; self._geom = None; self._arr = []
        head = tk.Frame(self, bg="white"); head.pack(fill="x")
        tk.Label(head, text=title, bg="white", fg=color, font=("Segoe UI", 10, "bold")).pack(side="left", padx=6)
        self.value = tk.StringVar(value="-"); tk.Label(head, textvariable=self.value, bg="white", fg="#374151").pack(side="left", padx=6)
        self.focus_btn = ttk.Button(head, text="专注", width=6, command=lambda: on_focus(self)); self.focus_btn.pack(side="right", padx=4, pady=2)
        self.canvas = tk.Canvas(self, height=120, bg="white", highlightthickness=0, cursor="fleur"); self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self._flag())
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)
        self.canvas.bind("<ButtonPress-1>", lambda e: on_pan("press", self, e))
        self.canvas.bind("<B1-Motion>", lambda e: on_pan("move", self, e))
        self.canvas.bind("<ButtonRelease-1>", lambda e: on_pan("release", self, e))

    def _flag(self): self.data_dirty = True

    def clear(self): self.data = []; self.value.set("-"); self.hover_x = None; self.data_dirty = True

    def append(self, t, v): self.data.append((t, v))

    def _on_motion(self, e): self.hover_x = e.x; self.hover_dirty = True

    def _on_leave(self, _e):
        if self.hover_x is not None: self.hover_x = None; self.hover_dirty = True

    def redraw(self, start, end):
        """Full redraw of the waveform layer for the sample window [start,end).

        A single pass over the window computes auto-range and the min/max
        envelope together; pixel mapping only touches the <=2*MAX_COLS envelope
        points, so cost stays flat regardless of how long collection has run."""
        c = self.canvas; c.delete("wave")
        w = max(c.winfo_width(), 80); h = max(c.winfo_height(), 50)
        arr = self._arr = self.data[start:end]
        n = len(arr)
        if self.data: last = self.data[-1]; self.value.set(f"t={last[0]} ms  y={last[1]:g}")
        if n < 2: self._geom = None; return
        left, right, top, bottom = 56, w - 10, 10, h - 22
        cols = max(1, min(int(right - left), MAX_COLS))
        step = n / cols if n > cols * 2 else 0
        env = []  # sample indices (within arr) to plot, in time order
        lo = hi = arr[0][1]
        if step:
            b = 0; s = 0
            while b < cols:
                e = min(n, max(s + 1, int((b + 1) * step)))
                lo_i = hi_i = s; lo_v = hi_v = arr[s][1]
                for k in range(s + 1, e):
                    v = arr[k][1]
                    if v < lo_v: lo_v, lo_i = v, k
                    elif v > hi_v: hi_v, hi_i = v, k
                if lo_v < lo: lo = lo_v
                if hi_v > hi: hi = hi_v
                env += (lo_i, hi_i) if lo_i <= hi_i else (hi_i, lo_i)
                b += 1; s = e
        else:
            env = list(range(n))
            for _, v in arr:
                if v < lo: lo = v
                elif v > hi: hi = v
        span = max(1.0, hi - lo)
        self._geom = (lo, hi, span, left, right, top, bottom, n)
        sx = (right - left) / max(1, n - 1); sy = (bottom - top) / span
        for k in range(6):  # horizontal grid + Y ticks
            gy = top + (bottom - top) * k / 5
            c.create_line(left, gy, right, gy, fill="#eef1f5", tags="wave")
            c.create_text(left - 4, gy, anchor="e", text=f"{hi - span * k / 5:.2f}", fill="#68707d", font=("Segoe UI", 8), tags="wave")
        for k in range(7):  # vertical grid + X (tick_ms) ticks
            gx = left + (right - left) * k / 6
            c.create_line(gx, top, gx, bottom, fill="#eef1f5", tags="wave")
            idx = min(n - 1, int((n - 1) * k / 6))
            c.create_text(gx, bottom + 4, anchor="n", text=f"{arr[idx][0]}", fill="#68707d", font=("Segoe UI", 8), tags="wave")
        c.create_line(left, top, left, bottom, fill="#d8dce2", tags="wave"); c.create_line(left, bottom, right, bottom, fill="#d8dce2", tags="wave")
        pts = []
        for i in env: pts += (left + i * sx, top + (hi - arr[i][1]) * sy)
        c.create_line(*pts, fill=self.color, width=2, tags="wave")
        self.draw_hover()

    def draw_hover(self):
        """Redraw only the hover overlay; leaves the waveform layer untouched."""
        c = self.canvas; c.delete("hover")
        if self.hover_x is None or self._geom is None: return
        _, hi, span, left, right, top, bottom, n = self._geom
        mx = min(max(self.hover_x, left), right)
        i = min(max(round((mx - left) * (n - 1) / max(1, right - left)), 0), n - 1)
        t, v = self._arr[i]
        x = left + i * (right - left) / max(1, n - 1); y = top + (hi - v) * (bottom - top) / span
        c.create_line(x, top, x, bottom, fill="#9aa3af", dash=(3, 3), tags="hover")
        c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=self.color, outline="white", tags="hover")
        txt = f"t={t} ms   y={v:g}"; tw = len(txt) * 7 + 12
        bx = max(left, min(x + 8, right - tw))
        c.create_rectangle(bx, top + 2, bx + tw, top + 22, fill="#111827", outline="", tags="hover")
        c.create_text(bx + 6, top + 12, anchor="w", text=txt, fill="white", font=("Segoe UI", 9), tags="hover")

    def plot_width(self):
        return max(1, self.canvas.winfo_width() - 10 - 56)


class App:
    def __init__(self, root):
        self.root = root; root.title("灵巧手 MLX90393 串联数据监视器"); root.geometry("1180x900"); root.minsize(920, 640)
        self.serial = None; self.stop = threading.Event(); self.rows = queue.Queue()
        self.count = 0; self.latest = None; self.charts = []; self.focused = None
        self.stream_histories = {}; self.stream_meta = {}; self.stream_counts = {}; self.stream_last_seen = {}; self.stream_order = []
        self.selected_stream = None
        self.port = tk.StringVar(); self.status = tk.StringVar(value="未连接")
        self.stats = tk.StringVar(value="总帧数 0 | 数据来源 0")
        self.stream_var = tk.StringVar(value=EMPTY_STREAM_LABEL)
        self.auto_source = tk.BooleanVar(value=True)
        self.sensor = tk.StringVar(value="mlx0")
        # view state
        self.follow = True; self.span = 2400; self.view_start = 0
        self.view_status = tk.StringVar(value="● 实时")
        self._drag = None; self._syncing = False; self._pan_active = False
        self.view_mode = "single"        # "single" | "all"
        self.diff_var = tk.BooleanVar(value=True)
        self._ui(); self.refresh_ports(); self._relayout(); self._sync_follow_ui()
        root.after(50, self.poll); root.after(75, self.render)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def _ui(self):
        outer = ttk.Frame(self.root, padding=10); outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="灵巧手 MLX90393 串联数据监视器", font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(
            outer,
            text="接收所有合法固件帧，按传感器类型和传感器id分类：0x01 指尖磁铁、0x02 指腹磁铁、0x03 指尖线圈、0x04 指腹线圈。",
            foreground="#4b5563",
            wraplength=980,
        ).pack(anchor="w", pady=(2, 0))
        bar = ttk.Frame(outer); bar.pack(fill="x", pady=8)
        ttk.Label(bar, text="串口").pack(side="left")
        self.combo = ttk.Combobox(bar, textvariable=self.port, state="readonly", width=28); self.combo.pack(side="left", padx=5)
        ttk.Button(bar, text="刷新串口", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="连接", command=self.toggle); self.connect_btn.pack(side="left", padx=5)
        ttk.Button(bar, text="一键刷新数据", command=self.reset).pack(side="left", padx=5)
        ttk.Label(bar, textvariable=self.status).pack(side="left", padx=10)
        sel = ttk.Frame(outer); sel.pack(fill="x", pady=(0, 4))
        ttk.Label(sel, text="数据来源:").pack(side="left")
        self.stream_box = ttk.Combobox(sel, textvariable=self.stream_var, state="readonly", width=44,
                                       values=[EMPTY_STREAM_LABEL])
        self.stream_box.current(0)
        self.stream_box.pack(side="left", padx=5)
        self.stream_box.bind("<<ComboboxSelected>>", lambda _e: self._pick_stream(self.stream_box.current()))
        ttk.Checkbutton(sel, text="跟随活跃来源", variable=self.auto_source).pack(side="left", padx=6)
        sel = ttk.Frame(outer); sel.pack(fill="x", pady=(0, 4))
        ttk.Label(sel, text="显示模式:").pack(side="left")
        self.mode_box = ttk.Combobox(sel, state="readonly", width=18,
                                      values=["单颗 MLX XYZ", "全部 MLX 9 通道"]); self.mode_box.current(0)
        self.mode_box.pack(side="left", padx=5)
        self.mode_box.bind("<<ComboboxSelected>>", lambda _e: self._pick_mode(self.mode_box.current()))
        ttk.Label(sel, text="显示MLX:").pack(side="left", padx=(10, 0))
        self.sensor_box = ttk.Combobox(sel, state="readonly", width=14, values=[f"{g} ({lbl})" for g, lbl in SENSORS]); self.sensor_box.current(0)
        self.sensor_box.pack(side="left", padx=5); self.sensor_box.bind("<<ComboboxSelected>>", lambda _e: self._pick_sensor(self.sensor_box.current()))
        self.diff_chk = ttk.Checkbutton(sel, text="附带差分 Bx'/By'/Bz'/Bz''", variable=self.diff_var, command=self.mark_relayout)
        self.diff_chk.pack(side="left", padx=6)
        # playback / history navigation bar
        nav = ttk.Frame(outer); nav.pack(fill="x", pady=(2, 4))
        self.pause_btn = ttk.Button(nav, text="暂停", width=6, command=self.pause_toggle); self.pause_btn.pack(side="left")
        ttk.Button(nav, text="回到最新", width=9, command=lambda: self.set_follow(True)).pack(side="left", padx=4)
        ttk.Label(nav, text="窗口:").pack(side="left", padx=(8, 2))
        self.win_box = ttk.Combobox(nav, state="readonly", width=8, values=[lbl for lbl, _ in WINDOWS]); self.win_box.current(2)
        self.win_box.pack(side="left"); self.win_box.bind("<<ComboboxSelected>>", lambda _e: self._pick_window(self.win_box.current()))
        ttk.Label(nav, textvariable=self.view_status, foreground="#2d6a4f", font=("Segoe UI", 9, "bold")).pack(side="left", padx=10)
        self.pan = ttk.Scale(nav, from_=0, to=1000, command=self.on_pan); self.pan.pack(side="left", fill="x", expand=True, padx=8)
        self.pan.bind("<ButtonPress-1>", lambda e: setattr(self, "_pan_active", True))
        self.pan.bind("<ButtonRelease-1>", lambda e: setattr(self, "_pan_active", False))
        ttk.Label(outer, textvariable=self.stats).pack(anchor="w")
        # fixed resizable grid (no scrolling): charts stretch to fill the window
        self.charea = ttk.Frame(outer); self.charea.pack(fill="both", expand=True, pady=(6, 0))
        self.charea.columnconfigure(0, weight=1); self.charea.columnconfigure(1, weight=1)
        for group, title, key, color in CHART_SPECS:
            self.charts.append(Chart(self.charea, group, title, key, color, self.focus_toggle, self.on_drag))

    # ---- stream classification ----------------------------------------------
    def _stream_key(self, row):
        return int(row.get("sensor_type", 0)), int(row.get("sensor_id", 0))

    def _stream_label(self, key):
        sensor_type, sensor_id = key
        meta = self.stream_meta.get(key, {})
        name = SENSOR_TYPE_NAMES.get(sensor_type) or meta.get("sensor_type_name") or f"未知类型0x{sensor_type:02X}"
        domain = meta.get("sensor_domain", "?")
        if isinstance(domain, int):
            domain = f"0x{domain:02X}"
        return f"0x{sensor_type:02X} {name} / id 0x{sensor_id:02X} / 传感器域 {domain}"

    def _empty_stream_history(self):
        return {key: [] for _, _, key, _ in CHART_SPECS}

    def _refresh_stream_box(self):
        values = [self._stream_label(key) for key in self.stream_order] or [EMPTY_STREAM_LABEL]
        self.stream_box.configure(values=values)
        if self.selected_stream in self.stream_order:
            index = self.stream_order.index(self.selected_stream)
            self.stream_box.current(index)
            self.stream_var.set(values[index])
        else:
            self.stream_box.current(0)
            self.stream_var.set(values[0])

    def _attach_stream(self, key, reset_view=True):
        self.selected_stream = key
        history = self.stream_histories[key]
        for chart in self.charts:
            chart.data = history[chart.key]
            chart.value.set("-")
            chart.hover_x = None
            chart.data_dirty = True
        self.focused = None
        if reset_view:
            self.view_start = 0
            self.set_follow(True)
        else:
            self.mark_dirty()
        self._refresh_stream_box()
        self._relayout()

    def _pick_stream(self, index):
        if 0 <= index < len(self.stream_order):
            self._attach_stream(self.stream_order[index])

    def _selected_stream_stale(self, now):
        if self.selected_stream is None:
            return True
        last_seen = self.stream_last_seen.get(self.selected_stream)
        return last_seen is None or now - last_seen > STREAM_STALE_SECONDS

    def _register_stream(self, row, now):
        key = self._stream_key(row)
        is_new = key not in self.stream_histories
        if is_new:
            self.stream_histories[key] = self._empty_stream_history()
            self.stream_counts[key] = 0
            self.stream_order.append(key)
        self.stream_last_seen[key] = now
        self.stream_meta[key] = {
            "sensor_type": key[0],
            "sensor_id": key[1],
            "sensor_domain": row.get("sensor_domain", "?"),
            "sensor_type_name": SENSOR_TYPE_NAMES.get(key[0]) or row.get("sensor_type_name") or f"未知类型0x{key[0]:02X}",
        }
        if is_new:
            self._refresh_stream_box()
        return key

    def _row_values(self, row):
        vals = {f"{g}_{ax}": row[f"{g}_{ax}"] for g, _ in SENSORS for ax in "xyz"}
        vals.update({"bz_diff": row["mlx0_z"] - row["mlx1_z"], "bx_diff": row["mlx0_x"] - row["mlx1_x"],
                     "by_diff": row["mlx0_y"] - row["mlx1_y"], "bz2_diff": row["mlx0_z"] - row["mlx2_z"]})
        return vals

    def _update_stats(self, last=None):
        selected_count = self.stream_counts.get(self.selected_stream, 0)
        selected_label = self._stream_label(self.selected_stream) if self.selected_stream else "未选择"
        text = f"总帧数 {self.count} | 数据来源 {len(self.stream_order)} | 当前 {selected_count} 帧 | {selected_label}"
        if self.selected_stream and self._selected_stream_stale(time.monotonic()):
            text += " | 当前来源未上报"
        if last is not None:
            text += f" | 最近 tick {last['tick_ms']} ms | GAIN_SEL {last['gain_sel']}"
        self.stats.set(text)

    # ---- view-window helpers -------------------------------------------------
    def total(self): return len(self.charts[0].data) if self.charts else 0

    def cur_span(self, total):
        return max(1, total) if self.span is None else max(1, min(self.span, total))

    def window(self):
        total = self.total()
        if total == 0: return 0, 0
        span = self.cur_span(total)
        maxstart = max(0, total - span)
        start = maxstart if self.follow else min(max(self.view_start, 0), maxstart)
        return start, start + span

    def mark_dirty(self):
        for c in self.charts:
            if c.visible: c.data_dirty = True

    def set_follow(self, follow):
        self.follow = follow
        if follow: self.view_start = max(0, self.total() - self.cur_span(self.total()))
        self._sync_follow_ui(); self.mark_dirty()

    def pause_toggle(self): self.set_follow(not self.follow)

    def _sync_follow_ui(self):
        self.pause_btn.configure(text="暂停" if self.follow else "继续")
        self.view_status.set("● 实时" if self.follow else "❚❚ 已暂停 · 拖动图表或滚动条回看")

    def _pick_window(self, index):
        prev_start = self.window()[0]
        self.span = WINDOWS[index][1]
        if not self.follow: self.view_start = prev_start  # keep left edge anchored
        self.mark_dirty()

    def on_pan(self, val):
        if self._syncing: return
        total = self.total(); span = self.cur_span(total); maxstart = max(0, total - span)
        if self.follow: self.set_follow(False)
        self.view_start = int(float(val) / 1000 * maxstart)
        self.mark_dirty()

    def on_drag(self, phase, chart, e):
        if phase == "press":
            self._drag = (e.x, self.window()[0])
            if self.follow: self.set_follow(False)
        elif phase == "move" and self._drag:
            x0, start0 = self._drag
            total = self.total(); span = self.cur_span(total); maxstart = max(0, total - span)
            delta = -(e.x - x0) * span / chart.plot_width()
            self.view_start = min(max(int(round(start0 + delta)), 0), maxstart)
            self.mark_dirty()
        elif phase == "release":
            self._drag = None

    def _sync_pan(self):
        if self._pan_active: return  # user is dragging the slider; don't fight it
        total = self.total(); span = self.cur_span(total); maxstart = max(0, total - span)
        start = self.window()[0]
        frac = 1.0 if (self.follow or maxstart == 0) else start / maxstart
        self._syncing = True; self.pan.set(frac * 1000); self._syncing = False

    # ---- layout --------------------------------------------------------------
    def _pick_sensor(self, index): self.sensor.set(SENSORS[index][0]); self.focused = None; self._relayout()

    def _pick_mode(self, index):
        self.view_mode = "all" if index == 1 else "single"
        self.focused = None
        self.sensor_box.configure(state="disabled" if self.view_mode == "all" else "readonly")
        self._relayout()

    def mark_relayout(self): self.focused = None; self._relayout()

    def _cols(self, n):
        for col in range(6): self.charea.columnconfigure(col, weight=1 if col < n else 0)

    def _clear_grid(self):
        for c in self.charts: c.grid_forget(); c.visible = False
        for r in range(8): self.charea.rowconfigure(r, weight=0)

    def _place(self, chart, r, col, span=1):
        chart.grid(row=r, column=col, columnspan=span, sticky="nsew", padx=3, pady=3)
        self.charea.rowconfigure(r, weight=1)
        chart.visible = True; chart.data_dirty = True
        chart.focus_btn.configure(text="专注")

    def _relayout(self):
        self._clear_grid()
        if self.focused:
            self._cols(2)
            self.focused.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=3, pady=3)
            self.charea.rowconfigure(0, weight=1)
            self.focused.visible = True; self.focused.data_dirty = True
            self.focused.focus_btn.configure(text="还原")
            return
        diffs = [c for c in self.charts if c.group == "diff"]
        if self.view_mode == "all":
            # 3 columns (mlx0/1/2) x 3 rows (X/Y/Z): each row is one axis across sensors
            self._cols(3)
            axis_row = {"x": 0, "y": 1, "z": 2}
            sensor_col = {g: i for i, (g, _) in enumerate(SENSORS)}
            for c in self.charts:
                if c.group in sensor_col:
                    self._place(c, axis_row[c.key.split("_")[1]], sensor_col[c.group])
            if self.diff_var.get():
                for i, c in enumerate(diffs): self._place(c, 3 + i // 3, i % 3)
        else:
            self._cols(2)
            shown = [c for c in self.charts if c.group == self.sensor.get()]
            if self.diff_var.get(): shown += diffs
            for i, c in enumerate(shown): self._place(c, i // 2, i % 2)

    def focus_toggle(self, chart): self.focused = None if self.focused is chart else chart; self._relayout()

    def refresh_ports(self):
        ps = sorted(list_ports.comports(), key=lambda p: p.device)
        self.combo["values"] = [f"{p.device} - {p.description}" for p in ps]
        if ps: self.combo.current(0)

    def toggle(self): self.disconnect() if self.serial else self.connect()

    def connect(self):
        if not self.port.get(): return messagebox.showwarning("串口", "未选择串口")
        try:
            self.serial = serial.Serial(self.port.get().split(" - ", 1)[0], BAUD_RATE, timeout=.2)
        except serial.SerialException as e:
            return messagebox.showerror("连接失败", str(e))
        self.stop.clear(); self.connect_btn.configure(text="断开"); self.status.set("已连接")
        threading.Thread(target=self.worker, daemon=True).start()

    def worker(self):
        try:
            for row in read_frames(self.serial, self.stop): self.rows.put(row)
        except Exception as e:
            if not self.stop.is_set(): self.rows.put(e)

    def disconnect(self):
        self.stop.set()
        if self.serial:
            try: self.serial.close()
            except serial.SerialException: pass
        self.serial = None; self.connect_btn.configure(text="连接"); self.status.set("未连接")

    def poll(self):
        last = None; got = False
        try:
            while True:
                row = self.rows.get_nowait()
                if isinstance(row, Exception):
                    messagebox.showerror("串口错误", str(row)); self.disconnect(); continue
                self.ingest(row); last = row; got = True
        except queue.Empty: pass
        self._update_stats(last)
        # new samples only force a redraw while following; paused view is static
        if got and self.follow: self.mark_dirty()
        self.root.after(50, self.poll)

    def render(self):
        start, end = self.window()
        self._sync_pan()
        for c in self.charts:
            if not c.visible: continue
            if c.data_dirty:
                c.data_dirty = c.hover_dirty = False; c.redraw(start, end)
            elif c.hover_dirty:
                c.hover_dirty = False; c.draw_hover()
        self.root.after(75, self.render)

    def ingest(self, row):
        self.latest = row; self.count += 1
        now = time.monotonic()
        selected_stale = self._selected_stream_stale(now)
        key = self._register_stream(row, now)
        self.stream_counts[key] += 1
        vals = self._row_values(row)
        history = self.stream_histories[key]
        for chart in self.charts:
            history[chart.key].append((row["tick_ms"], vals[chart.key]))
        first_key = self.charts[0].key
        if len(history[first_key]) > MAX_HISTORY:  # trim this source only, keep view anchored
            drop = len(history[first_key]) - MAX_HISTORY
            for samples in history.values():
                del samples[:drop]
            if key == self.selected_stream:
                self.view_start = max(0, self.view_start - drop)
        if self.selected_stream is None:
            self._attach_stream(key, reset_view=False)
        elif key != self.selected_stream and self.auto_source.get() and self.follow and selected_stale:
            self._attach_stream(key, reset_view=True)

    def reset(self):
        self.count = 0; self.latest = None; self.view_start = 0; self.set_follow(True)
        self.stream_histories.clear(); self.stream_meta.clear(); self.stream_counts.clear(); self.stream_last_seen.clear(); self.stream_order.clear()
        self.selected_stream = None
        for c in self.charts: c.clear()
        self._refresh_stream_box()
        self._update_stats()
        self.status.set("数据已刷新，重新计数")

    def close(self): self.disconnect(); self.root.destroy()


def main():
    root = tk.Tk(); App(root); root.mainloop()


if __name__ == "__main__":
    main()
