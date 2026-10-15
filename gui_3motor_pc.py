#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Windows GUI for up to seven RS00/EL05 motors on one pyserial AT adapter.

Keep this historical filename as the entry point. Position PP Sequence,
Teaching MIT and JOG share one serial connection. Importing opens no hardware.
"""

import math
import json
import logging
from pathlib import Path
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk
import serial

from robstride_driver_v0 import (
    RobStrideDriver,
    PROFILE_RS00,
    PROFILE_EL05,
    REG_RUN_MODE,
    REG_LOC_REF,
    REG_MECH_POS,
    REG_VEL_MAX,
    REG_ACC_SET,
    RUN_MODE_OPERATION,
    RUN_MODE_POSITION,
    RUN_MODE_POSITION_CSP,
    RUN_MODE_SPEED,
    RUN_MODE_CURRENT,
)

# ===================== CAU HINH MOTOR - SUA O DAY KHI THEM/BOT MOTOR =====
MOTOR_CONFIG = [
    {"id": 1, "profile": PROFILE_EL05, "label": "Motor 1 (EL05)"},
    {"id": 2, "profile": PROFILE_RS00, "label": "Motor 2 (RS00)"},
    {"id": 3, "profile": PROFILE_RS00, "label": "Motor 3 (RS00)"},
    {"id": 4, "profile": PROFILE_EL05, "label": "Motor 4 (EL05)"},
    {"id": 5, "profile": PROFILE_EL05, "label": "Motor 5 (EL05)"},
    {"id": 6, "profile": PROFILE_EL05, "label": "Motor 6 (EL05)"},
    {"id": 7, "profile": PROFILE_RS00, "label": "Motor 7 (RS00)"},
]
MOTOR_CONFIG_PATH = Path(__file__).with_name("motor_config_gui.json")
MAX_MOTORS = 7


class StopErrors(list):
    """Keep OS transport errors distinct from a missing motor confirmation."""
    link_error = None


def new_motor_state():
    return dict(deg=0.0, rad=0.0, vel=0.0, cur=0.0, torque=0.0,
                updated=0.0, requested=0.0, seq=0, vel_updated=0.0,
                vel_requested=0.0, vel_seq=0, cur_updated=0.0)


def validate_motor_config(rows):
    """Validate GUI addresses only. Never assign/change an actual motor ID."""
    if not isinstance(rows, list) or not 1 <= len(rows) <= MAX_MOTORS:
        raise ValueError("Cau hinh phai co tu 1 den 7 motor")
    profiles = {"RS00": PROFILE_RS00, "EL05": PROFILE_EL05}
    result, ids = [], set()
    for index, row in enumerate(rows, 1):
        if not isinstance(row, dict):
            raise ValueError(f"Dong {index}: cau hinh khong hop le")
        mid = row.get("id")
        if isinstance(mid, str):
            value = mid.strip()
            try:
                mid = int(value, 16 if value.lower().startswith("0x") else 10)
            except ValueError:
                raise ValueError(f"Dong {index}: CAN ID phai la so nguyen") from None
        if type(mid) is not int or not 1 <= mid <= 127:
            raise ValueError(f"Dong {index}: CAN ID phai trong 1..127")
        if mid in ids:
            raise ValueError(f"Trung CAN ID {mid}")
        name = str(row.get("profile", "")).strip().upper()
        if name not in profiles:
            raise ValueError(f"M{mid}: phai chon dung RS00 hoac EL05")
        ids.add(mid)
        result.append({"id": mid, "profile": profiles[name], "label": f"Motor {mid} ({name})"})
    return result


def load_motor_config(path=MOTOR_CONFIG_PATH):
    if not Path(path).exists():
        return [dict(cfg) for cfg in MOTOR_CONFIG]
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("File cau hinh phai la JSON object")
    return validate_motor_config(data.get("motors"))


def save_motor_config(config, path=MOTOR_CONFIG_PATH):
    data = {"motors": [{"id": cfg["id"], "profile": cfg["profile"].name} for cfg in config]}
    validate_motor_config(data["motors"])
    write_json_atomic(path, data)


def write_json_atomic(path, data):
    destination = Path(path)
    temporary = destination.with_name(destination.name + ".tmp")
    try:
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()


def waypoint_document(route, config):
    data = {"version": 1, "motors": [{"id": cfg["id"], "profile": cfg["profile"].name} for cfg in config],
            "waypoints": [{"id": wp["id"], "targets": {str(mid): value for mid, value in wp["targets"].items()},
                           "delay": wp.get("delay", 0.5)} for wp in route]}
    validate_waypoint_document(data, config)
    return data


def validate_waypoint_document(data, config):
    if not isinstance(data, dict) or type(data.get("version")) is not int or data["version"] != 1:
        raise ValueError("File waypoint khong dung dinh dang version 1")
    imported = validate_motor_config(data.get("motors"))
    expected = {cfg["id"]: cfg["profile"].name for cfg in config}
    if {cfg["id"]: cfg["profile"].name for cfg in imported} != expected:
        raise ValueError("CAN ID/loai motor cua file khac cau hinh hien tai; can day lai diem")
    entries = data.get("waypoints")
    if not isinstance(entries, list) or not entries:
        raise ValueError("File phai co it nhat mot waypoint")
    result, ids = [], set()
    for wp in entries:
        if not isinstance(wp, dict) or type(wp.get("id")) is not int or wp["id"] <= 0 or wp["id"] in ids:
            raise ValueError("ID waypoint khong hop le hoac bi trung")
        ids.add(wp["id"])
        targets = wp.get("targets")
        if not isinstance(targets, dict) or set(targets) != {str(mid) for mid in expected}:
            raise ValueError(f"Diem #{wp['id']} phai co du goc cua moi motor")
        values = list(targets.values()) + [wp.get("delay")]
        try:
            valid = all(type(value) in (int, float) and math.isfinite(value) for value in values)
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError("Goc va thoi gian cho phai la so huu han")
        if wp["delay"] < 0:
            raise ValueError("Thoi gian cho khong duoc am")
        result.append({"id": wp["id"], "targets": {mid: float(targets[str(mid)]) for mid in expected},
                       "delay": float(wp["delay"])})
    return result


def motor_ids_text(motors):
    return "/".join(f"M{mid}" for mid in motors)


def format_motor_state(cfg, state, now, connected=True):
    if not state.get("seq", 0):
        return f"{cfg['label']}:  Goc: -- (chua co feedback)"
    age = max(0.0, now - state["updated"])
    suffix = " | MAT KET NOI" if not connected else (f" | GOC CU {age:.1f}s" if age > POSITION_MAX_AGE_S else "")
    velocity = f"{state['vel']:.2f}" if 0 < state.get("vel_updated", 0) and now-state["vel_updated"] <= POSITION_MAX_AGE_S else "--"
    fresh_current = 0 < state.get("cur_updated", 0) and now-state["cur_updated"] <= CURRENT_MAX_AGE_S
    current = f"{state['cur']:.2f}" if fresh_current else "--"
    torque = f"{state.get('torque', 0):.2f}" if fresh_current else "--"
    return (f"{cfg['label']}: Goc: {state['deg']:>7.2f} deg ({state['rad']:>7.3f} rad) "
            f"| V: {velocity} rad/s | Iq: {current} A | M~: {torque} Nm{suffix}")
# ===========================================================================

# =================== THAY ĐỔI DUNG SAI THỰC TẾ ===================
STABLE_HOLD_S = 0.06       # Giảm thời gian chờ giữ ổn định từ 0.15s xuống 0.08s
ARRIVE_TOL_DEG = 2.0       # Tăng dung sai từ 1.0 deg lên 2.0 deg (phù hợp tay máy có tải)
ARRIVE_TIMEOUT_S = 6.0     # Timeout tối đa cho 1 điểm
SEQUENCE_TIMEOUT_S = 30.0  # Preserve the existing sequence timeout.
SEQUENCE_RETRY_S = 0.75
POSITION_MAX_AGE_S = 1.0
SEQUENCE_MIN_SPEED = 0.20
SEQUENCE_MIN_ACCEL = 0.50
SEQUENCE_MAX_ACCEL = 4.0
TEACHING_KD = 0.35
SEQUENCE_ACCEL = 2.0  # Sequence command acceleration, within existing 0.5..4 caps.
COMMAND_VERIFY_ATTEMPTS = 3
WAYPOINT_DISPATCH_ATTEMPTS = 3
SEQUENCE_CURRENT_INTERVAL_S = 0.5
CURRENT_MAX_AGE_S = 3.0
ARRIVE_VEL_TOL_RAD_S = 0.05
# Reference readback is distinct from arrival tolerance and V/A float equality.
# Field HOLD log differs by 0.0022..0.0066 deg; do not compare angles at 1e-6 rad.
TARGET_REF_TOL_DEG = 0.02
HOLD_REF_TOL_DEG = 0.10


def sequence_log(message):
    """Keep real sequence diagnostics even if the GUI status is clipped."""
    try:
        with Path(__file__).with_name("sequence_diagnostics.log").open("a", encoding="utf-8") as output:
            output.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {message}\n")
    except OSError:
        logging.getLogger(__name__).exception("Could not write sequence diagnostics")


def travel_time(distance, speed, accel):
    """Rest-to-rest, symmetric triangular/trapezoidal PP estimate (radians)."""
    if distance <= 0:
        return 0.0
    if distance <= speed * speed / accel:
        return 2.0 * math.sqrt(distance / accel)
    return distance / speed + speed / accel


def plan_waypoint(current, targets, motors, nominal_v):
    """Solve individual V/A against a common feasible duration, with floors.

    For a trapezoid: d = v*T - v*v/a. Use the Sequence acceleration, solve for v;
    if that falls below vmin, fix v=vmin and solve for a instead.
    Clip T for a tiny move to its slowest duration at the existing V/A floors.
    These estimates assume the motor firmware uses symmetric rest-to-rest PP.
    """
    if set(targets) != set(motors) or not math.isfinite(nominal_v):
        raise ValueError("Waypoint must contain all configured motors and finite speed")
    distances, caps = {}, {}
    for mid, drv in motors.items():
        if not math.isfinite(current[mid]) or not math.isfinite(float(targets[mid])):
            raise ValueError(f"M{mid}: non-finite position")
        distances[mid] = abs(math.radians(float(targets[mid]) - current[mid]))
        caps[mid] = min(nominal_v, 8.0, drv.profile.v_max)
        if caps[mid] < SEQUENCE_MIN_SPEED:
            raise ValueError(f"M{mid}: speed limit below the existing minimum")
    common_time = max(travel_time(distances[mid], caps[mid], SEQUENCE_ACCEL)
                      for mid in motors)
    commands = {}
    for mid in motors:
        d = distances[mid]
        speed, accel = SEQUENCE_MIN_SPEED, SEQUENCE_MIN_ACCEL
        if d > 0:
            slowest = travel_time(d, SEQUENCE_MIN_SPEED, SEQUENCE_MIN_ACCEL)
            target_time = min(common_time, slowest)
            accel = SEQUENCE_ACCEL
            disc = max(0.0, target_time ** 2 - 4.0 * d / accel)
            speed = 2.0 * d / (target_time + math.sqrt(disc))
            if speed < SEQUENCE_MIN_SPEED:
                speed = SEQUENCE_MIN_SPEED
                accel = speed ** 2 / max(speed * target_time - d, 1e-12)
            speed = max(SEQUENCE_MIN_SPEED, min(caps[mid], speed))
            accel = max(SEQUENCE_MIN_ACCEL, min(SEQUENCE_MAX_ACCEL, accel))
        commands[mid] = {"target_deg": float(targets[mid]), "speed": speed,
                         "accel": accel, "duration": travel_time(d, speed, accel)}
    return commands, common_time


class WaypointConfirmation:
    """Three distinct fresh rounds for all motors; never reuse a cache sample."""
    def __init__(self, targets, command_time, baseline, require_stopped=False):
        self.targets = targets
        self.command_time = command_time
        self.last_seq = {mid: baseline[mid]["seq"] for mid in targets}
        self.stable = 0
        self.require_stopped = require_stopped
        self.last_vel_seq = {mid: baseline[mid].get("vel_seq", 0) for mid in targets}

    def observe(self, snapshot, now):
        fresh = all(snapshot[mid]["seq"] > self.last_seq[mid]
                    and snapshot[mid]["updated"] > self.command_time
                    and snapshot[mid]["requested"] > self.command_time
                    and 0 <= now - snapshot[mid]["updated"] <= POSITION_MAX_AGE_S
                    for mid in self.targets)
        arrived = all(math.isfinite(snapshot[mid]["deg"])
                      and abs(snapshot[mid]["deg"] - target) <= ARRIVE_TOL_DEG
                      for mid, target in self.targets.items())
        if self.require_stopped:
            fresh = fresh and all(snapshot[mid].get("vel_seq", 0) > self.last_vel_seq[mid]
                                  and snapshot[mid].get("vel_requested", 0.0) > self.command_time
                                  and snapshot[mid].get("vel_updated", 0.0) > self.command_time
                                  and 0 <= now - snapshot[mid].get("vel_updated", 0.0) <= POSITION_MAX_AGE_S
                                  for mid in self.targets)
            arrived = arrived and all(math.isfinite(snapshot[mid].get("vel", float("inf")))
                                      and abs(snapshot[mid]["vel"]) <= ARRIVE_VEL_TOL_RAD_S
                                      for mid in self.targets)
        if not arrived or any(now - snapshot[mid]["updated"] > POSITION_MAX_AGE_S
                              for mid in self.targets):
            self.stable = 0
        if fresh:
            self.last_seq = {mid: snapshot[mid]["seq"] for mid in self.targets}
            self.last_vel_seq = {mid: snapshot[mid].get("vel_seq", 0) for mid in self.targets}
            self.stable = self.stable + 1 if arrived else 0
        return self.stable >= 3


class MotorApp:
    @property
    def motor_config(self):
        return getattr(self, "_motor_config", MOTOR_CONFIG)

    def __init__(self, root):
        self.root = root
        self._configuration_error = ""
        try:
            self._motor_config = load_motor_config()
        except (OSError, ValueError) as exc:
            self._motor_config = [dict(cfg) for cfg in MOTOR_CONFIG]
            self._configuration_error = str(exc)
        self.root.resizable(True, True)
        self.root.minsize(820, 600)
        self.root.geometry(f"{min(1180, self.root.winfo_screenwidth()-60)}x{min(940, self.root.winfo_screenheight()-100)}")

        self.motors = {}  # id -> RobStrideDriver
        self.is_connected = False
        self.is_teaching = False
        self.is_playing = False
        self.telemetry_running = False
        self._connection_lock = threading.Lock()
        self._connection_transition = threading.Event()
        self._connection_thread = None
        self._connection_generation = 0
        self._connection_fault = ""
        self._telemetry_stop = threading.Event()
        self._telemetry_thread = None
        self._closing = False
        self._manual_stop = threading.Event()
        self._manual_stop.set()
        self._manual_generation = 0
        self._manual_stop_pending = threading.Event()
        self._manual_threads = set()
        self._manual_threads_lock = threading.Lock()
        self._control_waiting = threading.Event()
        self._motion_blocked = threading.Event()
        self._jog_motor_id = None
        self._jog_pressed = threading.Event()

        self.sequence_stop_flag = threading.Event()
        self.sequence_running_flag = threading.Event()
        self._sequence_thread = None
        self._sequence_stop_thread = None
        self._teaching_fully_off = threading.Event()
        self._teaching_fully_off.set()

        # Protect motor_states while telemetry/sequence threads access it.
        self._state_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._sequence_command_lock = threading.Lock()
        self._telemetry_io_lock = threading.Lock()
        self._sequence_owns_telemetry = threading.Event()
        self._sequence_next_current_at = 0.0
        self._sequence_current_index = 0

        self._label_to_id = {cfg["label"]: cfg["id"] for cfg in self.motor_config}
        self.motor_states = {cfg["id"]: new_motor_state() for cfg in self.motor_config}
        self.waypoints = []
        self.last_telemetry_error = ""

        self._setup_ui()

    def _ui(self, fn):
        """Chay fn() tren main thread Tkinter - dung cho moi cap nhat widget
        tu cac thread nen (telemetry, sequence, teaching)."""
        if not self._closing:
            self.root.after(0, fn)

    # ======================================================================
    # UI
    # ======================================================================
    def _setup_ui(self):
        self.lbl_status = ttk.Label(self.root, text="He thong san sang.", relief="sunken", anchor="w")
        self.lbl_status.pack(side="bottom", fill="x")
        viewport = ttk.Frame(self.root)
        viewport.pack(fill="both", expand=True)
        canvas = tk.Canvas(viewport, highlightthickness=0)
        scroll = ttk.Scrollbar(viewport, orient="vertical", command=canvas.yview)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scroll.set)
        self._body = ttk.Frame(canvas)
        body_window = canvas.create_window((0, 0), window=self._body, anchor="nw")
        self._body.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(body_window, width=event.width))
        conn_frame = ttk.LabelFrame(self._body, text=" 1. Ket noi USB-CAN ", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=4)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.txt_port = ttk.Entry(conn_frame, width=16)
        self.txt_port.insert(0, "COM3")
        self.txt_port.grid(row=0, column=1, padx=5, pady=2)

        self.btn_connect = tk.Button(
            conn_frame, text="Ket noi", bg="#4CAF50", fg="white",
            font=("Arial", 9, "bold"), command=self.toggle_connect
        )
        self.btn_connect.grid(row=0, column=2, padx=15, pady=2)
        self.btn_motor_config = ttk.Button(conn_frame, text="Cau hinh 1-7 motor", command=self.show_motor_configuration)
        self.btn_motor_config.grid(row=0, column=3, padx=5)
        ttk.Button(conn_frame, text="Xem log", command=self.show_diagnostics).grid(row=0, column=4, padx=5)

        tele_frame = ttk.LabelFrame(self._body, padding=10)
        self.tele_frame = tele_frame
        tele_frame.pack(fill="x", padx=10, pady=4)

        self.lbl_motors = {}
        ttk.Label(self._body, text="M~ (Nm): mo-men uoc tinh tu Iq va Kt cua profile; -- = chua co/da cu feedback.").pack(fill="x", padx=20)

        ctrl_frame = ttk.LabelFrame(self._body, text=" 3. Dieu khien chap hanh (Tat ca motor) ", padding=10)
        ctrl_frame.pack(fill="x", padx=10, pady=4)

        tk.Button(
            ctrl_frame, text="Bat Luc (All)", width=14, bg="#2196F3", fg="white", font=("Arial", 9, "bold"),
            command=self.cmd_enable_all
        ).grid(row=0, column=0, padx=5, pady=4)

        tk.Button(
            ctrl_frame, text="STOP / Ngat All", width=14, bg="#D32F2F", fg="white", font=("Arial", 9, "bold"),
            command=self.cmd_disable_all
        ).grid(row=0, column=1, padx=5, pady=4)

        tk.Button(
            ctrl_frame, text="Goc 0 (Tat ca)", width=14, bg="#607D8B", fg="white",
            command=self.cmd_set_zero
        ).grid(row=0, column=2, padx=5, pady=4)

        self.btn_teach_mode = tk.Button(
            ctrl_frame, text="Bat Teaching (MIT)", width=18, bg="#9C27B0", fg="white",
            font=("Arial", 9, "bold"), command=self.toggle_teaching
        )
        self.btn_teach_mode.grid(row=0, column=3, padx=5, pady=4)

        jog_frame = ttk.LabelFrame(self._body, text=" 4. Dieu khien JOG (Nhap buoc & Giu chuot quay lien tuc) ", padding=10)
        jog_frame.pack(fill="x", padx=10, pady=4)

        row_sel = ttk.Frame(jog_frame)
        row_sel.pack(fill="x", pady=2)

        ttk.Label(row_sel, text="Chon Khop:").pack(side="left", padx=5)
        self.cb_jog_id = ttk.Combobox(
            row_sel, values=[cfg["label"] for cfg in self.motor_config], width=18, state="readonly"
        )
        self.cb_jog_id.set(self.motor_config[0]["label"])
        self.cb_jog_id.pack(side="left", padx=5)

        ttk.Label(row_sel, text="Buoc (deg):").pack(side="left", padx=5)
        self.spin_jog_step = ttk.Spinbox(row_sel, from_=0.5, to=90.0, increment=1.0, width=5)
        self.spin_jog_step.set(5.0)
        self.spin_jog_step.pack(side="left", padx=5)

        ttk.Label(row_sel, text="Toc do:").pack(side="left", padx=5)
        self.spin_jog_spd = ttk.Spinbox(row_sel, from_=0.2, to=8.0, increment=0.5, width=5)
        self.spin_jog_spd.set(1.5)
        self.spin_jog_spd.pack(side="left", padx=5)
        ttk.Label(row_sel, text="rad/s").pack(side="left")

        btn_step_row = ttk.Frame(jog_frame)
        btn_step_row.pack(fill="x", pady=4)

        tk.Button(
            btn_step_row, text="<-- Buoc (-)", width=16, bg="#546E7A", fg="white", font=("Arial", 9, "bold"),
            command=lambda: self.jog_step(-1)
        ).pack(side="left", padx=20)

        tk.Button(
            btn_step_row, text="Buoc (+) -->", width=16, bg="#546E7A", fg="white", font=("Arial", 9, "bold"),
            command=lambda: self.jog_step(1)
        ).pack(side="left", padx=20)

        btn_cont_row = ttk.Frame(jog_frame)
        btn_cont_row.pack(fill="x", pady=4)

        self.btn_jog_rev = tk.Button(
            btn_cont_row, text="<< GIU QUAY LUI (JOG-)", width=22, bg="#FF7043", fg="white", font=("Arial", 9, "bold")
        )
        self.btn_jog_rev.pack(side="left", padx=12)

        self.btn_jog_fwd = tk.Button(
            btn_cont_row, text="GIU QUAY TIEN (JOG+) >>", width=22, bg="#26A69A", fg="white", font=("Arial", 9, "bold")
        )
        self.btn_jog_fwd.pack(side="left", padx=12)

        EVT_PRESS = chr(60) + "Button-1" + chr(62)
        EVT_RELEASE = chr(60) + "ButtonRelease-1" + chr(62)

        self.btn_jog_rev.bind(EVT_PRESS, lambda e: self.jog_cont_start(-1))
        self.btn_jog_rev.bind(EVT_RELEASE, lambda e: self.jog_cont_stop())
        self.btn_jog_fwd.bind(EVT_PRESS, lambda e: self.jog_cont_start(1))
        self.btn_jog_fwd.bind(EVT_RELEASE, lambda e: self.jog_cont_stop())
        self.root.bind(EVT_RELEASE, lambda event: self.jog_cont_stop() if self._jog_motor_id is not None else None, add="+")
        self.root.bind("<FocusOut>", self._on_focus_out)

        wp_frame = ttk.LabelFrame(self._body, text=" 5. Day diem & Chay chuoi quy dao ", padding=10)
        wp_frame.pack(fill="both", expand=True, padx=10, pady=4)

        cols = ("ID",) + tuple(f"{cfg['label']} (deg)" for cfg in self.motor_config) + ("Cho (s)",)
        table_frame = ttk.Frame(wp_frame)
        table_frame.pack(fill="both", expand=True, pady=4)
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)
        self.tree_wp = ttk.Treeview(table_frame, columns=cols, show="headings", height=8)
        for c in cols:
            self.tree_wp.heading(c, text=c)
            self.tree_wp.column(c, width=140, anchor="center")
        self.tree_wp.grid(row=0, column=0, sticky="nsew")
        table_x = ttk.Scrollbar(table_frame, orient="horizontal", command=self.tree_wp.xview)
        table_y = ttk.Scrollbar(table_frame, orient="vertical", command=self.tree_wp.yview)
        table_x.grid(row=1, column=0, sticky="ew")
        table_y.grid(row=0, column=1, sticky="ns")
        self.tree_wp.configure(xscrollcommand=table_x.set, yscrollcommand=table_y.set)
        self.tree_wp.bind("<Double-1>", self.edit_waypoint_delay)
        file_box = ttk.Frame(wp_frame)
        file_box.pack(fill="x", pady=3)
        ttk.Button(file_box, text="Luu chuoi JSON", command=self.export_waypoints).pack(side="left", padx=5)
        ttk.Button(file_box, text="Mo chuoi JSON", command=self.import_waypoints).pack(side="left", padx=5)
        ttk.Label(file_box, text="Nhan dup cot Cho (s) de sua thoi gian dung.").pack(side="left", padx=8)

        btn_box = ttk.Frame(wp_frame)
        btn_box.pack(fill="x", pady=4)

        tk.Button(
            btn_box, text="[+] Luu Diem Hien Tai", bg="#3F51B5", fg="white", font=("Arial", 9, "bold"),
            command=self.save_current_point
        ).pack(side="left", padx=5)

        tk.Button(
            btn_box, text="Xoa Toan Bo Diem", bg="#E91E63", fg="white",
            command=self.clear_points
        ).pack(side="left", padx=5)

        self.btn_run_seq = tk.Button(
            btn_box, text="> CHAY CHUOI QUY DAO", bg="#009688", fg="white", font=("Arial", 10, "bold"),
            command=self.toggle_run_sequence
        )
        self.btn_run_seq.pack(side="right", padx=5)

        self._refresh_motor_views()
        if self._configuration_error:
            self.lbl_status.config(text=f"LOI CAU HINH: {self._configuration_error}; hay sua trong Cau hinh motor.")

    def _refresh_motor_views(self):
        self.root.title(f"RobStride {len(self.motor_config)}-Motor Local Controller - V29 Command Guard")
        self.tele_frame.config(text=f" 2. Trang thai {len(self.motor_config)} motor (toi da 7) ")
        for child in self.tele_frame.winfo_children():
            child.destroy()
        self.lbl_motors = {}
        for row, cfg in enumerate(self.motor_config):
            label = ttk.Label(self.tele_frame, text=f"{cfg['label']}: Goc: -- (chua co feedback)",
                              font=("Consolas", 9, "bold"), wraplength=1020, justify="left")
            label.grid(row=row, column=0, sticky="w", pady=3)
            self.lbl_motors[cfg["id"]] = label
        self.tele_frame.bind("<Configure>", lambda event: [label.configure(wraplength=max(400, event.width-24))
                                                           for label in self.lbl_motors.values()])
        choices = [cfg["label"] for cfg in self.motor_config]
        self.cb_jog_id.configure(values=choices)
        if self.cb_jog_id.get() not in choices:
            self.cb_jog_id.set(choices[0])
        columns = ("ID",) + tuple(f"{cfg['label']} (deg)" for cfg in self.motor_config) + ("Cho (s)",)
        self.tree_wp.configure(columns=columns)
        for index, column in enumerate(columns):
            self.tree_wp.heading(column, text=column)
            self.tree_wp.column(column, width=65 if index == 0 else 140, minwidth=60, anchor="center", stretch=False)

    def _apply_motor_configuration(self, rows, path=MOTOR_CONFIG_PATH, discard_waypoints=False):
        config = validate_motor_config(rows)
        if (self.is_connected or self.motors or self._connection_transition.is_set()
                or self._closing or self._sequence_busy() or self.is_teaching
                or not self._teaching_fully_off.is_set()):
            raise ValueError("Hay ngat ket noi va cho cac worker dung truoc khi doi cau hinh")
        if self.waypoints and not discard_waypoints:
            raise ValueError("Can xoa/day lai waypoint khi doi danh sach motor")
        if not self._motion_lock.acquire(blocking=False):
            raise ValueError("Worker dieu khien chua ket thuc")
        try:
            save_motor_config(config, path)
            self._connection_generation += 1
            self._motor_config = config
            self._configuration_error = ""
            self._label_to_id = {cfg["label"]: cfg["id"] for cfg in config}
            with self._state_lock:
                self.motor_states = {cfg["id"]: new_motor_state() for cfg in config}
            self.waypoints.clear()
            for row in self.tree_wp.get_children():
                self.tree_wp.delete(row)
            self._sequence_next_current_at = 0.0
            self._sequence_current_index = 0
            self._refresh_motor_views()
            self.lbl_status.config(text=f"Da luu {len(config)} motor: {motor_ids_text(self.motor_states)}. Chua ket noi COM.")
        finally:
            self._motion_lock.release()

    def show_motor_configuration(self):
        if (self.is_connected or self.motors or self._connection_transition.is_set()
                or self._sequence_busy() or self.is_teaching or self._closing):
            self.lbl_status.config(text="Hay ngat ket noi truoc khi cau hinh motor.")
            return
        existing = getattr(self, "_config_dialog", None)
        if existing is not None and existing.winfo_exists():
            existing.lift()
            return
        dialog = tk.Toplevel(self.root)
        self._config_dialog = dialog
        dialog.title("Cau hinh toi da 7 motor")
        dialog.transient(self.root)
        ttk.Label(dialog, text="Nhap ID va dung loai motor. De trong ca 2 o neu chua su dung.\n"
                              "Chi luu dia chi trong GUI; khong doi CAN ID tren dong co.").grid(
            row=0, column=0, columnspan=3, padx=12, pady=10)
        for column, title in enumerate(("Dong", "CAN ID (so hoac 0x...)", "Loai motor")):
            ttk.Label(dialog, text=title).grid(row=1, column=column, padx=8, pady=4)
        fields = []
        for index in range(MAX_MOTORS):
            ttk.Label(dialog, text=str(index+1)).grid(row=index+2, column=0)
            entry = ttk.Entry(dialog, width=18)
            profile = ttk.Combobox(dialog, values=("", "RS00", "EL05"), state="readonly", width=12)
            entry.grid(row=index+2, column=1, padx=8, pady=4)
            profile.grid(row=index+2, column=2, padx=8, pady=4)
            if index < len(self.motor_config):
                cfg = self.motor_config[index]
                entry.insert(0, str(cfg["id"]))
                profile.set(cfg["profile"].name)
            fields.append((entry, profile))

        def apply():
            rows = [{"id": entry.get().strip(), "profile": profile.get()}
                    for entry, profile in fields if entry.get().strip() or profile.get()]
            try:
                validate_motor_config(rows)
                discard = False
                if self.waypoints:
                    discard = messagebox.askyesno("Day lai waypoint", "Doi cau hinh se xoa waypoint cu. Tiep tuc?", parent=dialog)
                    if not discard:
                        return
                self._apply_motor_configuration(rows, discard_waypoints=discard)
            except (OSError, ValueError) as exc:
                messagebox.showerror("Loi cau hinh", str(exc), parent=dialog)
                return
            dialog.destroy()

        ttk.Button(dialog, text="Luu cau hinh", command=apply).grid(row=9, column=1, pady=12)
        ttk.Button(dialog, text="Huy", command=dialog.destroy).grid(row=9, column=2, pady=12)

    def show_diagnostics(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("Log dieu khien")
        text = tk.Text(dialog, width=120, height=28, wrap="word")
        scroll = ttk.Scrollbar(dialog, command=text.yview)
        scroll.pack(side="right", fill="y")
        text.pack(fill="both", expand=True)
        text.configure(yscrollcommand=scroll.set)
        try:
            content = Path(__file__).with_name("sequence_diagnostics.log").read_text(encoding="utf-8")
            content = "\n".join(content.splitlines()[-200:])
        except OSError as exc:
            content = f"Chua doc duoc log: {exc}"
        text.insert("end", content)
        text.configure(state="disabled")
        text.see("end")

    def _render_waypoints(self):
        for item in self.tree_wp.get_children():
            self.tree_wp.delete(item)
        for wp in self.waypoints:
            values = [wp["id"]] + [f"{wp['targets'][cfg['id']]:.2f} deg" for cfg in self.motor_config] + [wp["delay"]]
            self.tree_wp.insert("", "end", values=values)

    def export_waypoints(self):
        try:
            document = waypoint_document(self.waypoints, self.motor_config)
        except (ValueError, TypeError, KeyError) as exc:
            self.lbl_status.config(text=f"Chua luu file: {exc}")
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json", filetypes=[("Waypoint JSON", "*.json")])
        if not path:
            return
        try:
            write_json_atomic(path, document)
        except OSError as exc:
            self.lbl_status.config(text=f"Khong luu duoc file: {exc}")
            return
        self.lbl_status.config(text=f"Da luu {len(self.waypoints)} waypoint: {path}")

    def import_waypoints(self):
        if self._sequence_busy():
            self.lbl_status.config(text="Hay dung chuoi truoc khi mo file waypoint.")
            return
        path = filedialog.askopenfilename(parent=self.root, filetypes=[("Waypoint JSON", "*.json")])
        if not path:
            return
        try:
            route = validate_waypoint_document(json.loads(Path(path).read_text(encoding="utf-8-sig")), self.motor_config)
        except (OSError, ValueError) as exc:
            self.lbl_status.config(text=f"Khong mo duoc waypoint: {exc}")
            return
        if self.waypoints and not messagebox.askyesno("Thay waypoint", "Thay danh sach waypoint hien tai bang file vua chon?", parent=self.root):
            return
        self.waypoints = route
        self._render_waypoints()
        self.lbl_status.config(text=f"Da mo {len(route)} waypoint. Bam Chay chuoi khi san sang.")

    def edit_waypoint_delay(self, event):
        if self._sequence_busy():
            return
        item = self.tree_wp.identify_row(event.y)
        column = self.tree_wp.identify_column(event.x)
        if not item or column != f"#{len(self.motor_config)+2}":
            return
        index = self.tree_wp.index(item)
        value = simpledialog.askfloat("Thoi gian cho", "Dung tai diem nay bao nhieu giay?", minvalue=0.0,
                                      initialvalue=self.waypoints[index]["delay"], parent=self.root)
        if value is not None and math.isfinite(value):
            self.waypoints[index]["delay"] = value
            self._render_waypoints()

    def _get_selected_motor_id(self):
        return self._label_to_id.get(self.cb_jog_id.get(), self.motor_config[0]["id"])

    # ======================================================================
    # KET NOI / NGAT KET NOI
    # ======================================================================
    def toggle_connect(self):
        if self._connection_transition.is_set() or self._closing:
            self.lbl_status.config(text="Dang dong/mo COM; hay cho cac worker ket thuc.")
            return
        if self.is_connected or self.motors:
            self._begin_disconnect()
            return
        if getattr(self, "_configuration_error", ""):
            self.lbl_status.config(text="File cau hinh loi; hay sua trong Cau hinh 1-7 motor truoc khi ket noi.")
            return
        port = self.txt_port.get().strip()
        with self._connection_lock:
            self._connection_transition.set()
            self._connection_generation += 1
            generation = self._connection_generation
        self.btn_connect.config(state="disabled", text="Dang ket noi...")
        self.lbl_status.config(text=f"Dang mo cong {port}...")

        def connect():
            motors, first_drv, error = {}, None, None
            try:
                # Open once. All axes share the same serial object and lock.
                cfg = self.motor_config[0]
                first_drv = RobStrideDriver(port=port, profile=cfg["profile"], motor_id=cfg["id"])
                motors[cfg["id"]] = first_drv
                for cfg in self.motor_config[1:]:
                    motors[cfg["id"]] = RobStrideDriver(
                        port=port, profile=cfg["profile"], motor_id=cfg["id"],
                        _shared_ser=first_drv.ser, _shared_lock=first_drv._lock)
                for drv in motors.values():
                    drv.diagnostic_sink = sequence_log
            except Exception as e:
                error = f"{type(e).__name__}: {e}"
                if first_drv is not None:
                    try:
                        first_drv.ser.close()
                    except Exception:
                        pass
                sequence_log(f"CONNECT_ERROR port={port} {error}")

            def finish():
                if self._closing:
                    if first_drv is not None and error is None:
                        first_drv.ser.close()
                    self._connection_transition.clear()
                    self.root.destroy()
                    return
                self._connection_transition.clear()
                self.btn_connect.config(state="normal")
                if error is not None:
                    self.lbl_status.config(text=f"Khong the mo {port}: {error}")
                    self.btn_connect.config(text="Ket noi", bg="#4CAF50")
                    return
                self.motors = motors
                self._connection_fault = ""
                self.last_telemetry_error = ""
                self._telemetry_stop = threading.Event()
                with self._state_lock:
                    for state in self.motor_states.values():
                        state.update(new_motor_state())
                self.is_connected = True
                self.telemetry_running = True
                self._telemetry_thread = threading.Thread(
                    target=self._telemetry_loop,
                    args=(dict(motors), self._telemetry_stop, generation), daemon=True)
                self._telemetry_thread.start()
                self.btn_connect.config(text="Ngat KN", bg="#f44336")
                self.lbl_status.config(text=f"Da mo {port} cho {len(motors)} dia chi motor. Cho feedback goc; chua bat luc.")
            # This completion is also needed when Close was pressed during open.
            self.root.after(0, finish)

        self._connection_thread = threading.Thread(target=connect, daemon=True)
        self._connection_thread.start()

    def _begin_disconnect(self, reason=""):
        """Cancel/drain workers before closing the one shared serial handle.

        Can be requested from telemetry or Sequence; joins occur on a separate
        cleanup thread. Never automatically reconnect or replay a route.
        """
        with self._connection_lock:
            if self._connection_transition.is_set():
                return
            self._connection_transition.set()
            self._connection_generation += 1
            motors = dict(self.motors)
            self.is_connected = False  # Immediately reject new motor commands.
            self.telemetry_running = False
            self._telemetry_stop.set()
            if hasattr(self, "_manual_stop"):
                self._manual_stop.set()
                self._manual_generation += 1
            self.is_teaching = False
            self.sequence_stop_flag.set()
            if reason:
                self._connection_fault = reason
            workers = (self._sequence_thread, self._sequence_stop_thread, self._telemetry_thread)
            if hasattr(self, "_manual_threads_lock"):
                with self._manual_threads_lock:
                    workers += tuple(self._manual_threads)
        self._ui(lambda: (
            self.btn_connect.config(state="disabled", text="Dang ngat..."),
            self.lbl_status.config(text=(f"LOI COM: {reason} | dang dung worker..." if reason
                                         else "Dang dung worker va dong COM..."))))

        def disconnect():
            errors = []
            for worker in workers:
                if worker is not None:
                    worker.join()
            # Also drain an active manual/Teaching worker and pending RX.
            with self._motion_lock, self._telemetry_io_lock:
                errors = self._stop_sequence_motors(motors, self._sequence_command_lock) if motors else []
                if motors:
                    first = next(iter(motors.values()))
                    try:
                        with first._lock:
                            first.ser.close()
                    except Exception as e:
                        errors.append(f"close COM: {e}")
                self.motors = {}
            self._teaching_fully_off.set()
            sequence_log(f"DISCONNECTED reason={reason!r} stop_errors={errors}")

            def finish():
                self.is_playing = False
                self.is_teaching = False
                self.btn_run_seq.config(text="> CHAY CHUOI QUY DAO", bg="#009688")
                self.btn_teach_mode.config(text="Bat Teaching (MIT)", bg="#9C27B0")
                self.btn_connect.config(state="normal", text="Ket noi", bg="#4CAF50")
                self._connection_transition.clear()
                text = f"MAT KET NOI: {reason} | Hay kiem tra USB/COM va ket noi lai." if reason else "Da ngat ket noi."
                if errors:
                    text += " | STOP CHUA XAC NHAN: " + " | ".join(errors)
                self.lbl_status.config(text=text)
                self._push_telemetry_to_ui()
                if self._closing:
                    self.root.destroy()
            self.root.after(0, finish)

        self._connection_thread = threading.Thread(target=disconnect, daemon=True)
        self._connection_thread.start()

    def _handle_serial_fault(self, error):
        reason = f"{type(error).__name__}: {error}"
        self.last_telemetry_error = reason
        if not self._connection_transition.is_set() and self.is_connected:
            sequence_log(f"SERIAL_FAULT {reason}")
            self._begin_disconnect(reason)

    # ======================================================================
    # TELEMETRY NEN
    # ======================================================================
    def _record_position(self, mid, pos, requested):
        if pos is None or not math.isfinite(pos):
            return
        with self._state_lock:
            state = self.motor_states[mid]
            state.update(rad=pos, deg=math.degrees(pos), updated=time.monotonic(),
                         requested=requested, seq=state.get("seq", 0) + 1)

    def _record_current(self, mid, drv, current):
        if current is not None and math.isfinite(current):
            with self._state_lock:
                self.motor_states[mid].update(cur=current, cur_updated=time.monotonic(),
                                              torque=current * drv.profile.kt)

    def _position_snapshot(self):
        with self._state_lock:
            return {mid: dict(st) for mid, st in self.motor_states.items()}

    def _read_motor_state(self, mid, drv, teaching=False, stop_event=None, read_current=True, yield_event=None):
        """One real read per register; publish timestamps only after valid RX."""
        requested = time.monotonic()
        if teaching:
            pos, vel, cur = drv.mit_control(
                pos_rad=0.0, vel_rad=0.0, kp=0.0, kd=TEACHING_KD,
                torque_nm=0.0, timeout=0.006,
            )
            self._record_position(mid, pos, requested)
            velocity_requested = requested
        else:
            pos = drv.read_position_rad()
            self._record_position(mid, pos, requested)
            if (stop_event is not None and stop_event.is_set()) or (yield_event is not None and yield_event.is_set()):
                return
            velocity_requested = time.monotonic()
            vel = drv.read_velocity_rad()
            cur = None
        with self._state_lock:
            if vel is not None and math.isfinite(vel):
                self.motor_states[mid].update(
                    vel=vel, vel_updated=time.monotonic(), vel_requested=velocity_requested,
                    vel_seq=self.motor_states[mid].get("vel_seq", 0) + 1,
                )
        if not teaching and read_current:
            if (stop_event is not None and stop_event.is_set()) or (yield_event is not None and yield_event.is_set()):
                return
            cur = drv.read_current_a()
        self._record_current(mid, drv, cur)

    def _poll_sequence_states(self, motors):
        # Reference design: Sequence owns RX and keeps the GUI telemetry live.
        # The normal telemetry thread remains alive and resumes after cleanup.
        with self._telemetry_io_lock:
            for mid, drv in motors.items():
                if self.sequence_stop_flag.is_set():
                    break
                try:
                    self._read_motor_state(mid, drv, stop_event=self.sequence_stop_flag,
                                           read_current=False)
                except (serial.SerialException, OSError):
                    raise  # A broken COM link must not turn into a 6/30s timeout.
                except Exception as e:
                    self.last_telemetry_error = f"M{mid}: {type(e).__name__}: {e}"
            # Position/velocity are control-critical. Rotate one current read
            # Keep the original 1.5s rotation budget when adding more motors.
            now = time.monotonic()
            if motors and not self.sequence_stop_flag.is_set() and now >= self._sequence_next_current_at:
                mids = list(motors)
                mid = mids[self._sequence_current_index % len(mids)]
                self._sequence_current_index += 1
                interval = SEQUENCE_CURRENT_INTERVAL_S * min(1.0, 3.0 / len(mids))
                self._sequence_next_current_at = now + interval
                try:
                    cur = motors[mid].read_current_a()
                    self._record_current(mid, motors[mid], cur)
                except (serial.SerialException, OSError):
                    raise
                except Exception as e:
                    self.last_telemetry_error = f"M{mid}: current: {e}"
        self._push_telemetry_to_ui()

    def _telemetry_loop(self, motors=None, stop_event=None, generation=None):
        motors = dict(self.motors) if motors is None else motors
        stop_event = self._telemetry_stop if stop_event is None else stop_event
        generation = self._connection_generation if generation is None else generation

        def active():
            return (self.telemetry_running and not stop_event.is_set()
                    and generation == self._connection_generation)

        while active():
            waiting = getattr(self, "_control_waiting", None)
            if self._sequence_owns_telemetry.is_set() or (waiting is not None and waiting.is_set()):
                time.sleep(0.02)
                continue
            with self._telemetry_io_lock:
                for cfg in self.motor_config:
                    # Recheck after taking the lock: ownership may have changed.
                    if (not active() or self._sequence_owns_telemetry.is_set()
                            or (waiting is not None and waiting.is_set())):
                        break
                    mid = cfg["id"]
                    drv = motors.get(mid)
                    if drv is None:
                        continue
                    try:
                        self._read_motor_state(mid, drv, teaching=self.is_teaching, yield_event=waiting)
                    except (serial.SerialException, OSError) as e:
                        self._handle_serial_fault(e)
                        return
                    except Exception as e:
                        self.last_telemetry_error = f"M{mid}: {type(e).__name__}: {e}"
                        if self.is_teaching:
                            self.is_teaching = False
                            self._begin_disconnect(self.last_telemetry_error)
                            return
            self._push_telemetry_to_ui()
            time.sleep(0.01 if self.is_teaching else 0.03)

    def _push_telemetry_to_ui(self):
        snapshot = self._position_snapshot()
        generation = self._connection_generation

        def _update():
            if generation != self._connection_generation:
                return  # Do not display a sample queued by a previous connection.
            for cfg in self.motor_config:
                mid = cfg["id"]
                s = snapshot.get(mid)
                lbl = self.lbl_motors.get(mid)
                if s is None or lbl is None:
                    continue
                lbl.config(text=format_motor_state(cfg, s, time.monotonic(), self.is_connected))
        self._ui(_update)

    # ======================================================================
    # LENH DIEU KHIEN CHUNG (chay trong thread rieng de khong dong UI)
    # ======================================================================
    def _sequence_busy(self):
        return (self.is_playing or self.sequence_running_flag.is_set()
                or (self._sequence_thread is not None and self._sequence_thread.is_alive())
                or (self._sequence_stop_thread is not None and self._sequence_stop_thread.is_alive()))

    def _reject_during_sequence(self):
        if self._sequence_busy():
            self.lbl_status.config(text="Sequence dang giu quyen dieu khien; hay DUNG CHUOI truoc.")
            return True
        return False

    def _ensure_control_state(self):
        if not hasattr(self, "_control_waiting"):
            self._control_waiting = threading.Event()
            self._motion_blocked = threading.Event()
        if not hasattr(self, "_manual_stop"):
            self._manual_stop = threading.Event()
            self._manual_generation = 0
            self._manual_stop_pending = threading.Event()
            self._manual_threads = set()
            self._manual_threads_lock = threading.Lock()
            self._jog_motor_id = None
            self._jog_pressed = threading.Event()

    def _control_cancelled(self):
        return self._telemetry_stop.is_set() or (hasattr(self, "_manual_stop") and self._manual_stop.is_set())

    def _manual_cleanup(self, error, motors):
        errors = self._stop_sequence_motors(motors, self._sequence_command_lock)
        self.is_teaching = False
        self._teaching_fully_off.set()
        if errors:
            self._latch_stop_error(errors)
        if isinstance(error, (serial.SerialException, OSError)):
            self._handle_serial_fault(error)
        sequence_log(f"MANUAL_ENABLE_ERROR/CONTROL {type(error).__name__}: {error} stop_errors={errors}")
        cleanup = "STOP chua xac nhan" if errors else f"STOP da xac nhan {motor_ids_text(motors)}"
        result = RuntimeError(f"{error} | {cleanup}")
        result._motor_cleanup_done = True
        return result

    def _latch_stop_error(self, errors):
        self._ensure_control_state()
        self._motion_blocked.set()
        self._manual_stop.set()
        sequence_log("MOTION_BLOCKED STOP chua xac nhan: " + " | ".join(errors))
        if getattr(errors, "link_error", None) is not None:
            self._handle_serial_fault(errors.link_error)

    def _start_control_worker(self, fn, allow_sequence=False, on_cancel=None, stop_command=False, priority=False):
        """Drain old manual workers before Sequence; reject queued commands."""
        self._ensure_control_state()
        if self._motion_blocked.is_set() and not stop_command:
            self.lbl_status.config(text="STOP CHUA XAC NHAN. Bam STOP All de xac nhan lai; COM van mo de doc goc.")
            if on_cancel is not None:
                on_cancel()
            return
        if self._control_waiting.is_set() and not stop_command and not allow_sequence and not priority:
            self.lbl_status.config(text="Dang xu ly lenh truoc; khong xep them lenh trung.")
            if on_cancel is not None:
                on_cancel()
            return
        if self._manual_stop_pending.is_set() and not stop_command and not allow_sequence:
            self.lbl_status.config(text="Dang STOP tat ca motor; hay cho xac nhan.")
            if on_cancel is not None:
                on_cancel()
            return
        generation = self._connection_generation
        manual_generation = self._manual_generation
        self._control_waiting.set()
        self.lbl_status.config(text="Dang xu ly lenh... STOP All van san sang.")
        def work():
            try:
                with self._motion_lock:
                    if (not self.is_connected or self._connection_transition.is_set()
                            or generation != self._connection_generation
                            or (not stop_command and not allow_sequence and manual_generation != self._manual_generation)
                            or (not allow_sequence and self._sequence_busy())):
                        if on_cancel is not None:
                            on_cancel()
                        return
                    if not stop_command:
                        self._manual_stop.clear()
                        if not allow_sequence:
                            self.sequence_stop_flag.clear()
                    try:
                        fn()
                    except Exception as e:
                        if not getattr(e, "_motor_cleanup_done", False):
                            e = self._manual_cleanup(e, dict(self.motors))
                        self._ui(lambda e=e: (
                            self.btn_teach_mode.config(text="Bat Teaching (MIT)", bg="#9C27B0"),
                            self.lbl_status.config(text=f"LOI DIEU KHIEN: {e}")))
                        if on_cancel is not None:
                            on_cancel()
            finally:
                if stop_command:
                    self._manual_stop_pending.clear()
                with self._manual_threads_lock:
                    self._manual_threads.discard(threading.current_thread())
                    if not self._manual_threads:
                        self._control_waiting.clear()
        worker = threading.Thread(target=work, daemon=True)
        with self._manual_threads_lock:
            self._manual_threads.add(worker)
        worker.start()
        return True

    def _enable_position_hold(self, motors):
        errors = self._stop_sequence_motors(motors, self._sequence_command_lock)
        if errors:
            self._latch_stop_error(errors)
            raise RuntimeError("STOP truoc ENABLE chua xac nhan: " + " | ".join(errors))
        for mid, drv in motors.items():
            if self._control_cancelled():
                return False
            mode = drv.read_param_u8(REG_RUN_MODE, timeout=0.10)
            if mode != RUN_MODE_POSITION and not drv.write_mode_stopped(RUN_MODE_POSITION, self._manual_stop):
                raise RuntimeError(f"M{mid}: PP mode not confirmed; ENABLE not sent")
        for mid, drv in motors.items():
            position = None
            for attempt in range(COMMAND_VERIFY_ATTEMPTS):
                if self._control_cancelled():
                    return False
                requested = time.monotonic()
                position = drv.read_param(REG_MECH_POS, timeout=0.10)
                if position is not None and math.isfinite(position):
                    self._record_position(mid, position, requested)
                    break
            if position is None or not math.isfinite(position):
                raise RuntimeError(f"M{mid}: fresh HOLD position unavailable; ENABLE not sent")
            for attempt in range(COMMAND_VERIFY_ATTEMPTS):
                if self._control_cancelled():
                    return False
                # All axes are confirmed stopped: retry this same HOLD only
                # before ENABLE, never a moving PP target.
                drv.write_param_float(REG_LOC_REF, position, wait=0.009)
                actual = drv.read_param(REG_LOC_REF, timeout=0.10)
                if (actual is not None and math.isfinite(actual)
                        and abs(math.degrees(actual-position)) <= HOLD_REF_TOL_DEG):
                    break
            else:
                raise RuntimeError(f"M{mid}: HOLD not confirmed; ENABLE not sent")
        for mid, drv in motors.items():
            if self._control_cancelled() or not drv.enable_verified(self._manual_stop):
                return False
            sequence_log(f"MANUAL_ENABLE_CONFIRMED M{mid}")
        return True

    def cmd_enable_all(self):
        if self._reject_during_sequence():
            return
        if not self.is_connected:
            return
        self.is_teaching = False

        def _work():
            motors = dict(self.motors)
            self.is_teaching = False
            # Keep background telemetry from interleaving with manual mode /
            # HOLD / ENABLE confirmation. A displayed success needs all axes.
            with self._telemetry_io_lock:
                try:
                    if not self._enable_position_hold(motors):
                        raise RuntimeError("ENABLE cancelled")
                except Exception as exc:
                    raise self._manual_cleanup(exc, motors) from exc
            self._ui(lambda: self.lbl_status.config(text=f"Da xac nhan bat luc {motor_ids_text(motors)} (Position mode)."))
            self._teaching_fully_off.set()
        self._start_control_worker(_work)

    def cmd_disable_all(self):
        self._ensure_control_state()
        if self._manual_stop_pending.is_set():
            self.lbl_status.config(text="Dang STOP tat ca motor; hay cho xac nhan.")
            return
        self._manual_generation += 1
        self._manual_stop.set()
        self._jog_pressed.clear()
        self._jog_motor_id = None
        self.is_teaching = False
        if self._sequence_busy():
            self.toggle_run_sequence()
            return
        if not self.is_connected:
            return
        self.is_playing = False
        self.sequence_stop_flag.set()
        self._manual_stop_pending.set()
        self.lbl_status.config(text="Dang STOP tat ca motor...")

        def _work():
            with self._telemetry_io_lock:
                errors = self._stop_sequence_motors(dict(self.motors), self._sequence_command_lock)
                self.is_teaching = False
            self._teaching_fully_off.set()
            if errors:
                self._latch_stop_error(errors)
                raise RuntimeError("STOP CHUA XAC NHAN: " + " | ".join(errors))
            self._motion_blocked.clear()
            self._ui(lambda: (
                self.btn_teach_mode.config(text="Bat Teaching (MIT)", bg="#9C27B0"),
                self.lbl_status.config(text=f"STOP da xac nhan {motor_ids_text(self.motors)}")))
        self._start_control_worker(_work, stop_command=True)

    def cmd_set_zero(self):
        if self._reject_during_sequence():
            return
        if not self.is_connected:
            return
        if not messagebox.askyesno("Xac nhan", "Dat vi tri hien tai lam moc 0 rad cho TAT CA motor?"):
            return
        def _work():
            self.is_teaching = False
            confirmed, failed = [], []
            with self._telemetry_io_lock:
                errors = self._stop_sequence_motors(dict(self.motors), self._sequence_command_lock)
                if errors:
                    self._latch_stop_error(errors)
                    raise RuntimeError("Khong dat zero vi STOP chua xac nhan: " + " | ".join(errors))
                for mid, drv in self.motors.items():
                    if self._control_cancelled():
                        return
                    try:
                        if not self._zero_motor_stopped(mid, drv):
                            return
                        confirmed.append(mid)
                    except (serial.SerialException, OSError):
                        raise
                    except RuntimeError as exc:
                        failed.append(str(exc))
                        sequence_log(f"ZERO_ERROR {exc}")
            self._teaching_fully_off.set()
            if failed:
                raise RuntimeError(f"ZERO da xac nhan {motor_ids_text(confirmed) or 'khong co'}; loi: " + " | ".join(failed))
            self._ui(lambda: self.lbl_status.config(text="Da dat goc 0 va ngat luc. Can day lai waypoint theo moc moi."))
        if self._start_control_worker(_work):
            self.is_teaching = False
            self.waypoints.clear()
            self._render_waypoints()

    def _zero_motor_stopped(self, mid, drv):
        """Type 6 is blocked in PP (both manuals 4.2.6). Never ENABLE here."""
        original_mode = None
        for attempt in range(COMMAND_VERIFY_ATTEMPTS):
            if self._control_cancelled():
                return False
            original_mode = drv.read_param_u8(REG_RUN_MODE, timeout=0.10)
            if original_mode in (RUN_MODE_OPERATION, RUN_MODE_POSITION, RUN_MODE_SPEED, RUN_MODE_CURRENT, RUN_MODE_POSITION_CSP):
                break
        else:
            raise RuntimeError(f"M{mid}: khong doc duoc mode truoc ZERO")
        change_mode = original_mode not in (RUN_MODE_OPERATION, RUN_MODE_POSITION_CSP)
        link_failed = False
        try:
            if change_mode and not drv.write_mode_stopped(RUN_MODE_OPERATION, self._manual_stop):
                if self._control_cancelled():
                    return False
                raise RuntimeError(f"M{mid}: mode cho ZERO chua xac nhan")
            if not drv.set_zero_verified(self._manual_stop, math.radians(HOLD_REF_TOL_DEG)):
                if self._control_cancelled():
                    return False
                raise RuntimeError(f"M{mid}: zero khong xac nhan sau khi gui lai")
            self._record_position(mid, drv._last_zero_position, time.monotonic())
            sequence_log(f"ZERO_CONFIRMED M{mid} original_mode={original_mode} position={drv._last_zero_position}")
        except (serial.SerialException, OSError):
            link_failed = True
            raise
        finally:
            # A STOP request takes priority over restoring mode. The next
            # motion command will verify its mode again before any ENABLE.
            if change_mode and not link_failed and not self._control_cancelled():
                if not drv.write_mode_stopped(original_mode, self._manual_stop):
                    self._motion_blocked.set()
                    raise RuntimeError(f"M{mid}: chua xac nhan khoi phuc mode sau ZERO; bam STOP All")
        return True

    # ======================================================================
    # TEACHING (MIT)
    # ======================================================================
    def toggle_teaching(self):
        if self._reject_during_sequence():
            return
        if not self.is_connected:
            return
        if self.is_playing:
            messagebox.showwarning("Canh bao", "Dang chay chuoi diem, khong the bat Teaching luc nay.")
            return
        if not self.is_teaching:
            if not self._teaching_fully_off.is_set():
                self.lbl_status.config(text="Teaching dang chuyen mode, hay cho hoan tat.")
                return
            self._teaching_fully_off.clear()

            def _start():
                with self._telemetry_io_lock:
                    for mid, drv in self.motors.items():
                        if self._control_cancelled():
                            raise RuntimeError("Teaching cancelled")
                        if not drv.set_mode(RUN_MODE_OPERATION):
                            raise RuntimeError(f"M{mid}: MIT mode not confirmed")
                    for mid, drv in self.motors.items():
                        if self._control_cancelled() or not drv.enable_verified(self._manual_stop):
                            raise RuntimeError("Teaching ENABLE cancelled")
                    if self._control_cancelled():
                        raise RuntimeError("Teaching cancelled")
                    self.is_teaching = True
                self._ui(lambda: (
                    self.btn_teach_mode.config(text="DUNG Teaching", bg="#E91E63"),
                    self.lbl_status.config(text="TEACHING DANG BAT: Dat tay cac truc tu do!"),
                ))
            self._start_control_worker(_start, on_cancel=self._teaching_fully_off.set)
        else:
            self._stop_teaching()

    def _stop_teaching(self, for_sequence=False):
        self.is_teaching = False
        self._teaching_fully_off.clear()

        def _stop():
            try:
                with self._telemetry_io_lock:
                    if for_sequence:
                        errors = self._stop_sequence_motors(dict(self.motors), self._sequence_command_lock)
                        if errors:
                            self._latch_stop_error(errors)
                            self.sequence_stop_flag.set()
                            raise RuntimeError("Teaching STOP failed: " + " | ".join(errors))
                    elif not self._enable_position_hold(dict(self.motors)):
                        raise RuntimeError("Teaching HOLD cancelled")
                detail = motor_ids_text(self.motors)

                self._ui(
                    lambda detail=detail:
                    (
                        self.btn_teach_mode.config(
                            text="Bat Teaching (MIT)",
                            bg="#9C27B0"
                        ),
                        self.lbl_status.config(
                            text=f"Da dung Teaching: {detail}" + (" — chuyen sang Sequence" if for_sequence else " — HOLD da xac nhan")
                        )
                    )
                )
            finally:
                self._teaching_fully_off.set()

        self._start_control_worker(_stop, allow_sequence=for_sequence,
                                   on_cancel=self._teaching_fully_off.set)



    # ======================================================================
    # JOG
    # ======================================================================
    def jog_step(self, direction):
        if self._reject_during_sequence():
            return
        if not self.is_connected:
            return
        m_id = self._get_selected_motor_id()
        drv = self.motors.get(m_id)
        if not drv:
            return
        if self.is_teaching or not self._teaching_fully_off.is_set():
            self.lbl_status.config(text="Hay dung Teaching truoc khi JOG.")
            return
        try:
            step = float(self.spin_jog_step.get()) * direction
            spd = float(self.spin_jog_spd.get())
            if not math.isfinite(step) or not math.isfinite(spd) or not 0 < abs(step) <= 90 or not 0.2 <= spd <= 8:
                raise ValueError("Buoc/toc do JOG khong hop le")
        except ValueError as exc:
            self.lbl_status.config(text=str(exc))
            return
        def _work():
            with self._telemetry_io_lock:
                motors = {m_id: drv}
                if not self._enable_position_hold(motors):
                    return
                target_deg = self._position_snapshot()[m_id]["deg"] + step
                commands = {m_id: dict(target_deg=target_deg, speed=spd, accel=SEQUENCE_ACCEL)}
                if not self._prepare_verified_waypoint(motors, commands) or self._control_cancelled():
                    return
                drv.write_param_float(REG_LOC_REF, math.radians(target_deg))
                if self._verify_targets(motors, {m_id: target_deg}):
                    raise RuntimeError(f"M{m_id}: JOG target not confirmed")
        self._start_control_worker(_work)

    def jog_cont_start(self, direction):
        if self._reject_during_sequence():
            return
        if not self.is_connected:
            return
        m_id = self._get_selected_motor_id()
        drv = self.motors.get(m_id)
        if not drv:
            return
        if self.is_teaching or not self._teaching_fully_off.is_set():
            self.lbl_status.config(text="Hay dung Teaching truoc khi JOG.")
            return
        try:
            spd = float(self.spin_jog_spd.get()) * direction
            if not math.isfinite(spd) or not 0.2 <= abs(spd) <= 8:
                raise ValueError("Toc do JOG khong hop le")
        except ValueError as exc:
            self.lbl_status.config(text=str(exc))
            return
        self._ensure_control_state()
        if self._jog_motor_id is not None:
            return
        self._jog_motor_id = m_id
        pressed = threading.Event()
        pressed.set()
        self._jog_pressed = pressed
        self.lbl_status.config(text=f"Dang JOG giu: Motor {m_id} toc do {spd:.2f} rad/s")

        def _work():
            with self._telemetry_io_lock:
                if not pressed.is_set() or self._control_cancelled():
                    return
                if not drv.set_mode(RUN_MODE_SPEED):
                    raise RuntimeError(f"M{m_id}: Speed mode not confirmed")
                drv.set_velocity(0.0, current_limit=3.0)
                if not pressed.is_set() or self._control_cancelled():
                    return
                if not drv.enable_verified(self._manual_stop):
                    return
                if pressed.is_set() and not self._control_cancelled():
                    drv.set_velocity(spd, current_limit=3.0)
        self._start_control_worker(_work)

    def jog_cont_stop(self):
        if self._reject_during_sequence():
            return
        if not self.is_connected:
            return
        self._ensure_control_state()
        self._jog_pressed.clear()
        m_id = self._jog_motor_id
        self._jog_motor_id = None
        drv = self.motors.get(m_id)
        if not drv:
            return
        self.lbl_status.config(text=f"Da nha chuot - Khoa giu goc Motor {m_id}")

        def _work():
            with self._telemetry_io_lock:
                drv.set_velocity(0.0)
                if not self._enable_position_hold({m_id: drv}):
                    return
        self._start_control_worker(_work, priority=True)

    def _on_focus_out(self, event):
        if getattr(self, "_jog_motor_id", None) is None:
            return
        def check_focus():
            if self._jog_motor_id is not None and self.root.focus_displayof() is None:
                self.jog_cont_stop()
        self.root.after_idle(check_focus)

    # ======================================================================
    # DAY DIEM & CHAY CHUOI tren PC, qua cung ket noi serial
    # ======================================================================
    def save_current_point(self):
        if not self.is_connected:
            self.lbl_status.config(text="Can ket noi va co feedback goc truoc khi luu diem.")
            return
        pt_id = max((wp["id"] for wp in self.waypoints), default=0) + 1
        snapshot = self._position_snapshot()
        now = time.monotonic()
        missing = [cfg["id"] for cfg in self.motor_config
                   if (not snapshot[cfg["id"]]["seq"]
                       or not math.isfinite(snapshot[cfg["id"]]["deg"])
                       or not 0 <= now - snapshot[cfg["id"]]["updated"] <= POSITION_MAX_AGE_S)]
        if missing:
            self.lbl_status.config(text=f"Chua luu diem: thieu goc moi cua {motor_ids_text(missing)}")
            return
        targets = {cfg["id"]: round(snapshot[cfg["id"]]["deg"], 2) for cfg in self.motor_config}
        delay_s = 0.5
        self.waypoints.append({"id": pt_id, "targets": targets, "delay": delay_s})

        row_vals = [pt_id] + [f"{targets[cfg['id']]} deg" for cfg in self.motor_config] + [delay_s]
        self.tree_wp.insert("", "end", values=row_vals)

        detail = ", ".join(f"M{cfg['id']}={targets[cfg['id']]}" for cfg in self.motor_config)
        self.lbl_status.config(text=f"Da luu diem #{pt_id}: {detail}")

    def clear_points(self):
        self.waypoints.clear()
        for row in self.tree_wp.get_children():
            self.tree_wp.delete(row)
        self.lbl_status.config(text="Da xoa toan bo diem.")

    def toggle_run_sequence(self):
        if self._connection_transition.is_set() or self._closing:
            self.sequence_stop_flag.set()
            self.lbl_status.config(text="Dang dung worker/dong COM; chuoi da duoc yeu cau dung.")
            return
        # STOP must work even when the editable waypoint list has been cleared.
        worker_alive = self._sequence_thread is not None and self._sequence_thread.is_alive()
        if self.is_playing or self.sequence_running_flag.is_set() or worker_alive:
            self.sequence_stop_flag.set()
            self.lbl_status.config(text="DANG DUNG CHUOI...")
            if self._sequence_stop_thread is None or not self._sequence_stop_thread.is_alive():
                motors = dict(self.motors)
                worker = self._sequence_thread

                def _emergency_stop():
                    errors = self._stop_sequence_motors(motors, self._sequence_command_lock)
                    if worker is not None:
                        worker.join()
                    if errors:
                        self._latch_stop_error(errors)
                        self._ui(lambda errors=errors: self.lbl_status.config(
                            text="LOI GUI STOP: " + " | ".join(errors)
                        ))

                self._sequence_stop_thread = threading.Thread(target=_emergency_stop, daemon=True)
                self._sequence_stop_thread.start()
            return
        if self._sequence_stop_thread is not None and self._sequence_stop_thread.is_alive():
            self.lbl_status.config(text="Dang hoan tat STOP, hay cho worker ket thuc.")
            return
        if self._connection_transition.is_set() or not self.is_connected:
            self.lbl_status.config(text="COM chua san sang. Hay ket noi lai truoc khi chay chuoi.")
            return
        self._ensure_control_state()
        if self._motion_blocked.is_set():
            self.lbl_status.config(text="STOP CHUA XAC NHAN. Bam STOP All truoc khi chay chuoi.")
            return
        if not self.waypoints:
            return
        if not self.is_teaching and not self._teaching_fully_off.is_set():
            self.lbl_status.config(text="Teaching dang chuyen mode, hay cho hoan tat.")
            return

        # Capture Tk values and route on the GUI thread, before starting work.
        try:
            speed_input = float(self.spin_jog_spd.get())
            if not math.isfinite(speed_input):
                raise ValueError("Toc do phai la so huu han")
            nominal_v = max(0.2, min(8.0, speed_input))
            route = [
                {"id": wp["id"], "targets": dict(wp["targets"]),
                 "delay": float(wp.get("delay", 0.5))}
                for wp in self.waypoints
            ]
            expected_ids = {cfg["id"] for cfg in self.motor_config}
            for wp in route:
                if set(wp["targets"]) != expected_ids:
                    raise ValueError(f"Diem #{wp['id']} phai co du {motor_ids_text(sorted(expected_ids))}")
        except (TypeError, ValueError, KeyError) as e:
            self.lbl_status.config(text=f"Khong the chay chuoi: {e}")
            return
        self.is_playing = True
        self._ensure_control_state()
        self._manual_generation += 1
        self._manual_stop.set()
        self._jog_pressed.clear()
        self._jog_motor_id = None
        self.sequence_stop_flag.clear()
        self.sequence_running_flag.set()
        if self.is_teaching:
            self.lbl_status.config(text="Dang tat Teaching truoc khi chay chuoi...")
            self._stop_teaching(for_sequence=True)
        self.btn_run_seq.config(text="DUNG CHUOI", bg="#E91E63")
        self._sequence_thread = threading.Thread(
            target=self._run_sequence_worker,
            args=(route, nominal_v, dict(self.motors)), daemon=True,
        )
        self._sequence_thread.start()

    @staticmethod
    def _stop_sequence_motors(motors, command_lock=None):
        """Attempt STOP for every motor, even if one serial write fails."""
        errors = StopErrors()
        with command_lock or threading.Lock():
            for mid, drv in motors.items():
                try:
                    drv.disable(clear_fault=False)
                except Exception as e:
                    errors.append(f"M{mid}: {type(e).__name__}: {e}")
                    if isinstance(e, (serial.SerialException, OSError)):
                        errors.link_error = e
            # Send STOP to the entire group before waiting for any confirmation.
            # A missing reply on M1 must not delay the first STOP of M5/M6.
            for mid, drv in motors.items():
                try:
                    drv.disable_verified()
                except Exception as e:
                    errors.append(f"M{mid}: STOP confirmation: {type(e).__name__}: {e}")
                    if isinstance(e, (serial.SerialException, OSError)):
                        errors.link_error = e
        return errors

    def _wait_for_positions(self, motors, after=0.0, require_stopped=False):
        deadline = time.monotonic() + ARRIVE_TIMEOUT_S
        while not self.sequence_stop_flag.is_set():
            if self._sequence_owns_telemetry.is_set():
                self._poll_sequence_states(motors)
            snapshot = self._position_snapshot()
            now = time.monotonic()
            stopped = not require_stopped or all(
                snapshot[mid].get("vel_requested", 0.0) > after
                and 0 <= now - snapshot[mid].get("vel_updated", 0.0) <= POSITION_MAX_AGE_S
                and math.isfinite(snapshot[mid]["vel"])
                and abs(snapshot[mid]["vel"]) <= ARRIVE_VEL_TOL_RAD_S for mid in motors)
            if stopped and all(snapshot[mid]["seq"] > 0
                   and snapshot[mid]["requested"] > after
                   and 0 <= now - snapshot[mid]["updated"] <= POSITION_MAX_AGE_S
                   and math.isfinite(snapshot[mid]["deg"]) for mid in motors):
                return snapshot
            if now >= deadline:
                raise RuntimeError(f"Khong co telemetry moi/xac nhan dung cua du {motor_ids_text(motors)}")
            time.sleep(0.02)
        return None

    def _send_waypoint(self, motors, commands, prepare=True):
        with self._sequence_command_lock:
            if self.sequence_stop_flag.is_set():
                return None
            return RobStrideDriver.send_position_batch(
                motors, commands, self.sequence_stop_flag, prepare=prepare,
            )

    @staticmethod
    def _float_command_matches(actual, expected):
        return (actual is not None and math.isfinite(actual)
                and math.isclose(actual, expected, rel_tol=2e-6, abs_tol=1e-6))

    def _prepare_verified_waypoint(self, motors, commands):
        """Confirm V/A before targets, respecting each manual's permissions."""
        for mid, drv in motors.items():
            for reg, value in ((REG_VEL_MAX, commands[mid]["speed"]),
                               (REG_ACC_SET, commands[mid]["accel"])):
                confirmed = False
                for attempt in range(COMMAND_VERIFY_ATTEMPTS):
                    with self._sequence_command_lock:
                        if self.sequence_stop_flag.is_set():
                            return False
                        if drv.profile.name == PROFILE_EL05.name:
                            # EL05 260112 lists these registers W, not W/R.
                            confirmed = drv.write_param_float_ack(reg, value)
                            actual = "Type2 ACK" if confirmed else None
                            time.sleep(0.009)
                        else:
                            drv.write_param_float(reg, value, wait=0.009)
                            if self.sequence_stop_flag.is_set():
                                return False
                            actual = drv.read_param(reg, timeout=0.10)
                            confirmed = self._float_command_matches(actual, value)
                    if confirmed:
                        confirmed = True
                        break
                    sequence_log(f"VERIFY_RETRY M{mid} reg=0x{reg:04X} expected={value} actual={actual}")
                    time.sleep(0.01)
                if not confirmed:
                    raise RuntimeError(f"M{mid}: V/A register 0x{reg:04X} not confirmed; target not sent")
        return True

    def _verify_targets(self, motors, targets, tolerance_deg=TARGET_REF_TOL_DEG):
        """Read every loc_ref after grouped TX; retry reads, not motion writes."""
        pending = dict(motors)
        for attempt in range(COMMAND_VERIFY_ATTEMPTS):
            for mid, drv in list(pending.items()):
                with self._sequence_command_lock:
                    if self.sequence_stop_flag.is_set():
                        return None
                    actual = drv.read_param(REG_LOC_REF, timeout=0.06)
                if (actual is not None and math.isfinite(actual)
                        and abs(actual - math.radians(targets[mid])) <= math.radians(tolerance_deg)):
                    del pending[mid]
                else:
                    delta = math.degrees(actual) - targets[mid] if actual is not None else None
                    sequence_log(f"TARGET_UNCONFIRMED M{mid} expected={targets[mid]}deg actual={actual}rad delta_deg={delta} tolerance_deg={tolerance_deg} read={attempt+1}")
            if not pending:
                return []
            time.sleep(0.01)
        return list(pending)

    def _enable_sequence_motors(self, motors):
        for mid, drv in motors.items():
            with self._sequence_command_lock:
                if self.sequence_stop_flag.is_set():
                    return False
                if not drv.enable_verified(self.sequence_stop_flag):
                    return False
            sequence_log(f"ENABLE_CONFIRMED M{mid}")
        return True

    def _stage_hold_after_stop(self, motors):
        """Replace old targets with measured HOLD while motors are disabled.

        Otherwise re-enabling for recovery could restart the previous move
        before the new common plan is ready. Require fresh stopped telemetry.
        """
        after = time.monotonic()
        deadline = after + ARRIVE_TIMEOUT_S
        while not self.sequence_stop_flag.is_set():
            self._poll_sequence_states(motors)
            snapshot = self._position_snapshot()
            now = time.monotonic()
            stopped = all(snapshot[mid]["requested"] > after
                          and snapshot[mid].get("vel_requested", 0.0) > after
                          and 0 <= now - snapshot[mid]["updated"] <= POSITION_MAX_AGE_S
                          and 0 <= now - snapshot[mid].get("vel_updated", 0.0) <= POSITION_MAX_AGE_S
                          and math.isfinite(snapshot[mid]["deg"])
                          and math.isfinite(snapshot[mid]["vel"])
                          and abs(snapshot[mid]["vel"]) <= ARRIVE_VEL_TOL_RAD_S
                          for mid in motors)
            if stopped:
                hold = {mid: snapshot[mid]["deg"] for mid in motors}
                commands = {mid: dict(target_deg=value, speed=SEQUENCE_MIN_SPEED,
                                      accel=SEQUENCE_MIN_ACCEL) for mid, value in hold.items()}
                if self._send_waypoint(motors, commands, prepare=False) is None:
                    return False
                missing = self._verify_targets(motors, hold, tolerance_deg=HOLD_REF_TOL_DEG)
                if missing is None:
                    return False
                if missing:
                    raise RuntimeError(f"HOLD not confirmed after STOP on {missing}; no re-enable")
                sequence_log(f"HOLD_STAGED_WHILE_DISABLED {hold}")
                return True
            if now >= deadline:
                raise RuntimeError("Cannot confirm all motors stopped for resync; no re-enable")
            time.sleep(0.02)
        return False

    def _dispatch_verified_waypoint(self, motors, targets, nominal_v, pid):
        """Recover an unconfirmed target as a whole group, never one late axis."""
        for attempt in range(WAYPOINT_DISPATCH_ATTEMPTS):
            initial = self._wait_for_positions(motors, after=time.monotonic(), require_stopped=True)
            if initial is None:
                return None
            current = {mid: initial[mid]["deg"] for mid in motors}
            commands, common_time = plan_waypoint(current, targets, motors, nominal_v)
            if common_time >= SEQUENCE_TIMEOUT_S:
                raise RuntimeError(f"Diem #{pid}: thoi gian quy dao vuot timeout")
            if not self._prepare_verified_waypoint(motors, commands):
                return None
            sequence_log(f"WAYPOINT #{pid} dispatch={attempt+1} current={current} commands={commands} planned_time={common_time:.4f}s")
            sent = self._send_waypoint(motors, commands, prepare=False)
            if sent is None:
                return None
            missing = self._verify_targets(motors, targets)
            if missing is None:
                return None
            if not missing:
                sequence_log(f"TARGETS_CONFIRMED #{pid} verification_time={time.monotonic()-sent:.4f}s")
                return sent, commands, common_time
            # Other axes may already be moving. Stop all before retry/replan,
            # so one late motor cannot chase the remaining trajectory alone.
            sequence_log(f"RESYNC #{pid} unconfirmed={missing}; STOP all before retry")
            errors = self._stop_sequence_motors(motors, self._sequence_command_lock)
            if errors:
                self._latch_stop_error(errors)
                raise RuntimeError("Cannot resync after STOP errors: " + " | ".join(errors))
            if attempt + 1 == WAYPOINT_DISPATCH_ATTEMPTS:
                raise RuntimeError(f"Diem #{pid}: targets not confirmed on motors {missing}; STOP all")
            if not self._stage_hold_after_stop(motors):
                return None
            if not self._enable_sequence_motors(motors):
                return None
        return None

    def _run_sequence_worker(self, route, nominal_v, motors):
        """Execute every waypoint: TX, fresh confirmation, HOLD, then next."""
        total_points, completed = len(route), 0
        done, error = False, None
        # Manual workers already in progress must finish before Sequence owns
        # motion. Queued manual workers recheck the sequence reservation.
        while not self._teaching_fully_off.wait(timeout=0.05):
            pass
        self._motion_lock.acquire()
        try:
            self._sequence_owns_telemetry.set()
            # Drain the telemetry read currently in flight before mode/targets.
            with self._telemetry_io_lock:
                pass
            if not route or set(motors) != {cfg["id"] for cfg in self.motor_config}:
                raise RuntimeError(f"Chuoi phai co waypoint va du {motor_ids_text(cfg['id'] for cfg in self.motor_config)}")
            if self.sequence_stop_flag.is_set():
                return
            self._ui(lambda: self.lbl_status.config(text=f"STOP -> PP Mode -> ENABLE {motor_ids_text(motors)}..."))
            # All STOPs, then confirm Reset on all axes before mode writes.
            for action in (lambda drv: drv.disable(clear_fault=True),
                           lambda drv: drv.disable_verified(),
                           lambda drv: drv.write_param_u8(REG_RUN_MODE, RUN_MODE_POSITION, wait=0.03)):
                for drv in motors.values():
                    with self._sequence_command_lock:
                        if self.sequence_stop_flag.is_set():
                            return
                        action(drv)
            for mid, drv in motors.items():
                with self._sequence_command_lock:
                    if self.sequence_stop_flag.is_set():
                        return
                    confirmed = False
                    for attempt in range(COMMAND_VERIFY_ATTEMPTS):
                        if self.sequence_stop_flag.is_set():
                            return
                        mode = drv.read_param_u8(REG_RUN_MODE)
                        if mode == RUN_MODE_POSITION:
                            confirmed = True
                            sequence_log(f"MODE_CONFIRMED M{mid} PP=1")
                            break
                        if attempt + 1 < COMMAND_VERIFY_ATTEMPTS:
                            # Rewrite only while all motors are still disabled.
                            sequence_log(f"MODE_VERIFY_RETRY M{mid} actual={mode}")
                            drv.write_param_u8(REG_RUN_MODE, RUN_MODE_POSITION, wait=0.03)
                        time.sleep(0.02)
                    if not confirmed:
                        raise RuntimeError(f"M{mid}: khong xac nhan duoc PP mode 1")
            # Replace any stale PP target before ENABLE can resume it. V/A
            # must be configured at rest (both 260112 manuals, section 4.3.5).
            if not self._stage_hold_after_stop(motors):
                return
            if not self._enable_sequence_motors(motors):
                return

            sequence_log(f"START PP route={route} nominal_v={nominal_v}")
            for wp in route:
                if self.sequence_stop_flag.is_set():
                    break
                pid, targets, delay_s = wp["id"], wp["targets"], wp["delay"]
                if not math.isfinite(delay_s) or delay_s < 0:
                    raise ValueError(f"Diem #{pid}: delay khong hop le")
                self._ui(lambda pid=pid: self.lbl_status.config(text=f"Diem #{pid}: xac nhan V/A va target..."))
                started = time.monotonic()
                dispatched = self._dispatch_verified_waypoint(motors, targets, nominal_v, pid)
                if dispatched is None:
                    return
                command_time, commands, common_time = dispatched
                tracker = WaypointConfirmation(targets, command_time, self._position_snapshot(), require_stopped=True)
                last_status = started - 1.0

                while not self.sequence_stop_flag.is_set():
                    self._poll_sequence_states(motors)
                    if self.sequence_stop_flag.is_set():
                        break
                    now = time.monotonic()
                    snapshot = self._position_snapshot()
                    if now - started >= SEQUENCE_TIMEOUT_S:
                        detail = " | ".join(
                            f"M{mid}: {snapshot[mid]['deg']:.2f}/{targets[mid]:.2f} deg, "
                            f"v={snapshot[mid]['vel']:.3f}, pos_age={now-snapshot[mid]['updated']:.2f}s, "
                            f"vel_age={now-snapshot[mid].get('vel_updated', 0):.2f}s" for mid in motors)
                        raise RuntimeError(f"TIMEOUT tai diem #{pid} — khong bo qua waypoint. {detail}")
                    # Motor firmware interpolates each PP move. Verify fresh
                    # position AND low velocity on every axis before HOLD.
                    if tracker.observe(snapshot, now):
                        completed += 1
                        sequence_log(f"ARRIVED #{pid} snapshot={snapshot}")
                        self._ui(lambda pid=pid, completed=completed: self.lbl_status.config(
                            text=f"DA DEN diem #{pid} ({completed}/{total_points})"))
                        break
                    # The motor's loc_ref has already been verified. Do not
                    # repeatedly reset a moving/accepted PP target just because
                    # position feedback is late or the load is slow to settle.
                    if now - last_status >= 1.0:
                        last_status = now
                        detail = " | ".join(f"M{mid}:{snapshot[mid]['deg']:.2f}/{targets[mid]:.2f} deg" for mid in motors)
                        self._ui(lambda pid=pid, detail=detail, common_time=common_time: self.lbl_status.config(
                            text=f"Dang toi diem #{pid} | T muc tieu {common_time:.3f}s | {detail}"))
                    time.sleep(0.02)
                if self.sequence_stop_flag.is_set():
                    break
                self._ui(lambda pid=pid, delay_s=delay_s: self.lbl_status.config(text=f"HOLD diem #{pid}: {delay_s:.2f}s"))
                hold_end = time.monotonic() + delay_s
                while not self.sequence_stop_flag.is_set() and time.monotonic() < hold_end:
                    self._poll_sequence_states(motors)
                    time.sleep(min(0.02, max(0.0, hold_end - time.monotonic())))
            done = not self.sequence_stop_flag.is_set() and completed == total_points
        except Exception as e:
            error = f"{type(e).__name__}: {e}"
            self.sequence_stop_flag.set()
            sequence_log(f"ERROR {error} snapshot={self._position_snapshot()}")
            if isinstance(e, (serial.SerialException, OSError)):
                self._handle_serial_fault(e)
        finally:
            stop_errors = []
            if error is not None or self.sequence_stop_flag.is_set():
                stop_errors = self._stop_sequence_motors(motors, self._sequence_command_lock)
                if stop_errors:
                    self._latch_stop_error(stop_errors)
            sequence_log(f"FINISH completed={completed}/{total_points} done={done} stop_errors={stop_errors}")
            self._sequence_owns_telemetry.clear()
            self._motion_lock.release()
            self.sequence_running_flag.clear()

            def finish():
                self.is_playing = False
                self.btn_run_seq.config(text="> CHAY CHUOI QUY DAO", bg="#009688")
                if error is not None:
                    text = f"LOI CHAY CHUOI: {error} | " + ("STOP CHUA XAC NHAN" if stop_errors else f"STOP da xac nhan {motor_ids_text(motors)}")
                elif done and not self.sequence_stop_flag.is_set():
                    text = f"HOAN THANH 100% CHUOI ({completed}/{total_points}) — HOLD DIEM CUOI"
                else:
                    text = f"Da dung chuoi ({completed}/{total_points} diem) — " + ("STOP CHUA XAC NHAN." if stop_errors else "STOP da xac nhan.")
                if stop_errors:
                    text += " | LOI GUI STOP: " + " | ".join(stop_errors)
                self.lbl_status.config(text=text)
            self._ui(finish)

    # ======================================================================
    def on_close(self):
        if self._closing:
            return
        self._closing = True
        if self._connection_transition.is_set():
            return  # The connection completion destroys Tk after cleanup.
        self._begin_disconnect()


def launch_gui():
    root = tk.Tk()
    app = MotorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    launch_gui()
