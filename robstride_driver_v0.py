#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Driver v0 - RobStride RS00 / EL05 CAN Driver (qua module USB-CAN kieu AT-command)
===================================================================================
Tuong ung task B3: "Dung bench 1 motor: viet lop driver CAN cho RS00/EL05
(doc/ghi thong so; che do vi tri, van toc, mo-men, MIT kp/kd/tau)"
San pham ban giao: Driver v0 + tai lieu lenh

Ban nay doi chieu lai voi RS00User_Manual260112.pdf va EL05User_Manual260112.pdf
(RobStride Dynamics). Moi dia chi thanh ghi, dai scale MIT, trinh tu lenh deu
lay truc tiep tu 2 manual do - khong con la suy doan.

SUA LOI CHINH O BAN NAY: GUI Tkinter goi driver.read_velocity_rad() va
driver.read_voltage_v() nhung driver cu khong co 2 ham nay -> AttributeError
bi nuot boi "except Exception: pass" trong _telemetry_loop -> KHONG mot label
nao duoc cap nhat (ke ca vi tri, vi lenh doc van toc nam truoc lenh update
label vi tri trong code GUI). Da them 2 ham nay, dung dung thanh ghi
mechVel (0x701B) va VBUS (0x701C) co san trong manual.

PHAM VI v0 (giai doan bench, 1 motor):
    - Lop RobStrideDriver doc lap, KHONG phu thuoc socket/GUI.
    - Doc/ghi thong so chung (read_param/write_param) - dung lai cho B4.
    - 4 che do: Position PP, Position CSP, Velocity, Current/Torque.
    - Che do MIT (kp, kd, tau feedforward) - da doi chieu dung cau truc khung
      voi manual, dai scale khac nhau giua RS00/EL05 (xem MotorProfile).
    - Ho tro ca RS00 va EL05 qua PROFILE_RS00 / PROFILE_EL05.

KHONG thuoc pham vi v0 (de danh cho B8 - Driver v1 + API cho Du):
    Server TCP, GUI, vong lap feedback da luong lien tuc.
"""

import math
import struct
import threading
import time

import serial


# =========================================================================
# 1. HANG SO GIAO THUC (doi chieu manual, muc 4.1 cua ca 2 file)
# =========================================================================

COMM_GET_ID = 0
COMM_MOTION = 1       # "Operation control mode motor control instruction" = MIT
COMM_FEEDBACK = 2     # Khung phan hoi, TU DONG tra ve sau MOI lenh dieu khien
COMM_ENABLE = 3
COMM_STOP = 4
COMM_SET_ZERO = 6
COMM_READ_PARAM = 17
COMM_WRITE_PARAM = 18

# --- Dia chi thanh ghi (giong het nhau giua RS00 va EL05, theo manual) ---
REG_RUN_MODE = 0x7005
REG_IQ_REF = 0x7006
REG_LIMIT_TORQUE = 0x700B
REG_SPD_REF = 0x700A
REG_LOC_REF = 0x7016
REG_LIMIT_SPD = 0x7017    # CHI ap dung cho CSP mode ("Location mode (CSP) speed limit")
REG_LIMIT_CUR = 0x7018
REG_MECH_POS = 0x7019     # chi doc
REG_IQF = 0x701A          # chi doc
REG_MECH_VEL = 0x701B     # chi doc - "Speed of the load", -33..33 rad/s (RS00)
REG_VBUS = 0x701C         # chi doc - Bus voltage (V)
REG_ACC_RAD = 0x7022      # Velocity mode acceleration
REG_VEL_MAX = 0x7024      # PP mode speed (KHONG phai limit_spd)
REG_ACC_SET = 0x7025      # PP mode acceleration

RUN_MODE_OPERATION = 0    # = MIT mode, mac dinh khi vua cap nguon (manual 4.3.1)
RUN_MODE_POSITION = 1     # PP mode
RUN_MODE_SPEED = 2
RUN_MODE_CURRENT = 3
RUN_MODE_POSITION_CSP = 5


class MotorProfile:
    """
    Thong so rieng tung dong motor, lay truc tiep tu datasheet (muc 1.3
    "Electrical characteristic") va bang thanh ghi (muc 4.1.14) cua tung manual.
    """

    def __init__(self, name, kt, i_max, v_max, t_max, p_range=(-12.57, 12.57)):
        self.name = name
        self.kt = kt            # Nm/Arms - torque constant (datasheet muc 1.3)
        self.i_max = i_max      # A - dai iq_ref / limit_cur (0x7006 / 0x7018)
        self.v_max = v_max      # rad/s - dai scale van toc trong khung MIT (comm_type 1/2)
        self.t_max = t_max      # Nm  - dai scale mo-men trong khung MIT (comm_type 1/2)
        self.p_range = p_range  # rad - dai scale vi tri trong khung MIT (giong nhau 2 dong)


# RS00: Kt=1.48 Nm/Arms, iq_ref +-16A, MIT: vel +-33 rad/s, torque +-14 Nm
PROFILE_RS00 = MotorProfile(name="RS00", kt=1.48, i_max=16.0, v_max=33.0, t_max=14.0)

# EL05: Kt=0.94 Nm/Arms, iq_ref +-11A, MIT: vel +-50 rad/s, torque +-6 Nm theo
# bang comm_type 2 va limit_torque "0 to 6Nm". LUU Y: code mau C trong chinh
# manual EL05 lai dinh nghia T_MIN=-5.5f/T_MAX=5.5f - mau thuan noi bo. Dung
# 6.0 (khop "Peak load: 6 N.m") nhung NEN kiem chung tren bench truoc khi
# dung cho B5.
PROFILE_EL05 = MotorProfile(name="EL05", kt=0.94, i_max=11.0, v_max=50.0, t_max=6.0)


# =========================================================================
# 2. LOP DRIVER
# =========================================================================

class RobStrideDriver:
    def __init__(self, port, profile, motor_id=0x01, host_id=0xFD, baudrate=921600,
                 _shared_ser=None, _shared_lock=None):
        """
        _shared_ser / _shared_lock: dung khi NHIEU motor chung 1 bus CAN vat
        ly (nhieu RobStrideDriver tren cung 1 cong serial). Truyen 2 tham so
        nay (lay tu .ser / ._lock cua driver DAU TIEN da mo thanh cong) de
        TRANH mo serial.Serial(port=...) nhieu lan tren cung 1 cong - moi lan
        mo moi them 1 nguy co module USB-CAN bi reset giua chung (nhieu chip
        USB-to-serial tu dong tao xung DTR khi co file descriptor moi mo
        vao, bat ke code co set dtr=False ngay sau do hay khong). Voi 2
        motor, mo 2 lan con "may man" chay duoc; voi 3 motor tro len, rat de
        hong het ca bus. Chi driver DAU TIEN (khong truyen 2 tham so nay)
        moi thuc su goi serial.Serial() va bat tay AT+AT.
        """
        self.profile = profile
        self.motor_id = motor_id
        self.host_id = host_id
        self.diagnostic_sink = None
        self._last_request_detail = "no request yet"
        self._last_request_statuses = []

        if _shared_ser is not None:
            self.ser = _shared_ser
            self._lock = _shared_lock if _shared_lock is not None else threading.Lock()
            return  # Cong da duoc mo va bat tay AT+AT boi driver dau tien

        self._lock = threading.Lock()
        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.03, write_timeout=0.15,
            rtscts=False, dsrdtr=False,
        )
        try:
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.05)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self._send_raw(b"AT+AT\r\n", wait=0.05)
        except Exception:
            try:
                self.ser.close()
            except Exception:
                pass
            raise

    def set_can_id(self, new_id):
        """
        Doi CAN ID cua dong co dang duoc dia chi boi self.motor_id sang
        new_id, co hieu luc NGAY (comm_type=7, manual muc 4.1 "Communication
        type 7: Set motor CAN_ID"). Dung khi vua lap them 1 motor moi ma no
        con dang o ID mac dinh cua hang (thuong la 1) va can doi sang ID
        khac de khong trung voi motor da co san tren cung bus - trung ID se
        lam CA BUS loan vi 2 thiet bi cung tra loi 1 dia chi.

        CANH BAO: goi xong, self.motor_id cua CHINH INSTANCE NAY duoc cap
        nhat theo new_id luon - nhung neu dang dung chung 1 object cho nhieu
        noi (vi du da luu trong dict `motors` cua server theo ID CU), ban
        phai tu cap nhat lai key trong dict do, driver khong tu lam thay.
        """
        COMM_SET_CAN_ID = 7
        data2 = ((new_id & 0xFF) << 8) | (self.host_id & 0xFF)
        data = [0] * 8
        frame_can_id = ((COMM_SET_CAN_ID & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (self.motor_id & 0xFF)
        id_field = (frame_can_id << 3) | 0x04
        frame = b"AT" + id_field.to_bytes(4, "big") + bytes([8]) + bytes(data) + b"\r\n"
        with self._lock:
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(0.05)
        self.motor_id = new_id

    # ---------------------------------------------------------------
    # Tang truyen thong
    # ---------------------------------------------------------------
    def _write_all_locked(self, frame):
        """Finish a short serial write without retransmitting its AT header.

        Caller owns the shared lock. Zero progress or an exception is fatal;
        retrying a whole partially written frame could corrupt the byte stream.
        The real serial port also has a bounded write_timeout.
        """
        offset = 0
        deadline = time.monotonic() + 0.15
        while offset < len(frame):
            count = self.ser.write(frame[offset:])
            if count is None or not 0 < count <= len(frame) - offset:
                raise IOError("Serial write made no progress or returned an invalid count")
            offset += count
            if offset < len(frame) and time.monotonic() >= deadline:
                raise IOError("Serial write timed out before complete AT frame")
        # pyserial's Windows flush loops until the TX queue is empty, without
        # honoring write_timeout. Bound that wait as well; do not clear TX.
        deadline = time.monotonic() + 0.15
        while self.ser.out_waiting:
            if time.monotonic() >= deadline:
                raise serial.SerialTimeoutException("Serial TX queue did not drain within 0.15s")
            time.sleep(0.001)

    def _send_raw(self, raw_bytes, wait=0.02):
        with self._lock:
            self._write_all_locked(raw_bytes)
            if wait:
                time.sleep(wait)

    def _frame(self, comm_type, data2, data):
        can_id = ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (self.motor_id & 0xFF)
        id_field = (can_id << 3) | 0x04
        return b"AT" + id_field.to_bytes(4, "big") + bytes([len(data)]) + bytes(data) + b"\r\n"

    def _send(self, comm_type, data2, data, wait=0.006):
        frame = self._frame(comm_type, data2, data)
        with self._lock:
            self._write_all_locked(frame)
            time.sleep(wait)

    # ---------------------------------------------------------------
    # Doc/ghi thong so chung - dung lai cho B4 (nhan dang tham so)
    # ---------------------------------------------------------------
    def write_param_float(self, reg_addr, value, wait=0.006):
        data = list(reg_addr.to_bytes(2, "little")) + [0, 0] + list(struct.pack("<f", float(value)))
        self._send(COMM_WRITE_PARAM, self.host_id, data, wait)

    def write_param_u8(self, reg_addr, value, wait=0.02):
        data = list(reg_addr.to_bytes(2, "little")) + [0, 0, value & 0xFF, 0, 0, 0]
        self._send(COMM_WRITE_PARAM, self.host_id, data, wait)

    def write_param_float_ack(self, reg_addr, value, timeout=0.10):
        """Type 18 write followed by this motor's documented Type 2 reply.

        EL05 lists PP V/A registers as write-only (manual 260112, p.47).
        This confirms a fault-free reply, not an echoed register value.
        """
        data = reg_addr.to_bytes(2, "little") + b"\0\0" + struct.pack("<f", float(value))
        reply = self._request_response(self._frame(COMM_WRITE_PARAM, self.host_id, data),
                                       COMM_FEEDBACK, lambda payload: True,
                                       timeout, return_frame=True)
        if reply is None:
            return False
        faults = (reply[0] >> 16) & 0x3F
        if faults:
            raise RuntimeError(f"M{self.motor_id}: parameter write feedback fault bits 0x{faults:02X}")
        return True

    def _read_param_raw(self, reg_addr, timeout=0.03):
        data = list(reg_addr.to_bytes(2, "little")) + [0] * 6
        frame = self._frame(COMM_READ_PARAM, self.host_id, data)
        reg_bytes = reg_addr.to_bytes(2, "little")
        payload = self._request_response(
            frame, COMM_READ_PARAM,
            lambda data: data[:2] == reg_bytes and data[2:4] == b"\0\0", timeout,
        )
        return payload[4:8] if payload is not None else None

    @staticmethod
    def _extract_at_frames(buf):
        """Extract complete adapter frames; retain a fragmented frame in buf."""
        frames = []
        while buf:
            start = buf.find(b"AT")
            if start < 0:
                # The final A may be the beginning of the next header.
                buf[:] = b"A" if buf[-1:] == b"A" else b""
                break
            if start:
                del buf[:start]
            if len(buf) < 7:
                break

            id_field = int.from_bytes(buf[2:6], "big")
            dlc = buf[6]
            if dlc > 8 or (id_field & 0x07) != 0x04:
                del buf[0]
                continue
            frame_size = 7 + dlc + 2
            if len(buf) < frame_size:
                break
            if buf[frame_size - 2:frame_size] != b"\r\n":
                del buf[0]
                continue

            can_id = (id_field >> 3) & 0x1FFFFFFF
            frames.append((can_id, bytes(buf[7:7 + dlc])))
            del buf[:frame_size]
        return frames

    def _request_response(self, frame, response_type, matches_payload, timeout, return_frame=False,
                          matches_id=None):
        """One request owns RX until its matching response or timeout.

        All drivers on this serial connection use the same lock. Discard bytes
        already pending before TX, then parse every complete frame until the
        requested type, source motor, destination host and payload match.
        There is no request token on the wire: an identical late reply arriving
        after TX cannot be distinguished from the new reply by this protocol.
        """
        with self._lock:
            started = time.monotonic()
            received, parsed, discarded = bytearray(), [], b""
            observed_statuses = []
            outcome = "timeout"
            try:
                pending = self.ser.in_waiting
                if pending:
                    discarded = self.ser.read(pending)
                self._write_all_locked(frame)
                deadline = time.monotonic() + timeout
                buf = bytearray()
                while time.monotonic() < deadline:
                    available = self.ser.in_waiting
                    if available:
                        chunk = self.ser.read(available)
                        received.extend(chunk)
                        del received[:-256]  # Bounded diagnostic sample, not the parser buffer.
                        buf.extend(chunk)
                        for can_id, payload in self._extract_at_frames(buf):
                            comm_type = (can_id >> 24) & 0x1F
                            sender_id = (can_id >> 8) & 0xFF
                            receiver_id = can_id & 0xFF
                            detail = f"id=0x{can_id:08X}/type={comm_type}/src={sender_id}/dst={receiver_id}"
                            if comm_type == COMM_FEEDBACK:
                                detail += f"/status={(can_id >> 22) & 3}/fault=0x{(can_id >> 16) & 0x3F:02X}"
                                if sender_id == self.motor_id and receiver_id == self.host_id:
                                    observed_statuses.append((can_id >> 22) & 3)
                                    del observed_statuses[:-16]
                            parsed.append(detail)
                            del parsed[:-16]
                            if (comm_type == response_type
                                    and sender_id == self.motor_id
                                    and receiver_id == self.host_id
                                    and (response_type != COMM_READ_PARAM or (can_id >> 16) & 0xFF == 0)
                                    and len(payload) == 8
                                    and (matches_id is None or matches_id(can_id))
                                    and matches_payload(payload)):
                                outcome = "matched"
                                return (can_id, payload) if return_frame else payload
                    else:
                        time.sleep(0.001)
            except Exception as exc:
                outcome = f"exception:{type(exc).__name__}:{exc}"
                raise
            finally:
                self._last_request_statuses = observed_statuses
                self._last_request_detail = (
                    f"M{self.motor_id} outcome={outcome} elapsed_ms={(time.monotonic()-started)*1000:.1f} "
                    f"timeout_ms={timeout*1000:.0f} tx={frame.hex()} "
                    f"discarded={discarded[-64:].hex() or '-'} rx={received.hex() or '-'} parsed={parsed}"
                )
                command_type = (int.from_bytes(frame[2:6], "big") >> 27) & 0x1F
                if self.diagnostic_sink is not None and command_type in (COMM_ENABLE, COMM_STOP, COMM_WRITE_PARAM):
                    try:
                        self.diagnostic_sink("COMMAND_IO " + self._last_request_detail)
                    except Exception:
                        pass  # Diagnostic output must not prevent motor cleanup.
        return None

    def read_param(self, reg_addr, timeout=0.02):
        """Doc 1 thong so float32. None neu khong co phan hoi hop le."""
        raw = self._read_param_raw(reg_addr, timeout)
        return struct.unpack("<f", raw)[0] if raw else None

    def read_param_u8(self, reg_addr, timeout=0.02):
        raw = self._read_param_raw(reg_addr, timeout)
        return raw[0] if raw else None

    # ---------------------------------------------------------------
    # Lenh dieu khien chung
    # ---------------------------------------------------------------
    def enable(self):
        self._send(COMM_ENABLE, self.host_id, [0] * 8, 0.03)

    def enable_verified(self, stop_event, retries=3):
        """Existing Type 3 command, confirmed by this motor's Type 2 status.

        Mode status 2 means Run; the six fault bits must be zero. A missing
        response is retried a bounded number of times, never silently enabled.
        """
        frame = self._frame(COMM_ENABLE, self.host_id, [0] * 8)
        statuses = []
        for attempt in range(retries):
            if stop_event.is_set():
                return False
            reply = self._request_response(frame, COMM_FEEDBACK, lambda data: True,
                                           0.10, return_frame=True,
                                           matches_id=lambda cid: ((cid >> 16) & 0x3F) != 0
                                           or ((cid >> 22) & 3) == 2)
            statuses.extend(self._last_request_statuses)
            if reply is not None:
                can_id, _ = reply
                faults = (can_id >> 16) & 0x3F
                status = (can_id >> 22) & 0x03
                if faults:
                    raise RuntimeError(f"M{self.motor_id}: ENABLE feedback fault bits 0x{faults:02X}")
                if status == 2:
                    return True
            time.sleep(0.01)
        raise RuntimeError(f"M{self.motor_id}: no confirmed ENABLE/Run feedback; "
                           f"received_statuses={statuses}; {self._last_request_detail}")

    def disable(self, clear_fault=False):
        data = [0] * 8
        if clear_fault:
            data[0] = 1
        self._send(COMM_STOP, self.host_id, data, 0.02)

    def disable_verified(self, retries=3):
        """Confirm Type 4 by this motor's Type 2 Reset status; preserve faults.

        Cleanup must run even when the application's stop event is set.
        """
        frame = self._frame(COMM_STOP, self.host_id, [0] * 8)
        for attempt in range(retries):
            reply = self._request_response(frame, COMM_FEEDBACK, lambda data: True,
                                           0.10, return_frame=True,
                                           matches_id=lambda cid: ((cid >> 22) & 3) == 0)
            if reply is not None and ((reply[0] >> 22) & 0x03) == 0:
                return True
            time.sleep(0.01)
        raise RuntimeError(f"M{self.motor_id}: no confirmed STOP/Reset feedback")

    def set_zero(self):
        self._send(COMM_SET_ZERO, self.host_id, [1] + [0] * 7, 0.05)

    def set_mode(self, run_mode, retries=3):
        """Precautions #2 trong manual: khong doi mode khi dang chay - phai dung truoc."""
        for attempt in range(retries):
            self.disable(clear_fault=(attempt == 0))
            self.write_param_u8(REG_RUN_MODE, run_mode, wait=0.03)
            time.sleep(0.03)
            if self.read_param_u8(REG_RUN_MODE, timeout=0.10) == run_mode:
                return True
            time.sleep(0.05)
        return False

    # ---------------------------------------------------------------
    # Cac che do, dung trinh tu muc 4.3 cua manual
    # ---------------------------------------------------------------
    def set_position(self, angle_deg, speed_limit=None, accel=None):
        """
        PP mode (run_mode=1). Theo muc 4.3.5: ghi vel_max (0x7024) roi
        acc_set (0x7025) - KHONG dung limit_spd (do la cua CSP mode, xem
        set_position_csp). Giu nguyen ten tham so speed_limit de tuong
        thich voi GUI dang dung, nhung ben trong ghi dung thanh ghi vel_max.
        """
        if speed_limit is not None:
            self.write_param_float(REG_VEL_MAX, speed_limit, wait=0.008)
        if accel is not None:
            self.write_param_float(REG_ACC_SET, accel, wait=0.008)
        rad = math.radians(angle_deg)
        self.write_param_float(REG_LOC_REF, rad, wait=0.006)

    @staticmethod
    def send_position_batch(motors, commands, stop_event, prepare=True):
        """PP commands on one existing serial connection, without new CAN types.

        Configure all V/A first with the original parameter-write delays. Then
        write each ordinary loc_ref frame separately, flush, and retain the
        reference Sequence 10 ms target spacing (6 ms write + 4 ms pause). Do not assume the adapter accepts
        multiple AT frames in one write. RX cannot interleave with this group.
        Return TX completion time, or None if cancellation precedes the burst.
        The caller serializes this method and STOP with its command lock.
        """
        if not motors or set(commands) != set(motors):
            raise ValueError("Position batch must contain every motor")
        first = next(iter(motors.values()))
        if any(drv.ser is not first.ser or drv._lock is not first._lock
               for drv in motors.values()):
            raise ValueError("Position batch requires one serial and shared lock")

        def param_frame(drv, reg, value):
            data = reg.to_bytes(2, "little") + b"\0\0" + struct.pack("<f", float(value))
            return drv._frame(COMM_WRITE_PARAM, drv.host_id, data)

        settings, targets = [], []
        for mid, drv in motors.items():
            cmd = commands[mid]
            values = (cmd["target_deg"], cmd["speed"], cmd["accel"])
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"M{mid}: non-finite position command")
            if not 0 < cmd["speed"] <= drv.profile.v_max or cmd["accel"] <= 0:
                raise ValueError(f"M{mid}: invalid V/A")
            settings.extend((param_frame(drv, REG_VEL_MAX, cmd["speed"]),
                             param_frame(drv, REG_ACC_SET, cmd["accel"])))
            targets.append((mid, param_frame(drv, REG_LOC_REF, math.radians(cmd["target_deg"]))))

        with first._lock:
            if prepare:
                for frame in settings:
                    if stop_event.is_set():
                        return None
                    first._write_all_locked(frame)
                    time.sleep(0.009)  # Reference Sequence: 6 ms write + 3 ms pause.
            for mid, frame in targets:
                if stop_event.is_set():
                    return None
                first._write_all_locked(frame)
                time.sleep(0.010)  # Reference Sequence: 6 ms write + 4 ms pause.
            return time.monotonic()

    def set_position_csp(self, angle_deg, speed_limit=None):
        """CSP mode (run_mode=5). Theo muc 4.3.4: ghi limit_spd (0x7017) roi loc_ref."""
        if speed_limit is not None:
            self.write_param_float(REG_LIMIT_SPD, speed_limit, wait=0.008)
        rad = math.radians(angle_deg)
        self.write_param_float(REG_LOC_REF, rad, wait=0.006)

    @staticmethod
    def send_csp_batch(motors, commands, stop_event, prepare=True):
        """Existing private-protocol CSP frames, one target burst per tick.

        Configure limit_spd once; subsequent ticks contain only loc_ref frames.
        Do not add per-axis sleeps or allow RX between targets in a group.
        """
        if not motors or set(commands) != set(motors):
            raise ValueError("CSP batch must contain every motor")
        first = next(iter(motors.values()))
        if any(drv.ser is not first.ser or drv._lock is not first._lock
               for drv in motors.values()):
            raise ValueError("CSP batch requires one serial and shared lock")
        settings, targets = [], []
        for mid, drv in motors.items():
            cmd = commands[mid]
            if (not math.isfinite(cmd["target_deg"]) or not math.isfinite(cmd["speed"])
                    or not 0 < cmd["speed"] <= drv.profile.v_max):
                raise ValueError(f"M{mid}: invalid CSP command")
            for reg, value, output in ((REG_LIMIT_SPD, cmd["speed"], settings),
                                       (REG_LOC_REF, math.radians(cmd["target_deg"]), targets)):
                data = reg.to_bytes(2, "little") + b"\0\0" + struct.pack("<f", value)
                output.append(drv._frame(COMM_WRITE_PARAM, drv.host_id, data))
        with first._lock:
            if prepare:
                for frame in settings:
                    if stop_event.is_set():
                        return None
                    if first.ser.write(frame) != len(frame):
                        raise IOError("Incomplete CSP speed-limit serial write")
                    first.ser.flush()
                    time.sleep(0.008)
            if stop_event.is_set():
                return None
            burst = b"".join(targets)
            if first.ser.write(burst) != len(burst):
                raise IOError("Incomplete target burst serial write")
            first.ser.flush()
            return time.monotonic()

    def set_velocity(self, speed_rad, current_limit=None, accel=None):
        """Velocity mode (run_mode=2). Theo muc 4.3.3: limit_cur -> acc_rad -> spd_ref."""
        speed_rad = max(min(speed_rad, self.profile.v_max), -self.profile.v_max)
        if current_limit is not None:
            current_limit = min(current_limit, self.profile.i_max)
            self.write_param_float(REG_LIMIT_CUR, current_limit, wait=0.004)
        if accel is not None:
            self.write_param_float(REG_ACC_RAD, accel, wait=0.004)
        self.write_param_float(REG_SPD_REF, speed_rad, wait=0.004)

    def set_current(self, current_a):
        """Current/Torque mode (run_mode=3). Theo muc 4.3.2: ghi thang iq_ref."""
        current_a = max(min(current_a, self.profile.i_max), -self.profile.i_max)
        self.write_param_float(REG_IQ_REF, current_a, wait=0.004)

    # ---------------------------------------------------------------
    # Che do MIT: vi tri + van toc + kp/kd + mo-men feedforward, 1 khung
    # (Communication Type 1, manual muc 4.1.2)
    # ---------------------------------------------------------------
    def mit_control(self, pos_rad=0.0, vel_rad=0.0, kp=0.0, kd=0.5, torque_nm=0.0, timeout=0.015):
        """
        Lệnh Type 1 (MIT Mode) nâng cấp:
        - Phát lệnh điều khiển lực/trở kháng.
        - HỨNG VÀ GIẢI MÃ NGAY khung phản hồi Type 2 tự động của động cơ.
        - Trả về tuple: (pos_rad, vel_rad, current_a) theo thời gian thực.
        """
        def _scale_u16(x, lo, hi):
            x = max(min(x, hi), lo)
            return int(round((x - lo) / (hi - lo) * 65535))

        p_lo, p_hi = self.profile.p_range
        pos_u = _scale_u16(pos_rad, p_lo, p_hi)
        vel_u = _scale_u16(vel_rad, -self.profile.v_max, self.profile.v_max)
        kp_u = _scale_u16(kp, 0.0, 500.0)
        kd_u = _scale_u16(kd, 0.0, 5.0)
        trq_u = _scale_u16(torque_nm, -self.profile.t_max, self.profile.t_max)

        data = (
            list(pos_u.to_bytes(2, "big"))
            + list(vel_u.to_bytes(2, "big"))
            + list(kp_u.to_bytes(2, "big"))
            + list(kd_u.to_bytes(2, "big"))
        )
        can_id = ((COMM_MOTION & 0x1F) << 24) | ((trq_u & 0xFFFF) << 8) | (self.motor_id & 0xFF)
        id_field = (can_id << 3) | 0x04
        frame = b"AT" + id_field.to_bytes(4, "big") + bytes([8]) + bytes(data) + b"\r\n"

        ret_pos, ret_vel, ret_cur = None, None, None

        payload = self._request_response(
            frame, COMM_FEEDBACK, lambda data: True, timeout,
        )
        if payload is not None:
            p_raw, v_raw, t_raw, _ = struct.unpack(">HHHH", payload)
            ret_pos = (float(p_raw) / 65535.0) * (p_hi - p_lo) + p_lo
            ret_vel = (float(v_raw) / 65535.0) * (2.0 * self.profile.v_max) - self.profile.v_max
            ret_trq = (float(t_raw) / 65535.0) * (2.0 * self.profile.t_max) - self.profile.t_max
            ret_cur = ret_trq / self.profile.kt

        return ret_pos, ret_vel, ret_cur
    # ---------------------------------------------------------------
    # Doc nhanh - dung cho vong lap telemetry cua GUI
    # ---------------------------------------------------------------
    def read_position_rad(self):
        return self.read_param(REG_MECH_POS)

    def read_current_a(self):
        return self.read_param(REG_IQF)

    def read_velocity_rad(self):
        """Thanh ghi 0x701B 'mechVel' - 'Speed of the load' (R-only)."""
        return self.read_param(REG_MECH_VEL)

    def read_voltage_v(self):
        """Thanh ghi 0x701C 'VBUS' - Bus voltage (R-only)."""
        return self.read_param(REG_VBUS)

    def close(self):
        try:
            self.disable()
        finally:
            self.ser.close()


# =========================================================================
# 3. SELF-TEST / DEMO TREN BENCH
# =========================================================================

def self_test(profile=PROFILE_RS00, port=None):
    """Legacy moving bench demo: only an explicit caller supplies the port."""
    if not isinstance(port, str) or not port.strip():
        raise ValueError("Bench demo can move motors; specify port explicitly. Use gui_3motor_pc.py for normal operation.")
    port = port.strip()
    print(f"[*] Mo cong {port}, motor profile = {profile.name}")
    m = RobStrideDriver(port, profile, motor_id=0x01)

    print("\n== 1. Position PP mode ==")
    ok = m.set_mode(RUN_MODE_POSITION)
    print("  set_mode(PP):", "OK" if ok else "THAT BAI")
    m.set_zero()
    time.sleep(0.1)
    m.enable()
    time.sleep(0.1)
    for ang in (0, 30, -30, 0):
        m.set_position(ang, speed_limit=2.0, accel=10.0)
        time.sleep(1.5)
        pos = m.read_position_rad()
        vel = m.read_velocity_rad()
        if pos is not None:
            print(f"  dat {ang:+.0f} deg -> doc lai {math.degrees(pos):.2f} deg"
                  + (f", vel={vel:.2f} rad/s" if vel is not None else ""))

    print("\n== 2. Velocity mode ==")
    ok = m.set_mode(RUN_MODE_SPEED)
    print("  set_mode(SPEED):", "OK" if ok else "THAT BAI")
    m.enable()
    m.set_velocity(1.0, current_limit=3.0, accel=10.0)
    time.sleep(1.0)
    print(f"  dong dien khi chay 1.0 rad/s: {m.read_current_a()}")
    m.set_velocity(0.0)

    print("\n== 3. Current mode ==")
    ok = m.set_mode(RUN_MODE_CURRENT)
    print("  set_mode(CURRENT):", "OK" if ok else "THAT BAI")
    m.enable()
    m.set_current(0.3)
    time.sleep(0.5)
    print(f"  dong dien doc lai: {m.read_current_a()}")
    m.set_current(0.0)

    m.close()
    print("\n[*] Hoan tat self-test.")


def main():
    """Direct execution explains the GUI entry point; never runs a motor demo."""
    from pathlib import Path
    import sys
    gui_path = Path(__file__).resolve().with_name("gui_3motor_pc.py")
    print("Day la thu vien driver RobStride, khong phai chuong trinh GUI.")
    print("De dieu khien 3 motor tren Windows, chay:")
    print(f'"{sys.executable}" "{gui_path}"')
    print("Mo GUI, nhap cong COM va bam Ket noi. Driver chay rieng khong mo COM hay gui CAN.")


if __name__ == "__main__":
    main()
