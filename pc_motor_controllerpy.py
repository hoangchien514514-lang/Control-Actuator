#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cong cu kiem tra + sua xung dot CAN ID khi them motor moi.
Chay DOC LAP tren Pi (KHONG chay cung luc voi pi_server_dualmotor.py -
phai tat server truoc, vi ca 2 deu can doc quyen rieng cong serial).

    python3 fix_motor_id.py

Buoc 1: tam thoi RUT DAY CAN cua 2 motor cu (1 va 5), CHI de lai motor moi
        cam vao bus, roi chay script nay - no se quet va cho biet motor moi
        dang o ID nao that su.
Buoc 2: neu dung la dang o ID=1 (trung voi motor 1 cu), script se hoi va
        doi sang ID=6 ngay tai cho.
Buoc 3: cam lai day CAN cho 2 motor con lai, khoi dong lai
        pi_server_dualmotor.py nhu binh thuong.
"""

import glob
import sys
import time

from robstride_driver_v0 import RobStrideDriver, PROFILE_EL05

PORT_DEVICE = '/dev/ttyUSB0'


def find_port():
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    return ports[0] if ports else PORT_DEVICE


def main():
    port = find_port()
    print(f"[*] Mo cong {port} ...")
    print("[*] LUU Y: hay dam bao CHI motor moi dang noi vao bus luc nay")
    print("    (da rut day CAN cua motor 1 va 5 ra tam thoi), de tranh nham lan.")
    input("Nhan Enter de tiep tuc quet...")

    # Quet thu o vai ID hay gap: 1 (mac dinh hang), 6 (ID mong muon), va vai ID khac
    candidates = [1, 6, 2, 3, 4, 7]
    found_id = None
    for cid in candidates:
        try:
            drv = RobStrideDriver(port=port, profile=PROFILE_EL05, motor_id=cid)
            pos = drv.read_position_rad()
            drv.close()
            if pos is not None:
                print(f"[+] Motor TRA LOI o ID={cid} (doc duoc vi tri: {pos:.3f} rad)")
                found_id = cid
                break
            time.sleep(0.1)
        except Exception as e:
            print(f"[!] Loi khi thu ID={cid}: {e}")

    if found_id is None:
        print("[-] Khong tim thay motor o bat ky ID nao trong danh sach thu.")
        print("    Kiem tra lai day CAN, nguon 48V, dien tro 120 Ohm.")
        sys.exit(1)

    if found_id == 6:
        print("[*] Motor da dung o ID=6 roi, khong can doi gi them.")
        return

    print(f"\n[?] Motor dang o ID={found_id}. Doi sang ID=6 ngay bay gio? (y/n)")
    ans = input("> ").strip().lower()
    if ans != "y":
        print("[*] Da huy, khong doi gi.")
        return

    drv = RobStrideDriver(port=port, profile=PROFILE_EL05, motor_id=found_id)
    drv.set_can_id(6)
    time.sleep(0.2)
    print("[+] Da gui lenh doi ID sang 6.")

    # Xac nhan lai bang cach doc o ID moi
    drv2 = RobStrideDriver(port=port, profile=PROFILE_EL05, motor_id=6)
    pos = drv2.read_position_rad()
    drv2.close()
    if pos is not None:
        print(f"[+] XAC NHAN: motor gio da tra loi o ID=6 (vi tri: {pos:.3f} rad).")
        print("[*] Gio ban co the cam lai day CAN cho motor 1 va 5, roi chay lai pi_server_dualmotor.py.")
    else:
        print("[!] Khong xac nhan duoc o ID=6 - thu lai hoac kiem tra ket noi.")


if __name__ == "__main__":
    main()