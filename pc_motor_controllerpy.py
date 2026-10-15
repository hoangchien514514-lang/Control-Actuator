#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Windows entry point for the 7-motor GUI and read-only setup checks.

No CAN ID assignment or Linux device auto-scan. Importing opens no COM port.
"""
import argparse

from robstride_driver_v0 import RobStrideDriver  # Keep the existing import API.


def main(argv=None):
    parser = argparse.ArgumentParser(description="RobStride 7-motor Windows controller")
    parser.add_argument("--check-config", action="store_true", help="Print motor mapping without opening COM")
    parser.add_argument("--list-ports", action="store_true", help="List Windows serial ports without opening them")
    args = parser.parse_args(argv)
    if args.check_config:
        from gui_3motor_pc import load_motor_config
        config = load_motor_config()
        for cfg in config:
            print(f"ID={cfg['id']}: {cfg['profile'].name}")
        print(f"Configured motors: {len(config)}. No COM opened; no CAN sent.")
    if args.list_ports:
        from serial.tools import list_ports
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
    if not args.check_config and not args.list_ports:
        from gui_3motor_pc import launch_gui
        launch_gui()


if __name__ == "__main__":
    main()
