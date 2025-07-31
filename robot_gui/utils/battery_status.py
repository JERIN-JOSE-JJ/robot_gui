# utils/battery_status.py

import subprocess

def get_battery_status():
    """
    Returns:
        tuple: (battery_percent: int, is_charging: bool)
    """
    try:
        output = subprocess.check_output(["upower", "-i", "/org/freedesktop/UPower/devices/battery_BAT0"]).decode()

        percent = None
        charging = False

        for line in output.splitlines():
            if "percentage" in line:
                percent = int(line.strip().split()[-1].replace('%', ''))
            if "state" in line:
                charging = "charging" in line.lower()

        return (percent, charging)

    except Exception as e:
        print(f"[BatteryStatus] Error: {e}")
        return (None, False)
