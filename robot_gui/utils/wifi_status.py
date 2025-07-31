# utils/wifi_status.py

import subprocess

def get_wifi_status():
    """
    Returns:
        tuple: (ssid: str, signal_strength: int)
    """
    try:
        output = subprocess.check_output(["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL", "dev", "wifi"]).decode()
        for line in output.strip().split('\n'):
            parts = line.split(':')
            if parts[0] == 'yes':  # Currently connected network
                ssid = parts[1]
                signal_strength = int(parts[2])
                return (ssid, signal_strength)

        return (None, 0)

    except Exception as e:
        print(f"[WifiStatus] Error: {e}")
        return (None, 0)
