#!/usr/bin/env python3

import subprocess

def wifi_signal_strength():
    results = subprocess.getoutput(["netsh", "wlan", "show", "network", "mode=Bssid"])
    for row in results.splitlines():
        cols = row.split(' : ')
        key = cols[0].strip()
        if len(cols) == 2 and key == "Signal":
            return int(cols[1].split("%")[0])
    return 0

if __name__ == '__main__':
    print(wifi_signal_strength())
