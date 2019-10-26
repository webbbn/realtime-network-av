
import threading
import logging

import pyric
import pyudev as udev

# Create the context for interfacing with udev
udev_context = udev.Context()

class USBTetherRelay:
    """Wait for a phone to be connected via USB and configure UDP forwarding to it"""

    def __init__(self):

        # Create the monitoring thread
        self.thread = threading.Thread(target = self.monitor_udev)
        self.thread.start()

    def join(self):
        if self.thread:
            self.thread.join()
            self.thread = None

    def get_interface(self):
        """Return a USB tether interface if there is a phone tethered"""

        # Search for the tethered interface
        for iface in pyw.interfaces():
            print(iface)

        # We didn't find one
        return None

    def monitor_udev(self):

        # Monitor for any network adapters getiting plugged in or unplugged
        monitor = udev.Monitor.from_netlink(udev_context)
        #monitor.filter_by('net')
        for device in iter(monitor.poll, None):
            print(device.sys_name)
