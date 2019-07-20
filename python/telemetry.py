
import queue
import threading
import pymavlink.mavutil as mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink2

class SerialTelemetryRx(object):
    """Receive telemetry over a standard UART connection"""

    def __init__(self, queue, uart = "/dev/ttyS0", baudrate = 57600):
        self.queue = queue
        self.uart = uart
        self.baudrate = baudrate
        self.done = False
        self.mavs = mavutil.mavlink_connection(uart, baud=baudrate)
        self.thread = threading.Thread(target = self.start)
        self.thread.start()

    def __del__(self):
        self.done = True
        self.thread.join()

    def start(self):
        while not self.done:
            msg = self.mavs.recv_msg()
            if msg:
                self.queue.put(msg)

class UDPTelemetryRx(object):
    """Receive telemetry over a standard UDP port"""

    def __init__(self, queue, host, port):
        self.queue = queue
        self.host = host
        self.port = port
        self.done = False
        self.mavs = mavutil.mavlink_connection(host + ":" + str(port), write=False, input=True)
        self.thread = threading.Thread(target = self.start)
        self.thread.start()

    def __del__(self):
        self.done = True
        self.thread.join()

    def start(self):
        while not self.done:
            msg = self.mavs.recv_msg()
            if msg:
                self.queue.put(msg)

class UDPTelemetryTx(object):
    """Send telemetry over a UDP socket"""

    def __init__(self, queue, host, port, broadcast=False):
        self.queue = queue
        self.host = host
        self.port = port
        self.done = False
        self.mavs = mavutil.mavlink_connection(host + ":" + str(port), write=True, input=False)
        self.thread = threading.Thread(target = self.start)
        self.thread.start()

    def __del__(self):
        self.done = True
        self.thread.join()

    def start(self):
        while not self.done:
            msg = self.queue.get()
            self.mavs.write(msg.get_msgbuf())

class SerialTelemetryTx(object):
    """Seld telemetry over a standard UART connection"""

    def __init__(self, queue, uart = "/dev/ttyS0", baudrate = 57600):
        self.queue = queue
        self.uart = uart
        self.baudrate = baudrate
        self.done = False
        self.mavs = mavutil.mavlink_connection(uart, baud=baudrate)
        self.thread = threading.Thread(target = self.start)
        self.thread.start()

    def __del__(self):
        self.done = True
        self.thread.join()

    def start(self):
        while not self.done:
            msg = self.mavs.recv_msg()
            if msg:
                self.queue.put(msg)
