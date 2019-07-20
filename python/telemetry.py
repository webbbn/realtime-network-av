
import math
import time
import queue
import threading
import pymavlink.mavutil as mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink2
from MultiWii import MultiWii

class SerialTelemetryRx(object):
    """Receive telemetry over a standard UART connection"""

    def __init__(self, uart = "/dev/ttyS0", baudrate = 115200, protocol = "msp"):
        self.uart = uart
        self.baudrate = baudrate
        self.done = False
        self.mavs = mavutil.mavlink_connection(uart, baud=baudrate)

        if protocol == "msp":

            # Create the MSP interface to the flight controller.
            self.mw = MultiWii()
            self.mw.connect(uart, baudrate)

            # Create the mavlink UDP output port
            self.mavdown = mavutil.mavlink_connection('udpout:127.0.0.1:14550')

            # Create the procesing thread
            self.thread = threading.Thread(target = self.start_msp)

        elif protocol == "mavlink":
            self.thread = threading.Thread(target = self.start_mavlink)
        else:
            raise Exception("Unsupported telemetry format: " + protocol)

        # Start the processing thread
        self.thread.start()

    def __del__(self):
        self.done = True
        self.thread.join()

    def join(self):
        self.thread.join()

    def start_msp(self):
        counter = 0
        while not self.done:
            # High frequency messages
            att = self.mw.getAttitude()
            misc = self.mw.getMisc()

            # Send the attitude message
            roll = float(att["angx"]) * math.pi / 180.0
            pitch = float(att["angy"]) * math.pi / 180.0
            yaw = float(att["heading"]) * math.pi / 18.0
            rollspeed = 0; # rad/s
            pitchspeed = 0; # rad/s
            yawspeed = 0; # rad/s
            self.mavdown.mav.attitude_send(0, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
            
            # Low frequency messages
            if counter % 10:

                # Send a heartbeat message
                self.mavdown.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

            # Loop with a period of 100ms
            counter += 1
            time.sleep(0.1)

    def start_mavlink(self):
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

if __name__ == '__main__':
    queue = queue.Queue()
    telem = SerialTelemetryRx(queue, uart="/dev/ttyS1")
    time.sleep(20)
