#!/usr/bin/env python3

import os
import sys
import socket
import serial
import serial.threaded
import time

# /dev/ttyS0 for Raspberry Pi Zero W
# /dev/ttyS1 for nanopi duo2
serial_port = "/dev/ttyS1"

class SerialToNet(serial.threaded.Protocol):
    """serial->socket"""

    def __init__(self, port):
        self.port = port

        # Create the UDP broadcast port
        while True:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                break
            except:
                print("Send socket open error: " + os.strerr(error.errno) + "\n", file=sys.stderr)
                time.sleep(1)

    def __call__(self):
        return self

    def data_received(self, data):
        print(len(data))
        self.socket.sendto(data, ('<broadcast>', self.port))


if __name__ == '__main__':
    baudrate = 115200
    broadcast_port = 14550
    receive_port = 14551

    # Delay, just in case...
    time.sleep(20)

    # connect to serial port
    ser = serial.serial_for_url(serial_port, do_not_open=True)
    ser.baudrate = baudrate

    try:
        ser.open()
    except serial.SerialException as e:
        sys.stderr.write('Could not open serial port {}: {}\n'.format(ser.name, e))
        sys.exit(1)

    # Start the thread that broadcasts serial to a UDP port
    ser_to_net = SerialToNet(broadcast_port)
    serial_worker = serial.threaded.ReaderThread(ser, ser_to_net)
    serial_worker.start()

    # Create the UDP receive socket
    while True:
        try:
            recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            recv_socket.bind(("", receive_port))
            break
        except:
            print("Receive socket open error: " + os.strerr(error.errno) + "\n", file=sys.stderr)
            time.sleep(1)

    # Start the receive loop
    while True:
        data, addr = recv_socket.recvfrom(128)
        ser.write(data)

    serial_worker.stop()
