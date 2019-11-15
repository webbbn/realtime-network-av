#!/usr/bin/env python3

import socket
import struct
import array
import time
import logging
import math
import numpy as np
import subprocess
import multiprocessing as mp
#import py_v4l2 as v4l
#from py_v4l2 import Frame
#from py_v4l2 import Control

from wifibroadcast.format_as_table import format_as_table
from wifibroadcast import fec

def module_exists(module_name):
    try:
        __import__(module_name)
    except ImportError:
        return False
    else:
        return True

# Try loading picamera module
found_picamera = module_exists("picamera")
if found_picamera:
    import picamera

class FPSLogger(object):

    def __init__(self):
        self.bytes = 0
        self.count = 0
        self.blocks = 0
        self.prev_time = time.time()

    def log(self, frame_size, blocks = 0):
        self.bytes += frame_size
        self.blocks += blocks
        self.count += 1
        cur_time = time.time()
        dur = (cur_time - self.prev_time)
        if dur > 2.0:
            if self.blocks > 0:
                logging.debug("fps: %f  Mbps: %6.3f  blocks: %d" %
                              (self.count / dur, 8e-6 * self.bytes / dur, self.blocks))
            else:
                logging.debug("fps: %f  Mbps: %6.3f" % (self.count / dur, 8e-6 * self.bytes / dur))
            self.prev_time = cur_time
            self.bytes = 0
            self.count = 0

class UDPOutputStream(object):

    def __init__(self, host, port, broadcast = False, maxpacket = 1400, fec_ratio=0.0):
        self.log = FPSLogger()
        self.broadcast = broadcast
        self.maxpacket = maxpacket
        self.host = host
        self.port = port
        if fec_ratio > 0:
            self.fec = fec.PyFECBufferEncoder(maxpacket, fec_ratio)
        else:
            self.fec = None

        # Create the communication socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
    def write(self, s):
        self.log.log(len(s))
        if self.broadcast:
            host = '<broadcast>'
        else:
            host = self.host
        if self.fec:
            for b in self.fec.encode_buffer(s):
                self.sock.sendto(b, (host, self.port))
        else:
            for i in range(0, len(s), self.maxpacket):
                self.sock.sendto(s[i : min(i + self.maxpacket, len(s))], (host, self.port))


class WFBOutputStream(object):

    def __init__(self, dev, port = 0, code_blocks = 8, fec_blocks = 4, block_size = 1024, output = None):
        self.log = FPSLogger()
        self.code_blocks = code_blocks
        self.fec_blocks = fec_blocks
        self.block_size = block_size
        self.dev = dev
        self.port = port # The output port on the other side of the link.
        self.fec = fec.PyFECEncoder(self.code_blocks, self.fec_blocks, self.block_size, True)

        # Create the radiotap headerb"\x00\x00\x0c\x00\0x04\0x80\0x00\0x00\0x16\0x00\0x00"
        self.rt_header = bytearray([0x00, 0x00, # radiotap version
                                    0x0c, 0x00, # radiotap header length
                                    0x04, 0x80, 0x00, 0x00, # radiotap present flags (rate + tx flags)
                                    0x6c, # datarate (will be overwritten later)
                                    0x00, 0x00, 0x00])

        # Create the 802.11 header
        self.ieee_header = bytearray([0x08, 0x02, 0x00, 0x00, # frame control field (2bytes), duration (2 bytes)
                                      # port = 1st byte of IEEE802.11 RA (mac) must be something odd
                                      # (wifi hardware determines broadcast/multicast through odd/even check)
	                              0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	                              0x13, 0x22, 0x33, 0x44, 0x00, 0x00, # mac (last two bytes are the port number)
	                              0x13, 0x22, 0x33, 0x44, 0x55, 0x66, # mac
                                      # IEEE802.11 seqnum, (will be overwritten later by Atheros firmware/wifi chip)
	                              0x00, 0x00])
        self.ieee_header[14] = (port >> 8) & 0xff
        self.ieee_header[15] = port & 0xff

        # Create the communication socket
        self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)

        # Bind to the socket
        self.sock.bind((self.dev, 0))

        # Open the output file if requested
        if output:
            self.of = open(output, mode="wb")
            logging.info("Saving video to " + output)
        else:
            self.of = None

    def write(self, s):

        # Encode the packet
        self.fec.encode(s)

        # Retrieve the encoded blocks
        blocks = self.fec.get_blocks()

        # Trasmit the blocks
        count = 0
        for block in blocks:
            # Add the radiotap header and ieee header and send the packet.
            msg = np.append(self.rt_header + self.ieee_header, block)
            self.sock.send(msg)
            count += len(block)
        self.log.log(count, len(blocks))

        # Write the data to the output file if one was requested
        if self.of:
            self.of.write(s)

class UDSOutputStream(object):

    def __init__(self):
        self.log = FPSLogger()

        # Create the communication socket
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def connect(self, server_address):
        self.server_address = server_address

        # Try to connect to the UDS socket
        try:
            self.sock.connect(self.server_address)
        except:
            return False
        return True

    def write(self, s):
        self.log.log(len(s))
        self.sock.send(s)

class TCPOutputStream(object):

    def __init__(self, host, port):
        self.log = FPSLogger()
        self.host = host
        self.port = port
        self.conn = 0;

        # Create the communication socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Try connect to the receiver
        self.conn = self.sock.connect((self.host, self.port))

    def __del__(self):
        try:
            self.conn.close()
        except:
            pass

    def write(self, s):
        self.log.log(len(s))
        self.sock.send(s)

class Camera(object):

    def __init__(self, protocol, host, port, device=False, fec_ratio=0.0):
        self.streaming = False
        self.recording = False

        # Set some default parameters
        self.width = 1280
        self.height = 720
        self.bitrate = 6000000
        self.fps = 60
        self.intra_period = 3
        self.quality = 20
        self.inline_headers = True

        self.rec_width = 1920
        self.rec_height = 1080
        self.rec_bitrate = 25000000
        self.rec_intra_period = 30
        self.rec_quality = 30
        self.rec_inline_headers = True

        self.device = device

        self.using_socket = False

        # Create the connection for streaming the data on
        if protocol.upper() == "UDP":
            self.stream = UDPOutputStream(host, port, fec_ratio=fec_ratio)
        elif protocol.upper() == "UDPB":
            self.stream = UDPOutputStream(host, port, broadcast=True)
        elif protocol.upper() == "FECUDP":
            self.stream = FECUDPOutputStream(host, port, broadcast=True)
        elif protocol.upper() == "WFB":
            self.stream = WFBOutputStream(dev, port, data_blocks, fec_blocks, block_size, output)
        elif protocol.upper() == "UDS":
            self.stream = UDSOutputStream()
            count = 0
            self.stream.connect(host)
            while not self.stream.connect(host):
                if count == 10:
                    logging.debug("Waiting for the UDS socket to be created.")
                    count = 0
                count += 1
                time.sleep(1)
            logging.debug("Connected")
        elif protocol.upper() == "TCP":
            self.stream = TCPOutputStream(host, port)
        else:
            raise Exception("Invalid network protocol %s" % protocol)

    def __del__(self):
        self.stop_streaming()

    # Change the parameters for the video stream
    def streaming_params(self, width, height, bitrate, intra_period = 30, quality=20, fps = 60, inline_headers = True) :
        self.width = width
        self.height = height
        self.bitrate = bitrate
        self.intra_period = intra_period
        self.quality = quality
        self.fps = fps
        self.inline_headers = inline_headers

    # Change the parameters for the video recording
    def recording_params(self, width, height, bitrate, intra_period = 30, quality=20, fps = 60, inline_headers = True) :
        self.rec_width = width
        self.rec_height = height
        self.rec_bitrate = bitrate
        self.rec_intra_period = intra_period
        self.rec_quality = quality
        self.fps = fps
        self.rec_inline_headers = inline_headers

    def start_streaming(self, rec_filename = False, sock = None):

        # The camera frame size has to be the larger of the streaming and recording size
        if rec_filename:
            self.rec_width = max(self.width, self.rec_width)
            self.rec_height = max(self.height, self.rec_height)
        else:
            self.rec_width = self.width
            self.rec_height = self.height

        # Create the camera source
        if found_picamera:
            self.camera = picamera.PiCamera()

            # Initilize the camera parameters
            self.camera.resolution = (self.rec_width, self.rec_height)
            self.camera.framerate = self.fps
            self.camera.awb_mode = 'sunlight'

            # Are we recording and streaming, or just streaming?
            self.streaming = True
            if rec_filename:
                self.recording = True
                self.camera.start_recording(rec_filename, format='h264', intra_period=self.rec_intra_period,
                                            inline_headers=self.rec_inline_headers, bitrate=self.rec_bitrate, quality=self.rec_quality)
                self.camera.start_recording(self.stream, format='h264', intra_period=self.intra_period,
                                            inline_headers=self.inline_headers, bitrate=self.bitrate, quality=self.quality,
                                            splitter_port=2, resize=(self.width, self.height))
            else:
                self.camera.start_recording(self.stream, format='h264', intra_period=self.intra_period,
                                            inline_headers=self.inline_headers, bitrate=self.bitrate)

            while self.streaming:
                self.wait_streaming(1)

        elif self.using_socket:

            # Read from the socket and send it out
            while True:
                data = sock.recv(500000)
                self.stream.write(data)

        else:

            # We're using V4L2
            control = Control(self.device)
            control.set_control_value(9963800, 2)

            frame = Frame(self.device, self.width, self.height)
            self.streaming = True
            while self.streaming:
                frame_data = frame.get_frame()
                self.stream.write(frame_data)

    def wait_streaming(self, time):
        if self.camera:
            self.camera.wait_recording(time)

    def stop_streaming(self):
        if found_picamera:
            if self.recording:
                self.camera.stop_recording(splitter_port=2)
            if self.streaming:
                self.camera.stop_recording()
        self.streaming = False
        self.recording = False


class CameraProcess(object):

    def __init__(self, device = False, protocol = "UDP", host = "", port = 5600, \
                 width = 1280, height = 720, bitrate = 3000000, quality = 20, inline_headers = True, \
                 fps = 30, intra_period = 5, fec_ratio=0.0):
        self.device = device
        self.protocol = protocol
        self.host = host
        self.port = port
        self.width = width
        self.height = height
        self.bitrate = bitrate
        self.quality = quality
        self.inline_headers = inline_headers
        self.fps = fps
        self.intra_period = intra_period
        self.fec_ratio = fec_ratio

    def start(self):
        h264_device = None

        if self.host != "":
            host_port = self.host + ":" + str(self.port)
        else:
            host_port = str(self.port)

        # Read from the Raspberry Pi camera if it was found and if the user didn't specify an alternate device
        if not self.device and found_picamera:
            logging.info("Using picamera to stream %dx%d/%d video to %s at %f Mbps Using %s protocol " % \
                         (self.width, self.height, self.fps, host_port, self.bitrate, self.protocol))

        else:

            # Try finding a v4l2 device that will work
            if self.device :
                devices = [self.device]
            else:
                # Query the list of video devices
                devices = v4l.get_devices()

            # Try to find an H264 capable device
            for device in devices:
                logging.debug(device)
                logging.debug("")

                try:
                    control = Control(device)
                    controls = control.get_controls()
                    logging.debug(format_as_table(controls, controls[0].keys(), controls[0].keys(), 'name'))
                    formats = control.get_formats()
                    logging.debug(format_as_table(formats, formats[0].keys(), formats[0].keys(), 'format'))
                    for format in formats:
                        if format["format"] == "H264" and format["width"] == self.width and format["height"] == self.height:
                            logging.debug("Found requested format: %s - %dx%d on %s" % \
                                          (format["format"], format["width"], format["height"], device))
                            h264_device = device
                except Exception as e:
                    continue
            if h264_device == None:
                self.camera = None
                self.proc = None
                return False

            logging.info("Streaming %dx%d video to %s at %f Mbps Using %s protocol from %s" % \
                         (self.width, self.height, host_port, self.bitrate, self.protocol, h264_device))

        self.camera = Camera(self.protocol, self.host, self.port, h264_device, fec_ratio=self.fec_ratio)
        self.camera.streaming_params(self.width, self.height, self.bitrate, self.intra_period, self.quality,
                                     self.fps, self.inline_headers)
        self.proc = mp.Process(target=self.run)
        self.proc.start()
        return True

    def run(self):
        # Start streaming
        self.camera.start_streaming()

    def join(self):
        if self.proc:
            self.proc.join()

if __name__ == '__main__':
    logging.basicConfig(level='DEBUG')
    # cam = CameraProcess(host="192.168.1.38", port=5600)
    # cam = CameraProcess(host="127.0.0.1", port=5600)
    cam = CameraProcess(host="127.0.0.1", port=5700, fec_ratio=0.5)
    #cam = CameraProcess()
    if cam.start():
        cam.join()
    else:
        LOG_ERROR << "Error starting camera process"
