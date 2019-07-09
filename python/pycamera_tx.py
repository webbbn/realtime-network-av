#!/usr/bin/env python3

import os
import sys
import numpy as np

root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.join(root_dir, "lib")

if 'LD_LIBRARY_PATH' not in os.environ:
    os.environ['LD_LIBRARY_PATH'] = lib_dir
    try:
        os.execv(sys.argv[0], sys.argv)
    except Exception as exc:
        print('Failed re-exec:', exc)
        sys.exit(1)

# Add the python directory to the python path
python_dir = os.path.join(root_dir, "python")
sys.path.append(python_dir)

import io
import socket
import struct
import array
import time
import argparse
import signal
import logging
import math
import time

from format_as_table import format_as_table
import py_srt
import fec

using_picamera = False
using_socket = False
using_v4l2 = False

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

class SRTOutputStream(object):

    def __init__(self, host, port, maxpacket = 1310):
        self.maxpacket = maxpacket
        self.log = FPSLogger()

        # Create the communication socket
        self.sock = py_srt.create_socket()

        # Try connect to the receiver
        st = py_srt.connect(self.sock, host, port)
        if not st:
            raise Exception("Error connecting to SRT://%s:%d" % (host, port))

    def __del__(self):
        py_srt.close(self.sock)

    def write(self, s):
        self.log.log(len(s))
        for i in range(0, len(s), self.maxpacket):
            py_srt.sendmsg(self.sock, s[i : min(i + self.maxpacket, len(s))], ttl=250)

class UDPOutputStream(object):

    def __init__(self, host, port, broadcast = False, maxpacket = 1310):
        self.log = FPSLogger()
        self.broadcast = broadcast
        self.maxpacket = maxpacket
        self.host = host
        self.port = port

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
        for i in range(0, len(s), self.maxpacket):
            self.sock.sendto(s[i : min(i + self.maxpacket, len(s))], (host, self.port))

class FECUDPOutputStream(object):

    def __init__(self, host, port, broadcast = False, maxpacket = 1310):
        self.log = FPSLogger()
        self.broadcast = broadcast
        self.maxpacket = maxpacket
        self.code_blocks = 8
        self.fec_blocks = 4
        self.host = host
        self.port = port
        self.frame_id = 0
        #self.fec = FECCode(self.code_blocks, self.fec_blocks)
        self.fec = fec.FECCode(self.code_blocks, self.fec_blocks)

        # Create the communication socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def write(self, s):
        msg_len = len(s)
        count = 0
        sub_frame_len = self.maxpacket * self.code_blocks
        num_sub_frames = math.ceil(msg_len / sub_frame_len)
        sub_frame_id = 0
        for i in range(0, msg_len, sub_frame_len):
            sub_frame = s[i : i + sub_frame_len]
            # Encode the sub-frame into a set of FEC blocks
            fec_blocks = self.fec.encode_frame(sub_frame, frame_id=self.frame_id,
                                               sub_frame_id=sub_frame_id,
                                               num_sub_frames=num_sub_frames)
            for block in fec_blocks:
                count += len(block)
                self.sock.sendto(block, (host, self.port))
            sub_frame_id += 1
        self.frame_id = (self.frame_id + 1) % 255
        self.log.log(count)

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

    def __init__(self, protocol, host, port, device=False):
        self.streaming = False
        self.recording = False

        # Set some default parameters
        self.width = 1280
        self.height = 720
        self.bitrate = 5000000
        self.fps = 30
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

        # Create the connection for streaming the data on
        if protocol == "SRT":
            self.stream = SRTOutputStream(host, port)
        elif protocol.upper() == "UDP":
            self.stream = UDPOutputStream(host, port)
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
    def streaming_params(self, width, height, bitrate, intra_period = 30, quality=20, inline_headers = True) :
        self.width = width
        self.height = height
        self.bitrate = bitrate
        self.intra_period = intra_period
        self.quality = quality
        self.inline_headers = inline_headers

    # Change the parameters for the video recording
    def recording_params(self, width, height, bitrate, intra_period = 30, quality=20, inline_headers = True) :
        self.rec_width = width
        self.rec_height = height
        self.rec_bitrate = bitrate
        self.rec_intra_period = intra_period
        self.rec_quality = quality
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
        if using_picamera:
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
                                            inline_headers=self.intra_period, bitrate=self.bitrate, quality=self.quality)
        elif using_socket:

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
        if using_picamera:
            if self.recording:
                self.camera.stop_recording(splitter_port=2)
            if self.streaming:
                self.camera.stop_recording()
        self.streaming = False
        self.recording = False


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", help="the cameradevice name")
    parser.add_argument("-s", "--socket", help="the unix domain socket address")
    parser.add_argument("-l", "--loglevel", default="info", help="set output logging level (debug, info, warning, error, critical)")
    parser.add_argument("-sw", "--width", default=1280, help="the width of the transmitted video")
    parser.add_argument("-sh", "--height", default=720, help="the height of the transmitted video")
    parser.add_argument("-f", "--fps", default=30, help="the frames per second to sample the video at")
    parser.add_argument("-br", "--bitrate", default=5.0, help="the birtate of the transmitted video (in Mbps)")
    parser.add_argument("-ip", "--intra_period", default=3, help="the preodicity of I-frames")
    parser.add_argument("-q", "--quality", default=10, help="the encoding quality")
    parser.add_argument("-rw", "--rec_width", default=1920, help="the width of the recorded video")
    parser.add_argument("-rh", "--rec_height", default=1080, help="the height of the recorded video")
    parser.add_argument("-rbr", "--rec_bitrate", default=25, help="the periodicity of I-frames for recorded video")
    parser.add_argument("-rip", "--rec_intra_period", default=30, help="the birtate of the recorded video (in Mbps)")
    parser.add_argument("-rq", "--rec_quality", default=20, help="the recording encoding quality")
    parser.add_argument("-o", "--output", help="the output (recorded) video filename")
    parser.add_argument("-ho", "--host", help="the hostname/IP/address to stream the video to")
    parser.add_argument("-p", "--port", default=0, help="the port to stream the video to")
    parser.add_argument("-de", "--dev", help="the wifi device to send raw packets to")
    parser.add_argument("-db", "--data_blocks", default=8, help="the number of data blocks used in FEC encoding")
    parser.add_argument("-fb", "--fec_blocks", default=4, help="the number of FEC blocks used in FEC encoding")
    parser.add_argument("-bs", "--block_size", default=1024, help="the block size used in FEC encoding")
    parser.add_argument("protocol", help="the network protocol to use (UDP | UDPB (Broadcast) | TCP | SRT | FECUDP | UDS | WFB)")

    # Parse the options
    args = parser.parse_args()

    # Configure the logger
    log_level = getattr(logging, args.loglevel.upper())
    if not isinstance(log_level, int):
        print("Invalid log level: %s - setting to info" % (args.loglevel))
        log_level = logging.INFO
    logging.basicConfig(level=log_level, format="%(asctime)s %(levelname)s: %(message)s", datefmt="%H:%M:%S")

    protocol = args.protocol.upper()
    width = int(args.width)
    height = int(args.height)
    fps = int(args.fps)
    bitrate = int(float(args.bitrate) * 1e6)
    ip = int(args.intra_period)
    q = int(args.quality)
    rec_width = int(args.rec_width)
    rec_height = int(args.rec_height)
    rec_bitrate = int(float(args.rec_bitrate) * 1e6)
    rec_ip = int(args.rec_intra_period)
    rec_q = int(args.rec_quality)
    output = args.output
    host = args.host
    port = int(args.port)
    dev = args.dev
    data_blocks = args.data_blocks
    fec_blocks = args.fec_blocks
    block_size = args.block_size
    if protocol == "WFB":
        if args.dev == "":
            print("Must specify the wifi device (-de/--dev) when using WFB protocol")
            exit(1)
        else:
            host_port = dev + ":" + str(port)
    elif port > 0:
        host_port = host + ":" + str(port)
    else:
        port = None
        host_port = host
    h264_device = False

    # Force V4L2 if the user specified a device
    if args.device:
        using_v4l2 = True

    # Did the user specify a unix domain socket as input
    sock = None
    if not using_v4l2 and args.socket:

        # Remove the socket file if it exists
        if os.path.exists(args.socket):
            try:
                os.unlink(args.socket)
            except OSError:
                print("Error removing the Unix domain socket: " + args.socket)
                exit(1)

        # Create a UDS socket
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

        # Bind the socket to the port
        sock.bind(args.socket)
        using_socket = True
        logging.info("Streaming packets to %s at %f Mbps Using %s protocol from %s" % (host_port, bitrate, protocol, args.socket))

    if not using_socket and not using_v4l2:

        # Try loading picamera module
        try:
            import picamera
            logging.debug("Loaded pycamera module")
            using_picamera = True
            logging.info("Using picamera to stream %dx%d/%d video to %s at %f Mbps Using %s protocol " % (width, height, fps, host_port, bitrate, protocol))

        except:
            logging.debug("Pycamera not found - using video4linux2 interface")
            pass

    if not using_picamera and not using_socket:

        # Did the user specify a device?
        if args.device:

            h264_device = args.device

        else:

            # Try finding a v4l2 device that will work
            import py_v4l2 as v4l
            from py_v4l2 import Frame
            from py_v4l2 import Control

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
                        if format["format"] == "H264" and format["width"] == width and format["height"] == height:
                            logging.debug("Found requested format: %s - %dx%d on %s" % (format["format"], format["width"], format["height"], device))
                            h264_device = device
                except Exception as e:
                    logging.debug("Error reading controls/formats: " + str(e))
            if h264_device == False:
                logging.critical("No appropriate V4L2 device found")
                exit(1)
        logging.debug("Using " + h264_device)
        logging.info("Streaming %dx%d video to %s at %f Mbps Using %s protocol from %s" % (width, height, host_port, bitrate, protocol, h264_device))

    # Create the camera object
    global camera
    try:
        camera = Camera(protocol, host, port, h264_device)
    except Exception as e:
        print(e)
        exit()

    def exit_handler(sig, frame):
        global camera
        del camera
        sys.exit()

    # Set the streaming and recording parameters
    camera.streaming_params(width, height, bitrate, intra_period = ip, quality = q, inline_headers = True)
    if output:
        camera.recording_params(rec_width, rec_height, rec_bitrate, intra_period = rec_ip, quality = rec_q, inline_headers = True)

    # Setup an exit handler to gracefully exit
    signal.signal(signal.SIGINT, exit_handler)

    # Start streaming/recording
    camera.start_streaming(output, sock = sock)

    while(1):
        time.sleep(1);
