#!/usr/bin/env python3

import io
import os
import sys
import socket
import struct
import array
import time
import argparse
import signal
import logging
import math

import fec

# Add the python directory to the python path
root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
python_dir = os.path.join(root_dir, "python")
sys.path.append(python_dir)
lib_dir = os.path.join(root_dir, "lib")
if "LD_LIBRARY_PATH" in os.environ:
    os.environ["LD_LIBRARY_PATH"] = os.environ["LD_LIBRARY_PATH"] + ":" + lib_dir
else:
    os.environ["LD_LIBRARY_PATH"] = lib_dir

using_picamera = False

from format_as_table import format_as_table
import py_srt

class FPSLogger(object):

    def __init__(self, frequency = 30):
        self.frequency = frequency
        self.count = 0;
        self.fps = 0
        self.bps = 0
        self.prev_frame_time = time.time()

    def log(self, frame_size):
        frame_time = time.time()
        frame_dur = (frame_time - self.prev_frame_time)
        self.fps += 1.0 / frame_dur
        self.bps += frame_size * 8.0 / frame_dur
        self.count += 1
        self.prev_frame_time = frame_time
        if self.count == self.frequency:
            logging.debug("fps: %f  Mbps: %6.3f" % (self.fps / self.count, 1e-6 * self.bps / self.count))
            self.fps = 0;
            self.bps = 0;
            self.count = 0;

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

    def encode_cb(self, share_id, max_share_id, buf):
        if self.broadcast:
            host = '<broadcast>'
        else:
            host = self.host
        self.sock.sendto(buf, (host, self.port))
        self.count += len(buf)

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

    def start_streaming(self, rec_filename = False):

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
    parser.add_argument("protocol", help="the network protocol to use (UDP | UDPB (Broadcast) | TCP | SRT | FECUDP | OpenHD)")
    parser.add_argument("host", help="the hostname/IP to stream the video to")
    parser.add_argument("port", help="the port to stream the video to")

    # Parse the options
    args = parser.parse_args()

    # Configure the logger
    log_level = getattr(logging, args.loglevel.upper())
    if not isinstance(log_level, int):
        print("Invalid log level: %s - setting to info" % (args.loglevel))
        log_level = logging.INFO
    logging.basicConfig(level=log_level, format="%(asctime)s %(levelname)s: %(message)s", datefmt="%H:%M:%S")

    protocol = args.protocol
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
    h264_device = False

    # Are we on an raspberry pi?
    try:
        import picamera
        logging.debug("Loaded pycamera module")
        using_picamera = True
        logging.info("Using picamera to stream %dx%d/%d video to %s:%d at %f Mbps Using %s protocol " % (width, height, fps, host, port, bitrate, protocol))

    except:
        # If not, use V4L2
        import py_v4l2 as v4l
        from py_v4l2 import Frame
        from py_v4l2 import Control
        logging.debug("Pycamera not found - using video4linux2 interface")

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
                    if format["format"] == "H264" and format["width"] == args.rec_width and format["height"] == args.rec_height:
                        logging.debug("Found requested format: %s - %dx%d on %s" % (format["format"], format["width"], format["height"], device))
                        h264_device = device
            except Exception as e:
                logging.debug("Error reading controls/formats: " + str(e))
        if h264_device == False:
            logging.critical("No appropriate V4L2 device found")
            exit(1)
        logging.debug("Using " + h264_device)
        logging.info("Streaming %dx%d video to %s:%d at %f Mbps Using %s protocol from %s" % (width, height, host, port, bitrate, protocol, h264_device))

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
    camera.start_streaming(output)

    while(1):
        time.sleep(1);
