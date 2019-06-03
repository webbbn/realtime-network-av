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

# Add the python directory to the python path
root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
python_dir = os.path.join(root_dir, "python")
sys.path.append(python_dir)
lib_dir = os.path.join(root_dir, "lib")
if "LD_LIBRARY_PATH" in os.environ:
    os.environ["LD_LIBRARY_PATH"] = os.environ["LD_LIBRARY_PATH"] + ":" + lib_dir
else:
    os.environ["LD_LIBRARY_PATH"] = lib_dir

import picamera
import py_srt

class SRTOutputStream(object):

    def __init__(self, host, port, maxpacket = 1310):
        self.maxpacket = maxpacket

        # Create the communication socket
        self.sock = py_srt.create_socket()

        # Try connect to the receiver
        st = py_srt.connect(self.sock, host, port)
        if not st:
            raise Exception("Error connecting to SRT://%s:%d" % (host, port))

    def __del__(self):
        py_srt.close(self.sock)

    def write(self, s):
        for i in range(0, len(s), self.maxpacket):
            py_srt.sendmsg(self.sock, s[i : min(i + self.maxpacket, len(s))], ttl=250)

class UDPOutputStream(object):

    def __init__(self, host, port, broadcast = False, maxpacket = 1310):
        self.broadcast = broadcast
        self.maxpacket = maxpacket
        self.host = host
        self.port = port

        # Create the communication socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def write(self, s):
        if self.broadcast:
            host = '<broadcast>'
        else:
            host = self.host
        for i in range(0, len(s), self.maxpacket):
            self.sock.sendto(s[i : min(i + self.maxpacket, len(s))], (host, self.port))

class TCPOutputStream(object):

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.conn = 0;

        # Create the communication socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Try connect to the receiver
        print(self.host)
        print(self.port)
        self.conn = self.sock.connect((self.host, self.port))

    def __del__(self):
        try:
            self.conn.close()
        except:
            pass

    def write(self, s):
        self.sock.send(s)

class Camera(object):

    def __init__(self, protocol, host, port):
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

        # Create the connection for streaming the data on
        if protocol == "SRT":
            self.stream = SRTOutputStream(host, port)
        elif protocol == "UDP":
            self.stream = UDPOutputStream(host, port)
        elif protocol == "UDPB":
            self.stream = UDPOutputStream(host, port, broadcast=True)
        elif protocol == "TCP":
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
        self.camera = picamera.PiCamera()

        # Initilize the camera parameters
        self.camera.resolution = (self.rec_width, self.rec_height)
        self.camera.framerate = self.fps
        self.camera.awb_mode = 'sunlight'

        # Are we recording and streaming, or just streaming?
        if rec_filename:
            self.camera.start_recording(rec_filename, format='h264', intra_period=self.rec_intra_period,
                                        inline_headers=self.rec_inline_headers, bitrate=self.rec_bitrate, quality=self.rec_quality)
            self.camera.start_recording(self.stream, format='h264', intra_period=self.intra_period,
                                        inline_headers=self.inline_headers, bitrate=self.bitrate, quality=self.quality,
                                        splitter_port=2, resize=(self.width, self.height))
            self.recording = True
        else:
            self.camera.start_recording(self.stream, format='h264', intra_period=self.intra_period,
                                        inline_headers=self.intra_period, bitrate=self.bitrate, quality=self.quality)

    def wait_streaming(self, time):
        if self.camera:
            self.camera.wait_recording(time)

    def stop_streaming(self):
        if self.recording:
            self.camera.stop_recording(splitter_port=2)
        if self.streaming:
            self.camera.stop_recording()
        self.streaming = False
        self.recording = False

        #udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        #udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        #udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #udp_out.connect(('192.168.1.38', 1234))
        #udp_out_fd = udp_out.makefile('wb', 2000)
        #print(udp_out_fd)

        #tcp_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #tcp_out.bind(('192.168.1.116', 1234))
        #tcp_out.listen(1)
        #print("accepting")
        #conn, addr = tcp_out.accept()
        #print("connected" + str(addr))

        #tcp_out.connect(('192.168.1.38', 1234))
        #connection = conn.makefile('wb')

        # with picamera.PiCamera() as camera:

        #     # Initialize the high-resolution stream and record to a file
        #     camera.resolution = (1920, 1080)
        #     camera.framerate = 30
        #     camera.start_recording('highres.h264', format='h264', intra_period=30, inline_headers=True, bitrate=25000000, quality=20)

        #     # Process the low-resolution stream in real time
        #     camera.start_recording(OutputStream(), format='h264', intra_period=3, inline_headers=True, bitrate=5000000, quality=30, splitter_port=2, resize=(1280, 720))
        #     #camera.start_recording('lowres.h264', splitter_port=2, resize=(800, 480))
        #     #camera.start_recording('highres.h264')
        #     #camera.start_recording(udp_out_fd, format='h264')
        #     #camera.start_recording(sys.stdout, format='h264')
        #     camera.wait_recording(60)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
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
    parser.add_argument("protocol", help="the network protocol to use (UDP | UDPB (Broadcast) | TCP | SRT | OpenHD)")
    parser.add_argument("host", help="the hostname/IP to stream the video to")
    parser.add_argument("port", help="the port to stream the video to")

    # Parse the options
    args = parser.parse_args()
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
    verbose = args.verbose

    if verbose :
        print("Streaming %dx%d/%d video to %s:%d at %f Mbps Using %s protocol " %
              (width, height, fps, host, port, bitrate, protocol))

    # Create the camera object
    global camera
    try:
        camera = Camera(protocol, host, port)
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

    # Start streaming/recording
    camera.start_streaming(output)

    signal.signal(signal.SIGINT, exit_handler)

    while(1):
        time.sleep(1);
