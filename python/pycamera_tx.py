#!/usr/bin/env python3

import os
import sys

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
import argparse
import signal

import camera

using_picamera = False
using_socket = False
using_v4l2 = False

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
