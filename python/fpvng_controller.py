#!/usr/bin/env python3

import os
import sys

# Setup some paths based on the directory that this script was run frum.
root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

# Add the library directory to the LD_LIBRARY_PATH environement variable.
lib_dir = os.path.join(root_dir, "lib")
# Add the bin directory to PATH
bin_dir = os.path.join(root_dir, "bin")
if 'LD_LIBRARY_PATH' not in os.environ:
    os.environ['LD_LIBRARY_PATH'] = lib_dir
else:
    os.environ['LD_LIBRARY_PATH'] += ":" + lib_dir
os.environ['PATH'] += ":" + bin_dir
if 'RE_EXECED' not in os.environ:
    try:
        os.environ['RE_EXECED'] = "1"
        os.execv(sys.argv[0], sys.argv)
    except Exception as exc:
        print('Failed re-exec:', exc)
        sys.exit(1)

# Add the python directory to the python path
python_dir = os.path.join(root_dir, "lib/python3.7/site-packages")
sys.path.append(python_dir)

# The default configuration directory.
conf_dir = os.path.join(root_dir, "conf")

import io
import time
import queue
import argparse
import signal
import logging
import multiprocessing as mp

import network
import wfb_process
import camera
import rx_process as rx
import telemetry

# Define an exit handler to do a graceful shutdown
def exit_handler(sig, frame):
    sys.exit()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--loglevel", default="info", \
                        help="set output logging level (debug, info, warning, error, critical)")
    parser.add_argument("-b", "--bitrate", default=11, \
                        help="the bitrate to use on the wifi link")
    parser.add_argument("-ohd", "--openhd", help="Maintain compatability with OpenHD", \
                        action='store_true')

    # Parse the options
    args = parser.parse_args()
    bitrate = int(args.bitrate)
    openhd_mode = args.openhd

    # Configure the logger
    log_level = getattr(logging, args.loglevel.upper())
    if not isinstance(log_level, int):
        print("Invalid log level: %s - setting to info" % (args.loglevel))
        log_level = logging.INFO
    logger = logging.getLogger('FPVNG-controller')
    logging.basicConfig(level=log_level, format="%(asctime)s %(levelname)s: %(message)s", datefmt="%H:%M:%S")

    # Setup an exit handler to gracefully exit
    signal.signal(signal.SIGINT, exit_handler)

    # Try to start the camera
    cam = camera.CameraProcess()
    if cam.start():
        air_side = True
        logging.info("Camera found. Running as Air side.")
    else:
        air_side = False
        logging.info("Camera NOT found. Running as Ground side.")

    # Create the network configuration class.
    net = network.Network()
    ifaces = net.monitor_interfaces()
    iface = ifaces[0]

    # Configure the first interface
    mon = net.configure_interface(ifaces[0], bitrate=bitrate)
    if mon is None:
        logging.error("Error configuring the network interface: " + iface)
        exit(1)
    logging.info(mon.dev + " interface configured successfully")

    # Start the WFB interfaces
    if air_side:
        wfbp_tx_proc = wfb_process.WFBTxProcess(conf_dir + "/wifi_bridge_air.json", interface = mon.dev, port = 25)
        wfbp_rx_proc = wfb_process.WFBRxProcess(interface = mon.dev, ipaddr = "127.0.0.1", port = 26)
    else:
        wfbp_tx_proc = wfb_process.WFBTxProcess(conf_dir + "/wifi_bridge_ground.json", interface = mon.dev, port = 26)
        wfbp_rx_proc = wfb_process.WFBRxProcess(interface = mon.dev, ipaddr = "192.168.128.255", port = 25)

    if not air_side:
        rx_proc = rx.RxCrossfireProcess(device = "/dev/ttyS0")

    # Create the FC telemetry queue
    #fc_telem_queue = queue.Queue()

    # Start the telemetry parsers / forwarders
    if air_side:
        telem = telemetry.SerialTelemetryRx(uart="/dev/ttyS1", baudrate=115200)
    #     telemtx = telemetry.UDPTelemetryTx(fc_telem_queue, "127.0.0.1", 14550)
    # else:
    #     telemrx = telemetry.SerialTelemetryRx(fc_telem_queue, uart="/dev/ttyS0", baudrate=57600)
    #     telemtx = telemetry.UDPTelemetryTx(fc_telem_queue, "127.0.0.1", 14551)

    # Join with the processing threads before shutting down
    if air_side:
        telem.join()
    wfbp_tx_proc.join()
    wfbp_rx_proc.join()
