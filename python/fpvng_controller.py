#!/usr/bin/env python3

import os
import sys

# Setup some paths based on the directory that this script was run frum.
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
import time
import queue
import argparse
import signal
import logging
import multiprocessing as mp

import network
import wfb_process
import camera
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
    else:
        air_side = False

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
        wfbp_tx_proc = wfb_process.WFBTxProcess(interface = mon.dev, ports = [ "5600", "144500" ])
        wfbp_rx_proc = wfb_process.WFBRxProcess(interface = mon.dev, ipaddr = "127.0.0.1")
    else:
        wfbp_tx_proc = wfb_process.WFBTxProcess(interface = mon.dev, ports = [ "144501" ])
        wfbp_rx_proc = wfb_process.WFBRxProcess(interface = mon.dev, ipaddr = "127.0.0.1")

    # Create the FC telemetry queue
    fc_telem_queue = queue.Queue()

    # Start the telemetry parsers / forwarders
    if air_side:
        telemrx = telemetry.SerialTelemetryRx(fc_telem_queue, uart="/dev/ttyS1", baudrate=115200)
        telemtx = telemetry.UDPTelemetryTx(fc_telem_queue, "127.0.0.1", 14550)
    else:
        telemrx = telemetry.SerialTelemetryRx(fc_telem_queue, uart="/dev/ttyS0", baudrate=57600)
        telemtx = telemetry.UDPTelemetryTx(fc_telem_queue, "127.0.0.1", 14551)

    # Start the telemetry output
    wfbp_tx_proc.join()
    wfbp_rx_proc.join()
