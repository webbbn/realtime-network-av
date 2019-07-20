
import time
import socket
import subprocess
import multiprocessing as mp


class RxCrossfireProcess(object):

    def __init__(self, program = "/home/webbb/realtime-network-av/install/bin/csrf_receiver",
                 device = "/dev/ttyS0", period = 10, ip = "127.0.0.1", port=14552):
        self.program = program
        self.device = device
        self.period = period
        self.ip = ip
        self.port = port
        self.proc = mp.Process(target=self.start)
        self.proc.start()

    def execute(self, cmd):
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
        for stdout_line in iter(popen.stdout.readline, ""):
            yield stdout_line 
        popen.stdout.close()
        return_code = popen.wait()
        if return_code:
            raise subprocess.CalledProcessError(return_code, cmd)

    def start(self):
        # Create the UDP send socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for line in self.execute([self.program, "-p", str(self.period), self.device]):
            sock.sendto(bytearray(line, "utf-8"), (self.ip, self.port))

    def join(self):
        self.proc.join()
