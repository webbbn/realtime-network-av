
import time

import subprocess
import multiprocessing as mp


class WFBTxProcess(object):

    def __init__(self, program = "/home/webbb/realtime-network-av/install/bin/raw_wifi_bridge",
                 interface = "mon0", ports = [ ], args = []):
        self.program = program
        self.interface = interface
        self.ports = ports
        self.args = args
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
        for line in self.execute([self.program] + self.args + [self.interface] + self.ports):
            print(line)

    def join(self):
        self.proc.join()

class WFBRxProcess(object):

    def __init__(self, program = "/home/webbb/realtime-network-av/install/bin/raw_wifi_receiver",
                 interface = "mon0", ipaddr = "127.0.0.1", args = []):
        self.program = program
        self.interface = interface
        self.ipaddr = ipaddr
        self.args = args
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
        for line in self.execute([self.program] + self.args + [self.interface] + self.ipaddr):
            print(line)

    def join(self):
        self.proc.join()
