
import os
import time

import subprocess
import multiprocessing as mp

class WFBTxProcess(object):

    def __init__(self, conf, program = "wfb_bridge", interface = "mon0", args = [], port = 1):
        self.program = program
        self.interface = interface
        self.conf = conf
        self.args = args
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
        try:
            for line in self.execute([self.program] + self.args + [self.interface, "air", self.conf]):
                print(line)
        except Exception as e:
            print("Error starting subprocess")
            print(e)
            return False
        return True

    def join(self):
        self.proc.join()

class WFBRxProcess(object):

    def __init__(self, program = "raw_wifi_receiver", interface = "mon0", ipaddr = "127.0.0.1", args = [], port = 0):
        self.program = program
        self.interface = interface
        self.ipaddr = ipaddr
        self.args = args
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
        try:
            for line in self.execute([self.program] + self.args + [self.interface, str(self.port), self.ipaddr]):
                print(line)
        except Exception as e:
            print("Error starting subprocess")
            print(e)
            return False
        return True

    def join(self):
        self.proc.join()
