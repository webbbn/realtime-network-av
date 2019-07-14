
import time

import subprocess
import multiprocessing as mp


class WFBProcess(object):

    def __init__(self, program = "/home/webbb/realtime-network-av/install/bin/raw_wifi_bridge",
                 interface = "mon0", ports = [ "5600", "144500" ], args = []):
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
