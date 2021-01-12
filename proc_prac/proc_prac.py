#!/usr/bin/env python
# -*- coding: utf-8 -*-
 import subprocess
import time

cmd = "python3 while.py &"
cmd = cmd.split(" ")
print(cmd)
p = subprocess.Popen(cmd, shell=False)

print("PROCESS ID", p.pid)

time.sleep(10)
print("killing process", p.pid)
p.terminate()
p.wait()
