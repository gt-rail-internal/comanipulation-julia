import os
import signal
import subprocess
import time

p = subprocess.Popen(['python', 'topic2data2.py', '1', '2'])

time.sleep(10)

p.terminate()
p.kill()
