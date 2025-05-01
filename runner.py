#! /bin/python3

import subprocess
import sys

if len(sys.argv) < 6:
    print("usage: python3 runner.py <exec> <video> <imu> <output_dir> count")

executable = sys.argv[1]
video = sys.argv[2]
imu = sys.argv[3]
out_dir = sys.argv[4]
count = int(sys.argv[5])

command = sys.argv[1:5]
print(command)

avg_frame_time = 0
for i in range(count):
    result = subprocess.run(command, capture_output=True).stdout.decode("utf-8").split("\n")[-2]
    print(result)
    avg_frame_time+=float(result)

print(f"average frame processing time: {avg_frame_time/count}")

