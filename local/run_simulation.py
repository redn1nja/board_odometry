import subprocess
import time
import os
import sys

def main():
    sitl = None
    topics = None
    gazebo = None

    try:
        gazebo = subprocess.Popen("/usr/bin/gz sim -r iris_playpen.sdf".split(" "), stdout=sys.stdout)
        time.sleep(5)
        topics = subprocess.Popen("./bin/topics topics.txt output.txt output.avi".split(" "), stdout=sys.stdout)
        time.sleep(2)
        cur_dir = os.getcwd()
        os.chdir(os.environ['HOME'] + "/ardupilot")
        sitl = subprocess.Popen(f"xterm -e 'source ~/.profile && ./sim_vehicle.py -v copter -f gazebo-iris'".split(" "), stdout=sys.stdout)
        sitl.wait()
    finally:
        if sitl:
            sitl.terminate()
        if topics:
            topics.terminate()
        if gazebo:
            gazebo.terminate()


if __name__ == "__main__":
    main()
