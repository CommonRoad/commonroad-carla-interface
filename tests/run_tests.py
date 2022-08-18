import signal
import unittest
import os
import sys
import time
import subprocess

if __name__ == "__main__":
    print(os.getcwd())
    errors = 0
    failures = 0
    tests = 0
    with subprocess.Popen(["/opt/carla-simulator/CarlaUE4.sh", '-RenderOffScreen']) as process:
        try:
            print('Running Carla in process', process.pid)
            time.sleep(4)
            for x in os.walk(os.getcwd()):
                if '__' not in x[0] and '.' not in x[0]:
                    print(x[0])
                    all_tests = unittest.TestLoader().discover(x[0], pattern='test_*.py')
                    b = unittest.TextTestRunner().run(all_tests)
                    failures += len(b.failures)
                    errors += len(b.errors)
                    tests += b.testsRun
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            print('Carla Timed out - killing')
            process.kill()

    # Kill CarlaUE4-Linux
    try:
        # iterating through each instance of the process
        for line in os.popen("ps ax | grep " + 'CarlaUE4-Linux-' + " | grep -v grep"):
            fields = line.split()
            # extracting Process ID from the output
            pid = fields[0]
            # terminating process
            os.kill(int(pid), signal.SIGKILL)
        print("CarlaUE4-Linux  Successfully terminated")
    except Exception:
        print("Error Encountered while running script")

    print("Done")
    print('Executed {} tests; got {} fails and {} errors'.format(tests, failures, errors))
    if errors > 0 or failures > 0:
        sys.exit(1)
