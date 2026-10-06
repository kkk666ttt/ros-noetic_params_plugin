#!/usr/bin/env python3
"""Give each CTest a private ROS master and clean up its process group."""
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

with socket.socket() as sock:
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
with tempfile.TemporaryDirectory(prefix="ros-params-test-") as directory:
    env = dict(os.environ, ROS_MASTER_URI=f"http://127.0.0.1:{port}",
               ROS_HOSTNAME="127.0.0.1", ROS_HOME=directory,
               ROS_LOG_DIR=os.path.join(directory, "log"))
    with open(os.path.join(directory, "master.log"), "w+") as log:
        master = subprocess.Popen(["roscore", "-p", str(port)], env=env,
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while True:
                try:
                    proxy = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
                    if proxy.getPid("/ros_params_test_runner")[0] == 1:
                        break
                except (OSError, xmlrpc.client.Error):
                    pass
                if master.poll() is not None or time.monotonic() > deadline:
                    log.seek(0)
                    raise RuntimeError("Private ROS master did not start:\n" + log.read())
                time.sleep(0.05)
            result = subprocess.run(sys.argv[1:], env=env, timeout=40)
        finally:
            os.killpg(master.pid, signal.SIGTERM)
            try:
                master.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)
                master.wait()
        sys.exit(result.returncode)
