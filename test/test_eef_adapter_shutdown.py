"""The adapters' run loop on a SIGINT shutdown race (vNext M2.7): a clean stop exits 0.

rclpy (Humble) can raise RCLError ("failed to initialize wait set: the given context is not
valid") from ``executor.spin()`` when the SIGINT handler shuts the context down while the executor
rebuilds its wait set. ``adapter_node.run`` treats an exception raised after the context was shut
down as that clean stop (one STOPPED line on stderr), and any exception raised while the context is
still valid as a real failure. Both orders are reproduced exactly, in a child process (the default
rclpy context, the harness's domain, never the test session's): the race no longer depends on the
scheduler.
"""
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

import eef_harness as h

pytest.importorskip('rclpy', reason='the shutdown tests need a sourced ROS 2 Humble')

CHILD = textwrap.dedent("""
    import sys
    import rclpy
    from rclpy.impl.implementation_singleton import rclpy_implementation
    from rclpy.node import Node
    from trainit_hardware_layer.eef import adapter_node

    class Probe(Node):
        DEFAULT_NODE_NAME = '_shutdown_probe'     # '_': hidden from RosSession.visible()
        def __init__(self):
            super().__init__(self.DEFAULT_NODE_NAME)
        def recommended_threads(self):
            return 1
        def close(self, timeout_s=5.0):
            print('PROBE_CLOSED', flush=True)

    MODE = sys.argv[1]

    class RacingExecutor(adapter_node.MultiThreadedExecutor):
        def spin(self):
            if MODE == 'after_shutdown':
                # what the SIGINT handler's rcl_shutdown leaves: the same default context, invalid
                rclpy.get_default_context().shutdown()
            raise rclpy_implementation.RCLError(
                'failed to initialize wait set: the given context is not valid')

    adapter_node.MultiThreadedExecutor = RacingExecutor
    print('RUN_RETURNED', adapter_node.run(Probe), flush=True)
""")


def _child(mode: str, cwd) -> subprocess.CompletedProcess:
    # the child imports exactly the module this test process imports (never whatever its working
    # directory happens to hold): the package's root goes first on its PYTHONPATH
    import trainit_hardware_layer
    root = str(Path(trainit_hardware_layer.__file__).resolve().parents[1])
    path = os.pathsep.join([root] + [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep)
                                     if p])
    env = dict(os.environ, ROS_DOMAIN_ID=str(h.DOMAIN_ID), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(Path(cwd) / 'ros_log'), PYTHONPATH=path)
    return subprocess.run([sys.executable, '-c', CHILD, mode], env=env, cwd=str(cwd),
                          capture_output=True, text=True, timeout=60)


def _tokens(stdout: str) -> list:
    # Fast DDS prints its own diagnostics on stdout (a stale shared-memory lock, for one): only the
    # child's token lines are judged
    return [line for line in stdout.splitlines() if line.startswith(('PROBE_', 'RUN_'))]


def test_an_exception_after_the_context_was_shut_down_is_a_clean_stop(tmp_path):
    proc = _child('after_shutdown', tmp_path)
    assert proc.returncode == 0, proc.stderr
    assert _tokens(proc.stdout) == ['PROBE_CLOSED', 'RUN_RETURNED 0'], proc.stdout
    assert ('STOPPED RCLError from spin() after the context was shut down: failed to initialize '
            'wait set: the given context is not valid') in proc.stderr
    assert 'Traceback' not in proc.stderr


def test_an_exception_while_the_context_is_valid_is_a_real_failure(tmp_path):
    proc = _child('while_running', tmp_path)
    assert proc.returncode == 1, proc.stderr
    assert _tokens(proc.stdout) == ['PROBE_CLOSED'], proc.stdout   # the cleanup ran, no return
    assert 'RCLError: failed to initialize wait set: the given context is not valid' in proc.stderr
    assert 'STOPPED' not in proc.stderr
