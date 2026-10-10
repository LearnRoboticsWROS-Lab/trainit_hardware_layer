"""The two console scripts as child processes (domain 86): what a generated launch starts.

A launch hands an adapter its parameters as a parameter file, nested dicts flattened by the ROS
parameter parser exactly as launch_ros flattens them. These tests start the INSTALLED executables
(``lib/trainit_hardware_layer/<name>``), one process at a time, after checking that the installed
modules are this source tree byte for byte (a stale install fails instead of testing old code).
"""
import hashlib
import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

import eef_harness as h
from eef_harness import actuator

rclpy = pytest.importorskip('rclpy', reason='the console tests need a sourced ROS 2 Humble')

from std_srvs.srv import SetBool  # noqa: E402

SOURCE = Path(__file__).resolve().parent.parent / 'trainit_hardware_layer' / 'eef'


def _prefix() -> Path:
    from ament_index_python.packages import get_package_prefix
    return Path(get_package_prefix('trainit_hardware_layer'))


def _executable(name: str) -> Path:
    return _prefix() / 'lib' / 'trainit_hardware_layer' / name


def _params_file(tmp_path: Path, extra: dict, device_block: bool = True) -> Path:
    doc = yaml.safe_load(h.pinned('eef_conformance.yaml').read_text(encoding='utf-8'))
    params = doc['/**']['ros__parameters']
    if not device_block:
        params = {}
    params.update(extra)
    path = tmp_path / 'adapter_params.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}), encoding='utf-8')
    return path


def _start(name: str, args, tmp_path: Path) -> subprocess.Popen:
    env = dict(os.environ, ROS_DOMAIN_ID=str(h.DOMAIN_ID), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(tmp_path / 'ros_log'), PYTHONUNBUFFERED='1')
    return subprocess.Popen([str(_executable(name)), '--ros-args'] + list(args), env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def _stop(process: subprocess.Popen) -> int:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    return process.returncode


def test_the_installed_console_scripts_are_this_source():
    for name in ('mock_eef_adapter', 'isaac_eef_adapter'):
        exe = _executable(name)
        assert exe.is_file() and os.access(exe, os.X_OK), f'{exe}: not installed -- colcon build first'
    installed = (_prefix() / 'lib' / f'python{sys.version_info.major}.{sys.version_info.minor}'
                 / 'site-packages' / 'trainit_hardware_layer' / 'eef')
    sources = sorted(p.name for p in SOURCE.glob('*.py'))
    assert sources == sorted(p.name for p in installed.glob('*.py'))
    for name in sources:
        mine = hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()
        theirs = hashlib.sha256((installed / name).read_bytes()).hexdigest()
        assert mine == theirs, f'{name}: the install is stale -- colcon build first'


def test_mock_eef_adapter_runs_from_a_parameter_file(harness, tmp_path):
    params = _params_file(tmp_path, {'device_id': 'composite_tool',
                                     'mock': {'sites': {'site_b': {'decide_after_ms': 50}}}})
    probe = harness.helper('console_probe')
    watch = h.StateWatch(probe, '/c1/eef/composite_tool/state')
    process = _start('mock_eef_adapter', ['-r', '__ns:=/c1', '--params-file', str(params)], tmp_path)
    try:
        first = watch.wait(lambda m: m.status == 1, timeout=15.0, what='READY from the child')
        assert first.end_effector == '/c1/eef/composite_tool' and first.adapter == 'trainit_mock'
        assert [a.actuator for a in first.actuators] == ['linear_axis', 'rotary_axis', 'wrist_mechanism',
                                                         'gripper_a', 'gripper_b', 'binary_lock']
        assert '/c1/mock_eef_adapter' in harness.session.visible()
        client = probe.create_client(SetBool, '/c1/eef/composite_tool/actuators/gripper_b/set_engaged')
        assert h.call(client, SetBool.Request(data=True), timeout=10.0).success
        watch.wait(lambda m: actuator(m, 'gripper_b').last_command == 1 and h.site(m, 'site_b')[0] == 4)
    finally:
        code = _stop(process)
    assert code == 0, process.stderr.read()


@pytest.mark.parametrize('name,extra,device_block,refusal', [
    ('mock_eef_adapter', {'device_id': 'composite_tool',
                          'eef': {'composite_tool': {'scene_signal': 'adapter'}}}, True,
     'REFUSED eef.composite_tool.scene_signal: only a single-site device has a scene signal'),
    ('mock_eef_adapter', {'device_id': 'ghost'}, True, 'REFUSED device_id:'),
    ('isaac_eef_adapter', {'device_id': 'suction_simple'}, True, 'REFUSED stage_cmd_topic: required'),
    ('isaac_eef_adapter', {'device_id': 'composite_tool', 'stage_cmd_topic': 'stage/cmd'}, True,
     'REFUSED eef.composite_tool: trainit_isaac_surface_gripper serves one binary suction vacuum'),
])
def test_a_refused_start_exits_with_code_2(harness, tmp_path, name, extra, device_block, refusal):
    doc = yaml.safe_load(h.pinned('eef_conformance.yaml').read_text(encoding='utf-8'))
    params = doc['/**']['ros__parameters']
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(params.get(k), dict):
            for dev, keys in v.items():
                params[k][dev].update(keys)
        else:
            params[k] = v
    path = tmp_path / 'refused_params.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}), encoding='utf-8')
    process = _start(name, ['-r', '__ns:=/c2', '--params-file', str(path)], tmp_path)
    try:
        process.wait(timeout=20)
    finally:
        code = _stop(process)
    stderr = process.stderr.read()
    assert code == 2, stderr
    assert refusal in stderr, stderr
