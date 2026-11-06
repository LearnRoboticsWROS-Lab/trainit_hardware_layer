"""The INSTALLED package's Python metadata is this source's: one egg-info, this version, every console script.

colcon (setuptools) leaves the egg-info of an older version beside the new one when the version changes, and Python
may resolve the OLD metadata: a console script added since then fails at start with ``StopIteration`` (2026-10-30,
D0-C: ``realsense_source_info`` against a stale 0.2.1 egg-info). Fix: delete the stale
``install/trainit_hardware_layer/lib/python3.X/site-packages/trainit_hardware_layer-<old>-py3.X.egg-info``.
"""
import ast
import configparser
import re
import sys
from importlib.metadata import distribution
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = 'trainit_hardware_layer'


def _source_version():
    return re.search(r'<version>([^<]+)</version>', (ROOT / 'package.xml').read_text()).group(1).strip()


def _source_console_scripts():
    """The console-script names setup.py declares (read as Python, so a comment never shortens the list)."""
    for node in ast.walk(ast.parse((ROOT / 'setup.py').read_text())):
        if isinstance(node, ast.keyword) and node.arg == 'entry_points':
            return sorted(entry.split('=')[0].strip() for entry in ast.literal_eval(node.value)['console_scripts'])
    return []


def _site_packages():
    ament = pytest.importorskip('ament_index_python.packages')
    try:
        prefix = Path(ament.get_package_prefix(PACKAGE))
    except Exception:  # noqa: BLE001 -- not installed in this environment
        pytest.skip(f'{PACKAGE} is not installed in the sourced workspace')
    return prefix / 'lib' / f'python{sys.version_info.major}.{sys.version_info.minor}' / 'site-packages'


def test_the_install_holds_exactly_this_versions_metadata_with_every_console_script():
    version, site = _source_version(), _site_packages()
    infos = sorted(p.name for p in site.glob(f'{PACKAGE}-*.egg-info'))
    if not infos:
        pytest.skip('a symlink (develop) install keeps its metadata in the build tree')
    py = f'py{sys.version_info.major}.{sys.version_info.minor}'
    assert infos == [f'{PACKAGE}-{version}-{py}.egg-info'], \
        f'stale metadata in {site}: {infos} (keep only {version}; see this module\'s docstring)'
    entry_points = configparser.ConfigParser()
    entry_points.optionxform = str                      # keep the names' case
    entry_points.read(site / infos[0] / 'entry_points.txt')
    assert sorted(entry_points['console_scripts']) == _source_console_scripts()


def test_python_resolves_the_installed_distribution_to_this_version():
    _site_packages()
    resolved = distribution(PACKAGE.replace('_', '-'))
    assert resolved.version == _source_version()
    assert sorted(e.name for e in resolved.entry_points if e.group == 'console_scripts') == \
        _source_console_scripts()
