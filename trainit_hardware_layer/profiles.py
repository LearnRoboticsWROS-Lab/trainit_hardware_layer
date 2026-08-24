"""Load hardware profiles by role and model.

A profile describes one piece of hardware in the terms of ``contracts/``. Application
and adapter code reads a profile; it never hard-codes a model name, a topic or a frame.
That is the whole point of the layer -- see ``contracts/README.md``.
"""
from pathlib import Path
from typing import Any, Dict, List

import yaml

ROLES = ('robot', 'eef', 'camera')


def _root() -> Path:
    """Profiles directory, from the install share dir when installed, else the source."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('trainit_hardware_layer')) / 'profiles'
        if share.is_dir():
            return share
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / 'profiles'


def available(role: str) -> List[str]:
    """Model names available for a role."""
    if role not in ROLES:
        raise ValueError(f"unknown role '{role}' (expected one of {ROLES})")
    return sorted(p.stem for p in (_root() / role).glob('*.yaml'))


def load(role: str, model: str) -> Dict[str, Any]:
    """Return the profile dict for ``role``/``model``."""
    path = _root() / role / f'{model}.yaml'
    if not path.is_file():
        raise FileNotFoundError(
            f"no {role} profile '{model}'. Available: {', '.join(available(role)) or '(none)'}")
    with open(path) as fh:
        return yaml.safe_load(fh)


def backend(profile: Dict[str, Any], mode: str) -> Dict[str, Any]:
    """Return the backend entry of ``profile`` for ``mode`` (mock / isaac / real)."""
    backends = profile.get('backends') or {}
    if mode not in backends:
        raise KeyError(
            f"profile '{profile.get('model')}' declares no backend '{mode}' "
            f"(has: {', '.join(backends) or '(none)'})")
    return backends[mode] or {}
