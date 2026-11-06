"""The camera source-information document ``trainit-camera-source/1.0`` (ROS-free).

A camera backend MAY publish one document, latched, on ``<ns>/source_info`` (contracts/camera.md, "Source
information"). It answers: which physical device produced these frames, where it was attached, which host acquired
them, with which software and configuration, and how their stamps are produced. Every block says how its facts are
known:

- ``DISCOVERED``: queried from the device, the OS or the runtime on the ACQUISITION host;
- ``DECLARED``: given by the deployment (a launch argument);
- ``OBSERVED``: measured from the live stream or the running process.

A fact that cannot be established is ABSENT. It is never guessed, and nothing here is inferred from a topic name.

The document describes the ACQUISITION side only. A consumer on another host (a recorder) adds its own facts and
compares them with these (``host.boot_id_sha256``) to tell a local source from a remote one; it never re-reads the
device. Exact network addresses are deliberately not part of the document.
"""
import hashlib
import os
import platform
import re
import socket
from pathlib import Path

SCHEMA_NAME = 'trainit-camera-source'
SCHEMA_MAJOR = 1
SCHEMA_MINOR = 0
SCHEMA = f'{SCHEMA_NAME}/{SCHEMA_MAJOR}.{SCHEMA_MINOR}'

ORIGINS = ('DISCOVERED', 'DECLARED', 'OBSERVED')
# block -> the origin its facts have (one origin per block keeps the provenance of every field unambiguous)
BLOCK_ORIGINS = {'device': 'DISCOVERED', 'attachment': 'DISCOVERED', 'host': 'DISCOVERED',
                 'deployment': 'DECLARED', 'software': 'DISCOVERED', 'library': 'OBSERVED',
                 'configuration': 'DISCOVERED', 'timestamping': 'OBSERVED'}
REQUIRED = ('schema', 'backend', 'published_wall_utc_ns')

# The driver's per-frame clock domain (librealsense ``rs2_timestamp_domain``, as its metadata spells it) -> the
# TrainIt stamp origin of the header the driver writes. Only a domain whose header mapping is established is listed:
# ``global_time`` -- the device clock mapped onto the host clock by librealsense; ``system_time`` -- the host clock
# when the frame reached the library. ``hardware_clock`` is absent: the driver then re-bases the device clock on its
# own start time, which is no stamp origin of the vocabulary.
STAMP_ORIGIN_OF_CLOCK_DOMAIN = {'global_time': 'DRIVER_GLOBAL_TIME', 'system_time': 'RECEIPT'}

_USB_DEVICE_DIR = re.compile(r'^\d+-\d+(\.\d+)*$')        # 8-1.4: bus 8, root port 1, hub port 4


class SourceInfoError(ValueError):
    """A document that does not follow ``trainit-camera-source/1.0``."""


# ---- discovery helpers (the acquisition host) ------------------------------------------------------------------

def _read(path: Path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def boot_id_sha256(proc_root='/proc'):
    """sha256 of this boot's random id: equal on two processes exactly when they run on the same host, this boot.
    It identifies nothing beyond that (a new boot draws a new id)."""
    raw = _read(Path(proc_root) / 'sys' / 'kernel' / 'random' / 'boot_id')
    return None if not raw else hashlib.sha256(raw.encode('utf-8')).hexdigest()


def host_facts(proc_root='/proc') -> dict:
    """The DISCOVERED facts of the host this process runs on (no address, no machine id)."""
    out = {'origin': 'DISCOVERED', 'hostname': socket.gethostname(), 'machine': platform.machine(),
           'kernel': platform.release()}
    boot = boot_id_sha256(proc_root)
    if boot:
        out['boot_id_sha256'] = boot
    return {k: v for k, v in out.items() if v}


def usb_attachment(physical_port, sysfs_root='/sys'):
    """The DISCOVERED USB attachment of a device, from its sysfs node (e.g. a V4L2 video node under the device), or
    ``None`` when the port is not a sysfs path (another backend) or the device directory cannot be found.

    ``speed_mbps`` is the negotiated link speed, ``usb_version`` the device's bcdUSB; ``hub`` is the IMMEDIATE hub when
    the device hangs off one (``None`` for a root port), with its self-powered bit as the hub REPORTS it. The port path
    comes from the driver; only sysfs is read, and a path that leaves ``/sys`` is refused.
    """
    if not physical_port:
        return None
    port = os.path.normpath(str(physical_port))
    if not port.startswith('/sys/'):
        return None
    node = Path(sysfs_root) / Path(port).relative_to('/sys')
    device = next((p for p in [node, *node.parents] if _USB_DEVICE_DIR.match(p.name) and (p / 'idVendor').exists()),
                  None)
    if device is None:
        return None
    out = {'origin': 'DISCOVERED', 'via': 'sysfs (from the physical port the driver reports)', 'kind': 'USB',
           'port_id': device.name}
    for key, name, cast in (('bus', 'busnum', int), ('speed_mbps', 'speed', int), ('usb_version', 'version', str)):
        value = _read(device / name)
        if value:
            try:
                out[key] = cast(value)
            except ValueError:
                pass
    parent = device.parent
    if _USB_DEVICE_DIR.match(parent.name) and (parent / 'idVendor').exists():
        hub = {'port_id': parent.name}
        vendor, product = _read(parent / 'idVendor'), _read(parent / 'idProduct')
        if vendor and product:
            hub['vendor_product'] = f'{vendor}:{product}'
        for key, name in (('product', 'product'),):
            value = _read(parent / name)
            if value:
                hub[key] = value
        speed = _read(parent / 'speed')
        if speed and speed.isdigit():
            hub['speed_mbps'] = int(speed)
        attributes = _read(parent / 'bmAttributes')
        if attributes:
            try:
                hub['reports_self_powered'] = bool(int(attributes, 16) & 0x40)
            except ValueError:
                pass
        out['hub'] = hub
    else:
        out['hub'] = None                                   # discovered: a root port, no hub in between
    return out


def stamp_origin(clock_domains: dict):
    """The stamp origin every observed stream agrees on, else ``None`` (mixed, unknown or unmapped domains)."""
    origins = {STAMP_ORIGIN_OF_CLOCK_DOMAIN.get(d) for d in clock_domains.values()}
    return origins.pop() if len(origins) == 1 and None not in origins else None


def library_in_process(maps_text: str, library='librealsense2'):
    """``{name, version, path}`` of the shared library a process has mapped (``/proc/<pid>/maps``), else ``None``."""
    pattern = re.compile(r'(/\S*/' + re.escape(library) + r'\.so\.(\d+(?:\.\d+)*))\s*$')
    best = None
    for line in maps_text.splitlines():
        found = pattern.search(line)
        if found and (best is None or len(found.group(2)) > len(best[1])):
            best = (found.group(1), found.group(2))
    return None if best is None else {'name': library, 'version': best[1], 'path': best[0]}


# ---- the document ---------------------------------------------------------------------------------------------

def build(*, backend, published_wall_utc_ns, **blocks) -> dict:
    """A document from its blocks (``None`` blocks are left out: unknown is absent), validated."""
    doc = {'schema': SCHEMA, 'backend': backend, 'published_wall_utc_ns': int(published_wall_utc_ns)}
    for name, block in blocks.items():
        if block is not None:
            doc[name] = block
    return parse(doc)


def parse(document) -> dict:
    """The document, checked: the schema, the required fields, each known block's origin, JSON-plain values.

    A document of a NEWER minor version may carry blocks this reader does not know: they are kept as they are.
    Of this minor version, an unknown block is refused (a typo is not a new fact).
    """
    if not isinstance(document, dict):
        raise SourceInfoError('a source-information document is a JSON object')
    schema = document.get('schema')
    match = re.match(r'^' + re.escape(SCHEMA_NAME) + r'/(\d+)\.(\d+)$', str(schema))
    if not match or int(match.group(1)) != SCHEMA_MAJOR:
        raise SourceInfoError(f'schema {schema!r} is not {SCHEMA_NAME}/{SCHEMA_MAJOR}.x')
    newer = int(match.group(2)) > SCHEMA_MINOR
    for key in REQUIRED:
        if key not in document:
            raise SourceInfoError(f'{key!r} is required')
    if not isinstance(document['backend'], str) or not document['backend']:
        raise SourceInfoError('backend is a non-empty string')
    stamp = document['published_wall_utc_ns']
    if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
        raise SourceInfoError('published_wall_utc_ns is a positive integer')
    for key, block in document.items():
        if key in REQUIRED:
            continue
        if key not in BLOCK_ORIGINS:
            if newer:
                continue
            raise SourceInfoError(f'unknown block {key!r} in a {SCHEMA} document')
        if not isinstance(block, dict):
            raise SourceInfoError(f'{key} is a JSON object')
        if block.get('origin') != BLOCK_ORIGINS[key]:
            raise SourceInfoError(f'{key}.origin is {BLOCK_ORIGINS[key]} (got {block.get("origin")!r})')
    _json_plain(document, 'source_info')
    timestamping = document.get('timestamping')
    if timestamping is not None and 'stamp_origin' in timestamping:
        derived = stamp_origin(timestamping.get('clock_domains') or {})
        if timestamping['stamp_origin'] != derived:
            raise SourceInfoError(f'timestamping.stamp_origin {timestamping["stamp_origin"]!r} is not what the '
                                  f'observed clock domains establish ({derived!r})')
    return document


def _json_plain(value, where):
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise SourceInfoError(f'{where}: keys are strings')
            _json_plain(item, f'{where}.{key}')
    elif isinstance(value, list):
        for i, item in enumerate(value):
            _json_plain(item, f'{where}[{i}]')
    elif not (value is None or isinstance(value, (str, int, bool)) or
              (isinstance(value, float) and value == value and value not in (float('inf'), float('-inf')))):
        raise SourceInfoError(f'{where}: {type(value).__name__} is not a JSON value')


__all__ = ['SCHEMA', 'ORIGINS', 'BLOCK_ORIGINS', 'STAMP_ORIGIN_OF_CLOCK_DOMAIN', 'SourceInfoError', 'build',
           'parse', 'host_facts', 'boot_id_sha256', 'usb_attachment', 'stamp_origin', 'library_in_process']
