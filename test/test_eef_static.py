"""A static scan of the adapter sources (ADR-0022 D7, D8): what must never be in them.

- no absolute ROS name literal: every EEF name is relative and resolves in the cell namespace (an
  absolute name can only come from a parameter, e.g. the policy card's Trigger pair at the root);
- never ``/suction/attached`` (a second writer of the policy runtime's observation slot);
- no asyncio inside rclpy (the FAIRINO shim's defect: every goal ended ABORTED);
- no function-static cache: no ``global`` statement, no mutable default argument.
Docstrings and comments are not scanned for names; inside an f-string only a leading fixed part
is (f'{ns}/eef/{id}' composes a relative name with the node's namespace).
"""
import ast
import re
from pathlib import Path

import pytest

SOURCES = sorted((Path(__file__).resolve().parent.parent / 'trainit_hardware_layer' / 'eef').glob('*.py'))
ABSOLUTE_NAME = re.compile(r'^/[A-Za-z_~]')


def _docstrings(tree):
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            body = getattr(node, 'body', [])
            if body and isinstance(body[0], ast.Expr) and isinstance(getattr(body[0], 'value', None), ast.Constant):
                out.add(id(body[0].value))
    return out


def _fstring_parts(tree):
    """{id: is_first} for the fixed parts of f-strings: f'{ns}/eef/{id}' composes a relative name
    with a namespace, so only a LEADING fixed part may not be absolute (f'/x/{y}' is flagged)."""
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.JoinedStr):
            for i, v in enumerate(node.values):
                if isinstance(v, ast.Constant):
                    out[id(v)] = i == 0
    return out


@pytest.mark.parametrize('path', SOURCES, ids=[p.name for p in SOURCES])
def test_no_absolute_name_no_asyncio_no_attached_no_static_cache(path):
    text = path.read_text(encoding='utf-8')
    tree = ast.parse(text)
    docs = _docstrings(tree)
    parts = _fstring_parts(tree)
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docs:
            if parts.get(id(node), True):          # a whole literal, or the head of an f-string
                assert not ABSOLUTE_NAME.match(node.value), f'{path.name}:{node.lineno}: {node.value!r}'
            assert 'attached' not in node.value, f'{path.name}:{node.lineno}: {node.value!r}'
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names] + [getattr(node, 'module', None) or '']
            assert not any(n and n.split('.')[0] == 'asyncio' for n in names), f'{path.name}:{node.lineno}'
        assert not isinstance(node, (ast.Global, ast.Nonlocal)), f'{path.name}:{node.lineno}: global state'
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            for default in node.args.defaults + node.args.kw_defaults:
                assert not isinstance(default, (ast.List, ast.Dict, ast.Set)), (
                    f'{path.name}:{node.lineno}: a mutable default argument')
    assert 'async def' not in text and 'await ' not in text


def test_the_scan_sees_every_adapter_module():
    assert {p.name for p in SOURCES} == {'__init__.py', 'adapter_node.py', 'contract.py', 'core.py',
                                         'isaac_eef_adapter.py', 'mock_eef_adapter.py'}
