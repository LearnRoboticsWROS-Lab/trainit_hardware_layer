"""Token parity (ADR-0022 D3, D6): the hardware layer's tables equal the pinned canonical token file.

The token file is a byte copy (sha256-pinned) of the one trainit_interfaces, TMR and TSA check their
own tables against. Every group of the file is either owned by a table here and compared token for
token, or declared as not used by the hardware layer; a new group fails until it is classified.
The message constants (trainit_interfaces, the Python bindings) are compared too, so the core's
tables, the messages and the file can never drift apart.
"""
import eef_harness as h
from trainit_hardware_layer.eef import contract, core

# Groups the hardware layer does not interpret: the failure reasons belong to TMR's VerifyGrasp
# and to its commanders; an adapter reports causes in free text (`detail`).
NOT_USED_HERE = {'reason'}


def _ints(group):
    return {name: int(value) for name, value in h.load_tokens()[group].items()}


def test_the_token_file_is_pinned():
    assert h.sha256(h.DATA / 'eef_contract_tokens.txt') == h.PINS['eef_contract_tokens.txt']


def test_every_group_is_owned_or_declared_unused():
    owned = set(core.TOKEN_ENUMS) | {'accepted_evidence', 'result_policy', 'scene_signal', 'capability'}
    assert set(h.load_tokens()) == owned | NOT_USED_HERE


def test_the_core_enums_equal_the_token_file():
    for group, enum_cls in core.TOKEN_ENUMS.items():
        assert core.token_table(enum_cls) == _ints(group), group


def test_the_contract_tables_equal_the_token_file():
    tokens = h.load_tokens()
    interfaces = sorted((v, n) for n, v in _ints('command_interface').items() if n != 'unknown')
    assert contract.INTERFACES == tuple(n for _, n in interfaces)
    assert set(contract.RESULT_POLICIES) == set(tokens['result_policy'])
    assert set(contract.SCENE_SIGNALS) == set(tokens['scene_signal'])
    assert set(contract.ACCEPTED_EVIDENCE) == set(tokens['accepted_evidence'])
    # the classes a contract may accept are exactly the classes a TrainIt adapter publishes
    assert set(core.EVIDENCE_BY_NAME) == set(tokens['accepted_evidence'])
    assert {n: int(e) for n, e in core.EVIDENCE_BY_NAME.items()} == {
        n: v for n, v in _ints('evidence').items() if n in tokens['accepted_evidence']}
    assert {e.name.lower() for e in core.ADAPTER_EVIDENCE} == set(tokens['accepted_evidence'])
    assert {n: int(i) for n, i in core.INTERFACE_BY_NAME.items()} == {
        n: v for n, v in _ints('command_interface').items() if n != 'unknown'}


def test_the_capabilities_equal_the_token_file():
    assert {c.value for c in core.Capability} == set(h.load_tokens()['capability'])
    implied = {c for caps in core.IMPLIED_CAPABILITIES.values() for c in caps}
    assert implied <= set(core.Capability)
    assert set(core.IMPLIED_CAPABILITIES) == set(contract.INTERFACES)


def test_the_message_constants_equal_the_token_file():
    from trainit_interfaces.msg import EndEffectorActuatorState as A
    from trainit_interfaces.msg import EndEffectorState as S
    expected = {
        'tool_status': {n.lower()[len('STATUS_'):]: getattr(S, n) for n in dir(S) if n.startswith('STATUS_')},
        'grasp': {n.lower()[len('GRASP_'):]: getattr(S, n) for n in dir(S) if n.startswith('GRASP_')},
        'evidence': {n.lower()[len('EVIDENCE_'):]: getattr(S, n) for n in dir(S) if n.startswith('EVIDENCE_')},
        'command_interface': {n.lower()[len('INTERFACE_'):]: getattr(A, n) for n in dir(A)
                              if n.startswith('INTERFACE_')},
        'actuator_status': {n.lower()[len('STATUS_'):]: getattr(A, n) for n in dir(A) if n.startswith('STATUS_')},
        'binary': {n.lower()[len('BINARY_'):]: getattr(A, n) for n in dir(A) if n.startswith('BINARY_')},
        'command': {n.lower()[len('COMMAND_'):]: getattr(A, n) for n in dir(A) if n.startswith('COMMAND_')},
    }
    for group, constants in expected.items():
        assert constants == _ints(group), group
        assert constants == core.token_table(core.TOKEN_ENUMS[group]), group
