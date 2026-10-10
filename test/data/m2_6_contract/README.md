# Pinned byte copies of the vNext M2.6 end-effector contract files (ADR-0022)

Copied byte for byte from the vNext M2.6 forensics (`contract/`, outside every repository) on
2026-10-10. The same bytes live in the test data of `trainit_interfaces` (the token file),
`trainit_motion_runtime` and `trainit_setup_assistant`; no test reads a path in another
repository. `test/eef_harness.py` asserts each sha256 before any use.

| File | sha256 | What it is |
|---|---|---|
| `normalize_eef.py` | `9fba00c3981762974d9e916464fd670c8f83964392649c3e0854a94149290a0b` | the reference normaliser of the `eef` parameter block: the schema in executable form |
| `eef_conformance.yaml` | `873a1d582c4e3947a62026d153bd49ccb02d7b4d38e5cb3a5b4b3a8f5a27a04f` | the conformance fixture: four devices (composite, suction, jaw, binary jaw), synthetic names |
| `eef_conformance.expected.txt` | `1f6bd1d2ac2fb18865ee67a815877483537d0a325fbc9a5400cf2977710d242c` | the fixture's normalised dump |
| `eef_contract_tokens.txt` | `55048ea44d466979f57c28a634295a060a79b837fe614a30c49f60572a2bd5de` | the canonical tokens |

These files are never edited here and never re-frozen to make a test pass. A change is a change
to ADR-0022, made at the source and copied to every repository in lockstep.

History: the first copy (sha256 `57063387…bd384681`) raised TypeError, instead of refusing, on a
list where a site's `grip`, `engage` or `release` expects a name, and accepted non-ASCII letters
in a name token. Both were fixed at the source on 2026-10-10 (an `isinstance(x, str)` guard on the
three keys; `str.isascii()` in the token rule) and re-copied to every repository at once. The
reference and `contract.py` now give the same outcome for every input `test_eef_contract.py`
sweeps, a list in those keys included.

The second copy (sha256 `18acf5d0…af586f11`) accepted two blocks the runtime cannot honour, both
found by the M2.6 adversarial review and refused at the source on 2026-10-10, again in every
repository at once (the fixture and its dump did not change):
- a `set_bool` grip whose site swaps its targets (`engage: release`): the polarity of a SetBool
  actuator is the adapter's, so such a site is refused at `sites.<s>.engage`;
- a `gripper_command` actuator with two targets of equal value: an adapter tells ENGAGE, RELEASE
  and TARGET apart by the goal's position only, so the values must be pairwise distinct (refused
  at `actuators.<a>.targets`).
