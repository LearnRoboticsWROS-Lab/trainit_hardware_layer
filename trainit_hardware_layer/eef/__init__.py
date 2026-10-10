"""The canonical end-effector contract, adapter side (ADR-0022, trainit_hardware_layer 0.2.0).

- ``contract``: the parser of the ``eef`` parameter block TMR also reads (the port of the reference
  normaliser; same acceptance, same refusal keys, same dump).
- ``core``: the pure per-device state model (actuators, grasp sites, the LOST machine, the status
  aggregation, the publish/heartbeat rule) and the ``EndEffectorState`` message builder.
- ``adapter_node``: the ROS shell both adapters share (the state topic, the SetBool actuators, the
  scene signal, the compatibility endpoints).
- ``mock_eef_adapter`` / ``isaac_eef_adapter``: the two adapter rows ``trainit_mock`` and
  ``trainit_isaac_surface_gripper`` (console scripts of the same names).

Application code never imports this package: it names the contract (``eef/<id>/...``), and the
backend is a launch choice.
"""
