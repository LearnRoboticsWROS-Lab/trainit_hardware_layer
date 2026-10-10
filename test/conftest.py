"""pytest fixtures of the hardware layer's vNext M2.6 tests.

The pure tests (contract, tokens, core) never touch ROS. Every test that needs rclpy asks for
``harness``: the session's rclpy context on domain 86 (graph-guarded once, see eef_harness), plus
a per-test executor; after the test every node is gone again or the next test refuses to run.
"""
import pytest


@pytest.fixture(scope='session')
def ros(tmp_path_factory):
    pytest.importorskip('rclpy', reason='the rclpy tests need a sourced ROS 2 Humble')
    import eef_harness
    session = eef_harness.RosSession(tmp_path_factory.mktemp('m2_6_hw'))
    yield session
    session.close()


@pytest.fixture
def harness(ros, request):
    import eef_harness
    h = eef_harness.Harness(ros)
    yield h
    h.close()
    ros.expect_clean(f'the test after {request.node.name}')
