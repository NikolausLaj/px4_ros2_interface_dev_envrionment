"""Verify the simulated single-axis receiver's orientation."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from arva_sim.arva_sim_node import ArvaSim


@pytest.fixture
def node():
    rclpy.init(args=[])
    simulator = ArvaSim(parameter_overrides=[
        Parameter('transmitter_position_ned_m', value=[0.0, 0.0, 0.0]),
        Parameter('transmitter_heading_deg', value=0.0),
    ])
    yield simulator
    simulator.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('heading_deg, expected', [
    (0.0, None),
    (45.0, 10.0 * (1.5420 / math.sin(math.pi / 4)) ** (1 / 3)),
    (90.0, 10.0 * 1.5420 ** (1 / 3)),
    (-90.0, 10.0 * 1.5420 ** (1 / 3)),
    (180.0, None),
])
def test_distance_uses_receiver_forward_axis(node, heading_deg, expected):
    # This geometry produces a field pointing along the East axis.
    node._drone_pos = np.array([10.0, 0.0, 0.0])
    node._drone_heading = math.radians(heading_deg)
    distance, _ = node.computeArvaSignal()
    assert distance == pytest.approx(expected if expected is not None else -1.0)


def test_zero_forward_component_does_not_publish(node):
    node._drone_pos = np.array([10.0, 0.0, 0.0])
    node._drone_heading = 0.0
    messages = []
    node._arva_pub = SimpleNamespace(publish=messages.append)
    node.arvaTimer()
    assert messages == []


def test_near_field_uses_all_three_axes(node):
    node._drone_pos = np.array([1.0, 0.0, 0.0])
    for heading in (0.0, math.pi / 2):
        node._drone_heading = heading
        distance, angle = node.computeArvaSignal()
        assert distance == pytest.approx(1.5420 ** (1 / 3))
        assert angle == 0.0


def test_rotating_entire_scene_preserves_distance(node):
    node._tx_theta = -math.pi / 2
    node.spawnTransmitter()
    node._drone_pos = np.array([10.0, 0.0, -1.0])
    node._drone_heading = 0.0
    original, _ = node.computeArvaSignal()

    node._tx_theta = -math.pi
    node.spawnTransmitter()
    node._drone_pos = np.array([0.0, 10.0, -1.0])
    node._drone_heading = math.pi / 2
    rotated, _ = node.computeArvaSignal()
    assert original > 0.0
    assert rotated == pytest.approx(original)
