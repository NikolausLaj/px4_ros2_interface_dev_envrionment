"""Check transmitter configuration against the actual ROS node."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
import yaml
from px4_msgs.msg import VehicleLocalPosition

from arva_sim.arva_sim_node import ArvaSim


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


def test_saved_yaml_preserves_existing_transmitter():
    config = yaml.safe_load(
        (Path(__file__).parent.parent / 'cfg/arva_sim.yaml').read_text()
    )['arva_sim_node']['ros__parameters']
    node = ArvaSim(parameter_overrides=[
        Parameter(name, value=value) for name, value in config.items()
    ])
    try:
        np.testing.assert_allclose(node._tx_pos, [30.0, 20.0, 0.0])
        assert node._tx_theta == pytest.approx(-np.pi / 2)
        np.testing.assert_allclose(node._m_vec_wf, [-1.0, 0.0, 0.0], atol=1e-12)
    finally:
        node.destroy_node()


def test_parameters_override_position_and_dipole_rotation():
    node = ArvaSim(parameter_overrides=[
        Parameter('transmitter_position_ned_m', value=[-15.0, 40.0, 2.0]),
        Parameter('transmitter_heading_deg', value=0.0),
    ])
    try:
        np.testing.assert_allclose(node._tx_pos, [-15.0, 40.0, 2.0])
        np.testing.assert_allclose(node._m_vec_wf, [0.0, 1.0, 0.0])
        assert node._tx_theta == 0.0
    finally:
        node.destroy_node()


def test_direct_node_defaults_preserve_existing_transmitter():
    node = ArvaSim()
    try:
        np.testing.assert_allclose(node._tx_pos, [30.0, 20.0, 0.0])
        assert node._tx_theta == pytest.approx(-np.pi / 2)
    finally:
        node.destroy_node()


@pytest.mark.parametrize('name,value', [
    ('transmitter_position_ned_m', [0.0, 1.0]),
    ('transmitter_position_ned_m', [0.0, float('inf'), 0.0]),
    ('transmitter_heading_deg', float('nan')),
])
def test_invalid_transmitter_parameters_are_rejected(name, value):
    with pytest.raises(ValueError, match='transmitter_'):
        ArvaSim(parameter_overrides=[Parameter(name, value=value)])


def test_receiver_offset_preserves_ned_subtraction():
    node = ArvaSim(parameter_overrides=[
        Parameter('receiver_vertical_offset_m', value=1.2),
    ])
    try:
        node.localPositionCallback(VehicleLocalPosition(
            x=1.234, y=5.678, z=-2.987, heading=0.6,
        ))
        np.testing.assert_allclose(node._drone_pos, [1.23, 5.68, -4.19])
        assert node._drone_heading == pytest.approx(0.6)
    finally:
        node.destroy_node()


@pytest.mark.parametrize('distance,expected_valid', [
    (2.9, False), (3.0, False), (3.1, True),
])
def test_configured_angle_validity_threshold(distance, expected_valid):
    node = ArvaSim(parameter_overrides=[
        Parameter('angle_valid_min_distance_m', value=3.0),
    ])
    messages = []
    try:
        node._drone_pos = np.zeros(3)
        node.computeArvaSignal = lambda: (distance, 0.0)
        node._arva_pub = SimpleNamespace(publish=messages.append)
        node.arvaTimer()
        assert len(messages) == 1
        assert messages[0].angle_valid is expected_valid
        assert messages[0].distance_valid is True
    finally:
        node.destroy_node()


@pytest.mark.parametrize('detection_range,expected_detection', [(5.0, False), (15.0, True)])
def test_configured_range_limits_far_field_detection(detection_range, expected_detection):
    node = ArvaSim(parameter_overrides=[
        Parameter('transmitter_position_ned_m', value=[0.0, 0.0, 0.0]),
        Parameter('transmitter_heading_deg', value=90.0),
        Parameter('max_detection_range_m', value=detection_range),
    ])
    try:
        node._drone_pos = np.array([10.0, 0.0, 0.0])
        node._drone_heading = 0.0
        distance, _ = node.computeArvaSignal()
        if expected_detection:
            assert distance == pytest.approx(10.0 * (1.5420 / 2.0) ** (1.0 / 3.0))
        else:
            assert distance == -1.0
    finally:
        node.destroy_node()


@pytest.mark.parametrize('name,value', [
    ('receiver_vertical_offset_m', float('nan')),
    ('angle_valid_min_distance_m', -1.0),
    ('angle_valid_min_distance_m', float('inf')),
    ('max_detection_range_m', 0.0),
    ('max_detection_range_m', float('inf')),
])
def test_invalid_measurement_parameters_are_rejected(name, value):
    with pytest.raises(ValueError, match=name):
        ArvaSim(parameter_overrides=[Parameter(name, value=value)])
