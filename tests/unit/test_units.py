"""Unit tests for the units boundary (core/units.py).

These confirm that authored strings parse to the right SI floats, that
units convert (centimeters to meters, degrees to radians), and that a
value with the wrong dimension is rejected at the boundary rather than
silently accepted.
"""

import math

import pytest

from rigid_body.core.units import parse_quantity


def test_moment_of_inertia_parses_to_si():
    assert parse_quantity("0.5 kg*m^2", "kg*m^2") == pytest.approx(0.5)


def test_acceleration_parses_to_si():
    assert parse_quantity("9.81 m/s^2", "m/s^2") == pytest.approx(9.81)


def test_density_parses_to_si():
    assert parse_quantity("2700 kg/m^3", "kg/m^3") == pytest.approx(2700.0)


def test_length_converts_to_meters():
    assert parse_quantity("1 cm", "m") == pytest.approx(0.01)


def test_angle_in_degrees_converts_to_radians():
    assert parse_quantity("30 deg", "rad") == pytest.approx(math.pi / 6.0)


def test_damping_coefficient_parses():
    assert parse_quantity("1.0e-3 N*m*s", "N*m*s") == pytest.approx(1.0e-3)


def test_wrong_dimension_is_rejected():
    # A length offered where a moment of inertia is expected.
    with pytest.raises(ValueError):
        parse_quantity("0.5 m", "kg*m^2")


def test_unparseable_string_is_rejected():
    with pytest.raises(ValueError):
        parse_quantity("not a quantity", "m")
