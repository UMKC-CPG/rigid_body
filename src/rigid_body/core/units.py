"""The units boundary: authored unit strings in, bare SI floats out.

This is the one module permitted to touch the units library (ARCHITECTURE
Section 5.5, PSEUDOCODE Section 12.4). A scenario authors physical
quantities as human-readable strings -- ``"0.5 kg*m^2"``, ``"9.81 m/s^2"``,
``"30 deg"`` -- and this module parses each to a plain SI float exactly
once, on load. Everything below ``scenario/`` then works in bare SI, with
no unit object anywhere (DESIGN Section 2.1).

Parsing checks the dimension: a value offered for a moment of inertia must
actually have the dimension of a moment of inertia, so a mistyped unit is
rejected at the boundary rather than integrated into nonsense.
"""

import pint


# A single registry for the whole program; unit definitions are shared.
_UNIT_REGISTRY = pint.UnitRegistry()


def parse_quantity(text, expected_units):
    """Parse an authored quantity string to a bare SI float.

    ``text`` is the authored value such as ``"0.5 kg*m^2"``, and
    ``expected_units`` is a units string of the dimension it must have,
    such as ``"kg*m^2"``. The value is converted to SI base units and its
    magnitude returned. A dimension that does not match the expected one,
    or a string that cannot be parsed, raises ``ValueError``.

    Angles are treated as a dimension here: ``"30 deg"`` parses against
    ``"rad"`` and returns the angle in radians, the SI base.
    """
    try:
        quantity = _UNIT_REGISTRY.Quantity(text)
        expected = _UNIT_REGISTRY.Quantity(expected_units)
    except Exception as error:
        raise ValueError(
            f"could not parse quantity {text!r} against units "
            f"{expected_units!r}: {error}")

    if quantity.dimensionality != expected.dimensionality:
        raise ValueError(
            f"quantity {text!r} has dimension {quantity.dimensionality} "
            f"but {expected_units!r} expects "
            f"{expected.dimensionality}")

    return float(quantity.to_base_units().magnitude)
