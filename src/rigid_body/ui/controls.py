"""Scenario-editing controls: every edit is a new scenario, not a mutation.

This is the code form of PSEUDOCODE Section 15.4 and 15.6 (DESIGN Section
14.1, 14.5). The controls split in two (Section 15.1): the *time* controls,
which pace the display and live in ``dynamics/time_control.py``, and the
*scenario-editing* controls here, which set the body, the initial
conditions, the torque models, and the fidelity -- the physics zone.

The governing rule is that editing never mutates the running motion
(Section 15.1). A change of body constructs a new body, and more generally
a new *scenario* the engine runs from its initial condition: the running
trajectory is immutable, and a new setting is a new run. Because every edit
resolves into a scenario (Section 12), any state a student reaches by
fiddling is captured exactly and can be replayed or handed to the batch
tier -- exploration and reproducibility are one mechanism seen from two
ends (Section 15.4).

This module provides the pure, headless part of that surface: turning an
edit into a fresh scenario, and turning the labeled exaggeration factors
into honest on-screen text (Section 15.6). The actual widget bindings --
sliders and buttons reading a live vedo window into a
:class:`~rigid_body.dynamics.time_control.Controls` snapshot -- are the
renderer-bound part and are assembled with ``vedo_renderer.py``.
"""

import dataclasses


def apply_scenario_edit(scenario, edit):
    """Return a NEW scenario with the edited fields replaced.

    The realization of Section 15.4: it never mutates ``scenario`` (or the
    running motion it describes); it builds a fresh scenario record the
    session driver rebuilds the run from. ``edit`` is a mapping of
    top-level :class:`~rigid_body.scenario.scenario.Scenario` field names to
    their new values -- for a nested change (a single fidelity knob, say),
    build the replacement sub-record with :func:`replace_section` first. An
    empty or absent edit is a no-op that returns the scenario unchanged.
    """
    if not edit:
        return scenario
    return dataclasses.replace(scenario, **edit)


def replace_section(scenario, section_name, **field_changes):
    """Return a new scenario with one field of a sub-record replaced.

    A convenience for the common nested edit -- changing a single knob
    inside ``fidelity``, ``presentation``, ``initial_conditions``, and so
    on -- without the caller reconstructing the whole sub-record by hand.
    It replaces the named field on the sub-record, then replaces that
    sub-record on the scenario, so nothing is mutated in place (Section
    15.1).
    """
    section = getattr(scenario, section_name)
    edited_section = dataclasses.replace(section, **field_changes)
    return dataclasses.replace(scenario, **{section_name: edited_section})


def scale_setting_labels(scale_settings):
    """Return honest on-screen labels for the active exaggeration factors.

    Section 15.6, VISION Principle 12: whenever a control moves a quantity
    off its physical value, the factor is stated on screen. This turns the
    ``scale_settings`` a control carries into first-class label text the
    scene shows (Section 14.5). A factor of exactly one is left unlabeled,
    since it moves nothing off its physical value; every other factor is
    named, so a silent exaggeration is impossible.

    Returns a list of label strings, one per exaggerated quantity, ordered
    by the quantity name for a stable display.
    """
    labels = []
    for quantity_name in sorted(scale_settings):
        factor = scale_settings[quantity_name]
        if factor == 1:
            continue
        readable_name = quantity_name.replace("_", " ")
        labels.append(
            f"{readable_name} scaled {_format_factor(factor)}x")
    return labels


def _format_factor(factor):
    """Format a scale factor tidily: ``5`` not ``5.0``, ``2.5`` kept."""
    if float(factor) == int(factor):
        return str(int(factor))
    return str(factor)
