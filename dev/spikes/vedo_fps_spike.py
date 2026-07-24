#!/usr/bin/env python3
"""Architectural spike: measure vedo/VTK render throughput.

This is a throwaway benchmark whose only purpose is to retire the
rendering risk recorded in ARCHITECTURE.md section 6.3. It builds a
scene deliberately representative of the real simulation -- a rigid
body, its momental ellipsoid, the invariable plane, the polhode and
herpolhode traces, and both sets of coordinate axes -- and then
measures how many frames per second the renderer can sustain while
animating it.

The scene is drawn offscreen, so no X server or display is required.
That isolates the question of raw render throughput from the separate
question of how frames are delivered to a viewer.
"""

import argparse
import json
import os
import statistics
import sys
import time


def parse_command_line():
    """Collect the knobs that control scene complexity and run length."""

    parser = argparse.ArgumentParser(
        description="Measure vedo render throughput for a scene "
                    "representative of the rigid-body simulation.")
    parser.add_argument('--frames', type=int, default=300,
                        help='Number of frames to render and time.')
    parser.add_argument('--warmup-frames', type=int, default=30,
                        help='Frames rendered before timing begins.')
    parser.add_argument('--window-width', type=int, default=1280,
                        help='Render window width in pixels.')
    parser.add_argument('--window-height', type=int, default=960,
                        help='Render window height in pixels.')
    parser.add_argument('--ellipsoid-resolution', type=int, default=48,
                        help='Latitude/longitude divisions of the '
                             'momental ellipsoid wireframe.')
    parser.add_argument('--trace-points', type=int, default=4000,
                        help='Points retained in each of the polhode '
                             'and herpolhode traces.')
    parser.add_argument('--label', type=str, default='unlabeled',
                        help='Tag identifying this run in the output.')
    parser.add_argument('--save-image', type=str, default=None,
                        help='Write the final frame to this PNG path, '
                             'so the render can be inspected by eye.')
    return parser.parse_args()


def describe_opengl_context(plotter):
    """Extract the GL vendor and renderer strings from VTK.

    Knowing whether we landed on real GPU hardware or on a software
    rasterizer is the whole point of the exercise, so this string is
    reported alongside the frame rate.
    """

    try:
        capabilities = plotter.window.ReportCapabilities()
    except Exception as problem:               # pragma: no cover
        return f"<capabilities unavailable: {problem}>"

    interesting_prefixes = ("OpenGL vendor string",
                            "OpenGL renderer string",
                            "OpenGL version string")
    found_lines = [line.strip()
                   for line in capabilities.splitlines()
                   if line.strip().startswith(interesting_prefixes)]
    return " | ".join(found_lines) if found_lines else "<no GL strings>"


def verify_pixels_were_actually_drawn(plotter):
    """Confirm the renderer produced a real image, not an empty buffer.

    A render window without a valid OpenGL context can accept Render()
    calls and return immediately, which yields a spectacular and
    entirely fictitious frame rate. Timing that would be worse than
    not measuring at all, so before trusting any number we read the
    framebuffer back and check that the scene actually appears in it.

    Returns the fraction of pixels differing from the background and
    the count of distinct colors present.
    """

    import numpy as np

    try:
        captured_image = plotter.screenshot(asarray=True)
    except Exception as problem:               # pragma: no cover
        return {'pixels_verified': False,
                'reason': f"screenshot failed: {problem}"}

    if captured_image is None or captured_image.size == 0:
        return {'pixels_verified': False,
                'reason': 'empty framebuffer returned'}

    # The background is black, so any pixel with appreciable intensity
    # belongs to the scene we asked for.
    pixel_intensity = captured_image.reshape(-1, captured_image.shape[-1])
    is_foreground = pixel_intensity.max(axis=1) > 12
    foreground_fraction = float(is_foreground.mean())
    distinct_color_count = int(
        len(np.unique(pixel_intensity[::37], axis=0)))

    # A correct render of this scene covers a healthy share of the
    # frame with many distinct colors. A blank or near-blank buffer
    # fails both tests.
    looks_like_a_real_render = (foreground_fraction > 0.01
                                and distinct_color_count > 8)

    return {
        'pixels_verified': looks_like_a_real_render,
        'foreground_pixel_fraction': round(foreground_fraction, 4),
        'distinct_colors_sampled': distinct_color_count,
    }


def build_representative_scene(settings):
    """Assemble a scene with the same drawable content as the real tool.

    The geometry here is static and physically meaningless; only its
    *rendering cost* is representative. Vertex counts and object counts
    are what determine the frame rate, not whether the polhode shown is
    the true polhode.
    """

    import numpy as np
    import vedo

    scene_objects = []

    # The rigid body itself: a solid orthorhombic parallelepiped, which
    # is the shape used for the classic Dzhanibekov demonstration.
    rigid_body = vedo.Box(pos=(0, 0, 0), length=1.0, width=1.5,
                          height=0.2).c('tan').lighting('default')
    scene_objects.append(rigid_body)

    # The momental ellipsoid, drawn as a wireframe so the body inside
    # remains visible. This is typically the heaviest single object.
    momental_ellipsoid = vedo.Sphere(
        r=1.0, res=settings.ellipsoid_resolution)
    momental_ellipsoid.scale([1.6, 1.15, 0.75])
    momental_ellipsoid.wireframe(True).c('gray').alpha(0.4)
    scene_objects.append(momental_ellipsoid)

    # The invariable plane on which the ellipsoid rolls.
    invariable_plane = vedo.Plane(pos=(0, 0, -1.1), s=(4.0, 4.0))
    invariable_plane.c('lightblue').alpha(0.25)
    scene_objects.append(invariable_plane)

    # The polhode (traced on the ellipsoid) and the herpolhode (traced
    # on the invariable plane). Long polylines are a realistic and
    # non-trivial share of the per-frame vertex load.
    trace_parameter = np.linspace(0, 12 * np.pi, settings.trace_points)
    polhode_points = np.column_stack([
        1.6 * np.cos(trace_parameter) * 0.6,
        1.15 * np.sin(trace_parameter) * 0.6,
        0.75 * np.cos(3 * trace_parameter) * 0.3])
    herpolhode_points = np.column_stack([
        1.3 * np.cos(0.7 * trace_parameter),
        1.3 * np.sin(0.7 * trace_parameter),
        np.full_like(trace_parameter, -1.1)])
    polhode_trace = vedo.Line(polhode_points, lw=2).c('orange')
    herpolhode_trace = vedo.Line(herpolhode_points, lw=2).c('yellow')
    scene_objects.extend([polhode_trace, herpolhode_trace])

    # The angular momentum and angular velocity vectors.
    momentum_arrow = vedo.Arrow((0, 0, 0), (0, 0, 1.8), c='white')
    velocity_arrow = vedo.Arrow((0, 0, 0), (0.9, 0.5, 1.1), c='magenta')
    scene_objects.extend([momentum_arrow, velocity_arrow])

    # Both coordinate frames, since VISION Goal 5 requires showing the
    # body frame and the space frame at the same time.
    axis_directions = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    space_frame_colors = ['red', 'green', 'blue']
    body_frame_colors = ['darkred', 'darkgreen', 'darkblue']
    for direction, color in zip(axis_directions, space_frame_colors):
        tip = tuple(2.2 * component for component in direction)
        scene_objects.append(vedo.Arrow((0, 0, 0), tip, c=color))
    for direction, color in zip(axis_directions, body_frame_colors):
        tip = tuple(1.4 * component for component in direction)
        scene_objects.append(vedo.Arrow((0, 0, 0), tip, c=color))

    # The live numeric readout of conserved quantities.
    telemetry_text = vedo.Text2D(
        "t = 0.000 s\n|omega| = 0.938 rad/s\n|L| = 0.641 N.m\n"
        "E_kin = 0.2332 J\ndrift(E) = 1.2e-9",
        pos='top-left', s=0.8)
    scene_objects.append(telemetry_text)

    return scene_objects, rigid_body, momental_ellipsoid


def measure_frame_rate(settings):
    """Render the scene repeatedly and report frame-rate statistics."""

    import vedo

    vedo.settings.immediate_rendering = False

    scene_objects, rigid_body, momental_ellipsoid = \
        build_representative_scene(settings)

    plotter = vedo.Plotter(
        offscreen=True, bg='black', axes=0,
        size=(settings.window_width, settings.window_height))
    plotter.show(scene_objects, interactive=False, resetcam=True)

    opengl_description = describe_opengl_context(plotter)
    pixel_verification = verify_pixels_were_actually_drawn(plotter)

    # Warm-up frames absorb one-time costs: shader compilation, vertex
    # buffer uploads, and driver initialization. Timing these would
    # understate the sustained frame rate badly.
    for _ in range(settings.warmup_frames):
        rigid_body.rotate_z(1.0)
        plotter.render()

    frame_durations = []
    for _ in range(settings.frames):
        frame_start_time = time.perf_counter()

        # Animate the way the real tool will: the body and its momental
        # ellipsoid tumble together while the traces stay in place.
        rigid_body.rotate_z(1.0)
        rigid_body.rotate_x(0.6)
        momental_ellipsoid.rotate_z(1.0)
        momental_ellipsoid.rotate_x(0.6)

        plotter.render()
        frame_durations.append(time.perf_counter() - frame_start_time)

    if settings.save_image:
        plotter.screenshot(settings.save_image)

    plotter.close()

    sorted_durations = sorted(frame_durations)
    worst_five_percent_index = int(0.95 * len(sorted_durations)) - 1
    mean_duration = statistics.mean(frame_durations)

    return {
        'label': settings.label,
        'opengl': opengl_description,
        **pixel_verification,
        'frames': settings.frames,
        'resolution': f"{settings.window_width}x"
                      f"{settings.window_height}",
        'ellipsoid_resolution': settings.ellipsoid_resolution,
        'trace_points': settings.trace_points,
        'mean_fps': round(1.0 / mean_duration, 1),
        'median_fps': round(
            1.0 / statistics.median(frame_durations), 1),
        'worst_5pct_fps': round(
            1.0 / sorted_durations[worst_five_percent_index], 1),
        'mean_frame_ms': round(1000.0 * mean_duration, 2),
    }


def main():
    settings = parse_command_line()
    try:
        results = measure_frame_rate(settings)
    except Exception as problem:
        print(json.dumps({'label': settings.label,
                          'error': f"{type(problem).__name__}: "
                                   f"{problem}"}))
        return 1
    print(json.dumps(results))
    return 0


if __name__ == '__main__':
    sys.exit(main())
