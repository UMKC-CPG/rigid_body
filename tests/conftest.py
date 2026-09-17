"""Shared pytest configuration.

The importable library under ``src/`` is placed on ``sys.path`` here, so
that the tests run against the working tree with no install step, exactly
as the executable fronts in ``src/scripts/`` do (a ``pip install`` of the
package is the other route, ARCHITECTURE Section 9.5). Run the suite from
the physdemo suite's environment (``sdemo``), which supplies numpy, scipy,
and the rendering stack.
"""

import os
import sys

# Put the ``src`` directory on the import path so that ``rigid_body`` and
# its subpackages resolve without an editable install.
_source_directory = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src'))
if _source_directory not in sys.path:
    sys.path.insert(0, _source_directory)

# The suite always draws offscreen. Choose VTK's window class now, before
# any test imports the renderer, so that a DISPLAY that is set but dead
# cannot hang a render test (PSEUDOCODE Section 16.4).
from rigid_body.render.offscreen import prepare_offscreen  # noqa: E402

prepare_offscreen()
