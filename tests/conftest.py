"""Shared pytest configuration.

This project ships no packaging file; instead the importable library
under ``src/`` is placed on ``sys.path`` here, so that tests can
``import rigid_body`` exactly as the eventual entry-point scripts will.
Run the suite from inside the project's dedicated ``rigid`` virtual
environment, which supplies numpy, scipy, and the rendering stack.
"""

import os
import sys

# Put the ``src`` directory on the import path so that ``rigid_body`` and
# its subpackages resolve without an editable install.
_source_directory = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src'))
if _source_directory not in sys.path:
    sys.path.insert(0, _source_directory)
