"""The installed-copy guarantee (ARCHITECTURE 8.6(4), pseudocode
16.5): everything a run needs is inside the package, because the
package is all that `pip install` delivers. These are structural
checks; the end-to-end check (build a wheel, install it elsewhere, run
it) is manual and is recorded in dev/notes/."""

import ast
import importlib
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:                      # Python 3.10
    import tomli as tomllib

from rigid_body.cli.support import (CHECKED_DISTRIBUTIONS,
                                    PACKAGE_DEFAULTS_DIR, load_rc_defaults)

REPO = Path(__file__).resolve().parents[2]
PACKAGE = REPO / 'src' / 'rigid_body'
PYPROJECT = tomllib.loads((REPO / 'pyproject.toml').read_text())

# Import name -> the name pip knows the distribution by, where the two
# differ or where one distribution provides several modules.
DISTRIBUTION_OF = {'vtkmodules': 'vtk', 'tomli_w': 'tomli_w',
                   'tomllib': None}               # tomllib is stdlib 3.11+


def _declared():
    project = PYPROJECT['project']
    lines = list(project['dependencies'])
    for extra in project['optional-dependencies'].values():
        lines += extra
    names = set()
    for line in lines:
        name = line.split(';')[0]
        for separator in '><=!~ ':
            name = name.split(separator)[0]
        names.add(name.lower().replace('-', '_'))
    return names


def _third_party_imports(root):
    found = set()
    for path in root.rglob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                found |= {alias.name.split('.')[0] for alias in node.names}
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                found.add(node.module.split('.')[0])
    standard = set(sys.stdlib_module_names) | {'tomllib'}
    return {name for name in found
            if name not in standard and name != 'rigid_body'}


def test_rc_defaults_are_inside_the_package():
    assert PACKAGE in PACKAGE_DEFAULTS_DIR.parents
    for name in ('rbsimrc.py', 'rbbatchrc.py'):
        assert load_rc_defaults(name, {}, [PACKAGE_DEFAULTS_DIR])


def test_every_imported_module_is_a_declared_dependency():
    declared = _declared()
    for module in _third_party_imports(PACKAGE):
        distribution = DISTRIBUTION_OF.get(module, module)
        assert distribution.lower() in declared, \
            f'{module} is imported under src/rigid_body/ but ' \
            f'pyproject.toml does not declare {distribution}'


def test_self_check_reports_on_the_declared_dependencies():
    required = {line.split(';')[0].split('>')[0].strip().lower()
                for line in PYPROJECT['project']['dependencies']}
    required.discard('tomli')                    # 3.10 backport only
    assert {name.lower() for name in CHECKED_DISTRIBUTIONS} == required


def test_console_script_targets_exist():
    for target in PYPROJECT['project']['scripts'].values():
        module_name, function_name = target.split(':')
        assert callable(getattr(importlib.import_module(module_name),
                                function_name))


def test_example_run_files_are_package_data():
    data = PYPROJECT['tool']['setuptools']['package-data']
    assert data['rigid_body.examples'] == ['*.toml']
    assert (PACKAGE / 'examples' / '__init__.py').is_file()
    assert (PACKAGE / 'defaults' / '__init__.py').is_file()


def test_the_script_fronts_are_only_fronts():
    """Each src/scripts/*.py may import the standard library and
    rigid_body.cli, and nothing else: all behaviour is in the package,
    so both routes run the same code."""
    scripts = sorted((REPO / 'src' / 'scripts').glob('*.py'))
    assert [s.name for s in scripts] == ['rbbatch.py', 'rbsim.py']
    for script in scripts:
        tree = ast.parse(script.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert alias.name.split('.')[0] in \
                        sys.stdlib_module_names
            elif isinstance(node, ast.ImportFrom):
                assert node.module.split('.')[0] in \
                    sys.stdlib_module_names \
                    or node.module.startswith('rigid_body.cli')
        assert not any(isinstance(node, ast.FunctionDef)
                       for node in ast.walk(tree))
