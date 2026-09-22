"""The installed-copy guarantee (physdemo contract C2, C4, C5, C7,
C8, C9; PSEUDOCODE 4.8): everything a run needs is inside the
package, because the package is all that `pip install` delivers, and
both routes create commands of the same names. These are structural
checks; the end-to-end check (build a wheel, install it elsewhere,
run it) is manual and is recorded in dev/notes/. INHERITED from the
physdemo skeleton."""

import ast
import importlib
import re
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:                      # Python 3.10
    import tomli as tomllib

import rigid_body
from rigid_body.cli.support import PACKAGE_DEFAULTS_DIR, load_rc_defaults

PACKAGE_NAME = 'rigid_body'
REPO = Path(__file__).resolve().parents[2]
PACKAGE = REPO / 'src' / PACKAGE_NAME
SCRIPTS = REPO / 'src' / 'scripts'
PYPROJECT = tomllib.loads((REPO / 'pyproject.toml').read_text())

# Import name -> the name pip knows the distribution by, where the two
# differ or where one distribution provides several modules.
DISTRIBUTION_OF = {'vtkmodules': 'vtk', 'tomllib': None}


def _declared():
    project = PYPROJECT['project']
    lines = list(project['dependencies'])
    for extra in project.get('optional-dependencies', {}).values():
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
            if name not in standard and name != PACKAGE_NAME}


def _script_names():
    return sorted(path.stem for path in SCRIPTS.glob('*.py')
                  if not path.stem.endswith('rc'))


def test_rc_defaults_are_inside_the_package():
    assert PACKAGE in PACKAGE_DEFAULTS_DIR.parents
    for rc_file in PACKAGE_DEFAULTS_DIR.glob('*rc.py'):
        assert load_rc_defaults(rc_file.name, {}, [PACKAGE_DEFAULTS_DIR])


def test_the_version_has_one_source():
    assert re.fullmatch(r'\d+\.\d+\.\d+', rigid_body.__version__)
    assert 'version' in PYPROJECT['project']['dynamic']
    assert 'version' not in PYPROJECT['project']
    assert PYPROJECT['tool']['setuptools']['dynamic']['version'] == \
        {'attr': f'{PACKAGE_NAME}.__version__'}


def test_every_imported_module_is_a_declared_dependency():
    declared = _declared()
    for module in _third_party_imports(PACKAGE):
        distribution = DISTRIBUTION_OF.get(module, module)
        if distribution is None:
            continue
        assert distribution.lower() in declared, \
            f'{module} is imported under src/{PACKAGE_NAME}/ but ' \
            f'pyproject.toml does not declare {distribution}'


def test_self_check_reports_on_the_declared_dependencies():
    """Every command module that defines CHECKED_DISTRIBUTIONS (at
    least one must) names exactly the declared dependencies, minus
    the Python 3.10 TOML backport."""
    required = {line.split(';')[0].split('>')[0].strip().lower()
                for line in PYPROJECT['project']['dependencies']}
    required.discard('tomli')                    # 3.10 backport only
    checked = []
    for target in PYPROJECT['project']['scripts'].values():
        module = importlib.import_module(target.split(':')[0])
        if hasattr(module, 'CHECKED_DISTRIBUTIONS'):
            checked.append(module.CHECKED_DISTRIBUTIONS)
    assert checked, 'no command module defines CHECKED_DISTRIBUTIONS'
    for distributions in checked:
        assert {name.lower() for name in distributions} == required


def test_both_routes_create_the_same_command_names():
    """Contract C5: install_tool.sh names a command after its script
    file, pip after its [project.scripts] key; nothing else would
    notice if they differed."""
    console_scripts = PYPROJECT['project']['scripts']
    assert sorted(console_scripts) == _script_names()
    for name, target in console_scripts.items():
        assert target == f'{PACKAGE_NAME}.cli.{name}:console_main'
        module_name, function_name = target.split(':')
        assert callable(getattr(importlib.import_module(module_name),
                                function_name))


def test_example_run_files_are_package_data():
    data = PYPROJECT['tool']['setuptools']['package-data']
    assert data[f'{PACKAGE_NAME}.examples'] == ['*.toml']
    assert (PACKAGE / 'examples' / '__init__.py').is_file()
    assert (PACKAGE / 'defaults' / '__init__.py').is_file()
    assert list((PACKAGE / 'examples').glob('*.toml'))


def test_the_script_fronts_are_only_fronts():
    """Contract C2: a front may import the standard library and the
    package's cli, and nothing else; all behaviour is in the package,
    so both routes run the same code."""
    for name in _script_names():
        source = (SCRIPTS / f'{name}.py').read_text()
        assert source.startswith('#!/usr/bin/env python3\n')
        assert '.resolve()' in source or 'realpath(' in source
        assert 'abspath(' not in source
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert alias.name.split('.')[0] in \
                        sys.stdlib_module_names
            elif isinstance(node, ast.ImportFrom):
                assert node.module.split('.')[0] in \
                    sys.stdlib_module_names \
                    or node.module.startswith(f'{PACKAGE_NAME}.cli')
        assert not any(isinstance(node, (ast.FunctionDef, ast.ClassDef))
                       for node in ast.walk(tree))
