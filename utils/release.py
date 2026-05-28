#!/usr/bin/python3 -B

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Cut a unified release tag for pi3hat.

Usage:
    utils/release.py <bump|version>

pi3hat ships firmware, the Python client (moteus_pi3hat), and the C++
client together under a single version and a single `vX.Y.Z` tag.  This
helper bumps that one version everywhere it lives, commits, and creates
the annotated tag.  Pushing the tag fires build-release.yml, which builds
all three artifact kinds and attaches them to one GitHub Release.

Version sources kept in sync:
    lib/python/BUILD   VERSION="X.Y.Z"        (full PEP 440 string)
    CMakeLists.txt     project(... VERSION X.Y.Z)  (M.m.p only)

Bump types:
    major | minor | patch
    rc              iterate a pre-release suffix (1.0.0rc1 -> 1.0.0rc2)
    <explicit>      e.g. 1.0.0rc1, 1.1.0

When current is a pre-release, `patch` graduates it (1.0.0rc1 -> 1.0.0).

The version must be a normalized PEP 440 string (1.0.0, 1.0.0rc1,
1.0.0b1, 1.0.0.dev1).  PyPI and the bazel wheel build name artifacts by
the normalized form, so a semver-style spelling such as 1.0.0-rc1 is
rejected.  CMake's project(VERSION) only accepts M.m.p, so any
pre-release suffix is dropped from the CMakeLists value (but kept in the
tag and the Python version).
"""

import argparse
import pathlib
import re
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

PYTHON_VERSION_FILE = 'lib/python/BUILD'
PYTHON_VERSION_RE = r'^VERSION="([^"]+)"'

CMAKE_VERSION_FILE = 'CMakeLists.txt'
CMAKE_VERSION_RE = r'^(project\(moteus VERSION )[^)]+\)'


def git(*args, capture=False, check=True):
    return subprocess.run(['git', *args], cwd=REPO_ROOT, check=check,
                          capture_output=capture, text=True)


def read_version(path, regex):
    text = (REPO_ROOT / path).read_text()
    m = re.search(regex, text, re.MULTILINE)
    if not m:
        sys.exit(f'could not find version in {path}')
    return m.group(1)


def write_version(path, regex, replacement):
    full = REPO_ROOT / path
    full.write_text(re.sub(regex, replacement, full.read_text(),
                           count=1, flags=re.MULTILINE))


def compute_new_version(current, bump):
    m = re.match(r'^(\d+)\.(\d+)\.(\d+)(.*)$', current)
    if not m:
        sys.exit(f'could not parse current version: {current!r}')
    major, minor, patch = int(m.group(1)), int(m.group(2)), int(m.group(3))
    suffix = m.group(4)

    if bump == 'major':
        return f'{major+1}.0.0'
    if bump == 'minor':
        return f'{major}.{minor+1}.0'
    if bump == 'patch':
        # Graduate a pre-release: 1.0.0rc1 + patch -> 1.0.0.
        # Otherwise bump the patch number normally.
        return f'{major}.{minor}.{patch}' if suffix else f'{major}.{minor}.{patch+1}'
    if bump == 'rc':
        # Iterate a pre-release: increment the trailing number.
        if not suffix:
            sys.exit(f'current {current!r} has no pre-release suffix; '
                     f'use an explicit version to start one (e.g. '
                     f'{major}.{minor}.{patch+1}rc1)')
        m2 = re.match(r'^(.*?)(\d+)$', suffix)
        if not m2:
            sys.exit(f'pre-release {suffix!r} has no numeric tail to bump')
        return f'{major}.{minor}.{patch}{m2.group(1)}{int(m2.group(2))+1}'
    # Explicit version.
    if re.match(r'^\d+\.\d+\.\d+([.-]?[A-Za-z0-9.+-]*)?$', bump):
        return bump
    sys.exit(f'unknown bump or invalid version: {bump!r}')


def validate_python_version(version):
    """Reject versions that are not in normalized PEP 440 form.

    `python3 -m build` (and PyPI) name the wheel using the PEP 440
    *normalized* version, but the genrule in lib/python/BUILD declares
    its output filenames straight from the raw VERSION string.  A
    non-normalized spelling such as '1.0.0-rc1' would make the declared
    outputs never match what build produces, breaking the wheel build.
    """
    try:
        from packaging.version import Version, InvalidVersion
    except ImportError:
        sys.exit('release.py needs the `packaging` module to validate the '
                 'version (apt install python3-packaging, or pip install '
                 'packaging)')
    try:
        normalized = str(Version(version))
    except InvalidVersion:
        sys.exit(f'{version!r} is not a valid PEP 440 version')
    if normalized != version:
        sys.exit(
            f'\nERROR: version {version!r} is not normalized PEP 440.'
            f'\n\nPyPI and the bazel wheel build name artifacts by the '
            f'normalized form, so\nthis spelling would break the build.  Use '
            f'{normalized!r} instead:\n\n'
            f'    utils/release.py {normalized}\n')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bump',
                        help='major | minor | patch | rc | <explicit version>')
    args = parser.parse_args()

    current = read_version(PYTHON_VERSION_FILE, PYTHON_VERSION_RE)
    print(f'Current version: {current}')
    new = compute_new_version(current, args.bump)
    print(f'New     version: {new}')

    validate_python_version(new)

    tag = f'v{new}'
    if git('rev-parse', '-q', '--verify', f'refs/tags/{tag}',
           capture=True, check=False).returncode == 0:
        sys.exit(f'tag {tag} already exists')

    write_version(PYTHON_VERSION_FILE, r'^VERSION="[^"]+"',
                  f'VERSION="{new}"')

    # CMake project(... VERSION X.Y.Z) does not allow pre-release
    # identifiers; keep only the leading M.m.p for the file value.
    cmake_value = re.match(r'^(\d+\.\d+\.\d+)', new).group(1)
    write_version(CMAKE_VERSION_FILE, CMAKE_VERSION_RE,
                  rf'\g<1>{cmake_value})')

    changed = [f for f in (PYTHON_VERSION_FILE, CMAKE_VERSION_FILE)
               if git('diff', '--quiet', '--', f, check=False).returncode != 0]
    if changed:
        git('add', *changed)
        git('commit', '-m', f'Release {new}')
    else:
        print('(no version-file change to commit; tagging current HEAD)')

    git('tag', '-a', tag, '-m', f'pi3hat {new}')

    print(f'\nCreated tag: {tag}\n')
    print('Next steps:')
    print('  git push origin HEAD')
    print(f'  git push origin {tag}\n')
    print('After the tag is pushed, build-release.yml builds the firmware, '
          'Python,')
    print('and C++ artifacts and attaches them all to one draft GitHub '
          'Release.')


if __name__ == '__main__':
    main()
