#!/usr/bin/python3

# Copyright 2020-2022 Josh Pieper, jjp@pobox.com.
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

import argparse
import datetime
import subprocess
import sys


def run(cmd):
    print('>{}'.format(cmd))
    subprocess.check_call(cmd, shell=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--force', action='store_true')
    parser.add_argument('outdir')
    args = parser.parse_args()

    outdir = args.outdir

    # Make sure git is clean first.
    dirty = (subprocess.run("git diff-index --quiet HEAD --",
                            shell=True).returncode != 0)
    if dirty and not args.force:
        raise RuntimeError("git is dirty, cannot release!")
    git_hash = subprocess.check_output(
        "git rev-parse HEAD", shell=True).decode('utf8').strip()

    datestr = datetime.datetime.now().strftime('%Y%m%d')

    print("Details:")
    print(" Date:", datestr)
    print(" Output Directory:", outdir)
    print(" Git Hash:", git_hash)
    print("Building...")
    print()

    run('tools/bazel clean --expunge')
    run('tools/bazel build --cpu stm32g4 -c opt //:target')

    run(f'cp bazel-bin/fw/pi3_hat.elf {outdir}/{datestr}-pi3hat-{git_hash}.elf')

    for pyver in ['3.7', '3.9']:
        for arch in ['pi', 'pi64']:
            pyarch = {
                'pi' : 'armv7l',
                'pi64' : 'aarch64',
            }[arch]
            run(f'tools/bazel build --config={arch} --define PYTHON={pyver} -c opt //:pi3hat_tools')
            run(f'cp bazel-bin/pi3hat_tools.tar.bz2 {outdir}/{datestr}-pi3hat_tools-cp{pyver.replace(".","")}-{pyarch}-{git_hash}.tar.bz2')

            wheel_name = subprocess.check_output("tar --list -f bazel-bin/pi3hat_tools.tar.bz2 | grep \.whl$", shell=True).decode('utf8').strip()
            run(f'tar -C {outdir} --strip-components 2 --extract -f bazel-bin/pi3hat_tools.tar.bz2 {wheel_name}')


    print()
    print('DONE')

if __name__ == '__main__':
    main()
