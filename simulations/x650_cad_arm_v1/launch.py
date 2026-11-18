#!/usr/bin/env python3
"""Create a fresh fixed-base CAD-arm world and run Gazebo.

python launch.py                 # GUI; joint targets start at zero
python launch.py --exercise      # GUI; repeatable 10-second motion then hold
python launch.py --check         # headless, bounded 10-second audit

Manual commands in another terminal must use the printed GZ_PARTITION:
gz topic -t /x650_cad_arm/upper/position -m gz.msgs.Double -p 'data: 0.15'
gz topic -t /x650_cad_arm/crank/position -m gz.msgs.Double -p 'data: 0.10'
Targets are radians relative to the imported CAD zero pose, not absolute angles.
This is a fixed-base, collision-free bench model; it does not start PX4.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--exercise', action='store_true')
    p.add_argument('--check', action='store_true')
    p.add_argument('--package', type=Path)
    args = p.parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[1]
    build = root / 'generated/x650_cad_arm_build'
    subprocess.run(['cmake','-S',str(here),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],check=True,stdout=subprocess.DEVNULL)
    subprocess.run(['cmake','--build',str(build),'-j2'],check=True,stdout=subprocess.DEVNULL)
    cmd = [sys.executable,str(here / 'prepare.py')]
    if args.exercise or args.check:
        cmd.append('--exercise')
    if args.package:
        cmd.extend(['--package',str(args.package)])
    directory = Path(subprocess.check_output(cmd,text=True).strip())
    print('Run directory: ' + str(directory),flush=True)
    if args.check:
        raise SystemExit(subprocess.call([sys.executable,str(here / 'check.py'),str(directory)]))
    print('For manual commands: export GZ_PARTITION=x650-cad-' + directory.name.removeprefix('x650_cad_arm_'),flush=True)
    os.execvp('bash',['bash',str(directory / 'open.sh')])


if __name__ == '__main__':
    main()
