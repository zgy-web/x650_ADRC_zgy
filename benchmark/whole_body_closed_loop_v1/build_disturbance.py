#!/usr/bin/env python3
"""Compile/test only the isolated native disturbance plugin; never fly."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

HERE = Path(__file__).resolve().parent


def main():
    source = HERE / 'disturbance.cpp'
    sha = hashlib.sha256(source.read_bytes()).hexdigest()
    test = HERE / 'disturbance_self_test'
    test_command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic',
                    '-DWBCLOSED_DISTURBANCE_SELF_TEST', str(source), '-o', str(test)]
    subprocess.run(test_command, check=True)
    result = json.loads(subprocess.check_output([str(test)], text=True))
    if result.get('passed') is not True:
        raise RuntimeError('Disturbance self-test failed')
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-sim8'], text=True))
    library = HERE / 'libdisturbance.so'
    temporary = library.with_suffix('.so.building')
    command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-fPIC', '-shared',
               str(source), '-o', str(temporary), *flags]
    subprocess.run(command, check=True)
    if hashlib.sha256(source.read_bytes()).hexdigest() != sha:
        raise RuntimeError('Source changed during build')
    temporary.replace(library)
    result.update(source_sha256=sha, library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                  test_command=test_command, compile_command=command,
                  coverage='Pure schedule evaluator test plus native compile; live ECS enqueue validated in each run')
    (HERE / 'disturbance_build.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
