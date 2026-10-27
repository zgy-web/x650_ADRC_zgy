#!/usr/bin/env python3
"""Linux target helper: tar on stdin, RAM-only build/tests, persist four ELFs.

Invoke via SSH ``python3 -c <this source> --sha256 ... --runtime-root ...``.
All stdout/stderr must be collected by the workstation. No experiment corpus
is accepted and no application build/test log is persisted on the target.
"""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import io
import json
import os
import resource
import shutil
import subprocess
import sys
import tarfile
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sha256', required=True)
    parser.add_argument('--runtime-root', type=Path, required=True)
    parser.add_argument('--build-root', type=Path, default=Path('/dev/shm'),
        help='Server-only override if container /dev/shm is noexec; keep /dev/shm on NX')
    args = parser.parse_args()
    if len(args.sha256) != 64 or any(c not in '0123456789abcdef' for c in args.sha256):
        raise ValueError('Invalid package SHA')
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    data = sys.stdin.buffer.read(8*1024*1024+1)
    if len(data) > 8*1024*1024 or hashlib.sha256(data).hexdigest() != args.sha256:
        raise ValueError('Source package SHA/size mismatch')
    runtime = args.runtime_root.resolve()/('abi4_cpu_'+args.sha256[:16])
    if runtime.exists():
        raise FileExistsError(runtime)
    build_root = args.build_root.resolve()
    if os.statvfs(build_root).f_flag & os.ST_NOEXEC:
        raise ValueError('Build root is mounted noexec; select an executable directory on the server')
    job = Path(tempfile.mkdtemp(prefix='x650_abi4_build_', dir=str(build_root)))
    try:
        source, build, tmp = job/'source', job/'build', job/'tmp'
        source.mkdir(); tmp.mkdir()
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as tar:
            members = tar.getmembers()
            if len(members) >= 100 or len({m.name for m in members}) != len(members):
                raise ValueError('Source member limit/duplicate')
            if sum(m.size for m in members) > 16*1024*1024:
                raise ValueError('Source expanded size limit')
            for item in members:
                path = PurePosixPath(item.name)
                if not item.isfile() or path.is_absolute() or '..' in path.parts or '\\' in item.name:
                    raise ValueError('Unsafe source member')
                target = source.joinpath(*path.parts)
                if not target.resolve().is_relative_to(source.resolve()):
                    raise ValueError('Source outside RAM directory')
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(tar.extractfile(item).read())
        env = dict(os.environ, TMPDIR=str(tmp), PYTHONDONTWRITEBYTECODE='1',
            OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        commands = [
            ['cmake', '-S', str(source/'control_packages/x650_wholebody_v1/benchmark'),
                '-B', str(build), '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=ON'],
            ['cmake', '--build', str(build), '-j2'],
            ['ctest', '--test-dir', str(build), '--output-on-failure']]
        for command in commands:
            print('BUILD_COMMAND', json.dumps(command), flush=True)
            subprocess.run(command, env=env, check=True)
        runtime.mkdir(parents=True)
        lib = runtime/'lib'; lib.mkdir()
        paths = {'executable': (build/'x650_mpc_memory_benchmark', runtime/'x650_mpc_memory_benchmark'),
            'core_library': (build/'native/libx650_mpc_core.so', lib/'libx650_mpc_core.so'),
            'optimization_library': (build/'native/libx650_optimization.so', lib/'libx650_optimization.so'),
            'predictor': (build/'libx650_benchmark_predictor.so', lib/'libx650_benchmark_predictor.so')}
        checks = {}
        for key, (origin, destination) in paths.items():
            raw = origin.read_bytes()
            if raw[:4] != b'\x7fELF':
                raise ValueError('Only ELF code may persist')
            shutil.copy2(origin, destination)
            checks[key] = {'path': str(destination), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}
        result = subprocess.run(['ldd', str(paths['executable'][1])],
            env=dict(env, LD_LIBRARY_PATH=str(lib)), text=True, capture_output=True, check=True)
        if 'not found' in result.stdout:
            raise RuntimeError(result.stdout)
        report = dict(kind='x650_orin_code_build_v1', complete=True, architecture=os.uname().machine,
            package_sha256=args.sha256, runtime=str(runtime), files=checks, commands=commands,
            ldd=result.stdout, build_in_tmpfs=build_root == Path('/dev/shm').resolve(), tests_passed=True,
            application_input_or_output_files_created=False, only_four_elf_files_persist=True)
    finally:
        if job.resolve().parent != build_root or not job.name.startswith('x650_abi4_build_'):
            raise RuntimeError('Refusing unsafe RAM cleanup')
        shutil.rmtree(job)
    report['ram_build_removed'] = not job.exists()
    print('BUILD_REPORT_JSON', json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
