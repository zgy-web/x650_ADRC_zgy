#!/usr/bin/env python3
"""Stream code to tmpfs, link existing qualified libraries, persist one ELF only."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import resource
import shutil
import subprocess
import sys
import tarfile
import tempfile


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sha256', required=True)
    p.add_argument('--existing-runtime', type=Path, required=True)
    p.add_argument('--runtime-root', type=Path, required=True)
    p.add_argument('--build-root', type=Path, default=Path('/dev/shm'),
        help='Server-only override for a noexec /dev/shm; NX must use the tmpfs default')
    args = p.parse_args()
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    raw = sys.stdin.buffer.read(2*1024*1024+1)
    if len(raw)>2*1024*1024 or hashlib.sha256(raw).hexdigest()!=args.sha256:
        raise ValueError('Package SHA/size differs')
    runtime = args.existing_runtime.resolve()
    codes = {k: runtime/'lib'/n for k, n in (
        ('core', 'libx650_mpc_core.so'), ('optimization', 'libx650_optimization.so'),
        ('predictor', 'libx650_benchmark_predictor.so'))}
    before = {k: hashlib.sha256(v.read_bytes()).hexdigest() for k,v in codes.items()}
    output = args.runtime_root.resolve()/('pil_'+args.sha256[:16])
    if output.exists():
        raise FileExistsError(output)
    build_root=args.build_root.resolve()
    job = Path(tempfile.mkdtemp(prefix='x650_pil_build_', dir=str(build_root)))
    try:
        source=job/'source'; source.mkdir(); temp=job/'tmp';temp.mkdir()
        with tarfile.open(fileobj=io.BytesIO(raw), mode='r:gz') as tar:
            members=tar.getmembers()
            if len(members)>32 or sum(m.size for m in members)>2*1024*1024 or len({m.name for m in members})!=len(members):
                raise ValueError('Source archive bounds/duplicates')
            for m in members:
                n=PurePosixPath(m.name)
                if not m.isfile() or n.is_absolute() or '..' in n.parts or '\\' in m.name:
                    raise ValueError('Invalid source member')
                dest=source.joinpath(*n.parts);dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(tar.extractfile(m).read())
        env=dict(os.environ, TMPDIR=str(temp), PYTHONDONTWRITEBYTECODE='1')
        for cmd in ([ 'cmake', '-S', str(source/'control_packages/x650_wholebody_v1/pil'),
                      '-B', str(job/'build'), '-DCMAKE_BUILD_TYPE=Release', '-DX650_RUNTIME='+str(runtime)],
                    ['cmake', '--build', str(job/'build'), '-j2'],
                    ['ctest', '--test-dir', str(job/'build'), '--output-on-failure']):
            subprocess.run(cmd, env=env, check=True)
        binary=job/'build/x650_mpc_pil_worker';data=binary.read_bytes()
        if data[:4]!=b'\x7fELF':
            raise ValueError('Only executable code may persist')
        after={k: hashlib.sha256(v.read_bytes()).hexdigest() for k,v in codes.items()}
        if before!=after:
            raise ValueError('Qualified native libraries changed during build')
        output.mkdir(parents=True);installed=output/binary.name;shutil.copy2(binary, installed)
        report=dict(kind='x650_pil_code_build_v1', complete=True, source_sha256=args.sha256,
            executable=dict(path=str(installed),sha256=hashlib.sha256(data).hexdigest(),bytes=len(data)),
            libraries={k:dict(path=str(v),sha256=before[k]) for k,v in codes.items()},
            existing_math_binaries_unchanged=True, only_one_elf_persisted=True)
    finally:
        if job.resolve().parent!=build_root or not job.name.startswith('x650_pil_build_'):
            raise RuntimeError('Unsafe cleanup target')
        shutil.rmtree(job)
    report['ram_build_removed']=not job.exists()
    report['build_root_tmpfs']=build_root==Path('/dev/shm')
    print('BUILD_REPORT_JSON',json.dumps(report),flush=True)


if __name__ == '__main__':
    main()
