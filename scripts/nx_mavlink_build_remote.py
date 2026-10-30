"""RAM-only build/PTY tests; never invoke the real-device executable."""
import base64, hashlib, io, json, os, resource, subprocess, sys, tempfile, zipfile
from pathlib import Path


def emit(**r):print(json.dumps(r),flush=True)
def artifact(name,data,**extra):emit(event='artifact',name=name,sha256=hashlib.sha256(data).hexdigest(),base64=base64.b64encode(data).decode(),**extra)


def main():
    request=json.load(sys.stdin);resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    raw=base64.b64decode(request['archive'],validate=True)
    if hashlib.sha256(raw).hexdigest()!=request['sha256']:raise ValueError('Source archive changed')
    with tempfile.TemporaryDirectory(prefix='flyarm_uart_build_',dir='/dev/shm') as tmp:
        root=Path(tmp)
        try:
            with zipfile.ZipFile(io.BytesIO(raw)) as z:
                if set(z.namelist())!=set(request['files']):raise ValueError('Inventory mismatch')
                for name,sha in request['files'].items():
                    p=root/name
                    if not p.resolve().is_relative_to(root):raise ValueError('Unsafe member')
                    data=z.read(name)
                    if hashlib.sha256(data).hexdigest()!=sha:raise ValueError('Source changed')
                    p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
            env=os.environ.copy();env.update(TMPDIR=tmp,PYTHONDONTWRITEBYTECODE='1')
            def run(name,cmd,timeout=90):
                emit(event='phase',name=name)
                p=subprocess.run(list(map(str,cmd)),env=env,cwd=tmp,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout)
                artifact(name+'.log',p.stdout,returncode=p.returncode)
                if p.returncode:raise RuntimeError(name+' failed')
            run('configure',['cmake','-S',root/'source','-B',root/'build','-DCMAKE_BUILD_TYPE=Release','-DMAVLINK_INCLUDE_DIR='+str(root/'mavlink/include')])
            run('build',['cmake','--build',root/'build','-j2'])
            run('ctest',['ctest','--test-dir',root/'build','--output-on-failure'])
            run('pty_tests',['python3','-B',root/'source/tests/test_process.py',root/'build/flyarm_uart_bench_pty',root/'mavlink/dialect.py'])
            binary=(root/'build/flyarm_uart_bench').read_bytes()
            artifact('flyarm_uart_bench',binary,source_sha256=request['sha256'])
            emit(event='complete',passed=True,hardware_accessed=False,installed=False,source_sha256=request['sha256'])
        finally:
            pass
    emit(event='cleanup',removed=not root.exists(),persistent_files_created=[])


if __name__=='__main__':
    try:main()
    except Exception as e:emit(event='failure',error=repr(e));sys.exit(1)
