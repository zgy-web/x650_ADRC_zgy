"""Compile and test UART bench in NX RAM; no real device access or install."""
import argparse, base64, hashlib, io, json, re, shlex, subprocess, zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def payload(generated):
    profile=json.loads((ROOT/'configs/hardware/nx_mavlink_uart_v1.json').read_text())
    source=ROOT/'hardware/nx/mavlink_uart'
    text=(source/'src/bench.cpp').read_text()
    section=re.search(r'streams\{\{(.+?)\}\};',text,re.S).group(1)
    configured={int(a):int(b) for a,b in re.findall(r'\{(\d+),(\d+)\}',section)}
    requested={m['id']:m['bench_hz'] for m in profile['messages'] if m['direction']=='px4_to_nx'
               and m['bench_hz'] and m['name'] not in ('HEARTBEAT','TIMESYNC')}
    if configured!=requested:raise ValueError('C++ stream requests differ from budget profile')
    generated=Path(generated);manifest=json.loads((generated/'manifest.json').read_text())
    lock=json.loads((ROOT/'configs/hardware/nx_mavlink_dialect_lock.json').read_text())
    if manifest['lock']!=lock:raise ValueError('Wrong dialect source')
    files={}
    for p in source.rglob('*'):
        if p.is_file() and (p.suffix in ('.hpp','.cpp','.py') or p.name=='CMakeLists.txt'):
            files['source/'+p.relative_to(source).as_posix()]=p.read_bytes()
    for name,sha in manifest['files'].items():
        if name.startswith('include/') or name=='dialect.py':
            data=(generated/name).read_bytes()
            if hashlib.sha256(data).hexdigest()!=sha:raise ValueError('Generated file changed')
            files['mavlink/'+name]=data
    buf=io.BytesIO()
    with zipfile.ZipFile(buf,'w',zipfile.ZIP_DEFLATED) as z:
        for name,data in sorted(files.items()):
            info=zipfile.ZipInfo(name,(2026,1,1,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED;z.writestr(info,data)
    raw=buf.getvalue()
    return dict(archive=base64.b64encode(raw).decode(),sha256=hashlib.sha256(raw).hexdigest(),
                files={n:hashlib.sha256(d).hexdigest() for n,d in files.items()},profile=profile)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('generated','output','identity','known-hosts'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--ssh',default='ssh');p.add_argument('--host',default='192.168.30.145');a=p.parse_args()
    request=payload(a.generated);a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'source_manifest.json').write_text(json.dumps({k:v for k,v in request.items() if k!='archive'},indent=2))
    remote=Path(__file__).with_name('nx_mavlink_build_remote.py').read_text()
    cmd=[a.ssh,'-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','IdentitiesOnly=yes','-o','ConnectTimeout=8',
         '-i',str(a.identity),'-o','UserKnownHostsFile='+str(a.known_hosts.resolve()),'jetson@'+a.host,
         shlex.join(['env','PYTHONDONTWRITEBYTECODE=1','python3','-u','-c',remote])]
    child=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    child.stdin.write(json.dumps(request).encode());child.stdin.close();complete=cleanup=False
    with (a.output/'transport.jsonl').open('wb') as log:
        for line in child.stdout:
            log.write(line);log.flush()
            try:r=json.loads(line)
            except ValueError:print(line.decode(errors='replace'),end='');continue
            if 'base64' in r:
                data=base64.b64decode(r['base64'],validate=True)
                if hashlib.sha256(data).hexdigest()!=r['sha256'] or Path(r['name']).name!=r['name']:
                    raise ValueError('Artifact identity invalid')
                (a.output/r['name']).write_bytes(data)
            if r.get('event')=='complete':complete=r.get('passed') is True
            if r.get('event')=='cleanup':cleanup=r.get('removed') is True
            print(json.dumps({k:v for k,v in r.items() if k!='base64'}),flush=True)
    if child.wait() or not complete or not cleanup:raise RuntimeError('RAM build/testing incomplete')


if __name__=='__main__':main()
