"""Generate PC-side C headers from the exact audited PX4 MAVLink submodule."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import urllib.request
from pymavlink.generator import mavgen, mavparse

ROOT = Path(__file__).resolve().parents[1]


def prepare(output, source_dir=None):
    lock = json.loads((ROOT/'configs/hardware/nx_mavlink_dialect_lock.json').read_text())
    profile = json.loads((ROOT/'configs/hardware/nx_mavlink_uart_v1.json').read_text())
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    definitions = output/'definitions'; definitions.mkdir()
    metadata = {}
    for name, expected in lock['files'].items():
        if source_dir:
            data = (Path(source_dir)/name).read_bytes()
        else:
            url = f"https://raw.githubusercontent.com/mavlink/mavlink/{lock['mavlink_submodule_commit']}/message_definitions/v1.0/{name}"
            data = urllib.request.urlopen(url, timeout=20).read()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError('Pinned dialect hash mismatch: '+name)
        path = definitions/name; path.write_bytes(data)
        for m in mavparse.MAVXML(str(path), wire_protocol_version='2.0').message:
            metadata[m.name] = dict(id=m.id, max_payload=m.wire_length, crc_extra=m.crc_extra)
    for m in profile['messages']:
        if metadata[m['name']] != {k:m[k] for k in ('id','max_payload','crc_extra')}:
            raise ValueError('Budget does not match pinned definition: '+m['name'])
    opts = mavgen.Opts(str(output/'include'), wire_protocol='2.0', language='C', validate=False)
    if not mavgen.mavgen(opts, [str(definitions/'development.xml')]):
        raise RuntimeError('C header generation failed')
    opts = mavgen.Opts(str(output/'dialect.py'), wire_protocol='2.0', language='Python3', validate=False)
    if not mavgen.mavgen(opts, [str(definitions/'development.xml')]):
        raise RuntimeError('Python validation dialect generation failed')
    manifest = dict(lock=lock, generator_pymavlink_version=importlib.metadata.version('pymavlink'),
        metadata=metadata, files={p.relative_to(output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in sorted(output.rglob('*')) if p.is_file()})
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True); p.add_argument('--source-dir', type=Path)
    a = p.parse_args(); m = prepare(a.output, a.source_dir)
    print(json.dumps({'output':str(a.output.resolve()), 'generated_files':len(m['files']),
                      'generator':m['generator_pymavlink_version']}))
