#!/usr/bin/env python3
"""Remove four default ULog downsampling intervals on the pinned PX4 SITL."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PINNED_PX4 = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
DEFAULT_INTERVAL_MS = {"trajectory_setpoint": 200, "vehicle_local_position": 100,
                       "vehicle_local_position_setpoint": 100, "vehicle_attitude": 50}


def reviewed_text(text):
    """Preserve the whole default logging profile; alter only these intervals."""
    for topic, old_ms in DEFAULT_INTERVAL_MS.items():
        pattern = re.compile(r'(?m)^(\s*)add_topic\("'+topic+r'", (?:'+str(old_ms)+r'|0)\);$')
        text, count = pattern.subn(lambda m: m.group(1)+'add_topic("'+topic+'", 0);', text)
        if count != 1:
            raise ValueError("Missing, duplicate or unexpected default logger interval: "+topic)
    return text


def configure(px4, output, *, apply=False):
    px4, output = Path(px4).resolve(strict=True), Path(output).resolve()
    head = subprocess.run(["git","rev-parse","HEAD"],cwd=px4,check=True,capture_output=True,text=True).stdout.strip()
    if head != PINNED_PX4:
        raise ValueError("Logging profile requires pinned PX4 source")
    target = px4/'src/modules/logger/logged_topics.cpp'
    if target.is_symlink() or not target.resolve().is_relative_to(px4):
        raise ValueError("Logging target must be an ordinary file within PX4")
    before = target.read_bytes()
    after = reviewed_text(before.decode()).encode()
    digest = lambda value: hashlib.sha256(value).hexdigest()
    report = dict(kind='amsim_px4_upper_logging_v1',px4_commit=head,target=str(target),
        before_sha256=digest(before),after_sha256=digest(after),
        previous_intervals_ms=DEFAULT_INTERVAL_MS,requested_intervals_ms={key:0 for key in DEFAULT_INTERVAL_MS},
        changes_needed=before!=after,applied=False,build_required=before!=after,flight_started=False,
        note='Zero means log each available uORB update, not guaranteed delivery. Verify actual ULog coverage; custom logger_topics.txt overrides the default profile.')
    if apply:
        if (ROOT/'.amsim/runtime.json').exists() or (ROOT/'.amsim/x650_diagnostic_restore_pending.json').exists():
            raise RuntimeError('Active runtime or pending restoration prevents edits')
        if subprocess.run(['pgrep','-x','px4'],capture_output=True).returncode!=1:
            raise RuntimeError('PX4 must be stopped')
        output.mkdir(parents=True,exist_ok=False)
        (output/'logged_topics.before.cpp').write_bytes(before)
        (output/'logged_topics.after.cpp').write_bytes(after)
        target.write_bytes(after)
        if target.read_bytes()!=after:
            raise RuntimeError('Logger source write verification failed')
        report['applied']=True
        (output/'logging_config.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--apply',action='store_true')
    a=p.parse_args()
    print(json.dumps(configure(a.px4,a.output,apply=a.apply),indent=2))
