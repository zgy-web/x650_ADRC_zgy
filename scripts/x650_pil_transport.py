"""Workstation debug transport. Never owns an actuator or grants flight approval.

One persistent SSH channel, bounded frames, no remote input/output files.
Host roundtrip and NX local processing durations belong to different clocks.
"""
import hashlib
import json
import math
import queue
import struct
import subprocess
import threading
import time

MAX_FRAME = 131072


def encode(value):
    return json.dumps(value, separators=(',', ':'), allow_nan=False).encode('ascii')


def require(ok, why):
    if not ok:
        raise ValueError(why)


def validate_candidate(request, response, *, sent_ns, received_ns, source_sha256,
                       issue_ns=None, check_deadline=True):
    """Reference admission for the forthcoming C++ server bridge.

    issue_ns must be ORIGINAL server issue time for live use; never NX time.
    Recorded streams can check numerical lineage without a live-age claim.
    """
    require(received_ns >= sent_ns >= 0, 'Host clock regression')
    if issue_ns is not None:
        require(0 < issue_ns <= sent_ns, 'Original local issue is invalid')
    age_ns = received_ns - (sent_ns if issue_ns is None else issue_ns)
    require(not check_deadline or age_ns <= 50_000_000, 'End-to-end deadline')
    s = request['snapshot']
    require(response['request_sha256'] == hashlib.sha256(encode(request)).hexdigest(), 'Wrong request digest')
    require(response['source_sha256'] == source_sha256, 'Wrong loaded adapter source')
    require(response['status'] == 'candidate', 'NX rejected candidate')
    require(response['source_issue_ns'] == s['issued_monotonic_ns'], 'Original issue was refreshed')
    p = response['proposal_nx_clock']
    for key in ('schema_version', 'session_id', 'epoch', 'configuration_sha256', 'sequence', 'control_time_s', 'task_time_s'):
        require(p[key] == s[key], 'Candidate identity/time differs: '+key)
    require(p['snapshot_sequence'] == s['sequence'] and p['replay_only'] is False, 'Snapshot identity differs')
    require(p['status']==0 and p['accepted_optimum'] and p['nonlinear_check_passed'], 'Candidate quality rejected')
    require(1 <= p['subproblems'] <= 3 and p['primal_violation'] <= 2e-7 and p['kkt_residual'] <= .002, 'QP quality')
    require(response['hold_guard_passed'] and response['hold_guard_s']==.16, 'Hold guard missing')
    require(response['memory_step_count']==s['sequence'], 'Continuous Memory sequence differs')
    require(0 <= response['core_wall_ns'] <= 50_000_000 and 0 <= response['core_thread_ns'], 'NX core duration')
    require(0 < response['nx_received_ns'] <= p['compute_start_monotonic_ns'] <= p['compute_end_monotonic_ns'], 'NX clock order')
    require(p['compute_end_monotonic_ns']-response['nx_received_ns']<=50_000_000, 'NX local preparation duration')
    for name, n in (('position_reference_enu',3), ('velocity_feedforward_enu',3),
                    ('acceleration_feedforward_enu',3), ('base_velocity_target',3), ('joint_velocity',2)):
        require(len(p[name])==n and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in p[name]), 'Finite payload dimensions')
    for v, limit in zip(p['base_velocity_target']+p['joint_velocity'], [.55,.35,.45,.45,.35]):
        require(abs(v)<=limit+1e-9, 'Command speed limit')
    require(p['acceleration_feedforward_enu']==[0.,0.,0.], 'Unexpected acceleration feedforward')
    for i in range(3):
        reconstructed=p['velocity_feedforward_enu'][i]+s['position_gains'][i]*(p['position_reference_enu'][i]-s['position_enu'][i])
        require(abs(reconstructed-p['base_velocity_target'][i])<=1e-7, 'PX4 PVA reconstruction')
    return dict(host_roundtrip_ns=received_ns-sent_ns, original_issue_age_ns=age_ns,
        nx_core_ns=response['core_wall_ns'], nx_service_ns=response['nx_service_ns'],
        outside_nx_service_ns=max(0,received_ns-sent_ns-response['nx_service_ns']),
        outside_scope='host scheduling, serialization, SSH/network and response transit; not pure network latency')


class Transport:
    def __init__(self, command, stderr_path):
        self.stderr = stderr_path.open('xb')
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.stderr, bufsize=0)
        self.responses = queue.Queue(maxsize=2)
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        self.faulted=False

    def _reader(self):
        def read(n):
            data=bytearray()
            while len(data)<n:
                chunk=self.proc.stdout.read(n-len(data))
                if not chunk:
                    raise EOFError('Remote channel closed')
                data.extend(chunk)
            return bytes(data)
        try:
            while True:
                size=struct.unpack('!I',read(4))[0]
                require(0<size<=MAX_FRAME,'Remote frame bound')
                raw=read(size);stamp=time.perf_counter_ns()
                self.responses.put((raw,stamp))
        except BaseException as exc:
            self.responses.put(exc)

    def request(self, value, timeout=3.):
        require(not self.faulted, 'Local transport latched after failure')
        raw=encode(value);require(0<len(raw)<=MAX_FRAME,'Request frame bound')
        packet=struct.pack('!I',len(raw))+raw
        sent=time.perf_counter_ns()
        try:
            # A raw pipe write can be short: complete the exact frame.
            offset=0
            while offset<len(packet):
                n=self.proc.stdin.write(packet[offset:])
                require(n is not None and n>0, 'Request write failed');offset+=n
            item=self.responses.get(timeout=timeout)
            if isinstance(item,BaseException):
                raise item
            raw_response,received=item
            response=json.loads(raw_response, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            require(response['request_sha256']==hashlib.sha256(raw).hexdigest(),'Response/request binding failed')
            return response, sent, received
        except BaseException:
            self.faulted=True
            raise

    def close(self):
        if self.proc.stdin:
            self.proc.stdin.close()
        try:
            code=self.proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.proc.kill();code=self.proc.wait(timeout=3)
        self.thread.join(timeout=1)
        self.proc.stdout.close();self.stderr.close()
        return code
