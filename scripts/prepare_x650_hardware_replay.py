"""Bind historical snapshots to reconstructed PX4/JointState test messages.

This is NOT a raw ROS recording. Message validity/receipt metadata are explicit
test fixtures. No data are sent to a physical flight controller.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct


def f32(x):
    if not math.isfinite(x):
        raise ValueError("Nonfinite source")
    return struct.unpack("<f", struct.pack("<f", x))[0]


def quaternion(r):
    """Rotation matrix -> wxyz, independently of the C++ quaternion adapter."""
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = 2 * math.sqrt(1 + tr)
        return [s / 4, (r[2][1]-r[1][2])/s, (r[0][2]-r[2][0])/s, (r[1][0]-r[0][1])/s]
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + r[i][i] - r[j][j] - r[k][k])
    q = [0.] * 4
    q[0], q[i+1], q[j+1], q[k+1] = (r[k][j]-r[j][k])/s, s/4, (r[j][i]+r[i][j])/s, (r[k][i]+r[i][k])/s
    return q


def rotation(q):
    n = math.sqrt(sum(x*x for x in q))
    w, x, y, z = [a/n for a in q]
    return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]


def frame(case):
    state = case["input"]["state"]
    r = [state["rotation"][i:i+3] for i in (0, 3, 6)]
    # R_NED_FRD = T_ENU_NED * R_ENU_FLU * S_FRD_FLU.
    perm, rowsign, colsign = (1, 0, 2), (1, 1, -1), (1, -1, -1)
    rn = [[rowsign[i]*r[perm[i]][j]*colsign[j] for j in range(3)] for i in range(3)]
    q = [f32(x) for x in quaternion(rn)]
    quantized_rn = rotation(q)
    expected_r = [rowsign[i]*quantized_rn[perm[i]][j]*colsign[j] for i in range(3) for j in range(3)]
    def ned(v):
        return [f32(v[1]), f32(v[0]), f32(-v[2])]
    p, v, a = [ned(state[name]) for name in ("position", "velocity", "acceleration")]
    w = [f32(state["omega_body"][i]*colsign[i]) for i in range(3)]
    stamp_us = round(state["stamp"]*1e6)
    joint_ns = round(state["joint_stamp"]*1e9)
    expected = dict(state)
    expected.update(position=[p[1],p[0],-p[2]],velocity=[v[1],v[0],-v[2]],
                    acceleration=[a[1],a[0],-a[2]],omega_body=[w[0],-w[1],-w[2]],
                    rotation=expected_r,stamp=stamp_us*1e-6,joint_stamp=joint_ns*1e-9)
    return dict(id=case["id"],sample_us=stamp_us,joint_ns=joint_ns,q_ned_frd=q,
                position_ned=p,velocity_ned=v,acceleration_ned=a,omega_frd=w,
                q=state["q"],dq=state["dq"],expected_state=expected,
                # Deliberately nonzero: physical encoder must NOT add ref_alt.
                ignored_ref_alt_m=123.5,yaw_enu_rad=math.atan2(r[1][0],r[0][0]))


def prepare(root, manifest, output):
    declaration = json.loads(manifest.read_text(encoding="utf-8"))
    output.mkdir(parents=True, exist_ok=False)
    products = []
    for item in declaration["inputs"]:
        src = (root / item["source"].replace("\\", "/")).resolve()
        if not src.is_relative_to(root.resolve()):
            raise ValueError("Source escapes project")
        raw = src.read_bytes()
        if hashlib.sha256(raw).hexdigest() != item["sha256"] or len(raw) != item["bytes"]:
            raise ValueError("Historical input identity differs: " + str(src))
        recorded = json.loads(raw)
        if len(recorded["cases"]) != item["count"]:
            raise ValueError("Historical row coverage differs")
        data = dict(kind="x650_hardware_interface_replay_v1",recorded=recorded,
                    frames=[frame(c) for c in recorded["cases"]],
                    source_sha256=item["sha256"],
                    source_type="reconstructed_messages_from_saved_simulation_snapshots",
                    joint_names=["joint_1","joint_2"],
                    synthetic_metadata=["valid_flags","disarmed_status","receipt_times","aligned_body_topic_samples"],
                    raw_cdr_recording=False,physical_validation=False)
        encoded = json.dumps(data,allow_nan=False,separators=(",",":")).encode()
        path = output / (item["name"] + ".json")
        path.write_bytes(encoded)
        products.append(dict(name=item["name"],file=path.name,sha256=hashlib.sha256(encoded).hexdigest(),
                             bytes=len(encoded),count=item["count"],source_sha256=item["sha256"],
                             start_s=item["start_s"],end_s=item["end_s"]))
    result = dict(kind="x650_hardware_replay_preparation_v1",inputs=products,
                  row_count=sum(x["count"] for x in products),
                  source_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
                  raw_cdr_recording=False,physical_validation=False)
    (output/"manifest.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest",type=Path,default=Path("docs/validation/nmpc_interface_20261027/full_input_manifest.json"))
    p.add_argument("--output",type=Path,required=True)
    args = p.parse_args()
    print(json.dumps(prepare(Path(__file__).resolve().parents[1],args.manifest,args.output),indent=2))
