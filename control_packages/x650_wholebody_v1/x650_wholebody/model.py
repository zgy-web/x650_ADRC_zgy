"""Explicit planar-arm geometry for the composed X650, in SI / ENU / FLU.

The body origin is the imported ``base_link`` origin, which need not be a
centre of mass.  ``angle_offsets`` gives each link's absolute pitch at q=0;
the second link therefore uses offset[1] + q1 + q2.  Physical parameters are
loaded from a hash-bound preparation report, never from the old x500 defaults.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import numpy as np


def vector(value, size, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f"{name} must be a finite {size}-vector")
    return a


def rotation_matrix(value):
    a = np.asarray(value, dtype=float)
    if a.shape == (4,):
        w, x, y, z = vector(a, 4, "wxyz")
        n = np.linalg.norm(a)
        if n < 1e-12:
            raise ValueError("Zero quaternion")
        w, x, y, z = a/n
        a = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                      [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                      [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
    if (a.shape != (3, 3) or not np.isfinite(a).all()
            or not np.allclose(a.T@a, np.eye(3), atol=1e-8)
            or abs(np.linalg.det(a)-1) > 1e-8):
        raise ValueError("Require a proper FLU-to-ENU rotation or wxyz quaternion")
    return a


@dataclass(frozen=True)
class ArmModel:
    masses: tuple
    lengths: tuple
    com_distances: tuple
    angle_offsets: tuple
    mount_body: tuple
    inertia_diagonal: tuple
    total_mass: float
    base_com_body: tuple
    joint_lower: tuple
    joint_upper: tuple
    profile_name: str
    source_report_sha256: str = ""
    elbow_offset_body_m: tuple = (0., 0., 0.)

    def __post_init__(self):
        for name, count in (("masses", 2), ("lengths", 2), ("com_distances", 2),
                            ("angle_offsets", 2), ("mount_body", 3),
                            ("base_com_body", 3), ("joint_lower", 2), ("joint_upper", 2),
                            ("elbow_offset_body_m", 3)):
            a = vector(getattr(self, name), count, name)
            object.__setattr__(self, name, tuple(float(v) for v in a))
        if abs(self.elbow_offset_body_m[0])+abs(self.elbow_offset_body_m[2]) > 1e-12:
            raise ValueError("Supported elbow offset is along the common body-Y joint axis")
        if (not np.isfinite(self.total_mass) or self.total_mass <= sum(self.masses)
                or min(self.masses+self.lengths+self.com_distances) <= 0):
            raise ValueError("Positive physical link data and total mass including fixed body are required")
        if np.any(np.array(self.com_distances) > np.array(self.lengths)):
            raise ValueError("Supported link COM must lie within its length")
        if np.any(np.array(self.joint_lower) >= np.array(self.joint_upper)):
            raise ValueError("Physical joint intervals must be ordered")
        inertia = np.asarray(self.inertia_diagonal, dtype=float)
        if (inertia.shape != (2, 3) or not np.isfinite(inertia).all()
                or np.any(inertia <= 0)
                or np.any(2*inertia.max(axis=1) > inertia.sum(axis=1)+1e-12)):
            raise ValueError("Require positive, physically consistent diagonal COM inertias")
        object.__setattr__(self, "inertia_diagonal", tuple(tuple(float(v) for v in row) for row in inertia))
        if not isinstance(self.profile_name, str) or not self.profile_name:
            raise ValueError("Explicit profile_name is required")
        digest = self.source_report_sha256
        if digest and (len(digest) != 64 or set(digest) - set("0123456789abcdef")):
            raise ValueError("Invalid source-report SHA256")
        object.__setattr__(self, "total_mass", float(self.total_mass))

    @classmethod
    def from_report(cls, path):
        """Read a composed model only after its geometry/source audit passed.

        This accepts a *preflight* physical report; it does not require a prior
        flight, and it does not claim dynamics were calibrated.  The sibling
        URDF, kinematics and clearance certificate must match their hashes.
        """
        path = Path(path).resolve()
        content = path.read_bytes()
        report = json.loads(content)
        if (report.get("kind") != "x650_original_two_link_composition"
                or report.get("source_unchanged") is not True
                or report.get("joint_state_offset_convention_preserved") is not True
                or report.get("physical_geometry_validated") is not True
                or report.get("arm_self_collision_resolved") is not True):
            raise ValueError("Require an audited X650/two-link composition report")
        for filename, field in (("robot.urdf", "derived_urdf_sha256"),
                                ("kinematics.json", "kinematics_sha256"),
                                ("clearance_report.json", "clearance_certificate_sha256"),
                                ("arm_self_collision_report.json", "arm_self_collision_report_sha256")):
            actual = hashlib.sha256((path.parent/filename).read_bytes()).hexdigest()
            if report.get(field) != actual:
                raise ValueError("Preparation-report source hash mismatch: " + filename)
        clearance = json.loads((path.parent/"clearance_report.json").read_text(encoding="utf-8"))
        self_collision = json.loads((path.parent/"arm_self_collision_report.json").read_text(encoding="utf-8"))
        if (clearance.get("passed") is not True or clearance.get("physical_limits_changed") is not False
                or clearance.get("arm_self_collision", {}).get("passed") is not True
                or self_collision.get("passed") is not True):
            raise ValueError("Physical clearance certificate did not pass with original limits")
        fields = report.get("controller_model")
        if not isinstance(fields, dict):
            raise ValueError("Preparation report lacks explicit controller_model")
        names = set(cls.__dataclass_fields__) - {"source_report_sha256"}
        if not names.issubset(fields):
            raise ValueError("controller_model lacks fields: " + str(sorted(names-set(fields))))
        if (report.get("rigid_geometry_and_inertia_validated") is not True
                or fields.get("body_frame") != "FLU" or fields.get("world_frame") != "ENU"):
            raise ValueError("Physical model validation / coordinate convention is missing")
        model = cls(**{name: fields[name] for name in names},
                    source_report_sha256=hashlib.sha256(content).hexdigest())
        if abs(model.total_mass-float(report["total_mass_kg"])) > 1e-10:
            raise ValueError("Controller and aggregate mass differ")
        if abs(model.fixed_mass-float(fields["base_mass"])) > 1e-10:
            raise ValueError("Fixed body mass does not include the declared adapter/rotors")
        full_inertia = np.asarray(fields["inertia_com_kg_m2"], dtype=float)
        if (full_inertia.shape != (2, 3, 3)
                or not np.allclose(full_inertia, [np.diag(v) for v in model.inertia_diagonal], atol=1e-12)):
            raise ValueError("Supported arm requires audited diagonal COM inertias")
        k = json.loads((path.parent/"kinematics.json").read_text(encoding="utf-8"))
        checks = ((model.mount_body, k["mount_xyz_m"]), (model.lengths, k["lengths_m"]),
                  (model.joint_lower, np.array(k["joint_limits_rad"])[:, 0]),
                  (model.joint_upper, np.array(k["joint_limits_rad"])[:, 1]),
                  (model.angle_offsets, np.cumsum(k["initial_relative_angles_rad"])),
                  (model.elbow_offset_body_m, k["elbow_offset_body_m"]))
        if (k.get("body_frame") != "FLU" or k.get("world_frame") != "ENU"
                or k.get("gazebo_zero_is_folded_pose") is not True
                or not np.allclose(k["joint_axes_local"], [[0, 1, 0], [0, 1, 0]], atol=1e-12)
                or not np.allclose(k["mount_rpy_rad"], 0, atol=1e-12)
                or any(not np.allclose(a, b, rtol=0, atol=1e-10) for a, b in checks)):
            raise ValueError("Controller geometry conflicts with audited kinematics")
        return model

    @property
    def fixed_mass(self):
        return self.total_mass-sum(self.masses)

    def metadata(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


def endpoint_positions_body(q, model):
    q1, q2 = vector(q, 2, "q")
    a1, a2 = np.array(model.angle_offsets)+[q1, q1+q2]
    mount = np.array(model.mount_body)
    elbow = mount+model.lengths[0]*np.array([np.cos(a1), 0., -np.sin(a1)])
    tip = elbow+np.array(model.elbow_offset_body_m)+model.lengths[1]*np.array([np.cos(a2), 0., -np.sin(a2)])
    return np.array([mount, elbow, tip])


def tip_body(q, model):
    return endpoint_positions_body(q, model)[-1]


def tip_world(p, R, q, model):
    return vector(p, 3, "p")+rotation_matrix(R)@tip_body(q, model)


def joint_jacobian_body(q, model):
    q1, q2 = vector(q, 2, "q")
    a1, a2 = np.array(model.angle_offsets)+[q1, q1+q2]
    j1 = model.lengths[0]*np.array([-np.sin(a1), 0., -np.cos(a1)])
    j2 = model.lengths[1]*np.array([-np.sin(a2), 0., -np.cos(a2)])
    return np.column_stack((j1+j2, j2))


def joint_centripetal_body(q, dq, model):
    q1, q2 = vector(q, 2, "q")
    d1, d2 = vector(dq, 2, "dq")
    a1, a2 = np.array(model.angle_offsets)+[q1, q1+q2]
    return (-model.lengths[0]*np.array([np.cos(a1), 0., -np.sin(a1)])*d1*d1
            -model.lengths[1]*np.array([np.cos(a2), 0., -np.sin(a2)])*(d1+d2)**2)


def inverse_tip_body(target, near, model):
    """Return the IK branch nearest ``near`` without silently clipping q."""
    r = vector(target, 3, "target")-np.array(model.mount_body)-np.array(model.elbow_offset_body_m)
    near = vector(near, 2, "near")
    if abs(r[1]) > 1e-8:
        raise ValueError("Arm endpoint lies outside its body XZ plane")
    x, y = r[0], -r[2]
    l1, l2 = model.lengths
    c = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
    if abs(c) > 1+1e-10:
        raise ValueError("Arm target is unreachable")
    candidates = []
    for sign in (-1., 1.):
        beta = sign*np.arccos(np.clip(c, -1., 1.))
        theta = np.arctan2(y, x)-np.arctan2(l2*np.sin(beta), l1+l2*np.cos(beta))
        q = np.array([theta-model.angle_offsets[0], beta+model.angle_offsets[0]-model.angle_offsets[1]])
        q = near+(q-near+np.pi) % (2*np.pi)-np.pi
        candidates.append(q)
    return min(candidates, key=lambda q: np.linalg.norm(q-near)).copy()


def arm_com_kinematics(q, dq, ddq, model):
    """Two COM position/velocity/acceleration arrays relative to body origin."""
    q, dq, ddq = (vector(v, 2, n) for v, n in ((q, "q"), (dq, "dq"), (ddq, "ddq")))
    theta = np.array(model.angle_offsets)+[q[0], q.sum()]
    rate, accel = [dq[0], dq.sum()], [ddq[0], ddq.sum()]
    def segment(length, i):
        direction = np.array([np.cos(theta[i]), 0., -np.sin(theta[i])])
        tangent = np.array([-np.sin(theta[i]), 0., -np.cos(theta[i])])
        return length*direction, length*tangent*rate[i], length*(tangent*accel[i]-direction*rate[i]**2)
    s1, elbow, s2 = segment(model.com_distances[0], 0), segment(model.lengths[0], 0), segment(model.com_distances[1], 1)
    return (np.array([np.array(model.mount_body)+s1[0], np.array(model.mount_body)+elbow[0]+np.array(model.elbow_offset_body_m)+s2[0]]),
            np.array([s1[1], elbow[1]+s2[1]]), np.array([s1[2], elbow[2]+s2[2]]))
