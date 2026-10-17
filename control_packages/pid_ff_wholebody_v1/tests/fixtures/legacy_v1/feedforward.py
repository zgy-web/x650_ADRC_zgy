"""Two-link arm reaction prediction for x500_underfold, in SI units.

The output used by the PX4 position controller is *only* an acceleration
feedforward.  It compensates the arm's motion relative to the base.  Total
vehicle mass (including the arm) already supplies nominal gravity and base
acceleration, so adding arm weight again would be incorrect.

The full arm reaction moment is reported for diagnosis, but is not claimed to
be applied by a position/velocity/acceleration MAVLink setpoint.  No external
force is applied to the Gazebo plant by this module.

World uses ENU; body uses Gazebo FLU.  R_world_body maps body vectors to world.
omega_body and alpha_body are physical base angular velocity/acceleration
expressed in body axes.  q/dq are measured; ddq may be the coordinated planned
acceleration, which makes this model-based prediction rather than a measured
disturbance observer.  It excludes contact, aerodynamic and joint tracking
model errors.  A caller must log the actual source of each input.
"""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class ArmParameters:
    """Defaults match the archived Gazebo x500_underfold model snapshot."""

    masses: tuple = (0.25, 0.20)
    lengths: tuple = (0.30, 0.25)
    com_distances: tuple = (0.15, 0.125)
    angle_offsets: tuple = (2.61799387799, 0.0)
    mount_body: tuple = (0.0, 0.0, -0.10)
    inertia_diagonal: tuple = (
        (6.66666666667e-5, 0.00190833333333, 0.00190833333333),
        (4.08333333333e-5, 0.00106208333333, 0.00106208333333),
    )
    total_mass: float = 2.6493076923076924

    def __post_init__(self):
        if self.total_mass <= sum(self.masses) or min(self.masses) <= 0:
            raise ValueError("Total mass must include positive arm and base masses")
        if min(self.lengths + self.com_distances) <= 0:
            raise ValueError("Link geometry must be positive")

    @classmethod
    def from_sdf(cls, path):
        """Load and validate this supported two-link model, without guessing.

        A flattened model SDF is required.  Changing axes, COM orientations or
        topology requires updating this model, rather than silently reusing it.
        """
        root = ET.parse(Path(path)).getroot()
        model = root if root.tag == "model" else root.find("model")
        if model is None or model.findall("include") or model.findall("model"):
            raise ValueError("A flattened, single model SDF is required")
        links = {e.attrib["name"]: e for e in model.findall("link")}
        joints = {e.attrib["name"]: e for e in model.findall("joint")}

        def nums(element, name, default=None):
            text = element.findtext(name, default)
            if text is None:
                raise ValueError("Missing SDF value " + name)
            return np.array([float(x) for x in text.split()])

        base_inertial = nums(links["base_link"], "inertial/pose", "0 0 0 0 0 0")
        if not np.allclose(base_inertial, 0.0, atol=1e-10):
            raise ValueError("This API requires base_link origin at its COM")
        masses, lengths, coms, offsets, inertias, poses = [], [], [], [], [], []
        for i in (1, 2):
            link, joint = links[f"arm_link_{i}"], joints[f"arm_joint_{i}"]
            if joint.findtext("child") != f"arm_link_{i}" or joint.findtext("parent") != ("base_link" if i == 1 else "arm_link_1"):
                raise ValueError("Unexpected arm chain topology")
            axis = joint.find("axis/xyz")
            if axis is None or axis.get("expressed_in") != "base_link" or not np.allclose(nums(joint, "axis/xyz"), [0, 1, 0]):
                raise ValueError("Only revolute axes along +body Y are supported")
            pose_element = link.find("pose")
            if pose_element is None or pose_element.get("relative_to") != "base_link":
                raise ValueError("Arm zero poses must be expressed in base_link")
            pose = nums(link, "pose")
            com = nums(link, "inertial/pose", "0 0 0 0 0 0")
            if not np.allclose(pose[[1, 3, 5]], 0, atol=1e-10) or not np.allclose(com[1:], 0, atol=1e-10):
                raise ValueError("Only planar pitch links with +X COM offset are supported")
            inertia = link.find("inertial/inertia")
            if not np.allclose([float(inertia.findtext(k, "0")) for k in ("ixy", "ixz", "iyz")], 0):
                raise ValueError("Only diagonal link inertias are supported")
            box = nums(link, "collision/geometry/box/size")
            masses.append(float(link.findtext("inertial/mass")))
            lengths.append(float(box[0]))
            coms.append(float(com[0]))
            offsets.append(float(pose[4]))
            inertias.append(tuple(float(inertia.findtext(k)) for k in ("ixx", "iyy", "izz")))
            poses.append(pose)
        expected_elbow = poses[0][:3] + _ry(offsets[0]) @ np.array([lengths[0], 0, 0])
        if not np.allclose(poses[1][:3], expected_elbow, atol=1e-8):
            raise ValueError("Second link origin does not match first link endpoint")
        total_mass = sum(float(e.findtext("inertial/mass", "0")) for e in links.values())
        return cls(tuple(masses), tuple(lengths), tuple(coms), tuple(offsets),
                   tuple(poses[0][:3]), tuple(inertias), total_mass)


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite vector of length {size}")
    return result


def _ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def _segment(length, theta, rate, acceleration):
    direction = np.array([np.cos(theta), 0., -np.sin(theta)])
    tangent = np.array([-np.sin(theta), 0., -np.cos(theta)])
    return (length * direction, length * tangent * rate,
            length * (tangent * acceleration - direction * rate ** 2))


def arm_com_kinematics(q, dq=(0., 0.), ddq=(0., 0.), params=None):
    """Return COM positions/velocities/accelerations relative to body origin.

    Derivatives here are taken in body coordinates; base rotation is applied
    separately by arm_reaction_feedforward().  Shape of each returned array is
    (2 links, 3 body coordinates).
    """
    params = params or ArmParameters()
    q, dq, ddq = (_vector(v, 2, n) for v, n in ((q, "q"), (dq, "dq"), (ddq, "ddq")))
    theta = np.array(params.angle_offsets) + [q[0], q.sum()]
    rates, accelerations = [dq[0], dq.sum()], [ddq[0], ddq.sum()]
    s1 = _segment(params.com_distances[0], theta[0], rates[0], accelerations[0])
    elbow = _segment(params.lengths[0], theta[0], rates[0], accelerations[0])
    s2 = _segment(params.com_distances[1], theta[1], rates[1], accelerations[1])
    mount = np.asarray(params.mount_body)
    positions = np.array([mount + s1[0], mount + elbow[0] + s2[0]])
    velocities = np.array([s1[1], elbow[1] + s2[1]])
    accelerations = np.array([s1[2], elbow[2] + s2[2]])
    return positions, velocities, accelerations


def arm_reaction_feedforward(q, dq, ddq, *, R_world_body=None,
                             omega_body=(0., 0., 0.), alpha_body=(0., 0., 0.),
                             base_acceleration_world=(0., 0., 0.),
                             gravity_world=(0., 0., -9.81), params=None,
                             acceleration_limit=1.0):
    """Predict arm reactions and return bounded acceleration FF in world ENU.

    Exact model identity (before bounding):
        M_total * (a_base - g) + F_relative
        = M_fixed * (a_base - g) - predicted_arm_reaction_force.

    Thus ``acceleration_world`` = F_relative / M_total is ADDED to the common
    trajectory acceleration.  It has no arm-weight term.  The returned moment
    is the complete arm-on-base moment about base_link origin (NOT about the
    arm root); ``required_arm_moment_body`` is its opposite for diagnostics.

    acceleration_limit bounds the vector norm, preserving direction.  Set it
    to None only for analysis/tests; live callers should retain a finite bound.
    """
    params = params or ArmParameters()
    q, dq, ddq = (_vector(v, 2, n) for v, n in ((q, "q"), (dq, "dq"), (ddq, "ddq")))
    omega = _vector(omega_body, 3, "omega_body")
    alpha = _vector(alpha_body, 3, "alpha_body")
    base_acc = _vector(base_acceleration_world, 3, "base_acceleration_world")
    gravity = _vector(gravity_world, 3, "gravity_world")
    R = np.eye(3) if R_world_body is None else np.asarray(R_world_body, dtype=float)
    if R.shape != (3, 3) or not np.all(np.isfinite(R)) or not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or np.linalg.det(R) < 0:
        raise ValueError("R_world_body must be a proper rotation matrix")
    if acceleration_limit is not None and (not np.isfinite(acceleration_limit) or acceleration_limit <= 0):
        raise ValueError("acceleration_limit must be finite and positive or None")

    positions, velocities, relative_acc = arm_com_kinematics(q, dq, ddq, params)
    inertial_relative_acc_body = (relative_acc + 2 * np.cross(omega, velocities)
                                  + np.cross(alpha, positions)
                                  + np.cross(omega, np.cross(omega, positions)))
    inertial_relative_acc_world = inertial_relative_acc_body @ R.T
    absolute_acc_world = base_acc + inertial_relative_acc_world
    masses = np.asarray(params.masses)
    required_forces = masses[:, None] * (absolute_acc_world - gravity)
    relative_force_world = masses @ inertial_relative_acc_world
    raw_acceleration_world = relative_force_world / params.total_mass
    norm = np.linalg.norm(raw_acceleration_world)
    scale = 1.0 if acceleration_limit is None or norm <= acceleration_limit else acceleration_limit / norm

    required_moment_world = np.cross(positions @ R.T, required_forces).sum(axis=0)
    theta = np.array(params.angle_offsets) + [q[0], q.sum()]
    axis = np.array([0., 1., 0.])
    for i, (rate, accel) in enumerate(zip((dq[0], dq.sum()), (ddq[0], ddq.sum()))):
        link_R = R @ _ry(theta[i])
        I_world = link_R @ np.diag(params.inertia_diagonal[i]) @ link_R.T
        link_omega_world = R @ (omega + axis * rate)
        link_alpha_world = R @ (alpha + axis * accel + np.cross(omega, axis * rate))
        required_moment_world += I_world @ link_alpha_world + np.cross(link_omega_world, I_world @ link_omega_world)
    moment_body = R.T @ required_moment_world

    return {
        "acceleration_world": raw_acceleration_world * scale,
        "raw_acceleration_world": raw_acceleration_world,
        "relative_force_world": relative_force_world,
        "predicted_reaction_force_world": -required_forces.sum(axis=0),
        "predicted_reaction_moment_body": -moment_body,
        "required_arm_moment_body": moment_body,
        "saturated": bool(scale < 1.0),
        "saturation_scale": float(scale),
        "total_mass_kg": float(params.total_mass),
        "arm_com_position_body": positions,
        "arm_com_acceleration_world": absolute_acc_world,
    }
