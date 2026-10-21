"""Bind one declared world target to a fixed PX4 local-NED position setpoint.

The flight controller's position loop remains in charge. No per-sample truth,
truth-minus-estimate correction, velocity feedback cancellation, or fitted
translation is accepted by this interface. The caller owns mode transitions,
fusion-state gates, sending, and command logging. An estimator-origin/reset
change after binding aborts this commissioning reference instead of silently
moving the target or re-centering the evaluation.
"""
from __future__ import annotations
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PINNED_PX4 = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
FRAME_MODES = ("global_wgs84", "raw_ev_world")
POSITION_YAW_MASK = 2552  # Ignore vx/vy/vz, ax/ay/az, yaw_rate; retain xyz/yaw.
EARTH_A, EARTH_B, PROJECTION_RADIUS = 6378137., 6356752.314245, 6371000.


def _finite(value, label):
    if isinstance(value, bool):
        raise ValueError(label+" must be a finite number")
    try:
        result = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(label+" must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(label+" must be finite")
    return result


def _vector(value, label):
    if len(value) != 3:
        raise ValueError(label+" must have three coordinates")
    return tuple(_finite(item, label) for item in value)


def read_world_datum(path):
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    worlds = [root] if root.tag == "world" else root.findall("world")
    if len(worlds) != 1:
        raise ValueError("Fixed hover requires one explicit Gazebo world")
    spherical = worlds[0].find("spherical_coordinates")
    names = ("latitude_deg", "longitude_deg", "elevation", "heading_deg")
    if spherical is None or any(spherical.findtext(name) is None for name in names):
        raise ValueError("World must declare its spherical latitude/longitude/elevation/heading")
    values = {name: _finite(spherical.findtext(name), "world "+name) for name in names}
    if (spherical.findtext("surface_model", "EARTH_WGS84") != "EARTH_WGS84"
            or spherical.findtext("world_frame_orientation", "ENU") != "ENU"
            or abs(values["heading_deg"]) > 1e-10):
        raise ValueError("Only WGS84 ENU worlds with zero heading are supported")
    if not -90 <= values["latitude_deg"] <= 90 or not -180 <= values["longitude_deg"] <= 180:
        raise ValueError("Invalid world geodetic origin")
    return {"source_path": str(path), "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "world_name": worlds[0].get("name"), **values}


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    latitude, longitude = math.radians(latitude_deg), math.radians(longitude_deg)
    e2 = 1-(EARTH_B/EARTH_A)**2
    radius = EARTH_A/math.sqrt(1-e2*math.sin(latitude)**2)
    return ((radius+altitude_m)*math.cos(latitude)*math.cos(longitude),
            (radius+altitude_m)*math.cos(latitude)*math.sin(longitude),
            (radius*(1-e2)+altitude_m)*math.sin(latitude))


def ecef_to_geodetic(point):
    x, y, z = _vector(point, "ECEF")
    p = math.hypot(x, y)
    e2 = 1-(EARTH_B/EARTH_A)**2
    if p < 1e-9:
        if abs(z) < 1.:
            raise ValueError("ECEF point is not near an Earth surface")
        return math.copysign(90., z), 0., abs(z)-EARTH_B
    latitude, longitude = math.atan2(z, p*(1-e2)), math.atan2(y, x)
    for _ in range(20):
        radius = EARTH_A/math.sqrt(1-e2*math.sin(latitude)**2)
        altitude = p/math.cos(latitude)-radius
        updated = math.atan2(z, p*(1-e2*radius/(radius+altitude)))
        if abs(updated-latitude) < 1e-15:
            latitude = updated
            break
        latitude = updated
    else:
        raise ValueError("WGS84 inverse conversion did not converge")
    radius = EARTH_A/math.sqrt(1-e2*math.sin(latitude)**2)
    altitude = p/math.cos(latitude)-radius
    return math.degrees(latitude), math.degrees(longitude), altitude


def world_to_geodetic(point, world):
    east, north, up = _vector(point, "world target")
    latitude, longitude = math.radians(world["latitude_deg"]), math.radians(world["longitude_deg"])
    origin = geodetic_to_ecef(world["latitude_deg"], world["longitude_deg"], world["elevation"])
    # Transpose of the WGS84 ECEF->ENU rotation, as used by Gazebo spherical coordinates.
    displacement = (-math.sin(longitude)*east-math.sin(latitude)*math.cos(longitude)*north+math.cos(latitude)*math.cos(longitude)*up,
                    math.cos(longitude)*east-math.sin(latitude)*math.sin(longitude)*north+math.cos(latitude)*math.sin(longitude)*up,
                    math.cos(latitude)*north+math.sin(latitude)*up)
    return ecef_to_geodetic(tuple(a+b for a, b in zip(origin, displacement)))


def project_north_east(latitude_deg, longitude_deg, reference_latitude_deg, reference_longitude_deg):
    """Pinned PX4 MapProjection::project (spherical azimuthal equidistant)."""
    latitude, longitude, lat0, lon0 = map(math.radians, (latitude_deg, longitude_deg, reference_latitude_deg, reference_longitude_deg))
    delta = longitude-lon0
    argument = math.sin(lat0)*math.sin(latitude)+math.cos(lat0)*math.cos(latitude)*math.cos(delta)
    c = math.acos(min(1., max(-1., argument)))
    if c > math.pi-.01:
        raise ValueError("Target is outside the local projection domain")
    scale = 1. if abs(c) < 1e-12 else c/math.sin(c)
    return (scale*(math.cos(lat0)*math.sin(latitude)-math.sin(lat0)*math.cos(latitude)*math.cos(delta))*PROJECTION_RADIUS,
            scale*math.cos(latitude)*math.sin(delta)*PROJECTION_RADIUS)


def position_snapshot(position):
    required = ("x", "y", "z", "ref_lat", "ref_lon", "ref_alt", "ref_timestamp",
                "xy_valid", "z_valid", "xy_global", "z_global", "xy_reset_counter", "z_reset_counter", "heading_reset_counter")
    missing = [name for name in required if not hasattr(position, name)]
    if missing:
        raise ValueError("VehicleLocalPosition lacks origin contract fields: "+str(missing))
    if not bool(position.xy_valid) or not bool(position.z_valid):
        raise RuntimeError("PX4 local XYZ estimate must be valid before fixed-position control")
    output = {name: _finite(getattr(position, name), name) for name in ("x", "y", "z", "ref_lat", "ref_lon", "ref_alt", "ref_timestamp")}
    output.update({name: bool(getattr(position, name)) for name in ("xy_valid", "z_valid", "xy_global", "z_global")})
    for name in ("xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
        value = _finite(getattr(position, name), name)
        if not value.is_integer() or not 0 <= value <= 255:
            raise ValueError("Invalid PX4 reset counter: "+name)
        output[name] = int(value)
    if not -90 <= output["ref_lat"] <= 90 or not -180 <= output["ref_lon"] <= 180:
        raise ValueError("Invalid PX4 reference latitude/longitude")
    return output


def _raw_ev_audit_origin(document):
    if document.get("frame_mode") != "raw_ev_world":
        raise ValueError("This inverse helper supports raw_ev_world only; use source-backed WGS84 conversion for global_wgs84")
    if document.get("px4_source_contract_commit") != PINNED_PX4:
        raise ValueError("Raw EV inverse requires the verified pinned PX4 source contract")
    return _finite(document["px4_reference_at_bind"]["ref_alt"], "bound altitude origin")


def local_positions_to_world(positions_nx3, document):
    """Audit-only raw-EV inverse; caller must verify logged origin/reset stability.

    Returns plain lists to keep the live reference module independent of numpy.
    Global WGS84 records use the existing source-backed hover-chain conversion,
    never this raw EV inverse or a fitted translation.
    """
    altitude_origin = _raw_ev_audit_origin(document)
    return [[east, north, altitude_origin-down]
            for north, east, down in (_vector(row, "local NED position") for row in positions_nx3)]


def local_velocities_to_world(velocities_nx3, document):
    """Raw-EV NED velocity -> world ENU; no origin or position-error correction."""
    _raw_ev_audit_origin(document)
    return [[east, north, -down]
            for north, east, down in (_vector(row, "local NED velocity") for row in velocities_nx3)]


class FixedWorldHoverReference:
    """One immutable target and one bound origin; no feedback-dependent encoding."""
    @classmethod
    def bind(cls, *, position, target_world_enu_m, world_sdf, yaw_ned, frame_mode="global_wgs84"):
        if frame_mode not in FRAME_MODES:
            raise ValueError("Unsupported fixed-hover reference frame mode")
        target = _vector(target_world_enu_m, "target_world_enu_m")
        yaw = _finite(yaw_ned, "yaw_ned")
        if not -math.pi <= yaw <= math.pi:
            raise ValueError("Fixed NED yaw must be in [-pi, pi]")
        reference = position_snapshot(position)
        world = read_world_datum(world_sdf)
        if frame_mode == "global_wgs84":
            if not reference["xy_global"] or not reference["z_global"] or reference["ref_timestamp"] <= 0:
                raise RuntimeError("Global world binding requires actual valid PX4 global reference")
            latitude, longitude, altitude = world_to_geodetic(target, world)
            north, east = project_north_east(latitude, longitude, reference["ref_lat"], reference["ref_lon"])
            local = (north, east, reference["ref_alt"]-altitude)
            conversion = {"target_geodetic": {"latitude_deg": latitude, "longitude_deg": longitude, "altitude_m": altitude},
                "position_semantics": "Gazebo WGS84 ENU -> ECEF -> geodetic -> actual PX4 MapProjection NED",
                "global_altitude_semantics": "Gazebo world geodetic altitude, consistent with native GNSS bridge; not re-zeroed"}
        else:
            # Explicitly selected only after the caller verifies EV XYZ/velocity
            # fusion, EV height reference, and no GNSS/baro aid. The pinned EKF
            # resets localXY to EV worldNE, but resets *global* altitude to
            # -EVz=worldZ while retaining the previously established ref_alt.
            # A source-proven conversion, NOT a fitted truth-minus-estimate bias.
            if not reference["z_global"] or reference["ref_timestamp"] <= 0:
                raise RuntimeError("Raw EV mode currently requires a previously established finite PX4 altitude origin")
            local = (target[1], target[0], reference["ref_alt"]-target[2])
            conversion = {"position_semantics": "Pinned raw Gazebo EV worldNE -> localNE; localD=retained_ref_alt-worldZ",
                "global_altitude_semantics": "Raw EV height reference makes reported global altitude equal Gazebo worldZ, NOT AMSL; never audit this mode with global_wgs84 conversion",
                "required_external_gate": "Caller verifies live EV position/height/velocity fusion, EV HGT_REF and no GNSS/baro aiding before binding",
                "source_contract": ["GZBridge::odometryCallback: EVz=-worldZ",
                    "ev_pos_control: no GNSS -> resetHorizontalPositionTo(EV measurement), EV bias reset",
                    "ev_height_control: EV height reference -> resetAltitudeTo(-measurement)",
                    "position_fusion: resetAltitudeTo retains previously established local origin altitude"]}
        value = cls()
        value._position, value._yaw, value._reference = tuple(local), yaw, deepcopy(reference)
        value._document = {"kind": "x650_fixed_world_hover_reference", "schema_version": 1,
            "profile": "fixed_world_position_v1", "frame_mode": frame_mode, "world": world,
            "target_world_enu_m": list(target), "target_local_ned_m": list(local), "fixed_yaw_ned_rad": yaw,
            "px4_reference_at_bind": reference, "px4_source_contract_commit": PINNED_PX4,
            "coordinate_frame": 1, "type_mask": POSITION_YAW_MASK,
            "controller": "PX4 native position loop, velocity/acceleration fields ignored",
            "target_frozen": True, "fit_estimator_translation": False, "truth_feedback_in_setpoint": False,
            "reset_policy": "Reject origin/global-validity or position/heading reset changes; never silently rebind or re-center evaluation",
            "ref_timestamp_policy": "Diagnostic only after bind because DDS may translate epoch; lat/lon/alt and reset counters establish identity",
            "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), **conversion}
        return value

    @property
    def document(self):
        return deepcopy(self._document)

    def check(self, position):
        current = position_snapshot(position)
        for name, tolerance in (("ref_lat", 1e-11), ("ref_lon", 1e-11), ("ref_alt", 1e-6)):
            if abs(current[name]-self._reference[name]) > tolerance:
                raise RuntimeError("PX4 origin changed after fixed world reference binding: "+name)
        for name in ("xy_global", "z_global", "xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
            if current[name] != self._reference[name]:
                raise RuntimeError("PX4 origin/reset state changed after fixed world reference binding: "+name)
        return current

    def setpoint(self):
        return {"coordinate_frame": 1, "type_mask": POSITION_YAW_MASK, "position": list(self._position),
                "velocity": [0., 0., 0.], "acceleration": [0., 0., 0.], "yaw": self._yaw, "yaw_rate": 0.}
