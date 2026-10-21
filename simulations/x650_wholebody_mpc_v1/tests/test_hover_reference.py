"""Fixed target/frame tests without ROS, Gazebo, or flight-controller access."""
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hover_reference import (EARTH_A, EARTH_B, FixedWorldHoverReference,
    POSITION_YAW_MASK, ecef_to_geodetic, geodetic_to_ecef,
    local_positions_to_world, local_velocities_to_world,
    project_north_east, read_world_datum, world_to_geodetic)


def position(**changes):
    values = dict(x=.1, y=-.2, z=-2., ref_lat=0., ref_lon=0., ref_alt=488.,
        ref_timestamp=1000000, xy_valid=True, z_valid=True, xy_global=True,
        z_global=True, xy_reset_counter=2, z_reset_counter=1,
        heading_reset_counter=0)
    values.update(changes)
    return SimpleNamespace(**values)


class FixedWorldReferenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.world = Path(self.temporary.name)/"world.sdf"
        self.world.write_text('''<sdf version="1.9"><world name="test">
          <spherical_coordinates><surface_model>EARTH_WGS84</surface_model>
          <world_frame_orientation>ENU</world_frame_orientation>
          <latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg>
          <elevation>488</elevation><heading_deg>0</heading_deg>
          </spherical_coordinates></world></sdf>''', encoding="utf-8")

    def bind(self, **changes):
        args = dict(position=position(), target_world_enu_m=[0., 0., 2.37],
                    world_sdf=self.world, yaw_ned=.25)
        args.update(changes)
        return FixedWorldHoverReference.bind(**args)

    def assertVectorClose(self, actual, expected, tolerance=1e-7):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def test_wgs84_known_axes_and_geodetic_round_trip(self):
        self.assertVectorClose(geodetic_to_ecef(0., 0., 0.), [EARTH_A, 0., 0.])
        self.assertVectorClose(geodetic_to_ecef(0., 90., 10.), [0., EARTH_A+10., 0.])
        self.assertVectorClose(ecef_to_geodetic([0., 0., EARTH_B+12.]), [90., 0., 12.])
        for geodetic in ((47.397742, 8.545594, 490.37), (-34., -179., 12.), (80., 45., 1500.)):
            recovered = ecef_to_geodetic(geodetic_to_ecef(*geodetic))
            self.assertVectorClose(recovered[:2], geodetic[:2], tolerance=1e-10)
            self.assertAlmostEqual(recovered[2], geodetic[2], delta=1e-7)

    def test_world_conversion_known_equatorial_east_point(self):
        # At lat=lon=0: ECEF=(A+elevation+worldZ,worldX,worldY).
        east, up = 12., 2.37
        actual = world_to_geodetic([east, 0., up], read_world_datum(self.world))
        expected = (0., math.degrees(math.atan2(east, EARTH_A+488.+up)),
                    math.hypot(EARTH_A+488.+up, east)-EARTH_A)
        self.assertVectorClose(actual, expected)
        north, mapped_east = project_north_east(actual[0], actual[1], 0., 0.)
        self.assertAlmostEqual(north, 0., delta=1e-9)
        self.assertAlmostEqual(mapped_east, math.radians(expected[1])*6371000., delta=1e-8)

    def test_global_binding_uses_declared_world_and_actual_estimator_origin(self):
        reference = self.bind()
        self.assertVectorClose(reference.setpoint()["position"], [0., 0., -2.37])
        # A different *actual reference origin* must change the encoded target,
        # without using the estimate's current x/y/z or shifting the world task.
        shifted = self.bind(position=position(ref_lon=.0001, ref_alt=489., x=99., y=-88., z=77.))
        self.assertVectorClose(shifted.setpoint()["position"],
                               [0., -math.radians(.0001)*6371000., -1.37])
        self.assertEqual(shifted.document["target_world_enu_m"], [0., 0., 2.37])
        self.assertFalse(shifted.document["fit_estimator_translation"])
        self.assertFalse(shifted.document["truth_feedback_in_setpoint"])
        self.assertEqual(len(shifted.document["world"]["source_sha256"]), 64)

    def test_position_and_timestamp_changes_do_not_move_frozen_target(self):
        reference = self.bind()
        packet = reference.setpoint()
        reference.check(position(x=-100., y=200., z=8., ref_timestamp=1792600000000000))
        self.assertEqual(reference.setpoint(), packet)
        reference.check(position(x=3., y=4., z=5.))
        self.assertEqual(reference.setpoint(), packet)

    def test_origin_reset_or_validity_changes_fail_closed(self):
        reference = self.bind()
        cases = ({"ref_lat": .000001}, {"ref_lon": .000001}, {"ref_alt": 488.01},
                 {"xy_reset_counter": 3}, {"z_reset_counter": 2},
                 {"heading_reset_counter": 1}, {"xy_global": False},
                 {"z_global": False}, {"xy_valid": False}, {"z_valid": False})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                reference.check(position(**changes))
        with self.assertRaises(ValueError):
            reference.check(position(x=float("nan")))

    def test_raw_ev_retained_altitude_origin_and_explicit_audit_contract(self):
        reference = self.bind(frame_mode="raw_ev_world", target_world_enu_m=[3., -4., 2.37])
        self.assertVectorClose(reference.setpoint()["position"], [-4., 3., 485.63])
        self.assertEqual(reference.document["frame_mode"], "raw_ev_world")
        self.assertIn("NOT AMSL", reference.document["global_altitude_semantics"])
        self.assertIn("Caller verifies", reference.document["required_external_gate"])
        # The reused nominal latitude/longitude do not define raw EV local XY.
        other = self.bind(frame_mode="raw_ev_world", target_world_enu_m=[3., -4., 2.37],
                          position=position(ref_lat=47., ref_lon=8.))
        self.assertEqual(reference.setpoint(), other.setpoint())
        with self.assertRaises(RuntimeError):
            self.bind(frame_mode="raw_ev_world", position=position(z_global=False))

    def test_mavlink_mask_retains_only_xyz_and_yaw(self):
        packet = self.bind().setpoint()
        self.assertEqual(packet["coordinate_frame"], 1)
        self.assertEqual(packet["type_mask"], POSITION_YAW_MASK)
        for bit in (0, 1, 2, 9, 10):  # Position active, acceleration not force, yaw active.
            self.assertFalse(packet["type_mask"] & (1 << bit))
        for bit in (3, 4, 5, 6, 7, 8, 11):
            self.assertTrue(packet["type_mask"] & (1 << bit))
        self.assertEqual(packet["yaw"], .25)

    def test_raw_ev_audit_inverse_preserves_declared_world_target(self):
        reference = self.bind(frame_mode="raw_ev_world", target_world_enu_m=[3., -4., 2.37])
        world = local_positions_to_world([reference.setpoint()["position"]], reference.document)
        self.assertVectorClose(world[0], [3., -4., 2.37])
        self.assertEqual(local_velocities_to_world([[1., -2., 3.]], reference.document), [[-2., 1., -3.]])
        with self.assertRaisesRegex(ValueError, "WGS84"):
            local_positions_to_world([[0., 0., 0.]], self.bind().document)
        mismatched = reference.document
        mismatched["px4_source_contract_commit"] = "unverified"
        with self.assertRaisesRegex(ValueError, "pinned"):
            local_velocities_to_world([[1., 2., 3.]], mismatched)
        with self.assertRaises(ValueError):
            local_positions_to_world([[0., 0., float("nan")]], reference.document)

    def test_mutating_audit_or_packet_copy_cannot_change_bound_target(self):
        reference = self.bind()
        original = reference.setpoint()
        packet, document = reference.setpoint(), reference.document
        packet["position"][0] = 100.
        document["target_local_ned_m"][0] = 100.
        document["px4_reference_at_bind"]["ref_alt"] = 0.
        self.assertEqual(reference.setpoint(), original)
        self.assertEqual(reference.document["px4_reference_at_bind"]["ref_alt"], 488.)
        reference.check(position())

    def test_invalid_or_missing_reference_and_world_contract_are_rejected(self):
        for changes in ({"xy_global": False}, {"z_global": False}, {"ref_timestamp": 0}):
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                self.bind(position=position(**changes))
        for args in ({"yaw_ned": float("nan")}, {"yaw_ned": 4.},
                     {"target_world_enu_m": [0., 0.]}, {"frame_mode": "fit_estimator"},
                     {"position": position(ref_alt=float("nan"))}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.bind(**args)
        incomplete = position()
        del incomplete.ref_lat
        with self.assertRaisesRegex(ValueError, "origin contract"):
            self.bind(position=incomplete)
        original = self.world.read_text(encoding="utf-8")
        for replacement in (original.replace("<heading_deg>0", "<heading_deg>90"),
                            original.replace("<elevation>488</elevation>", ""),
                            original.replace("EARTH_WGS84", "MOON_SCS")):
            self.world.write_text(replacement, encoding="utf-8")
            with self.assertRaises(ValueError):
                self.bind()


if __name__ == "__main__":
    unittest.main()
