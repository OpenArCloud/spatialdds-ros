#!/usr/bin/env python3
"""
The frame arithmetic, checked by composing it rather than by reading it.

An inverted or mis-scaled transform produces a well-formed pose with
plausible-looking numbers and places everything near the centre of the earth.
Nothing about the record is malformed, so nothing a schema check does will
find it. What finds it is using the transform and seeing where the answer
lands.

**On comparing against the prototype.** `spatialdds_demo/plaza.py` has the
same arithmetic and the two agree to 0.0 — but both use the same textbook
trace-branching quaternion extraction on the same ENU matrix, so that
agreement shows the transcription is faithful and *not* that two independent
derivations concur. The independent checks are the ones below: a round trip
through a different code path (iterative geodetic inversion), and a property
check that rotates basis vectors with the quaternion and asks where they
point.
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_ros.config import Anchor                      # noqa: E402
from spatialdds_ros.frames import (                           # noqa: E402
    earth_fixed_frame, enu_to_ecef_pose, frame_uuid, geodetic_of,
    georeference, robot_frame, yaw_quaternion)
from spatialdds_ros.topics import EARTH_FIXED_FQN             # noqa: E402

#: Somewhere with a sign on both axes, so a swapped or negated term shows.
ANCHOR = Anchor(30.28396857092828, -97.73974829101985, 145.083)

#: And a second one in a different hemisphere pair, because a formula can be
#: right at one latitude and wrong at another.
SOUTH_EAST = Anchor(-33.8688, 151.2093, 58.0)


def _rotate(q, v):
    """Rotate `v` by quaternion `q` = (x, y, z, w). Written out rather than
    imported, so this does not share code with what it is checking."""
    x, y, z, w = q
    t = [2 * (y * v[2] - z * v[1]),
         2 * (z * v[0] - x * v[2]),
         2 * (x * v[1] - y * v[0])]
    return [v[0] + w * t[0] + (y * t[2] - z * t[1]),
            v[1] + w * t[1] + (z * t[0] - x * t[2]),
            v[2] + w * t[2] + (x * t[1] - y * t[0])]


class TheProjectionRoundTrips(unittest.TestCase):
    def test_the_origin_comes_back(self):
        for anchor in (ANCHOR, SOUTH_EAST):
            with self.subTest(lat=anchor.lat_deg):
                t, _ = enu_to_ecef_pose(anchor.lat_deg, anchor.lon_deg,
                                        anchor.height_m)
                lat, lon, height = geodetic_of(t)
                self.assertAlmostEqual(lat, anchor.lat_deg, places=9)
                self.assertAlmostEqual(lon, anchor.lon_deg, places=9)
                self.assertAlmostEqual(height, anchor.height_m, places=3)

    def test_the_origin_is_one_earth_radius_from_the_centre(self):
        """
        The giveaway for an inverted transform is a translation near zero.
        """
        t, _ = enu_to_ecef_pose(ANCHOR.lat_deg, ANCHOR.lon_deg,
                                ANCHOR.height_m)
        self.assertGreater(math.hypot(*t), 6.0e6)


class TheBasisPointsWhereItShould(unittest.TestCase):
    """
    The independent check on the quaternion: rotate the local axes and compare
    against east/north/up derived separately from the trigonometry.
    """

    def _expected(self, anchor):
        lat, lon = math.radians(anchor.lat_deg), math.radians(anchor.lon_deg)
        sl, cl, sp, cp = (math.sin(lon), math.cos(lon),
                          math.sin(lat), math.cos(lat))
        return {"east": (-sl, cl, 0.0),
                "north": (-sp * cl, -sp * sl, cp),
                "up": (cp * cl, cp * sl, sp)}

    def test_x_is_east_y_is_north_z_is_up(self):
        for anchor in (ANCHOR, SOUTH_EAST):
            _, q = enu_to_ecef_pose(anchor.lat_deg, anchor.lon_deg,
                                    anchor.height_m)
            expected = self._expected(anchor)
            for axis, local in (("east", [1, 0, 0]), ("north", [0, 1, 0]),
                                ("up", [0, 0, 1])):
                with self.subTest(lat=anchor.lat_deg, axis=axis):
                    got = _rotate(q, local)
                    for a, b in zip(got, expected[axis]):
                        self.assertAlmostEqual(a, b, places=12)

    def test_up_at_the_anchor_points_away_from_the_earth_centre(self):
        """A sanity check that does not depend on the matrix at all."""
        t, q = enu_to_ecef_pose(ANCHOR.lat_deg, ANCHOR.lon_deg,
                                ANCHOR.height_m)
        up = _rotate(q, [0, 0, 1])
        radial = [c / math.hypot(*t) for c in t]
        dot = sum(a * b for a, b in zip(up, radial))
        self.assertGreater(dot, 0.999, "up should be very nearly radial")


class TheGeoreferenceSaysWhichEndIsWhich(unittest.TestCase):
    def test_parent_is_earth_fixed_and_child_is_the_local_frame(self):
        """
        The assertion that fails if someone follows the 1.7 announce's
        `from`/`to` names instead of the roles.
        """
        ft = georeference("map/bare-scene", ANCHOR)
        self.assertEqual(ft.parent_ref.fqn, EARTH_FIXED_FQN)
        self.assertEqual(ft.child_ref.fqn, "map/bare-scene")

    def test_composing_the_transform_lands_the_origin_on_the_anchor(self):
        """
        The composed check: take the local origin through the transform and see
        whether it arrives where the anchor says. This is what catches an
        inversion; reading the fields does not.
        """
        ft = georeference("map/bare-scene", ANCHOR)
        lat, lon, height = geodetic_of(ft.T_parent_child.t)
        self.assertAlmostEqual(lat, ANCHOR.lat_deg, places=9)
        self.assertAlmostEqual(lon, ANCHOR.lon_deg, places=9)
        self.assertAlmostEqual(height, ANCHOR.height_m, places=3)

    def test_a_point_ten_metres_north_lands_ten_metres_north(self):
        """
        Scale and orientation together, which is the half a round trip of the
        origin alone cannot see: the origin would still be right if the basis
        were rotated or the units wrong.
        """
        ft = georeference("map/bare-scene", ANCHOR)
        t, q = ft.T_parent_child.t, ft.T_parent_child.q
        offset = _rotate(q, [0.0, 10.0, 0.0])
        moved = [t[i] + offset[i] for i in range(3)]
        lat, lon, _ = geodetic_of(moved)
        north_m = (lat - ANCHOR.lat_deg) * 111_320.0
        east_m = ((lon - ANCHOR.lon_deg) * 111_320.0
                  * math.cos(math.radians(ANCHOR.lat_deg)))
        self.assertAlmostEqual(north_m, 10.0, delta=0.05)
        self.assertAlmostEqual(east_m, 0.0, delta=0.05)

    def test_the_covariance_arm_is_chosen_explicitly(self):
        """A union has to say which arm it is; there is no implicit absent."""
        ft = georeference("map/bare-scene", ANCHOR)
        self.assertEqual(ft.cov.discriminator.name, "COV_NONE")

    def test_it_survives_the_wire(self):
        from spatialdds_idl18.spatial.core import FrameTransform
        ft = georeference("map/bare-scene", ANCHOR)
        back = FrameTransform.deserialize(ft.serialize())
        self.assertEqual(back.child_ref.fqn, "map/bare-scene")
        self.assertEqual(back.T_parent_child.t, ft.T_parent_child.t)
        self.assertEqual(back.child_ref.scale_status.name, "SCALE_DECLARED")


class TheFrameRefsDeclareWhatTheyKnow(unittest.TestCase):
    def test_the_robot_frame_declares_metres(self):
        """REP-103 makes ROS metric, so this is a true statement rather than a
        guess — and SCALE_DECLARED rather than DERIVED, because nobody
        recovered it from a similarity transform."""
        ref = robot_frame("map/bare-scene")
        self.assertTrue(ref.has_scale)
        self.assertEqual(ref.scale_status.name, "SCALE_DECLARED")
        self.assertEqual(ref.meters_per_unit, 1.0)
        self.assertEqual(ref.display_unit, "m")

    def test_earth_fixed_claims_no_axis_convention(self):
        """ECEF is not a convention `CoordConvention` can name, so claiming one
        would be false and the flag is how the type says nothing."""
        self.assertFalse(earth_fixed_frame().has_coord_convention)

    def test_the_uuid_is_derived_from_the_name(self):
        """So three participants naming one frame produce one uuid, not three."""
        self.assertEqual(frame_uuid("map/x"), frame_uuid("map/x"))
        self.assertNotEqual(frame_uuid("map/x"), frame_uuid("map/y"))
        import uuid as _uuid
        self.assertEqual(frame_uuid("map/x"),
                         str(_uuid.uuid5(_uuid.NAMESPACE_URL, "map/x")))


class YawIsAboutUp(unittest.TestCase):
    def test_a_quarter_turn_takes_east_to_north(self):
        got = _rotate(yaw_quaternion(math.pi / 2), [1.0, 0.0, 0.0])
        for a, b in zip(got, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(a, b, places=12)

    def test_zero_yaw_is_identity(self):
        self.assertEqual(yaw_quaternion(0.0), [0.0, 0.0, 0.0, 1.0])


if __name__ == "__main__":
    unittest.main()
