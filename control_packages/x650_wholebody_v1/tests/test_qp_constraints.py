"""Polytope equivalence, including signed rows and numerical proof margins."""
from itertools import product
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from x650_wholebody.qp_constraints import solver_constraint_view


class SolverConstraintViewTests(unittest.TestCase):
    def test_signed_zero_rows_may_remain_distinct_without_changing_feasibility(self):
        C = np.array([[1., 0.], [1., -0.], [0., 1.]])
        a, l, h, info = solver_constraint_view(C, [-1., -.5, -1.], [1., .5, 1.])
        self.assertEqual(info["duplicate_rows_merged"], 0)
        self.assertEqual(info["solver_rows"], 3)
        self.assertFalse(np.signbit(a[0, 1]))
        self.assertTrue(np.signbit(a[1, 1]))
        np.testing.assert_array_equal(l, [-1., -.5, -1.])
        np.testing.assert_array_equal(h, [1., .5, 1.])

    def test_overflow_or_unbounded_proof_retains_original_rows(self):
        for C, lo, hi in (
            ([[1e-320, 0.], [0., 1.], [1., 1.]], [-1., -1., -1e308], [1., 1., 1e308]),
            ([[1., 0.], [0., 1.], [1e308, 1e308]], [-2., -2., -1e308], [2., 2., 1e308]),
        ):
            with np.errstate(over="raise", invalid="raise"):
                a, l, h, info = solver_constraint_view(C, lo, hi)
            self.assertEqual(info["box_implied_rows_omitted"], 0)
            np.testing.assert_array_equal(a, C)
            np.testing.assert_array_equal(l, lo)
            np.testing.assert_array_equal(h, hi)

    def test_identical_rows_intersect_bounds_without_merging_nearby_rows(self):
        C = np.array([[1., 0.], [1., 0.], [1., 1e-25], [0., 1.]])
        a, l, h, info = solver_constraint_view(C, [-1., -.5, -.1, -1.], [1., .3, .1, 1.])
        self.assertEqual(info["duplicate_rows_merged"], 1)
        np.testing.assert_array_equal(a, C[[0, 2, 3]])
        np.testing.assert_array_equal(l, [-.5, -.1, -1.])
        np.testing.assert_array_equal(h, [.3, .1, 1.])

    def test_negative_singletons_keep_all_deleted_rows_true_at_box_vertices(self):
        C = np.array([[-2., 0.], [0., 4.], [1., 1.], [1., -1.], [0., 0.]])
        lo, hi = np.array([-.4, -.4, -.8, -.05, -.1]), np.array([.2, .8, .8, .05, .1])
        a, l, h, info = solver_constraint_view(C, lo, hi)
        self.assertEqual(info["box_implied_rows_omitted"], 2)
        # Independently derived true singleton box x=[-.1,.2], y=[-.1,.2].
        for point in product([-.1, .2], repeat=2):
            value = C@point
            self.assertGreaterEqual(value[2], lo[2])
            self.assertLessEqual(value[2], hi[2])
        np.testing.assert_array_equal(a, C[[0, 1, 3]])

    def test_boundary_rows_are_not_omitted_by_roundoff_or_small_coefficients(self):
        C = np.array([[1., 0.], [0., 1.], [1., 1.], [1., 1e-300]])
        a, _, _, info = solver_constraint_view(C, [-1., -1., -2., -.5], [1., 1., 2., .5])
        self.assertEqual(info["box_implied_rows_omitted"], 0)
        np.testing.assert_array_equal(a, C)

    def test_empty_retained_box_refuses_and_infeasible_constant_is_kept(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            solver_constraint_view([[1.], [1.]], [-1., .5], [-.5, 1.])
        with self.assertRaisesRegex(ValueError, "single-variable"):
            solver_constraint_view([[1.], [-1.]], [.5, .5], [1., 1.])
        C = np.array([[1., 0.], [0., 1.], [0., 0.]])
        a, l, h, _ = solver_constraint_view(C, [-1., -1., 1.], [1., 1., 2.])
        np.testing.assert_array_equal(a, C)
        self.assertEqual(l[-1], 1.)

    def test_random_polytope_membership_is_unchanged(self):
        rng = np.random.default_rng(82026)
        for _ in range(10):
            n = 4
            boxlo, boxhi = rng.uniform(-.5, -.1, n), rng.uniform(.1, .5, n)
            other = rng.normal(size=(10, n))
            C = np.vstack((np.eye(n), -2*np.eye(n), other, other[:2]))
            lo = np.r_[boxlo, -2*boxhi, np.full(10, -1.), np.full(2, -.8)]
            hi = np.r_[boxhi, -2*boxlo, np.full(10, 1.), np.full(2, .8)]
            a, l, h, _ = solver_constraint_view(C, lo, hi)
            z = rng.uniform(-1., 1., (2000, n))
            before = np.all((C@z.T >= lo[:, None]) & (C@z.T <= hi[:, None]), axis=0)
            after = np.all((a@z.T >= l[:, None]) & (a@z.T <= h[:, None]), axis=0)
            np.testing.assert_array_equal(after, before)


if __name__ == "__main__":
    unittest.main()
