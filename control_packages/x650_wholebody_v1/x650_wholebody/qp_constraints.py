"""Equivalent linear-constraint view for the existing dense QP solver.

The full original constraints remain authoritative for independent quality
checks. No tolerance-based merging or approximate active-set selection occurs.
"""
from __future__ import annotations

import numpy as np


def solver_constraint_view(matrix, lower, upper):
    """Merge identical rows and omit rows implied by retained scalar bounds.

Each singleton row stays in the solver. Its quotient bounds are rounded
outwards before interval arithmetic, and a conservative floating-point guard
is required before dropping any other row. This does not assume that the
current warm start remains active or that a particular future QP solution is
near it. Near-boundary, equality, and unbounded proofs remain uncompressed.
"""
    C, lo, hi = np.asarray(matrix, dtype=float), np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if (C.ndim != 2 or lo.shape != (len(C),) or hi.shape != lo.shape
            or not np.isfinite(C).all() or not np.isfinite(lo).all()
            or not np.isfinite(hi).all()):
        raise ValueError("Finite complete linear constraints required")
    if np.any(lo > hi):
        raise ValueError("Inconsistent original linear-constraint interval")
    rows, lows, highs, identities = [], [], [], {}
    for row, low, high in zip(C, lo, hi):
        # Signed zero variants need not merge; equality of byte-identical
        # float vectors is sufficient and never conflates nearby normals.
        key = row.tobytes()
        index = identities.get(key)
        if index is None:
            identities[key] = len(rows)
            rows.append(row)
            lows.append(low)
            highs.append(high)
        else:
            lows[index] = max(lows[index], low)
            highs[index] = min(highs[index], high)
    if not rows:
        return C.copy(), lo.copy(), hi.copy(), {"original_rows": 0, "solver_rows": 0,
            "duplicate_rows_merged": 0, "box_implied_rows_omitted": 0}
    merged = np.asarray(rows)
    lows, highs = np.asarray(lows), np.asarray(highs)
    if np.any(lows > highs):
        raise ValueError("Inconsistent duplicate linear-constraint intervals")
    singleton = np.count_nonzero(merged, axis=1) == 1
    box_lo, box_hi = np.full(C.shape[1], -np.inf), np.full(C.shape[1], np.inf)
    for row, low, high in zip(merged[singleton], lows[singleton], highs[singleton]):
        column = np.flatnonzero(row)[0]
        with np.errstate(over="ignore", invalid="ignore"):
            a, b = sorted((low/row[column], high/row[column]))
            box_lo[column] = max(box_lo[column], np.nextafter(a, -np.inf))
            box_hi[column] = min(box_hi[column], np.nextafter(b, np.inf))
    if np.any(box_lo > box_hi):
        raise ValueError("Inconsistent retained single-variable constraint bounds")
    omitted = np.zeros(len(merged), dtype=bool)
    if (np.isfinite(box_lo).all() and np.isfinite(box_hi).all()
            and np.all(box_lo <= box_hi) and np.all(lows <= highs)):
        positive, negative = np.maximum(merged, 0.), np.minimum(merged, 0.)
        with np.errstate(over="ignore", invalid="ignore"):
            minimum = positive@box_lo+negative@box_hi
            maximum = positive@box_hi+negative@box_lo
            magnitude = np.abs(merged)@np.maximum(abs(box_lo), abs(box_hi))
            guard = 64*np.finfo(merged.dtype).eps*(C.shape[1]+1)*(1+magnitude+abs(lows)+abs(highs))
            omitted = (~singleton & np.isfinite(minimum) & np.isfinite(maximum)
                       & np.isfinite(guard) & (minimum > lows+guard) & (maximum < highs-guard))
    keep = ~omitted
    return merged[keep], lows[keep], highs[keep], {
        "original_rows": len(C), "solver_rows": int(keep.sum()),
        "duplicate_rows_merged": len(C)-len(merged),
        "box_implied_rows_omitted": int(omitted.sum()),
        "proof": "retained singleton bounds; outward quotient rounding and guarded row intervals",
        "full_constraints_retained_for_quality": True}
