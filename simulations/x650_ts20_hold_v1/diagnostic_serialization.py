"""TS20 diagnostic JSON without copying an already supported container tree.

The shared recorder's converter remains the authority for every unsupported
input.  In particular, dictionary key coercion and the converter's intentionally
non-recursive ndarray.tolist()/numpy-scalar.item() behavior are not broadened.
This module changes neither collection policy nor the diagnostic JSON schema.
"""
from __future__ import annotations

import json
import math
import numpy as np


_JSON_ATOMS = frozenset((str, int, float, bool, type(None)))
_NUMERIC_SCALARS = frozenset((np.bool_, np.int8, np.int16, np.int32, np.int64,
                            np.uint8, np.uint16, np.uint32, np.uint64,
                            np.float16, np.float32, np.float64))
_MAX_CONTAINER_DEPTH = 32


def _supported(item, active):
    # Keep recursion at module scope: a per-call recursive closure would itself
    # create an avoidable function/cell reference cycle on every encoded row.
    kind = type(item)
    if kind in _JSON_ATOMS:
        return True
    if kind in _NUMERIC_SCALARS:
        # float64 can bypass default() as a Python-float subclass.  Some
        # Python/NumPy versions then include its NumPy repr in the NaN/Inf
        # error; the original helper first converts it to a Python float.
        return kind not in (np.float16, np.float32, np.float64) or math.isfinite(item)
    if kind is np.ndarray:
        return item.dtype.type in _NUMERIC_SCALARS
    if kind not in (dict, list, tuple):
        return False
    # json_value's Python recursion can fail before the C encoder would fail.
    # Deep inputs are outside this diagnostic-specific fast domain; let the
    # original converter retain its own acceptance/error behavior for them.
    if len(active) >= _MAX_CONTAINER_DEPTH:
        return False
    identity = id(item)
    if identity in active:
        return False
    active.add(identity)
    try:
        if kind is dict:
            for key, child in item.items():
                if type(key) is not str or not _supported(child, active):
                    return False
        else:
            for child in item:
                if not _supported(child, active):
                    return False
        return True
    finally:
        active.remove(identity)


def supports_fast_tree(value):
    """Recognize a narrow value domain without constructing a second tree.

    Exact built-in containers/keys exclude custom iteration/string-coercion
    behavior.  Object, structured, complex, string, datetime and long-double
    arrays/scalars use the original converter, including its original errors.
    Cycles and inputs beyond 32 container levels go to that converter rather
    than acquiring a new acceptance/error policy. Shared acyclic references
    are valid.
    """
    try:
        return _supported(value, set())
    except RecursionError:
        return False


class _NumericEncoder(json.JSONEncoder):
    def default(self, value):
        # Domain recognition has already excluded non-recursive .tolist/.item
        # counterexamples.  Numeric array conversion is still required by JSON;
        # surrounding dict/list/tuple containers are passed to the C encoder.
        if type(value) is np.ndarray and value.dtype.type in _NUMERIC_SCALARS:
            return value.tolist()
        if type(value) in _NUMERIC_SCALARS:
            return value.item()
        return super().default(value)


def encode_diagnostic_row(row, *, reference_converter):
    """Return the original JSON line bytes as text, or the original error.

    The explicitly supplied converter is the shared runner's current
    ``json_value`` function.  The unsupported path is its original expression,
    not a more permissive JSONEncoder.default fallback.
    """
    if supports_fast_tree(row):
        return json.dumps(row, cls=_NumericEncoder, allow_nan=False) + "\n"
    return json.dumps(reference_converter(row), allow_nan=False) + "\n"
