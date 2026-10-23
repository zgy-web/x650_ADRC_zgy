"""Borrowed-pointer layout and synchronous lifetime, without a native library."""
import ctypes
from pathlib import Path
import sys
import unittest
import weakref

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from x650_wholebody.ts20_prediction import _borrow_native_pointer


class BorrowedNativePointerTests(unittest.TestCase):
    def test_float_integer_and_multidimensional_layout_preserve_exact_addresses(self):
        for dtype, element, shape in ((np.float64, ctypes.c_double, (8,)),
                                     (np.int32, ctypes.c_int, (9,)),
                                     (np.float64, ctypes.c_double, (7, 5))):
            array = np.arange(np.prod(shape), dtype=dtype).reshape(shape)
            pointer = _borrow_native_pointer(array, ctypes.POINTER(element))
            self.assertEqual(ctypes.addressof(pointer.contents), array.ctypes.data)
            self.assertEqual(pointer[0], array.ravel()[0])
            self.assertEqual(pointer[array.size-1], array.ravel()[-1])
            self.assertIsNone(pointer._objects)
            self.assertFalse(hasattr(pointer, "_arr"))

    def test_dtype_and_noncontiguous_inputs_refuse(self):
        for array in (np.zeros(3, np.float32), np.zeros(3, np.int64),
                      np.arange(8, dtype=np.float64)[::2], np.zeros(3, dtype=">f8")):
            with self.assertRaisesRegex(ValueError, "contiguous ndarray dtype"):
                _borrow_native_pointer(array, ctypes.POINTER(ctypes.c_double))

    def test_pointer_has_no_owner_cycle_and_dies_without_waiting_for_gc(self):
        array = np.arange(8, dtype=np.float64)
        pointer = _borrow_native_pointer(array, ctypes.POINTER(ctypes.c_double))
        array_ref, pointer_ref = weakref.ref(array), weakref.ref(pointer)
        del pointer, array
        self.assertIsNone(pointer_ref())
        self.assertIsNone(array_ref())

    def test_explicit_owner_tuple_keeps_array_alive_through_synchronous_call(self):
        array = np.arange(8, dtype=np.float64)
        owner = (array,)
        array_ref = weakref.ref(array)
        pointer = _borrow_native_pointer(array, ctypes.POINTER(ctypes.c_double))
        del array
        observed = []
        callback_type = ctypes.CFUNCTYPE(ctypes.c_double, ctypes.POINTER(ctypes.c_double))
        def synchronous(pointer_arg):
            observed.append(array_ref() is not None)
            return pointer_arg[0]+pointer_arg[7]
        callback = callback_type(synchronous)
        self.assertEqual(callback(pointer), 7.)
        self.assertEqual(observed, [True])
        del owner
        self.assertIsNone(array_ref())
        self.assertIsNone(pointer._objects)


if __name__ == "__main__":
    unittest.main()
