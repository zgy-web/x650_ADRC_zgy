"""Private numeric graph cloning retains deepcopy ownership and protocols."""
from copy import deepcopy
from pathlib import Path
import pickle
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'control_packages/x650_wholebody_v1'))
from x650_wholebody.adapter import _copy_numeric_diagnostics


class CustomCopy:
    def __init__(self, value):
        self.value = value

    def __deepcopy__(self, memo):
        result = type(self)(deepcopy(self.value, memo))
        memo[id(self)] = result
        result.used_copy_protocol = True
        return result


class NumericDiagnosticCopyTests(unittest.TestCase):
    def test_numeric_graph_bytes_values_order_and_shared_descendants_match(self):
        matrix = np.arange(150., dtype=float).reshape(15, 10)
        events = [{'time_s': i*.005, 'q': np.array([i*.001, -.02]),
                   'lower': [-.2, -.3], 'passed': True} for i in range(80)]
        graph = {'diagnostics': {'matrix': matrix, 'events': events},
            'matrix_alias': matrix, 'events_alias': events, 'label': 'held',
            'nothing': None, 'integer': 2**63+3, 'scalar': np.float64(.5)}
        expected, actual = deepcopy(graph), _copy_numeric_diagnostics(graph)
        self.assertEqual(pickle.dumps(expected, protocol=5), pickle.dumps(actual, protocol=5))
        self.assertIs(actual['matrix_alias'], actual['diagnostics']['matrix'])
        self.assertIs(actual['events_alias'], actual['diagnostics']['events'])
        self.assertFalse(np.shares_memory(actual['matrix_alias'], matrix))
        saved = pickle.dumps(actual, protocol=5)
        matrix[:] = 999.; events[0]['q'][:] = -999.; events[0]['lower'][0] = 321.
        self.assertEqual(pickle.dumps(actual, protocol=5), saved)

    def test_array_layout_dtype_readonly_and_scalar_shapes_match_deepcopy(self):
        base = np.arange(24, dtype=np.int64).reshape(4, 6)
        arrays = [base, base.T, base[:, ::2], np.asfortranarray(base),
            np.array(3.5), np.array([], dtype=np.float32),
            np.array([True, False]), np.array([1+2j]),
            np.array([(1., 2)], dtype=[('f', 'f8'), ('i', 'i8')])]
        for source in arrays:
            source.setflags(write=False)
            a, b = deepcopy(source), _copy_numeric_diagnostics(source)
            self.assertEqual(pickle.dumps(a, protocol=5), pickle.dumps(b, protocol=5))
            self.assertEqual(a.flags.writeable, b.flags.writeable)
            self.assertEqual(a.flags.c_contiguous, b.flags.c_contiguous)
            self.assertEqual(a.flags.f_contiguous, b.flags.f_contiguous)
            self.assertFalse(np.shares_memory(source, b))

    def test_cycles_and_existing_memo_preserve_graph_identity(self):
        graph = {}; values = [graph]; graph['values'] = values; graph['self'] = graph
        actual = _copy_numeric_diagnostics(graph)
        self.assertIs(actual['self'], actual)
        self.assertIs(actual['values'][0], actual)
        self.assertEqual(pickle.dumps(deepcopy(graph)), pickle.dumps(actual))
        replacement = []
        memo = {id(values): replacement}
        copied = _copy_numeric_diagnostics(graph, memo)
        self.assertIs(copied['values'], replacement)
        self.assertIs(memo[id(graph)], copied)

    def test_unknown_type_and_object_array_retain_deepcopy_protocol(self):
        item = CustomCopy([1., 2.])
        obj = np.empty(1, dtype=object); obj[0] = item
        graph = {'custom': item, 'custom_alias': item, 'objects': obj}
        a, b = deepcopy(graph), _copy_numeric_diagnostics(graph)
        self.assertEqual(pickle.dumps(a), pickle.dumps(b))
        self.assertTrue(b['custom'].used_copy_protocol)
        self.assertIs(b['custom'], b['custom_alias'])
        self.assertIs(b['objects'][0], b['custom'])
        self.assertIsNot(b['custom'].value, item.value)

    def test_tuple_list_cycle_and_mutation_are_detached(self):
        items = []; value = (items,); items.append(value)
        graph = {'tuple': value, 'list': items}
        a, b = deepcopy(graph), _copy_numeric_diagnostics(graph)
        self.assertEqual(pickle.dumps(a), pickle.dumps(b))
        self.assertIs(b['tuple'][0], b['list'])
        self.assertIs(b['list'][0], b['tuple'])
        b['list'].append('local')
        self.assertEqual(len(items), 1)

    def test_container_subclasses_keep_their_type_and_extra_state(self):
        class ListSubclass(list):
            pass
        class DictSubclass(dict):
            pass
        items = ListSubclass([np.array([.1, .2])]); items.note = {'x': [3.]}
        graph = DictSubclass(items=items); graph.label = ['extra']
        actual = _copy_numeric_diagnostics(graph)
        self.assertIs(type(actual), DictSubclass)
        self.assertIs(type(actual['items']), ListSubclass)
        self.assertEqual(actual.label, graph.label)
        self.assertIsNot(actual.label, graph.label)
        self.assertIsNot(actual['items'].note, items.note)
        self.assertFalse(np.shares_memory(actual['items'][0], items[0]))


if __name__ == '__main__':
    unittest.main()
