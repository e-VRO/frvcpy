"""This module offers unit tests covering the translator, the solver, and its algorithm.

Typical usage:
    From the command-line:
        frvcpy-test
    From python:
        import frvcpy.test
        frvcpy.test.runAll()

"""

import json
import unittest

import pkg_resources

from frvcpy import solver
from frvcpy import translator

def assertDeepAlmostEqual(test_case, expected, actual, *args, **kwargs):
    """
    Assert that two complex structures have almost equal contents.

    Compares lists, dicts and tuples recursively. Checks numeric values
    using test_case's :py:meth:`unittest.TestCase.assertAlmostEqual` and
    checks all other values with :py:meth:`unittest.TestCase.assertEqual`.
    Accepts additional positional and keyword arguments and pass those
    intact to assertAlmostEqual() (that's how you specify comparison
    precision).

    :param test_case: TestCase object on which we can call all of the basic
    'assert' methods.
    :type test_case: :py:class:`unittest.TestCase` object

    See https://github.com/larsbutler/oq-engine/blob/master/tests/utils/helpers.py

    """
    
    is_root = not '__trace' in kwargs
    trace = kwargs.pop('__trace', 'ROOT')
    try:
        if isinstance(expected, (int, float, complex)):
            test_case.assertAlmostEqual(expected, actual, *args, **kwargs)
        elif isinstance(expected, (list, tuple)):
            test_case.assertEqual(len(expected), len(actual))
            for index in range(len(expected)):
                v1, v2 = expected[index], actual[index]
                assertDeepAlmostEqual(test_case, v1, v2,
                                      __trace=repr(index), *args, **kwargs)
        elif isinstance(expected, dict):
            test_case.assertEqual(set(expected), set(actual))
            for key in expected:
                assertDeepAlmostEqual(test_case, expected[key], actual[key],
                                      __trace=repr(key), *args, **kwargs)
        else:
            test_case.assertEqual(expected, actual)
    except AssertionError as exc:
        exc.__dict__.setdefault('traces', []).append(trace)
        if is_root:
            trace = ' -> '.join(reversed(exc.traces))
            exc = AssertionError("%s\nTRACE: %s" % (exc.message, trace))
        raise exc


class TestFrvcpAlgo(unittest.TestCase):
    """Unit tests for the translator and solver/algorithm"""

    RESULTS_FNAME = pkg_resources.resource_filename(
        "frvcpy.test", "data/testdata.json")
    MANUSCRIPT_INSTANCE = pkg_resources.resource_filename(
        "frvcpy.test", "data/vrprep-instance.xml")
    REFERENCE_INSTANCE = pkg_resources.resource_filename(
        "frvcpy.test", "data/frvcpy-instance.json")
    MANUSCRIPT_INSTANCE_NAME = "tc0c40s8cf0"
    MANUSCRIPT_ROUTE_NAME = "route_tc0c40s8cf0_23"

    def setUp(self):

        self.results = None
        with open(self.RESULTS_FNAME, 'r') as results_file:
            self.results = json.load(results_file)

        self.ref_instance = None
        with open(self.REFERENCE_INSTANCE, 'r') as inst_file:
            self.ref_instance = json.load(inst_file)

        self.q_init = self.ref_instance['max_q']

    def test_translation(self):
        """Translate an instance, compare it to a known reference"""

        frvcp_instance = translator.translate(self.MANUSCRIPT_INSTANCE)
        assertDeepAlmostEqual(self, frvcp_instance, self.ref_instance, 3)

    def test_manuscript_instance(self):
        """Test the algorithm on the route from the manuscript."""

        route = self.results[self.MANUSCRIPT_ROUTE_NAME]['route']
        known_obj = self.results[self.MANUSCRIPT_ROUTE_NAME]['obj']

        frvcp_solver = solver.Solver(
            self.REFERENCE_INSTANCE, route, self.q_init)
        obj, _ = frvcp_solver.solve()

        self.assertAlmostEqual(obj, known_obj, 3)

    def test_evrpnl_instances(self) -> bool:
        """Run solver on each of the E-VRP-NL routes for the instance."""

        for rte_info in self.results.values():
            frvcp_solver = solver.Solver(
                self.ref_instance, rte_info['route'], self.q_init)
            obj, _ = frvcp_solver.solve()
            self.assertAlmostEqual(obj, rte_info['obj'], 3)


def runAll():
    """Runs all unittests

    Effectively the same as running from the command line
        python -m unittest frvcpy.test.test -v

    """

    loader = unittest.TestLoader()
    test_dir = pkg_resources.resource_filename('frvcpy.test','.')
    suite = loader.discover(test_dir)

    runner = unittest.TextTestRunner(verbosity=2)
    runner.run(suite)

if __name__ == "__main__":
    unittest.main()
