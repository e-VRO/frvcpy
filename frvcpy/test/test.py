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
        self.assertEqual(frvcp_instance, self.ref_instance)

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
