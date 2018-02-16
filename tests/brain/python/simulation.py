# Copyright (c) 2016, EPFL/Blue Brain Project
#                     Juan Hernando <juan.hernando@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#
# This library is free software; you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License version 3.0 as published
# by the Free Software Foundation.
#
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

import setup
import brain
import numpy

import unittest

class TestSimulationOpen(unittest.TestCase):

    def test_bad_open(self):
        self.assertRaises(RuntimeError, lambda: brain.Simulation("foo"))

    def test_open(self):
        simulation = brain.Simulation(brain.test.blue_config)

class TestSimulation(unittest.TestCase):
    def setUp(self):
        self.simulation = brain.Simulation(brain.test.blue_config)

    def test_open_circuit(self):
        circuit = self.simulation.open_circuit();

    def test_open_spike_report(self):
        report = self.simulation.open_spike_report();

    def test_open_compartment_report(self):
        names = self.simulation.compartment_report_names()
        for name in names:
            report = self.simulation.open_compartment_report(name);

    def test_gids(self):
        gids = self.simulation.gids()

if __name__ == '__main__':
    unittest.main()


