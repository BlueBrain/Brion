# Copyright (c) 2017, EPFL/Blue Brain Project
#                     Juan Hernando <juan.hernando@epfl.ch>
#                     Mohamed-Ghaith Kaabi <mohamed.kaabi@epfl.ch>
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
import brion
import numpy
import unittest
from brion import *

report_path = brion.test.root_data_path + "/local/simulations/may17_2011/Control/voltage.h5"
all_compartments_report_path = brion.test.root_data_path + "/local/simulations/may17_2011/Control/allCompartments.h5"


class TestMetaData(unittest.TestCase):
    def setUp(self):
        self.report = CompartmentReport(all_compartments_report_path)

    def test_metadata(self):
        metadata = self.report.metadata
        assert(metadata['data_unit'] == 'mV')
        assert(metadata['time_unit'] == 'ms')
        assert(metadata['start_time'] == 0.0)
        assert(metadata['end_time'] == 10.0)
        assert(numpy.isclose(metadata['time_step'], 0.1))
        assert(metadata['frame_count'] == 100)
        assert(self.report.cell_count == 35)


class TestReader(unittest.TestCase):
    def setUp(self):
        self.report  = CompartmentReport(report_path)

    def test_metadata(self):
        metadata = self.report.metadata
        assert(metadata['data_unit'] == 'mV')
        assert(metadata['time_unit'] == 'ms')
        assert(metadata['start_time'] == 0.0)
        assert(metadata['end_time'] == 10.0)
        assert(numpy.isclose(metadata['time_step'], 0.1))
        assert(metadata['frame_count'] == 100)
        assert(self.report.cell_count == 600)

    def test_gids(self):
        assert((self.report.gids == numpy.arange(1, 601, 1)).all())

    def test_create_view(self):
        view = self.report.create_view({1, 2, 3})
        assert((view.gids == [1, 2, 3]).all())

    def test_mapping(self):
        view = self.report.create_view({1, 2, 3})
        mapping = view.mapping
        assert(mapping.num_compartments(1) == 1)
        assert(mapping.index.tolist() == [(1, 0), (2, 0), (3, 0)])
        assert(mapping.offsets == [[0], [1], [2]])
        assert(mapping.compartment_counts() ==  [[1], [1], [1]])
        assert(mapping.frame_size == 3)

    def test_frames(self):

        view = self.report.create_view({1, 2, 3})
        timestamps, frames = view.load(0.0, 0.2)

        assert(numpy.isclose(timestamps,[0.0, 0.1]).all())
        assert(numpy.isclose(frames, [[-65., -65., -65.],
            [-65.14350891, -65.29447937, -65.44480133]]).all())

        # load(start, end)
        timestamps, frames = view.load(0.05, 0.25)
        # This window overlaps frames [0, 0.1), [0.1, 0.2), [0.2, 03)
        assert(len(timestamps) == 3)
        assert(frames.shape == (3, 3))
        timestamps, frames = view.load(0.09, 0.12)
        # Despite the time window is smaller than the timestep, each edge is
        # falling on a different frame, so we get two frames.
        assert(len(timestamps) == 2)
        assert(frames.shape == (2, 3))

        # load(start, end, step)
        timestamps, frames = view.load(0.0, 1.0, 0.2)
        assert(len(timestamps) == 5)
        assert(numpy.isclose(timestamps,[0.0, 0.2, 0.4, 0.6, 0.8]).all())
        assert(frames.shape == (5, 3))

        timestamps, frames = view.load(0.0, 1.0, 0.3)
        assert(len(timestamps) == 4)
        assert(numpy.isclose(timestamps,[0.0, 0.3, 0.6, 0.9]).all())
        assert(frames.shape == (4, 3))

        timestamps, frames = view.load(0.0, 0.001, 0.1)
        assert(len(timestamps) == 1)
        assert(numpy.isclose(timestamps,[0.0]).all())
        assert(frames.shape == (1, 3))

        # load_all()
        timestamps, frames = view.load_all()
        assert(len(timestamps) == 100)
        timestamp, frame = view.load(0.1)
        assert(numpy.isclose(timestamp, 0.1))
        assert(frame.shape == (3,))
        assert((frame ==
            [-65.14350891113281, -65.29447937011719, -65.4448013305664]).all())

        # temp view
        timestamp, frame = self.report.create_view({1, 2, 3}).load(0.1)
        assert(numpy.isclose(timestamp, 0.1))
        assert(frame.shape == (3,))
        assert((frame ==
            [-65.14350891113281, -65.29447937011719, -65.4448013305664]).all())

        # single neuron
        view = self.report.create_view({1})

        timestamp, frames = view.load(0.0)
        assert(numpy.isclose(timestamp, 0.0))
        assert(numpy.isclose(frames,[-65.]).all())

        timestamps, frames = view.load(0.0, 0.2)
        assert(numpy.isclose(timestamps, [0.0, 0.1]).all())
        assert(numpy.isclose(frames,[[-65.], [-65.14350891]]).all())

        timestamps, frames = view.load_all()
        assert(len(timestamps) == 100)
        assert(frames.shape == (100, 1))

        # temp view
        timestamps, frames = self.report.create_view({1, 2, 3}).load(0.0, 0.2)
        assert(numpy.isclose(timestamps,[ 0.0, 0.1]).all())
        assert(numpy.isclose(frames, [[-65., -65., -65.],
            [-65.14350891, -65.29447937, -65.44480133]]).all())

        #load all
        timestamps, frames = self.report.create_view().load_all()
        assert(len(timestamps) == 100)
        assert(frames.shape == (100, 600))

class TestReaderExceptions(unittest.TestCase):
    def setUp(self):
        self.report = CompartmentReport(report_path)
        self.view = self.report.create_view({1, 2, 3})
        metadata = self.report.metadata
        self.start = metadata['start_time']
        self.end = metadata['end_time']
        self.step = metadata['time_step']

    def test_load_timestamp(self):

        self.assertRaises(RuntimeError, CompartmentReportView.load, self.view,
                          self.start - 1)
        self.assertRaises(RuntimeError, CompartmentReportView.load, self.view,
                          self.end + 1)

    def test_load_range(self):

        self.assertRaises(RuntimeError, CompartmentReportView.load, self.view,
                          self.end, self.start)
        ts, fs = self.view.load(self.start - 20, self.start)
        assert(len(ts) == 0 and len(fs) == 0)
        ts, fs = self.view.load(self.end, self.end + 10)
        assert(len(ts) == 0 and len(fs) == 0)
        ts, fs = self.view.load(self.start - 10, self.end + 10)
        assert(len(ts) == 100 and len(fs) == 100)

    def test_load_range_step(self):

        self.view.load(0, 100, self.step * 1000000000)

        self.assertRaises(RuntimeError, CompartmentReportView.load, self.view,
                          self.start, self.end, self.step * 0.9999)
        self.assertRaises(RuntimeError, CompartmentReportView.load, self.view,
                          self.start, self.end, self.step * 1.001)
        self.assertRaises(RuntimeError, CompartmentReportView.load, self.view,
                          self.start, self.end, -self.step)
        self.assertRaises(RuntimeError, CompartmentReportView.load, self.view,
                          self.start, self.end, 0)

class TestMemoryManagement(unittest.TestCase):

    def test_view(self):
        view = CompartmentReport(report_path).create_view({1, 2, 3})
        assert((view.gids == [1, 2, 3]).all())

    def test_frame(self):
        timestamp, frame = CompartmentReport(report_path).create_view(
            {1, 2, 3}).load(0.1)
        assert(numpy.isclose(timestamp, 0.1))
        assert((frame ==
            [-65.14350891113281, -65.29447937011719, -65.4448013305664]).all())

    def test_mapping(self):
        mapping = CompartmentReport(report_path).create_view(
            {1, 2, 3}).mapping
        assert(mapping.num_compartments(1) == 1)
        assert(mapping.index.tolist() == [(1, 0), (2, 0), (3, 0)])

if __name__ == '__main__':
    unittest.main()
