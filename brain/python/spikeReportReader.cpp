/* Copyright (c) 2006-2018, Ahmet Bilgili <ahmet.bilgili@epfl.ch>
 *                          Juan Hernando <jhernando@fi.upm.es>
 *
 * This file is part of Brion <https://github.com/BlueBrain/Brion>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <boost/python.hpp>

#include "arrayHelpers.h"
#include "helpers.h"
#include "pythonDocumentation.h"

#include <brain/spikeReportReader.h>
#include <brain/types.h>

namespace bp = boost::python;

namespace brain
{
namespace
{
using namespace brain_python;

SpikeReportReaderPtr _initURI(const std::string& uri)
{
    return SpikeReportReaderPtr(new SpikeReportReader(brion::URI(uri)));
}

SpikeReportReaderPtr _initURIandGIDSet(const std::string& uri, bp::object gids)
{
    return SpikeReportReaderPtr(
        new SpikeReportReader(brion::URI(uri), gidsFromPython(gids)));
}

bp::object SpikeReportReader_getSpikes(SpikeReportReader& reader,
                                       const float startTime,
                                       const float endTime)
{
    return toNumpy(reader.getSpikes(startTime, endTime));
}
}

void export_SpikeReportReader()
{
    // clang-format off
const auto selfarg = bp::arg("self");

bp::class_<SpikeReportReader, boost::noncopyable, SpikeReportReaderPtr>(
    "SpikeReportReader", bp::no_init)
    .def("__init__", bp::make_constructor(_initURI), SPIKEREPORTREADER_CONSTRUCTOR_DOXY)
    .def("__init__", bp::make_constructor(_initURIandGIDSet),
         SPIKEREPORTREADER_CTOR_URI_GIDSET_DOXY)
    .def("close", &SpikeReportReader::close, SPIKEREPORTREADER_CLOSE_DOXY)
    .def("get_spikes", SpikeReportReader_getSpikes,
         (selfarg, bp::arg("start_time"), bp::arg("stop_time")), SPIKEREPORTREADER_GETSPIKES_DOXY)
    .add_property("end_time", &SpikeReportReader::getEndTime, SPIKEREPORTREADER_GETENDTIME_DOXY)
    .add_property("has_ended", &SpikeReportReader::hasEnded, SPIKEREPORTREADER_HASENDED_DOXY);
    // clang-format on
}
}
