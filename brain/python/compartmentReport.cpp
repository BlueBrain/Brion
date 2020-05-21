/* Copyright (c) 2017, EPFL/Blue Brain Project
 *                     Juan Hernando <juan.hernando@epfl.ch>
 *                     Mohamed-Ghaith Kaabi <mohamed.kaabi@epfl.ch>
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

#include "arrayHelpers.h"
#include "helpers.h"
#include "pythonDocumentation.h"

#include <brain/compartmentReport.h>
#include <brain/compartmentReportMapping.h>

#include <brain/types.h>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace brain
{
using namespace brain_python;

typedef boost::shared_ptr<CompartmentReportView> CompartmentReportViewPtr;

// This proxy object is needed because when converting C++ vectors to numpy
// arrays we need a shared_ptr to act as a custodian. As
// CompartmentReportMapping
// is indeed a wrapper than holds a pointer to a CompartmentReportMapping, the
// best solution for the wrapping is to make the proxy be a wrapper of a
// shared_ptr to a view. This way we can use that pointer as the custodian.
class CompartmentReportMappingProxy
{
public:
    CompartmentReportMappingProxy(const CompartmentReportViewPtr& view_)
        : view(view_)
    {
    }
    CompartmentReportMappingProxy(const CompartmentReportMappingProxy& other)
        : view(other.view)
    {
    }

    size_t getNumCompartments(const size_t index) const
    {
        if (index >= view->getMapping().getOffsets().size())
        {
            PyErr_SetString(PyExc_IndexError, "Cell index out of bounds");
            throw bp::error_already_set();
        }
        return view->getMapping().getNumCompartments(index);
    }

    size_t getFrameSize() const { return view->getMapping().getFrameSize(); }
    CompartmentReportViewPtr view;
};

CompartmentReportPtr CompartmentReport_initURI(const std::string& uri)
{
    return std::make_shared<CompartmentReport>(brion::URI(uri));
}

CompartmentReportViewPtr CompartmentReport_createViewEmptyGIDs(
    CompartmentReport& reader)
{
    auto view = reader.createView();
    return CompartmentReportViewPtr(new CompartmentReportView(std::move(view)));
}

CompartmentReportViewPtr CompartmentReport_createView(CompartmentReport& reader,
                                                      bp::object gids)
{
    auto view = reader.createView(gidsFromPython(gids));
    return CompartmentReportViewPtr(new CompartmentReportView(std::move(view)));
}

bp::object CompartmentReport_getMetaData(const CompartmentReport& reader)
{
    const CompartmentReportMetaData& md = reader.getMetaData();

    bp::dict dict;
    dict["start_time"] = md.startTime;
    dict["end_time"] = md.endTime;
    dict["time_step"] = md.timeStep;
    dict["time_unit"] = md.timeUnit;
    dict["data_unit"] = md.dataUnit;
    dict["frame_count"] = md.frameCount;

    return dict;
}

bp::object CompartmentReport_getGids(const CompartmentReport& report)
{
    return toNumpy(toVector(report.getGIDs()));
}

bp::object CompartmentReportView_getGids(const CompartmentReportView& view)
{
    return toNumpy(toVector(view.getGIDs()));
}

CompartmentReportMappingProxy CompartmentReportView_getMapping(
    const CompartmentReportViewPtr& view)
{
    return view;
}

bp::object CompartmentReportView_loadAt(CompartmentReportView& view,
                                        double time)
{
    return frameToTuple(view.load(time).get());
}

bp::object CompartmentReportView_load(CompartmentReportView& view,
                                      const double start, const double end)
{
    return framesToTuple(view.load(start, end).get());
}

bp::object CompartmentReportView_load2(CompartmentReportView& view,
                                       const double start, const double end,
                                       const double stride)
{
    return framesToTuple(view.load(start, end, stride).get());
}

bp::object CompartmentReportView_loadAll(CompartmentReportView& view)
{
    return framesToTuple(view.loadAll().get());
}

bp::object CompartmentReportMapping_getIndex(
    const CompartmentReportMappingProxy& mapping)
{
    static_assert(sizeof(CompartmentReportMapping::IndexEntry) ==
                      sizeof(uint32_t) + sizeof(uint32_t),
                  "Bad alignment of IndexEntry");
    return toNumpy(mapping.view->getMapping().getIndex(), mapping.view);
}

bp::object CompartmentReportMapping_getOffsets(
    const CompartmentReportMappingProxy& mapping)
{
    boost::python::list result;

    auto& offsets = mapping.view->getMapping().getOffsets();
    for (auto& offset : offsets)
        result.append(toNumpy(offset, mapping.view));

    return result;
}

bp::object CompartmentReportMapping_getCompartmentCounts(
    const CompartmentReportMappingProxy& mapping)
{
    boost::python::list result;

    auto& counts = mapping.view->getMapping().getCompartmentCounts();
    for (auto& count : counts)
        result.append(toNumpy(count, mapping.view));

    return result;
}

void export_CompartmentReport()
// clang-format off
{
const auto selfarg = bp::arg("self");

bp::class_<CompartmentReport, boost::noncopyable, CompartmentReportPtr>(
    "CompartmentReport", bp::no_init)
    .def("__init__", bp::make_constructor(CompartmentReport_initURI),
         COMPARTMENTREPORT_CONSTRUCTOR_DOXY)
    .add_property("metadata", CompartmentReport_getMetaData,
                  COMPARTMENTREPORT_GEMETADATA_DOXY)
    .add_property("gids", CompartmentReport_getGids,
                  COMPARTMENTREPORT_GETGIDS_DOXY)
    .add_property("cell_count", &CompartmentReport::getCellCount,
                  COMPARTMENTREPORT_GETCELLCOUNT_DOXY)
    .def("create_view", CompartmentReport_createView,
         (selfarg, bp::arg("gids")), COMPARTMENTREPORT_CREATEVIEW_DOXY)
    .def("create_view", CompartmentReport_createViewEmptyGIDs, (selfarg),
         COMPARTMENTREPORT_CREATEVIEW_EMPTY_DOXY);

bp::class_<CompartmentReportMappingProxy>("CompartmentReportMapping",
                                          bp::no_init)
    .def("num_compartments", &CompartmentReportMappingProxy::getNumCompartments,
         COMPARTMENTREPORTMAPPINGPROXY_GETNUMCOMPARTMENTS_DOXY)
    .add_property("index", CompartmentReportMapping_getIndex,
                  COMPARTMENTREPORTMAPPINGPROXY_GETINDEX_DOXY)
    .add_property("offsets", CompartmentReportMapping_getOffsets,
                  COMPARTMENTREPORTMAPPINGPROXY_GETOFFSETS_DOXY)
    .add_property("frame_size", &CompartmentReportMappingProxy::getFrameSize,
                  COMPARTMENTREPORTMAPPINGPROXY_GETFRAMESIZE_DOXY)
    .def("compartment_counts",
         CompartmentReportMapping_getCompartmentCounts, (selfarg),
         COMPARTMENTREPORTMAPPINGPROXY_GETCOMPARTMENTCOUNTS_DOXY);

bp::class_<CompartmentReportView, CompartmentReportViewPtr, boost::noncopyable>(
    "CompartmentReportView", bp::no_init)
    .add_property("gids", CompartmentReportView_getGids,
                  COMPARTMENTREPORTVIEW_GETGIDS_DOXY)
    .add_property("mapping", CompartmentReportView_getMapping,
                  COMPARTEMENTREPORTVIEW_GETMAPPING_DOXY)
    .def("load", CompartmentReportView_loadAt, (selfarg, bp::arg("time")),
         COMPARTMENTREPORTVIEW_LOAD_DOXY)
    .def("load", CompartmentReportView_load,
         (selfarg, bp::arg("start"), bp::arg("end")),
         COMPARTMENTREPORTVIEW_LOAD_DBLDBL_DOXY)
    .def("load", CompartmentReportView_load2,
         (selfarg, bp::arg("start"), bp::arg("end"), bp::arg("stride")),
         COMPARTMENTREPORTVIEW_LOAD_DBLDBLDBL_DOXY)
    .def("load_all", CompartmentReportView_loadAll, (selfarg),
         COMPARTMENTREPORTVIEW_LOADALL_DOXY);
}
// clang-format on
}
