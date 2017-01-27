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
#include "docstrings.h"
#include "helpers.h"

#include <brain/compartmentReportReader.h>
#include <brain/types.h>

#include <boost/python.hpp>

namespace bp = boost::python;

namespace brain
{
typedef boost::shared_ptr<CompartmentReportView> CompartmentReportViewPtr;
typedef boost::shared_ptr<CompartmentReportFrame> CompartmentReportFramePtr;

// This proxy object is needed because when converting C++ vectors to numpy
// arrays we need a shared_ptr to act as a custodian. As CompartmentReportMapping
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
        return view->getMapping().getNumCompartments(index);
    }

    CompartmentReportViewPtr view;
};

CompartmentReportReaderPtr CompartmentReportReader_initURI(
    const std::string& uri)
{
    return std::make_shared<CompartmentReportReader>(brion::URI(uri));
}

CompartmentReportViewPtr CompartmentReportReader_createView(
    CompartmentReportReader& reader, bp::object gids)
{
    auto view = reader.createView(gidsFromPython(gids));
    return CompartmentReportViewPtr(new CompartmentReportView(std::move(view)));
}

bp::object CompartmentReportReader_getMetaData(
    const CompartmentReportReader& reader)
{
    const CompartmentReportMetaData& md = reader.getMetaData();

    bp::object dict{bp::handle<>(PyDict_New())};

    PyDict_SetItemString(dict.ptr(), "start_time",
                         PyFloat_FromDouble(md.startTime));
    PyDict_SetItemString(dict.ptr(), "end_time",
                         PyFloat_FromDouble(md.endTime));
    PyDict_SetItemString(dict.ptr(), "time_step",
                         PyFloat_FromDouble(md.timeStep));

    PyDict_SetItemString(dict.ptr(), "time_unit",
                         PyUnicode_FromString(md.timeUnit.c_str()));
    PyDict_SetItemString(dict.ptr(), "data_unit",
                         PyUnicode_FromString(md.dataUnit.c_str()));

    return dict;
}

bp::object CompartmentReportView_getGids(const CompartmentReportView& view)
{
    return toPythonSet(view.getGIDs());
}

CompartmentReportMappingProxy CompartmentReportView_getMapping(
    const CompartmentReportViewPtr& view)
{
    return view;
}

CompartmentReportFramePtr CompartmentReportView_loadAt(
    CompartmentReportView& view, float time)
{
    return CompartmentReportFramePtr(
        new CompartmentReportFrame(view.load(time).get()));
}

bp::object CompartmentReportView_load(CompartmentReportView& view,
                                      const float start, const float end)
{
    auto frames = view.load(start, end).get();
    boost::python::list result;
    for (auto& frame : frames)
        result.append(CompartmentReportFramePtr(
            new CompartmentReportFrame(std::move(frame))));
    return result;
}

bp::object CompartmentReportView_loadAll(CompartmentReportView& view)
{
    auto frames = view.loadAll().get();
    boost::python::list result;
    for (auto& frame : frames)
        result.append(CompartmentReportFramePtr(
            new CompartmentReportFrame(std::move(frame))));
    return result;
}

bp::object CompartmentReportFrame_getData(
    const CompartmentReportFramePtr& frame)
{
    return toNumpy(frame->getData(), frame);
}

bp::object CompartmentReportMapping_getIndex(
    const CompartmentReportMappingProxy& mapping)
{
    static_assert(sizeof(CompartmentReportMapping::IndexEntry) ==
                  sizeof(uint64_t) + sizeof(uint32_t) +
                  sizeof(uint16_t) + sizeof(uint16_t),
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

void export_CompartmentReportReader()
{
// clang-format off
const auto selfarg = bp::arg("self");

bp::class_<CompartmentReportReader, boost::noncopyable>(
    "CompartmentReportReader", bp::no_init)
    .def("__init__", bp::make_constructor(CompartmentReportReader_initURI),
         DOXY_FN(brain::CompartmentReportReader::CompartmentReportReader))
    .add_property("metadata", CompartmentReportReader_getMetaData,
                  DOXY_FN(brain::CompartmentReportReader::getMetaData))
    .def("create_view", CompartmentReportReader_createView,
         (selfarg, bp::arg("gids")),
         DOXY_FN(brain::CompartmentReportReader::createView(const GIDSet&)))
    .def("create_view",
         (CompartmentReportView(CompartmentReportReader::*)())
         &CompartmentReportReader::createView,
         DOXY_FN(brain::CompartmentReportReader::createView()));

bp::class_<CompartmentReportMappingProxy>("CompartmentReportMapping",
                                          bp::no_init)
    .def("num_compartments", &CompartmentReportMappingProxy::getNumCompartments,
         DOXY_FN(brain::CompartmentReportMapping::getNumCompartments))
    .add_property("index", CompartmentReportMapping_getIndex,
                  DOXY_FN(brain::CompartmentReportMapping::getIndex))
    .add_property("offsets", CompartmentReportMapping_getOffsets,
                  DOXY_FN(brain::CompartmentReportMapping::getOffsets))
    .def("compartment_counts",
         CompartmentReportMapping_getCompartmentCounts, (selfarg),
         DOXY_FN(brain::CompartmentReportMapping::getCompartmentCounts));

bp::class_<CompartmentReportView, CompartmentReportViewPtr, boost::noncopyable>(
    "CompartmentReportView", bp::no_init)
    .add_property("gids", CompartmentReportView_getGids,
                  DOXY_FN(brain::CompartmentReportView::getGIDs))
    .add_property("mapping", CompartmentReportView_getMapping,
                  DOXY_FN(brain::CompartmentReportView::getMapping))
    .def("load", &CompartmentReportView_loadAt, (selfarg, bp::arg("time")),
         DOXY_FN(brain::CompartmentReportView::load(float)))
    .def("load", &CompartmentReportView_load,
         (selfarg, bp::arg("start"), bp::arg("end")),
         DOXY_FN(brain::CompartmentReportView::load(float,float)))
    .def("load_all", &CompartmentReportView_loadAll, (selfarg),
         DOXY_FN(brain::CompartmentReportView::loadAll));

bp::class_<CompartmentReportFrame, CompartmentReportFramePtr,
           boost::noncopyable>(
    "CompartmentReportFrame", bp::no_init)
    .def_readonly("empty", &CompartmentReportFrame::empty,
                  DOXY_FN(brain::CompartmentReportFrame::empty))
    .def_readonly("timestamp", &CompartmentReportFrame::getTimestamp,
                  DOXY_FN(brain::CompartmentReportFrame::getTimestamp))
    .add_property("data", CompartmentReportFrame_getData,
                  DOXY_FN(brain::CompartmentReportFrame::getData));
// clang-format on
}
}
