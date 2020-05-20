/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Juan Hernando <juan.hernando@epfl.ch>
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
#include <boost/python/stl_iterator.hpp>

#include "../arrayHelpers.h"
#include "../helpers.h"
#include "../pythonDocumentation.h"

#include <brain/neuron/morphology.h>
#include <brain/neuron/section.h>
#include <brain/neuron/soma.h>

namespace bp = boost::python;

namespace brain
{
namespace neuron
{
namespace
{
using namespace brain_python;

template <typename Part>
class MorphologyPartWrapper : public Part
{
public:
    MorphologyPartWrapper(const Part& part, const MorphologyPtr& morphology_)
        : Part(part)
        , morphology(morphology_)
    {
    }

    bp::object getChildren()
    {
        const Sections& sections = Part::getChildren();
        bp::list result;
        for (const auto& i : sections)
            result.append(MorphologyPartWrapper<Section>(i, morphology));
        return result;
    }

    MorphologyPtr morphology;
};

typedef MorphologyPartWrapper<Soma> SomaWrapper;
typedef MorphologyPartWrapper<Section> SectionWrapper;

bp::object Soma_getProfilePoints(const SomaWrapper& soma)
{
    return toNumpy(soma.getProfilePoints());
}

glm::vec4 Section_getIndex(const SectionWrapper& section, int index)
{
    if (index >= int(section.getNumSamples()) ||
        index <= -int(section.getNumSamples()))
    {
        PyErr_SetString(PyExc_IndexError, "Sample index out of range");
        bp::throw_error_already_set();
    }
    return section[index];
}

#define GET_SECTION_ARRAY(Array)                                 \
    bp::object Section_get##Array(const SectionWrapper& section) \
    {                                                            \
        return toNumpy(section.get##Array());                    \
    }

GET_SECTION_ARRAY(Samples)
GET_SECTION_ARRAY(SampleDistancesToSoma)

bp::object Section_getSamplesFromPositions(const SectionWrapper& section,
                                           bp::object points)
{
    const floats pointVector =
        vectorFromIterable<float>(points,
                                  "Cannot convert argument to float vector");
    return toNumpy(section.getSamples(pointVector));
}

bp::object Section_getParent(const SectionWrapper& section)
{
    if (section.hasParent())
        return bp::object(
            SectionWrapper(section.getParent(), section.morphology));
    return bp::object();
}

MorphologyPtr Morphology_initFromURI(const std::string& uri)
{
    return MorphologyPtr(new Morphology(URI(uri)));
}

MorphologyPtr Morphology_initFromURIAndTransform(const std::string& uri,
                                                 bp::object transform)
{
    return MorphologyPtr(
        new Morphology(URI(uri), fromNumpy<glm::mat4>(transform)));
}

#define GET_MORPHOLOGY_ARRAY(Array)                                   \
    bp::object Morphology_get##Array(const MorphologyPtr& morphology) \
    {                                                                 \
        return toNumpy(morphology->get##Array(), morphology);         \
    }

GET_MORPHOLOGY_ARRAY(Points)
GET_MORPHOLOGY_ARRAY(Sections)

bp::object Morphology_getSectionIDs(const MorphologyPtr& morphology,
                                    bp::object types)
{
    const SectionTypes typeVector = vectorFromIterable<SectionType>(
        types, "Cannot convert argument to SectionType list");
    return toNumpy(morphology->getSectionIDs(typeVector));
}

bp::object Morphology_getSectionTypes(const MorphologyPtr& morphology)
{
    return toNumpy(morphology->getSectionTypes(), morphology);
}

bp::object Morphology_getSectionsByType(const MorphologyPtr& morphology,
                                        bp::object types)
{
    SectionTypes typeVector;
    try
    {
        bp::extract<SectionType> extractor(types);
        if (extractor.check())
        {
            typeVector.push_back((SectionType)extractor);
        }
        else
        {
            bp::stl_input_iterator<SectionType> i(types), end;
            for (; i != end; ++i)
                typeVector.push_back(*i);
        }
    }
    catch (...)
    {
        PyErr_SetString(PyExc_ValueError,
                        "Cannot convert argument to SectionType list");
        bp::throw_error_already_set();
    }
    const Sections& sections = morphology->getSections(typeVector);
    bp::list result;
    for (const auto& section : sections)
        result.append(SectionWrapper(section, morphology));
    return result;
}

SectionWrapper Morphology_getSection(const MorphologyPtr& morphology,
                                     const uint32_t id)
{
    return SectionWrapper(morphology->getSection(id), morphology);
}

SomaWrapper Morphology_getSoma(const MorphologyPtr& morphology)
{
    return SomaWrapper(morphology->getSoma(), morphology);
}

bp::object Morphology_getTransformation(const MorphologyPtr& morphology)
{
    return toNumpy(morphology->getTransformation());
}
}
// clang-format off
void export_Morphology()
{

const auto selfarg = bp::arg("self");

bp::class_<SomaWrapper>(
    "Soma", SOMA_CLASS_DOXY, bp::no_init)
    .def("profile_points", Soma_getProfilePoints, (selfarg), SOMA_GETPROFILEPOINTS_DOXY)
    .def("mean_radius", &Soma::getMeanRadius, (selfarg), SOMA_GETMEANRADIUS_DOXY)
    .def("max_radius", &Soma::getMaxRadius, (selfarg), SOMA_GETMAXRADIUS_DOXY)
    .def("centroid", &Soma::getCentroid, (selfarg), SOMA_GETCENTROID_DOXY)
    .def("children", &SomaWrapper::getChildren, (selfarg), SOMA_GETCHILDREN_DOXY);

bp::class_<SectionWrapper>(
    "Section", SECTION_CLASS_DOXY, bp::no_init)
    .def(bp::self == bp::self)
    .def("id", &Section::getID, (selfarg), SECTION_GETID_DOXY)
    .def("__getitem__", Section_getIndex)
    .def("type", &Section::getType, (selfarg), SECTION_GETTYPE_DOXY)
    .def("length", &Section::getLength, (selfarg), SECTION_GETLENGTH_DOXY)
    .def("samples", Section_getSamples, (selfarg), SECTION_GETSAMPLES_DOXY)
    .def("samples", Section_getSamplesFromPositions,
         (selfarg, bp::arg("positions")), SECTION_GETSAMPLES_FLT_DOXY)
    .def("num_samples", &Section::getNumSamples, (selfarg), SECTION_GETNUMSAMPLES_DOXY)
    .def("distance_to_soma", &Section::getDistanceToSoma, (selfarg),
         SECTION_GETDISTANCETOSOMA_DOXY)
    .def("sample_distances_to_soma", Section_getSampleDistancesToSoma, (selfarg),
         SECTION_GETSAMPLEDISTANCESTOSOMA_DOXY)
    .def("parent", Section_getParent, (selfarg), SECTION_GETPARENT_DOXY)
    .def("children", &SectionWrapper::getChildren, (selfarg), SECTION_GETCHILDREN_DOXY);

bp::class_<Morphology, boost::noncopyable, MorphologyPtr>(
    "Morphology", MORPHOLOGY_CLASS_DOXY, bp::no_init)
    .def("__init__", bp::make_constructor(Morphology_initFromURI), MORPHOLOGY_CONSTRUCTOR_DOXY)
    .def("__init__",
         bp::make_constructor(Morphology_initFromURIAndTransform), MORPHOLOGY_CTOR_MATRIX_URI_DOXY)

    .def("points", Morphology_getPoints, (selfarg), MORPHOLOGY_GETPOINTS_DOXY)
    .def("sections", Morphology_getSections, (selfarg), MORPHOLOGY_GETSECTIONS_DOXY)
    .def("section_types", Morphology_getSectionTypes, (selfarg), MORPHOLOGY_GETSECTIONTYPES_DOXY)
    .def("section_ids", Morphology_getSectionIDs, (selfarg, bp::arg("types")),
         MORPHOLOGY_GETSECTIONIDS_DOXY)
    .def("sections", Morphology_getSectionsByType, (selfarg, bp::arg("type")),
         MORPHOLOGY_GETSECTIONS_BYTYPE_DOXY)
    .def("section", Morphology_getSection, (selfarg, bp::arg("id")),
         MORPHOLOGY_GETSECTIONS_BYTYPE_NC_DOXY)
    .def("soma", Morphology_getSoma, (selfarg), MORPHOLOGY_GETSOMA_DOXY)
    .def("transformation", Morphology_getTransformation, (selfarg),
         MORPHOLOGY_GETTRANSFORMATION_DOXY);
}
// clang-format on
}
}
