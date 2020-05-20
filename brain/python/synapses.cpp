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

#include "arrayHelpers.h"
#include "pythonDocumentation.h"
#include "types.h"

#include <brain/synapse.h>
#include <brain/synapses.h>

namespace bp = boost::python;

namespace brain
{
namespace
{
using namespace brain_python;

bool nonzero(const Synapses&)
{
    return true;
}

bp::object Synapse_getGID(const Synapse& synapse)
{
    auto gid = synapse.getGID();
    return bp::make_tuple(gid.first, gid.second);
}

Synapse Synapses_get(const Synapses& synapses, long int index)
{
    if (index < 0)
        index = synapses.size() + index;
    if (index < 0 || size_t(index) >= synapses.size())
    {
        PyErr_SetString(PyExc_IndexError, "Index out of bounds");
        bp::throw_error_already_set();
    }
    return synapses[index];
}

#define GET_SYNAPSES_ARRAY_PROPERTY(type, name)                           \
    bp::object Synapses_##name(const Synapses& synapses)                  \
    {                                                                     \
        if (!synapses.name())                                             \
            return bp::object();                                          \
        return toNumpy(synapses.name(), synapses.size(), synapses._impl); \
    }

bp::object Synapses_indices(const Synapses& synapses)
{
    try
    {
        return toNumpy(synapses.indices(), synapses.size(), synapses._impl);
    }
    catch (...)
    {
        return bp::object();
    }
}

GET_SYNAPSES_ARRAY_PROPERTY(uin32_t, preGIDs)
GET_SYNAPSES_ARRAY_PROPERTY(uin32_t, preSectionIDs)
GET_SYNAPSES_ARRAY_PROPERTY(uin32_t, preSegmentIDs)
GET_SYNAPSES_ARRAY_PROPERTY(float, preDistances)
GET_SYNAPSES_ARRAY_PROPERTY(float, preSurfaceXPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, preSurfaceYPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, preSurfaceZPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, preCenterXPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, preCenterYPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, preCenterZPositions)
GET_SYNAPSES_ARRAY_PROPERTY(uint32_t, postGIDs)
GET_SYNAPSES_ARRAY_PROPERTY(uint32_t, postSectionIDs)
GET_SYNAPSES_ARRAY_PROPERTY(uint32_t, postSegmentIDs)
GET_SYNAPSES_ARRAY_PROPERTY(float, postDistances)
GET_SYNAPSES_ARRAY_PROPERTY(float, postSurfaceXPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, postSurfaceYPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, postSurfaceZPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, postCenterXPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, postCenterYPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, postCenterZPositions)
GET_SYNAPSES_ARRAY_PROPERTY(float, delays)
GET_SYNAPSES_ARRAY_PROPERTY(float, conductances)
GET_SYNAPSES_ARRAY_PROPERTY(float, utilizations)
GET_SYNAPSES_ARRAY_PROPERTY(float, depressions)
GET_SYNAPSES_ARRAY_PROPERTY(float, facilitations)
GET_SYNAPSES_ARRAY_PROPERTY(float, decays)
GET_SYNAPSES_ARRAY_PROPERTY(int, efficacies)
}

// clang-format off
void export_Synapses()
{
const auto selfarg = bp::arg( "self" );

bp::class_< Synapse >( "Synapse", SYNAPSE_CLASS_DOXY, bp::no_init )
    .def( "gid", Synapse_getGID, ( selfarg ), SYNAPSE_GETGID_DOXY)
    .def( "pre_gid", &Synapse::getPresynapticGID, ( selfarg ), SYNAPSE_GETPRESYNAPTICGID_DOXY)
    .def( "pre_section", &Synapse::getPresynapticSectionID, ( selfarg ),
          SYNAPSE_GETPRESYNAPTICSECIONID_DOXY)
    .def( "pre_segment", &Synapse::getPresynapticSegmentID, ( selfarg ),
          SYNAPSE_GETPRESYNAPTICSEGMENTID_DOXY)
    .def( "pre_distance", &Synapse::getPresynapticDistance, ( selfarg ),
          SYNAPSE_GETPRESYNAPTICDISTANCE_DOXY)
    .def( "pre_center_position", &Synapse::getPresynapticCenterPosition,
          ( selfarg ), SYNAPSE_GETPRESYNAPTICCENTERPOSITION_DOXY)
    .def( "pre_surface_position", &Synapse::getPresynapticSurfacePosition,
          ( selfarg ), SYNAPSE_GETPRESYNAPTICSURFACEPOSITION_DOXY)
    .def( "post_gid", &Synapse::getPostsynapticGID, ( selfarg ), SYNAPSE_GETPOSTSYNAPTICGID_DOXY)
    .def( "post_section", &Synapse::getPostsynapticSectionID, ( selfarg ),
          SYNAPSE_GETPOSTSYNAPTICSECTIONID_DOXY)
    .def( "post_segment", &Synapse::getPostsynapticSegmentID, ( selfarg ),
          SYNAPSE_GETPOSTSYNAPTICSEGMENTID_DOXY)
    .def( "post_distance", &Synapse::getPostsynapticDistance, ( selfarg ),
          SYNAPSE_GETPOSTSYNAPTICDISTANCE_DOXY)
    .def( "post_center_position", &Synapse::getPostsynapticCenterPosition,
          ( selfarg ), SYNAPSE_GETPOSTSYNAPTICCENTERPOSITION_DOXY)
    .def( "post_surface_position", &Synapse::getPostsynapticSurfacePosition,
          ( selfarg ), SYNAPSE_GETPOSSYNAPTICSURFACEPOSITION_DOXY)
    .def( "delay", &Synapse::getDelay, ( selfarg ), SYNAPSE_GETDELAY_DOXY)
    .def( "conductance", &Synapse::getConductance, ( selfarg ), SYNAPSE_GETCONDUCTANCE_DOXY)
    .def( "utilization", &Synapse::getUtilization, ( selfarg ), SYNAPSE_GETUTILIZATION_DOXY)
    .def( "depression", &Synapse::getDepression, ( selfarg ), SYNAPSE_GETDEPRESSION_DOXY)
    .def( "facilitation", &Synapse::getFacilitation, ( selfarg ), SYNAPSE_GETFACILITATION_DOXY)
    .def( "decay", &Synapse::getDecay, ( selfarg ), SYNAPSE_GETDECAY_DOXY)
    .def( "efficacy", &Synapse::getEfficacy, ( selfarg ), SYNAPSE_GETEFFICACY_DOXY);

bp::class_< Synapses >( "Synapses", SYNAPSES_CLASS_DOXY, bp::no_init )
    .def( "__nonzero__", nonzero )
    .def( "__len__", &Synapses::size )
    .def( "__getitem__", Synapses_get )
    // There is no need to wrap the iterator, Python provides one out of the
    // box thanks to __len__ and __getitem__
    .def( "empty", &Synapses::empty, ( selfarg ), SYNAPSES_EMPTY_DOXY)
    .def( "indices", Synapses_indices, ( selfarg ), SYNAPSES_INDICES_DOXY)
    .def( "pre_gids", Synapses_preGIDs, ( selfarg ), SYNAPSES_PREGIDS_DOXY)
    .def( "pre_section_ids", Synapses_preSectionIDs, ( selfarg ), SYNAPSES_PRESECTIONIDS_DOXY)
    .def( "pre_segment_ids", Synapses_preSegmentIDs, ( selfarg ), SYNAPSES_PRESEGMENTIDS_DOXY)
    .def( "pre_distances", Synapses_preDistances, ( selfarg ), SYNAPSES_PREDISTANCES_DOXY)
    .def( "pre_surface_x_positions", Synapses_preSurfaceXPositions,
          ( selfarg ), SYNAPSES_PRESURFACEXPOSITIONS_DOXY)
    .def( "pre_surface_y_positions", Synapses_preSurfaceYPositions,
          ( selfarg ), SYNAPSES_PRESURFACEYPOSITIONS_DOXY)
    .def( "pre_surface_z_positions", Synapses_preSurfaceZPositions,
          ( selfarg ), SYNAPSES_PRESURFACEZPOSITIONS_DOXY)
    .def( "pre_center_x_positions", Synapses_preCenterXPositions, ( selfarg ),
          SYNAPSES_PRECENTERXPOSITIONS_DOXY)
    .def( "pre_center_y_positions", Synapses_preCenterYPositions, ( selfarg ),
          SYNAPSES_PRECENTERYPOSITIONS_DOXY)
    .def( "pre_center_z_positions", Synapses_preCenterZPositions, ( selfarg ),
          SYNAPSES_PRECENTERZPOSITIONS_DOXY)
    .def( "post_gids", Synapses_postGIDs, ( selfarg ), SYNAPSES_POSTGIDS_DOXY)
    .def( "post_section_ids", Synapses_postSectionIDs, ( selfarg ), SYNAPSES_POSTSECTIONIDS_DOXY)
    .def( "post_segment_ids", Synapses_postSegmentIDs, ( selfarg ), SYNAPSES_POSTSEGMENTIDS_DOXY)
    .def( "post_distances", Synapses_postDistances, ( selfarg ), SYNAPSES_POSTDISTANCES_DOXY)
    .def( "post_surface_x_positions", Synapses_postSurfaceXPositions,
          ( selfarg ), SYNAPSES_POSTSURFACEXPOSITIONS_DOXY)
    .def( "post_surface_y_positions", Synapses_postSurfaceYPositions,
          ( selfarg ), SYNAPSES_POSTSURFACEYPOSITIONS_DOXY)
    .def( "post_surface_z_positions", Synapses_postSurfaceZPositions,
          ( selfarg ), SYNAPSES_POSTSURFACEZPOSITIONS_DOXY)
    .def( "post_center_x_positions", Synapses_postCenterXPositions, ( selfarg ),
          SYNAPSES_POSTCENTERXPOSITIONS_DOXY)
    .def( "post_center_y_positions", Synapses_postCenterYPositions, ( selfarg ),
          SYNAPSES_POSTCENTERYPOSITIONS_DOXY)
    .def( "post_center_z_positions", Synapses_postCenterZPositions, ( selfarg ),
          SYNAPSES_POSTCENTERZPOSITIONS_DOXY)
    .def( "delays", Synapses_delays, ( selfarg ), SYNAPSES_DELAYS_DOXY)
    .def( "conductances", Synapses_conductances, ( selfarg ), SYNAPSES_CONDUCTANCES_DOXY)
    .def( "utilizations", Synapses_utilizations, ( selfarg ), SYNAPSES_UTILIZATIONS_DOXY)
    .def( "depressions", Synapses_depressions, ( selfarg ), SYNAPSES_DEPRESSIONS_DOXY)
    .def( "facilitations", Synapses_facilitations, ( selfarg ), SYNAPSES_FACILITATIONS_DOXY)
    .def( "decays", Synapses_decays, ( selfarg ), SYNAPSES_DECAYS_DOXY)
    .def( "efficacies", Synapses_efficacies, ( selfarg ), SYNAPSES_EFFICACIES_DOXY);
}
// clang-format on
}
