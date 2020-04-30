/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
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

#ifndef BRAIN_TYPES
#define BRAIN_TYPES

#include <brion/types.h>

/** @namespace brain Algorithmic interface to Blue Brain data model */
namespace brain
{
/**
 * Loading of data during SynapsesStream::read(), otherwise load happens
 * on-demand.
 */
enum class SynapsePrefetch
{
    none = 0,                    //!< only loads pre- and post GIDs
    attributes = 1 << 0,         //!< topological information (section,
                                 //!  segment, distance) and model
                                 //!  attributes
    positions = 1 << 1,          //!< pre/post surface/center positions
    all = attributes | positions //!< all synapse data
};

class Circuit;
class CompartmentReport;
class CompartmentReportFrame;
class CompartmentReportMapping;
class CompartmentReportView;
class Simulation;
class SpikeReportReader;
class SpikeReportWriter;
class CompartmentReport;
class Synapse;
class Synapses;
class SynapsesIterator;
class SynapsesStream;

using brion::GIDSet;
using brion::Strings;
using brion::URI;
using brion::URIs;
using brion::Vector2is;
using brion::Vector3fs;
using brion::Vector4fs;
using brion::floats;
using brion::uint32_ts;
using brion::size_ts;

using brion::SectionOffsets;
using brion::Spike;
using brion::Spikes;
using brion::CompartmentCounts;

using CompartmentReportFrames = std::vector<CompartmentReportFrame>;
using Matrix4fs = std::vector<glm::mat4>;
using Quaternionfs = std::vector<glm::quat>;

using SpikeReportReaderPtr = std::shared_ptr<SpikeReportReader>;
using SpikeReportWriterPtr = std::shared_ptr<SpikeReportWriter>;

using CompartmentReportPtr = std::shared_ptr<CompartmentReport>;

/**
 * The GID of a synapse is the a tuple of two numbers:
 * - The GID of the post-synaptic cell.
 * - The index of the synapse in the array of afferent contacts
 *   of the post-synaptic cell before pruning/filtering.
 * GIDs are invariant regardless of how the structural touches are
 * converted into functional synapses during circuit building.
 */
using SynapseGID = std::pair<uint32_t, size_t>;

namespace detail
{
struct SynapsesStream;
}
}
#endif
