/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Daniel.Nachbaur@epfl.ch
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
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "synapses.h"

#include "circuit.h"
#include "log.h"
#include "synapse.h"
#include "synapsesIterator.h"
#include "synapsesStream.h"

#include "detail/circuit.h"
#include "detail/synapsesStream.h"

#include <brion/synapse.h>
#include <brion/synapseSummary.h>

#include <bbp/sonata/edges.h>

#include <mutex>

namespace
{
class EdgePopulation
{
public:
    static auto open(const std::string& file, const std::string& population)
    {
        auto name = _extractName(file, population);
        return bbp::sonata::EdgePopulation(file, "", name);
    }

private:
    static std::string _extractName(const std::string& file, const std::string& population)
    {
        if (!population.empty())
        {
            return population;
        }

        auto storage = bbp::sonata::EdgeStorage(file);
        auto populations = storage.populationNames();

        if (populations.empty())
        {
            throw std::invalid_argument("No edge populations found in " + file);
        }

        return *populations.begin();
    }
};

class GidConverter
{
public:
    static std::vector<uint64_t> toSonata(const brion::GIDSet& gids)
    {
        if (gids.find(0) != gids.end())
        {
            throw std::invalid_argument("0-based GID on BlueConfig circuit");
        }

        auto result = std::vector<uint64_t>();
        result.reserve(gids.size());
        for (auto gid : gids)
        {
            result.push_back(gid - 1);
        }
        return result;
    }

    static std::vector<uint32_t> toBlueConfig(const std::vector<uint64_t>& nodeIds)
    {
        auto result = std::vector<uint32_t>();
        result.reserve(nodeIds.size());
        for (auto nodeId : nodeIds)
        {
            result.push_back(static_cast<uint32_t>(nodeId + 1));
        }
        return result;
    }
};
} // namespace

namespace brain
{

struct Synapses::InternalBaseImpl : public Synapses::BaseImpl
{
    InternalBaseImpl(const Circuit& circuit, const GIDSet& gids, bool afferent, SynapsePrefetch prefetch)
        : _circuit(circuit._impl)
        , _gids(prefetch != SynapsePrefetch::all ? gids : GIDSet())
        , _afferent(afferent)
        , _size(0)
    {
    }

    InternalBaseImpl(const Circuit& circuit, const GIDSet& gids, const std::string& source, SynapsePrefetch prefetch)
        : _circuit(circuit._impl)
        , _gids(prefetch != SynapsePrefetch::all ? gids : GIDSet())
        , _afferent(true)
        , _externalSource(source)
        , _size(0)
    {
    }

    void _allocateAttributes(const size_t size, const bool allocateIndex)
    {
        if (allocateIndex)
        {
            _index.resize(size);
        }

        _preSectionID.resize(size);
        _preSegmentID.resize(size);
        _preDistance.resize(size);

        _postSectionID.resize(size);
        _postSegmentID.resize(size);
        _postDistance.resize(size);

        _delay.resize(size);
        _conductance.resize(size);
        _utilization.resize(size);
        _depression.resize(size);
        _facilitation.resize(size);
        _decay.resize(size);
        _efficacy.resize(size);
    }

    void _allocatePositions(const size_t size)
    {
        _preSurfacePositionX.resize(size);
        _preSurfacePositionY.resize(size);
        _preSurfacePositionZ.resize(size);
        _preCenterPositionX.resize(size);
        _preCenterPositionY.resize(size);
        _preCenterPositionZ.resize(size);

        _postSurfacePositionX.resize(size);
        _postSurfacePositionY.resize(size);
        _postSurfacePositionZ.resize(size);
        _postCenterPositionX.resize(size);
        _postCenterPositionY.resize(size);
        _postCenterPositionZ.resize(size);
    }

    const std::shared_ptr<const Circuit::Impl> _circuit;
    GIDSet _gids;
    bool _afferent;
    std::string _externalSource;

    size_t _size;

    std::vector<size_t> _index;

    std::vector<uint32_t> _preGID;
    std::vector<uint32_t> _preSectionID;
    std::vector<uint32_t> _preSegmentID;
    std::vector<float> _preDistance;
    std::vector<float> _preSurfacePositionX;
    std::vector<float> _preSurfacePositionY;
    std::vector<float> _preSurfacePositionZ;
    std::vector<float> _preCenterPositionX;
    std::vector<float> _preCenterPositionY;
    std::vector<float> _preCenterPositionZ;

    std::vector<uint32_t> _postGID;
    std::vector<uint32_t> _postSectionID;
    std::vector<uint32_t> _postSegmentID;
    std::vector<float> _postDistance;
    std::vector<float> _postSurfacePositionX;
    std::vector<float> _postSurfacePositionY;
    std::vector<float> _postSurfacePositionZ;
    std::vector<float> _postCenterPositionX;
    std::vector<float> _postCenterPositionY;
    std::vector<float> _postCenterPositionZ;

    std::vector<float> _delay;
    std::vector<float> _conductance;
    std::vector<float> _utilization;
    std::vector<float> _depression;
    std::vector<float> _facilitation;
    std::vector<float> _decay;
    std::vector<int> _efficacy;

    std::once_flag _attributeFlag;
    std::once_flag _positionFlag;
    std::once_flag _indexFlag;
    // Besides the call_once flags, we still need to ensure exclusive access
    // to state.
    std::mutex _mutex;
};

struct Synapses::Impl : public Synapses::InternalBaseImpl
{
    Impl(const Circuit& circuit, const GIDSet& gids, const bool afferent, const SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, afferent, prefetch)
    {
        _loadConnectivity();

        if (int(prefetch) & int(SynapsePrefetch::attributes))
        {
            std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
        }
        if (int(prefetch) & int(SynapsePrefetch::positions))
        {
            std::call_once(_positionFlag, [&]() { _loadPositions(); });
        }
    }

    Impl(const Circuit& circuit, const GIDSet& gids, const std::string& source, const SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, source, prefetch)
    {
        // We don't have a summary file for projected afferent synapses.
        // But at least we have to figure out the size of the container.
        const auto& synapses = _circuit->getAfferentProjectionAttributes(source);
        _size = synapses.getNumSynapses(gids);

        if (int(prefetch) & int(SynapsePrefetch::attributes))
        {
            std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
        }
    }

    void _loadConnectivity()
    {
        const brion::SynapseSummary& synapseSummary = _circuit->getSynapseSummary();

        for (const auto gid : _gids)
        {
            const auto& summary = synapseSummary.read(gid);

            for (size_t i = 0; i < summary.shape()[0]; ++i)
            {
                const uint32_t peerGid = summary[i][0];

                for (size_t j = 0; j < summary[i][_afferent ? 2 : 1]; ++j)
                {
                    _preGID.push_back(peerGid);
                    _postGID.push_back(gid);
                }
            }
        }

        _size = _preGID.size();

        if (!_afferent)
        {
            _preGID.swap(_postGID);
        }
    }

    void _loadAttributes()
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (!_efficacy.empty())
        {
            return;
        }

        const auto& synapseAttributes = _externalSource.empty()
                                            ? _circuit->getSynapseAttributes(_afferent)
                                            : _circuit->getAfferentProjectionAttributes(_externalSource);
        const auto synapseExtra = _externalSource.empty() ? _circuit->getSynapseExtra() : nullptr;

        // For external afferent projections we haven't had the chance to
        // get the connectivity.
        const bool haveGIDs = _externalSource.empty();
        if (!haveGIDs)
        {
            _preGID.resize(_size);
            _postGID.resize(_size);
        }

        const bool haveExtra = _afferent && synapseExtra;
        _allocateAttributes(_size, haveExtra || _afferent);

        size_t i = 0;
        for (auto gid : _gids)
        {
            auto&& attr = synapseAttributes.read(gid, brion::SYNAPSE_ALL_ATTRIBUTES);
            auto&& extra = haveExtra ? synapseExtra->read(gid, 1) : brion::SynapseMatrix();
            for (size_t j = 0; j < attr.shape()[0]; ++j)
            {
                const uint32_t connected = attr[j][0];

                if (!haveGIDs)
                {
                    // This code path is exclusive of external projections,
                    // which are afferent only
                    assert(_afferent);
                    _preGID[i] = connected;
                    _postGID[i] = gid;
                }
                assert(_preGID[i] == _afferent ? connected : gid);
                assert(_postGID[i] == _afferent ? gid : connected);
                assert(i < _size);

                if (haveExtra)
                    _index[i] = extra[j][0];
                else if (_afferent)
                    // Fallback assigment of synapses index for afferent views
                    _index[i] = j;

                _delay[i] = attr[j][1];
                _postSectionID[i] = attr[j][2];
                _postSegmentID[i] = attr[j][3];
                _postDistance[i] = attr[j][4];
                _preSectionID[i] = attr[j][5];
                _preSegmentID[i] = attr[j][6];
                _preDistance[i] = attr[j][7];
                _conductance[i] = attr[j][8];
                _utilization[i] = attr[j][9];
                _depression[i] = attr[j][10];
                _facilitation[i] = attr[j][11];
                _decay[i] = attr[j][12];
                _efficacy[i] = attr[j][17];
                ++i;
            }
        }
    }

    void _loadPositions()
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (!_externalSource.empty())
        {
            BRAIN_THROW("Synapse positions not available for projections")
        }

        if (!_preCenterPositionX.empty())
        {
            return;
        }

        _allocatePositions(_size);

        Strings keys;
        CachedSynapses loaded;

        // delay the opening of the synapse file as much as possible, even
        // though the code looks ugly... As the circuit impl keeps the file
        // opened, we can safely just get a loose pointer here.
        const brion::Synapse* positions = nullptr;

        size_t i = 0;
        for (const auto gid : _gids)
        {
            const auto readFromFile = [&]
            {
                if (!positions)
                {
                    positions = &_circuit->getSynapsePositions(_afferent);
                }

                if (positions->getNumAttributes() == brion::SYNAPSE_POSITION_ALL)
                    return positions->read(gid, brion::SYNAPSE_POSITION);

                return positions->read(gid, brion::SYNAPSE_OLD_POSITION);
            };

            const brion::SynapseMatrix pos = readFromFile();

            for (size_t j = 0; j < pos.size(); ++j)
            {
                size_t offset = 0;
                if (pos.shape()[1] == brion::SYNAPSE_POSITION_ALL)
                {
                    _preSurfacePositionX[i] = pos[j][1];
                    _preSurfacePositionY[i] = pos[j][2];
                    _preSurfacePositionZ[i] = pos[j][3];
                    _postSurfacePositionX[i] = pos[j][4];
                    _postSurfacePositionY[i] = pos[j][5];
                    _postSurfacePositionZ[i] = pos[j][6];
                    offset = 6;
                }

                _preCenterPositionX[i] = pos[j][1 + offset];
                _preCenterPositionY[i] = pos[j][2 + offset];
                _preCenterPositionZ[i] = pos[j][3 + offset];
                _postCenterPositionX[i] = pos[j][4 + offset];
                _postCenterPositionY[i] = pos[j][5 + offset];
                _postCenterPositionZ[i] = pos[j][6 + offset];

                ++i;
            }
        }
    }

    void _ensureGIDs() override
    {
        if (_externalSource.empty())
        {
            return;
        }

        std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
    }

    void _ensureAttributes() override
    {
        std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
    }

    void _ensurePositions() override
    {
        std::call_once(_positionFlag, [&]() { _loadPositions(); });
    }
};

struct Synapses::SonataImpl : public Synapses::InternalBaseImpl
{
    SonataImpl(const Circuit& circuit, const GIDSet& gids, bool afferent, SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, afferent, prefetch)
        , _edges(EdgePopulation::open(_circuit->getSynapseSource(), _circuit->getSynapsePopulation()))
        , _nodeIds(GidConverter::toSonata(gids))
        , _selection(_afferent ? _edges.afferentEdges(_nodeIds) : _edges.efferentEdges(_nodeIds))
    {
        BRAIN_WARN << "SONATA format support is experimental. It is encouraged to use libsonata instead" << std::endl;

        _size = _selection.flatSize();

        _loadConnectivity();

        if (int(prefetch) & int(SynapsePrefetch::attributes))
        {
            std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
        }
        if (int(prefetch) & int(SynapsePrefetch::positions))
        {
            std::call_once(_positionFlag, [&]() { _loadPositions(); });
        }
    }

    SonataImpl(const Circuit& circuit, const GIDSet& gids, const std::string& source, const SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, source, prefetch)
        , _edges(EdgePopulation::open(_circuit->getSynapseProjectionSource(source),
                                      _circuit->getSynapseProjectionPopulation(source)))
        , _nodeIds(GidConverter::toSonata(gids))
        , _selection(_afferent ? _edges.afferentEdges(_nodeIds) : _edges.efferentEdges(_nodeIds))
    {
        BRAIN_WARN << "SONATA format support is experimental. It is encouraged to use libsonata instead" << std::endl;

        _size = _selection.flatSize();

        _loadProjectionConnectivity();

        if (int(prefetch) & int(SynapsePrefetch::attributes))
        {
            std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
        }
    }

    void _loadConnectivity()
    {
        auto selection = _afferent ? _edges.afferentEdges(_nodeIds) : _edges.efferentEdges(_nodeIds);
        auto preGidsTemp = _afferent ? _edges.targetNodeIDs(selection) : _edges.sourceNodeIDs(selection);
        auto postGidsTemp = _afferent ? _edges.sourceNodeIDs(selection) : _edges.targetNodeIDs(selection);
        _preGID = GidConverter::toBlueConfig(preGidsTemp);
        _postGID = GidConverter::toBlueConfig(postGidsTemp);
    }

    void _loadProjectionConnectivity()
    {
        _preGID = GidConverter::toBlueConfig(_edges.sourceNodeIDs(_selection));
        _postGID = GidConverter::toBlueConfig(_edges.targetNodeIDs(_selection));
    }

    void _loadAttributes()
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (!_delay.empty())
        {
            return;
        }

        _delay = _edges.getAttribute<float>("delay", _selection);
        _decay = _edges.getAttribute<float>("decay_time", _selection);
        _preSegmentID = _edges.getAttribute<uint32_t>("afferent_segment_id", _selection);
        _preSectionID = _edges.getAttribute<uint32_t>("afferent_section_id", _selection);
        _postSegmentID = _edges.getAttribute<uint32_t>("efferent_segment_id", _selection);
        _postSectionID = _edges.getAttribute<uint32_t>("efferent_section_id", _selection);
        _conductance = _edges.getAttribute<float>("conductance", _selection);
        _facilitation = _edges.getAttribute<float>("facilitation_time", _selection);
        _depression = _edges.getAttribute<float>("depression_time", _selection);

        if (!_afferent)
        {
            swap(_preGID, _postGID);
        }
    }

    void _loadPositions()
    {
        if (!_externalSource.empty())
        {
            BRAIN_THROW("Synapse positions not available for projections")
        }

        std::lock_guard<std::mutex> lock(_mutex);

        if (!_preCenterPositionX.empty())
        {
            return;
        }

        _preCenterPositionX = _edges.getAttribute<float>("afferent_center_x", _selection);
        _preCenterPositionY = _edges.getAttribute<float>("afferent_center_y", _selection);
        _preCenterPositionZ = _edges.getAttribute<float>("afferent_center_z", _selection);
        _preSurfacePositionX = _edges.getAttribute<float>("afferent_surface_x", _selection);
        _preSurfacePositionY = _edges.getAttribute<float>("afferent_surface_y", _selection);
        _preSurfacePositionZ = _edges.getAttribute<float>("afferent_surface_z", _selection);
        _postCenterPositionX = _edges.getAttribute<float>("efferent_center_x", _selection);
        _postCenterPositionY = _edges.getAttribute<float>("efferent_center_y", _selection);
        _postCenterPositionZ = _edges.getAttribute<float>("efferent_center_z", _selection);
        _postSurfacePositionX = _edges.getAttribute<float>("efferent_surface_x", _selection);
        _postSurfacePositionY = _edges.getAttribute<float>("efferent_surface_y", _selection);
        _postSurfacePositionZ = _edges.getAttribute<float>("efferent_surface_z", _selection);

        if (!_afferent)
        {
            swap(_preCenterPositionX, _postCenterPositionX);
            swap(_preCenterPositionY, _postCenterPositionY);
            swap(_preCenterPositionZ, _postCenterPositionZ);
            swap(_preSurfacePositionX, _postSurfacePositionX);
            swap(_preSurfacePositionY, _postSurfacePositionY);
            swap(_preSurfacePositionZ, _postSurfacePositionZ);
        }
    }

    void _ensureGIDs() override
    {
        if (_externalSource.empty())
        {
            return;
        }

        std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
    }

    void _ensureAttributes() override
    {
        std::call_once(_attributeFlag, [&]() { _loadAttributes(); });
    }

    void _ensurePositions() override
    {
        std::call_once(_positionFlag, [&]() { _loadPositions(); });
    }

private:
    bbp::sonata::EdgePopulation _edges;
    std::vector<uint64_t> _nodeIds;
    bbp::sonata::Selection _selection;
};

Synapses::Synapses(const Circuit& circuit, const GIDSet& gids, bool afferent, SynapsePrefetch prefetch)
{
    auto synapseSource = circuit._impl->getSynapseSource();
    if (synapseSource.find("sonata") != std::string::npos || synapseSource.find("edges.h5") != std::string::npos)
    {
        _impl = std::make_shared<SonataImpl>(circuit, gids, afferent, prefetch);
    }
    else
    {
        _impl = std::make_shared<Impl>(circuit, gids, afferent, prefetch);
    }
}

Synapses::Synapses(const Circuit& circuit, const GIDSet& gids, const std::string& source, SynapsePrefetch prefetch)
{
    auto synapseSource = circuit._impl->getSynapseSource();
    if (synapseSource.find("sonata") != std::string::npos || synapseSource.find("edges.h5") != std::string::npos)
    {
        _impl = std::make_shared<SonataImpl>(circuit, gids, source, prefetch);
    }
    else
    {
        _impl = std::make_shared<Impl>(circuit, gids, source, prefetch);
    }
}

Synapses::Synapses(const SynapsesStream& stream)
{
    auto& streamImpl = *stream._impl;
    auto& projection = streamImpl._externalSource;
    auto& circuit = streamImpl._circuit;
    auto& gids = streamImpl._gids;
    auto afferent = streamImpl._afferent;
    auto prefetch = streamImpl._prefetch;

    if (projection.empty())
    {
        *this = Synapses(circuit, gids, afferent, prefetch);
    }
    else
    {
        *this = Synapses(circuit, gids, projection, prefetch);
    }
}

Synapses::Synapses(const Synapses& rhs) noexcept
    : _impl(rhs._impl)
{
}

Synapses::Synapses(Synapses&& rhs) noexcept
    : _impl(std::move(rhs._impl))
{
}

Synapses::~Synapses() {}

Synapses& Synapses::operator=(const Synapses& rhs) noexcept
{
    if (this != &rhs)
        _impl = rhs._impl;
    return *this;
}

Synapses& Synapses::operator=(Synapses&& rhs) noexcept
{
    if (this != &rhs)
        _impl = std::move(rhs._impl);
    return *this;
}

size_t Synapses::size() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    return impl._size;
}

bool Synapses::empty() const
{
    return size() == 0;
}

Synapses::const_iterator Synapses::begin() const
{
    return const_iterator(*this, 0);
}

Synapses::const_iterator Synapses::end() const
{
    return const_iterator(*this, size());
}

Synapse Synapses::operator[](const size_t index_) const
{
    return Synapse(*this, index_);
}

const size_t* Synapses::indices() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    if (impl._index.empty())
    {
        BRAIN_THROW("Synapse index not available")
    }
    return impl._index.data();
}

const uint32_t* Synapses::preGIDs() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureGIDs();
    return impl._preGID.data();
}

const uint32_t* Synapses::preSectionIDs() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._preSectionID.data();
}

const uint32_t* Synapses::preSegmentIDs() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._preSegmentID.data();
}

const float* Synapses::preDistances() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._preDistance.data();
}

const float* Synapses::preSurfaceXPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preSurfacePositionX.data();
}

const float* Synapses::preSurfaceYPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preSurfacePositionY.data();
}

const float* Synapses::preSurfaceZPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preSurfacePositionZ.data();
}

const float* Synapses::preCenterXPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preCenterPositionX.data();
}

const float* Synapses::preCenterYPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preCenterPositionY.data();
}

const float* Synapses::preCenterZPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preCenterPositionZ.data();
}

const uint32_t* Synapses::postGIDs() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureGIDs();
    return impl._postGID.data();
}

const uint32_t* Synapses::postSectionIDs() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._postSectionID.data();
}

const uint32_t* Synapses::postSegmentIDs() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._postSegmentID.data();
}

const float* Synapses::postDistances() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._postDistance.data();
}

const float* Synapses::postSurfaceXPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postSurfacePositionX.data();
}

const float* Synapses::postSurfaceYPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postSurfacePositionY.data();
}

const float* Synapses::postSurfaceZPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postSurfacePositionZ.data();
}

const float* Synapses::postCenterXPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postCenterPositionX.data();
}

const float* Synapses::postCenterYPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postCenterPositionY.data();
}

const float* Synapses::postCenterZPositions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postCenterPositionZ.data();
}

const float* Synapses::delays() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._delay.data();
}

const float* Synapses::conductances() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._conductance.data();
}

const float* Synapses::utilizations() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._utilization.data();
}

const float* Synapses::depressions() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._depression.data();
}

const float* Synapses::facilitations() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._facilitation.data();
}

const float* Synapses::decays() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._decay.data();
}

const int* Synapses::efficacies() const
{
    Impl& impl = static_cast<Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._efficacy.data();
}
} // namespace brain
