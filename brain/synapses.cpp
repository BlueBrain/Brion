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

namespace brain
{
namespace
{
template <typename T>
void _allocate(T& data, const size_t size)
{
    if (data)
        return;

    void* ptr;
    if (posix_memalign(&ptr, 32, size * sizeof(typename T::element_type)))
    {
        BRAIN_WARN << "Memory alignment failed. Trying normal allocation"
                   << std::endl;
        ptr = calloc(size, sizeof(typename T::element_type));
        if (!ptr)
            BRAIN_THROW_IMPL(std::bad_alloc())
    }
    data.reset((typename T::element_type*)ptr);
    // cppcheck-suppress memleak
}
} // namespace

struct Synapses::InternalBaseImpl : public Synapses::BaseImpl
{
    InternalBaseImpl(const Circuit& circuit, const GIDSet& gids,
                     const GIDSet& filterGIDs, const bool afferent,
                     const SynapsePrefetch prefetch)
        : _circuit(circuit._impl)
        , _gids(prefetch != SynapsePrefetch::all ? gids : GIDSet())
        , _filterGIDs(prefetch != SynapsePrefetch::all ? filterGIDs : GIDSet())
        , _afferent(afferent)
        , _size(0)
    {
    }

    InternalBaseImpl(const Circuit& circuit, const GIDSet& gids,
                     const std::string& source, const SynapsePrefetch prefetch)
        : _circuit(circuit._impl)
        , _gids(prefetch != SynapsePrefetch::all ? gids : GIDSet())
        , _afferent(true)
        , _externalSource(source)
        , _size(0)
    {
    }

#define FILTER(gid)                                                         \
    if (!filterGIDs->empty() && filterGIDs->find(gid) == filterGIDs->end()) \
        continue;

    virtual ~InternalBaseImpl() {}

    virtual void _loadConnectivity(const GIDSet* gids,
                                   const GIDSet* filterGIDs) const = 0;
    virtual void _loadAttributes(const GIDSet* gids,
                                 const GIDSet* filterGIDs) const = 0;
    virtual void _loadPositions(const GIDSet* gids,
                                const GIDSet* filterGIDs) const = 0;
    virtual void _ensureGIDs() const = 0;
    virtual void _ensureAttributes() const = 0;
    virtual void _ensurePositions() const = 0;

    void _allocateAttributes(const size_t size, const bool allocateIndex) const
    {
        if (allocateIndex)
            _allocate(_index, size);

        _allocate(_preSectionID, size);
        _allocate(_preSegmentID, size);
        _allocate(_preDistance, size);

        _allocate(_postSectionID, size);
        _allocate(_postSegmentID, size);
        _allocate(_postDistance, size);

        _allocate(_delay, size);
        _allocate(_conductance, size);
        _allocate(_utilization, size);
        _allocate(_depression, size);
        _allocate(_facilitation, size);
        _allocate(_decay, size);
        _allocate(_efficacy, size);
    }

    void _allocatePositions(const size_t size) const
    {
        _allocate(_preSurfacePositionX, size);
        _allocate(_preSurfacePositionY, size);
        _allocate(_preSurfacePositionZ, size);
        _allocate(_preCenterPositionX, size);
        _allocate(_preCenterPositionY, size);
        _allocate(_preCenterPositionZ, size);

        _allocate(_postSurfacePositionX, size);
        _allocate(_postSurfacePositionY, size);
        _allocate(_postSurfacePositionZ, size);
        _allocate(_postCenterPositionX, size);
        _allocate(_postCenterPositionY, size);
        _allocate(_postCenterPositionZ, size);
    }

    void _clearPositions() const
    {
        _preSurfacePositionX.reset();
        _preSurfacePositionY.reset();
        _preSurfacePositionZ.reset();
        _preCenterPositionX.reset();
        _preCenterPositionY.reset();
        _preCenterPositionZ.reset();
        _postSurfacePositionX.reset();
        _postSurfacePositionY.reset();
        _postSurfacePositionZ.reset();
        _postCenterPositionX.reset();
        _postCenterPositionY.reset();
        _postCenterPositionZ.reset();
    }

    size_t _getSize() const { return _size; }

    const std::shared_ptr<const Circuit::Impl> _circuit;
    const GIDSet _gids;
    const GIDSet _filterGIDs;
    const bool _afferent;
    std::string _externalSource;

    template <typename T>
    struct FreeDeleter
    {
        void operator()(T* ptr) { free(ptr); }
    };

    typedef std::unique_ptr<uint32_t, FreeDeleter<uint32_t>> UIntPtr;
    typedef std::unique_ptr<int, FreeDeleter<int>> IntPtr;
    typedef std::unique_ptr<size_t, FreeDeleter<size_t>> size_tPtr;
    typedef std::unique_ptr<float, FreeDeleter<float>> floatPtr;

    mutable size_t _size;

    mutable size_tPtr _index;

    mutable UIntPtr _preGID;
    mutable UIntPtr _preSectionID;
    mutable UIntPtr _preSegmentID;
    mutable floatPtr _preDistance;
    mutable floatPtr _preSurfacePositionX;
    mutable floatPtr _preSurfacePositionY;
    mutable floatPtr _preSurfacePositionZ;
    mutable floatPtr _preCenterPositionX;
    mutable floatPtr _preCenterPositionY;
    mutable floatPtr _preCenterPositionZ;

    mutable UIntPtr _postGID;
    mutable UIntPtr _postSectionID;
    mutable UIntPtr _postSegmentID;
    mutable floatPtr _postDistance;
    mutable floatPtr _postSurfacePositionX;
    mutable floatPtr _postSurfacePositionY;
    mutable floatPtr _postSurfacePositionZ;
    mutable floatPtr _postCenterPositionX;
    mutable floatPtr _postCenterPositionY;
    mutable floatPtr _postCenterPositionZ;

    mutable floatPtr _delay;
    mutable floatPtr _conductance;
    mutable floatPtr _utilization;
    mutable floatPtr _depression;
    mutable floatPtr _facilitation;
    mutable floatPtr _decay;
    mutable IntPtr _efficacy;

    mutable std::once_flag _attributeFlag;
    mutable std::once_flag _positionFlag;
    mutable std::once_flag _indexFlag;
    // Besides the call_once flags, we still need to ensure exclusive access
    // to state.
    mutable std::mutex _mutex;
};

struct Synapses::Impl : public Synapses::InternalBaseImpl
{
    Impl(const Circuit& circuit, const GIDSet& gids, const GIDSet& filterGIDs,
         const bool afferent, const SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, filterGIDs, afferent,
                                     prefetch)
    {
        _loadConnectivity(&gids, &filterGIDs);

        if (int(prefetch) & int(SynapsePrefetch::attributes))
            std::call_once(_attributeFlag, &Impl::_loadAttributes, this, &gids,
                           &filterGIDs);
        if (int(prefetch) & int(SynapsePrefetch::positions))
            std::call_once(_positionFlag, &Impl::_loadPositions, this, &gids,
                           &filterGIDs);
    }

    Impl(const Circuit& circuit, const GIDSet& gids, const std::string& source,
         const SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, source, prefetch)
    {
        // We don't have a summary file for projected afferent synapses.
        // But at least we have to figure out the size of the container.
        const auto& synapses =
            _circuit->getAfferentProjectionAttributes(source);
        _size = synapses.getNumSynapses(gids);

        if (int(prefetch) & int(SynapsePrefetch::attributes))
        {
            GIDSet empty;
            std::call_once(_attributeFlag, &Impl::_loadAttributes, this, &gids,
                           &empty);
        }
    }

    void _loadConnectivity(const GIDSet* gids, const GIDSet* filterGIDs) const
    {
        const brion::SynapseSummary& synapseSummary =
            _circuit->getSynapseSummary();

        uint32_ts pres, posts;
        for (const auto gid : *gids)
        {
            const auto& summary = synapseSummary.read(gid);

            for (size_t i = 0; i < summary.shape()[0]; ++i)
            {
                const uint32_t peerGid = summary[i][0];
                FILTER(peerGid);

                for (size_t j = 0; j < summary[i][_afferent ? 2 : 1]; ++j)
                {
                    pres.push_back(peerGid);
                    posts.push_back(gid);
                }
            }
        }

        _size = pres.size();
        _allocate(_preGID, _size);
        _allocate(_postGID, _size);
        memcpy(_preGID.get(), pres.data(), _size * sizeof(uint32_t));
        memcpy(_postGID.get(), posts.data(), _size * sizeof(uint32_t));

        if (!_afferent)
            _preGID.swap(_postGID);
    }

    void _loadAttributes(const GIDSet* gids, const GIDSet* filterGIDs) const
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (_efficacy)
            return;

        const brion::Synapse& synapseAttributes =
            _externalSource.empty()
                ? _circuit->getSynapseAttributes(_afferent)
                : _circuit->getAfferentProjectionAttributes(_externalSource);
        const brion::Synapse* synapseExtra =
            _externalSource.empty() ? _circuit->getSynapseExtra() : nullptr;

        // For external afferent projections we haven't had the chance to
        // get the connectivity.
        const bool haveGIDs = _externalSource.empty();
        if (!haveGIDs)
        {
            _allocate(_preGID, _size);
            _allocate(_postGID, _size);
        }

        const bool haveExtra = _afferent && synapseExtra;
        _allocateAttributes(_size, haveExtra || _afferent);

        size_t i = 0;
        for (const auto gid : *gids)
        {
            auto&& attr =
                synapseAttributes.read(gid, brion::SYNAPSE_ALL_ATTRIBUTES);
            auto&& extra =
                haveExtra ? synapseExtra->read(gid, 1) : brion::SynapseMatrix();
            for (size_t j = 0; j < attr.shape()[0]; ++j)
            {
                const uint32_t connected = attr[j][0];
                FILTER(connected);

                if (!haveGIDs)
                {
                    // This code path is exclusive of external projections,
                    // which are afferent only
                    assert(_afferent);
                    _preGID.get()[i] = connected;
                    _postGID.get()[i] = gid;
                }
                assert(_preGID.get()[i] == _afferent ? connected : gid);
                assert(_postGID.get()[i] == _afferent ? gid : connected);
                assert(i < _size);

                if (haveExtra)
                    _index.get()[i] = extra[j][0];
                else if (_afferent)
                    // Fallback assigment of synapses index for afferent views
                    _index.get()[i] = j;

                _delay.get()[i] = attr[j][1];
                _postSectionID.get()[i] = attr[j][2];
                _postSegmentID.get()[i] = attr[j][3];
                _postDistance.get()[i] = attr[j][4];
                _preSectionID.get()[i] = attr[j][5];
                _preSegmentID.get()[i] = attr[j][6];
                _preDistance.get()[i] = attr[j][7];
                _conductance.get()[i] = attr[j][8];
                _utilization.get()[i] = attr[j][9];
                _depression.get()[i] = attr[j][10];
                _facilitation.get()[i] = attr[j][11];
                _decay.get()[i] = attr[j][12];
                _efficacy.get()[i] = attr[j][17];
                ++i;
            }
        }
    }

    void _loadPositions(const GIDSet* gids, const GIDSet* filterGIDs) const
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (!_externalSource.empty())
            BRAIN_THROW(
                "Synapse positions are not available for external projection "
                "synapses")

        if (_preCenterPositionX)
            return;

        _allocatePositions(_size);

        Strings keys;
        CachedSynapses loaded;

        // delay the opening of the synapse file as much as possible, even
        // though the code looks ugly... As the circuit impl keeps the file
        // opened, we can safely just get a loose pointer here.
        const brion::Synapse* positions = nullptr;

        size_t i = 0;
        bool haveSurfacePositions = false;
        for (const auto gid : *gids)
        {
            const auto readFromFile = [&] {
                if (!positions)
                {
                    try
                    {
                        positions = &_circuit->getSynapsePositions(_afferent);
                    }
                    catch (...)
                    {
                        // Leave arrays unmodified for exception safety
                        _clearPositions();
                        throw;
                    }
                }

                if (positions->getNumAttributes() ==
                    brion::SYNAPSE_POSITION_ALL)
                    return positions->read(gid, brion::SYNAPSE_POSITION);
                else
                    return positions->read(gid, brion::SYNAPSE_OLD_POSITION);
            };

            const brion::SynapseMatrix pos = readFromFile();

            for (size_t j = 0; j < pos.size(); ++j)
            {
                const uint32_t preGid = pos[j][0];
                FILTER(preGid);

                if (pos.shape()[1] == brion::SYNAPSE_POSITION_ALL)
                {
                    haveSurfacePositions = true;
                    _preSurfacePositionX.get()[i] = pos[j][1];
                    _preSurfacePositionY.get()[i] = pos[j][2];
                    _preSurfacePositionZ.get()[i] = pos[j][3];
                    _postSurfacePositionX.get()[i] = pos[j][4];
                    _postSurfacePositionY.get()[i] = pos[j][5];
                    _postSurfacePositionZ.get()[i] = pos[j][6];
                    _preCenterPositionX.get()[i] = pos[j][7];
                    _preCenterPositionY.get()[i] = pos[j][8];
                    _preCenterPositionZ.get()[i] = pos[j][9];
                    _postCenterPositionX.get()[i] = pos[j][10];
                    _postCenterPositionY.get()[i] = pos[j][11];
                    _postCenterPositionZ.get()[i] = pos[j][12];
                }
                else
                {
                    _preCenterPositionX.get()[i] = pos[j][1];
                    _preCenterPositionY.get()[i] = pos[j][2];
                    _preCenterPositionZ.get()[i] = pos[j][3];
                    _postCenterPositionX.get()[i] = pos[j][4];
                    _postCenterPositionY.get()[i] = pos[j][5];
                    _postCenterPositionZ.get()[i] = pos[j][6];
                }
                ++i;
            }
        }

        if (!haveSurfacePositions)
        {
            _preSurfacePositionX.reset();
            _preSurfacePositionY.reset();
            _preSurfacePositionZ.reset();
            _postSurfacePositionX.reset();
            _postSurfacePositionY.reset();
            _postSurfacePositionZ.reset();
        }
    }

    void _ensureGIDs() const
    {
        if (_externalSource.empty())
            return;

        // Until C++17 call_once is using decay_copy
        std::call_once(_attributeFlag, &Impl::_loadAttributes, this, &_gids,
                       &_filterGIDs);
    }

    void _ensureAttributes() const
    {
        // Until C++17 call_once is using decay_copy
        std::call_once(_attributeFlag, &Impl::_loadAttributes, this, &_gids,
                       &_filterGIDs);
    }

    void _ensurePositions() const
    {
        // Until C++17 call_once is using decay_copy
        std::call_once(_positionFlag, &Impl::_loadPositions, this, &_gids,
                       &_filterGIDs);
    }
};

struct Synapses::SonataImpl : public Synapses::InternalBaseImpl
{
#define TRY_GET_ATTRIBUTE(buffer, source, type, name, selection) \
    try                                                          \
    {                                                            \
        buffer = source.getAttribute<type>(name, selection);     \
    }                                                            \
    catch (...)                                                  \
    {                                                            \
    }

#define ADD_TO_STORAGE(storage, storageIndex, source, sourceIndex) \
    if (!source.empty())                                           \
        storage.get()[storageIndex] = source[sourceIndex];

    SonataImpl(const Circuit& circuit, const GIDSet& gids,
               const GIDSet& filterGIDs, const bool afferent,
               const SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, filterGIDs, afferent,
                                     prefetch)
    {
        BRAIN_WARN << "The SONATA format support is experimental and not "
                      "officially supported. "
                   << "It is encouraged to use libsonata instead" << std::endl;

        GIDSet fixedGids;
        for (const auto gid : gids)
            fixedGids.insert(gid - 1);
        GIDSet fixedFilters;
        for (const auto gid : filterGIDs)
            fixedFilters.insert(gid - 1);
        _loadConnectivity(&fixedGids, &fixedFilters);

        if (int(prefetch) & int(SynapsePrefetch::attributes))
            std::call_once(_attributeFlag, &SonataImpl::_loadAttributes, this,
                           &fixedGids, &fixedFilters);
        if (int(prefetch) & int(SynapsePrefetch::positions))
            std::call_once(_positionFlag, &SonataImpl::_loadPositions, this,
                           &fixedGids, &fixedFilters);
    }

    SonataImpl(const Circuit& circuit, const GIDSet& gids,
               const std::string& source, const SynapsePrefetch prefetch)
        : Synapses::InternalBaseImpl(circuit, gids, source, prefetch)
    {
        BRAIN_WARN << "The SONATA format support is experimental and not "
                      "officially supported. "
                   << "It is encouraged to use libsonata instead" << std::endl;

        GIDSet fixedGids;
        for (const auto gid : gids)
            fixedGids.insert(gid - 1);

        const std::string projSourceFile =
            _circuit->getSynapseProjectionSource(source);
        const std::string& synapsePopulation =
            _circuit->getSynapseProjectionPopulation(source);
        // We don't have a summary file for projected afferent synapses.
        // But at least we have to figure out the size of the container.
        const bbp::sonata::EdgeStorage edgeStorage(projSourceFile);
        if (!synapsePopulation.empty())
        {
            const bbp::sonata::EdgePopulation edges(projSourceFile, "",
                                                    synapsePopulation);
            _size += edges.size();
        }
        else
        {
            for (const auto& popName : edgeStorage.populationNames())
            {
                const bbp::sonata::EdgePopulation edges(projSourceFile, "",
                                                        popName);
                _size += edges.size();
            }
        }

        if (int(prefetch) & int(SynapsePrefetch::attributes))
        {
            GIDSet empty;
            std::call_once(_attributeFlag, &SonataImpl::_loadAttributes, this,
                           &fixedGids, &empty);
        }
    }

    void _loadConnectivity(const GIDSet* gids, const GIDSet* filterGIDs) const
    {
        const std::string synapseFilePath = _circuit->getSynapseSource();

        std::set<uint64_t> uniqueIds;
        // Input ids must be decreased by 1
        for (uint32_t oldId : *gids)
        {
            if (oldId > 0)
                uniqueIds.insert(oldId - 1);
        }

        uint32_ts filteredPreGids, filteredPostGids;
        const std::vector<uint64_t> nodeIds(uniqueIds.begin(), uniqueIds.end());
        const bbp::sonata::EdgeStorage edgeStorage(synapseFilePath);

        auto synapseGIDsFunc = [&](const std::string& populationName) {
            const bbp::sonata::EdgePopulation edges(synapseFilePath, "",
                                                    populationName);
            const bbp::sonata::Selection s = _afferent
                                                 ? edges.afferentEdges(nodeIds)
                                                 : edges.efferentEdges(nodeIds);

            const std::vector<uint64_t> preGidsTemp = edges.sourceNodeIDs(s);
            const std::vector<uint64_t> postGidsTemp = edges.targetNodeIDs(s);

            // Result ids must be incremented by one to return in the same
            // range as the input source GIDs
            for (size_t i = 0; i < preGidsTemp.size(); i++)
            {
                uint32_t preN = static_cast<uint32_t>(preGidsTemp[i]);
                FILTER(preN)

                filteredPreGids.push_back(preN + 1);
                filteredPostGids.push_back(
                    static_cast<uint32_t>(postGidsTemp[i]) + 1);
            }
        };

        const std::string& synapsePopulation = _circuit->getSynapsePopulation();
        if (!synapsePopulation.empty())
            synapseGIDsFunc(synapsePopulation);
        else
            synapseGIDsFunc(*edgeStorage.populationNames().begin());

        _size = filteredPreGids.size();
        _allocate(_preGID, _size);
        _allocate(_postGID, _size);
        memcpy(_preGID.get(), filteredPreGids.data(), _size * sizeof(uint32_t));
        memcpy(_postGID.get(), filteredPostGids.data(),
               _size * sizeof(uint32_t));
    }

    void _loadAttributes(const GIDSet* gids, const GIDSet* filterGIDs) const
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (_efficacy)
            return;

        std::string synapseFilePath;
        std::string synapsePop;
        if (_externalSource.empty())
        {
            synapseFilePath = _circuit->getSynapseSource();
            synapsePop = _circuit->getSynapsePopulation();
        }
        else
        {
            synapseFilePath =
                _circuit->getSynapseProjectionSource(_externalSource);
            synapsePop =
                _circuit->getSynapseProjectionPopulation(_externalSource);
        }

        std::set<uint64_t> uniqueIds;
        // Input ids must be decreased by 1
        for (uint32_t oldId : *gids)
        {
            if (oldId > 0)
                uniqueIds.insert(oldId - 1);
        }
        const std::vector<uint64_t> nodeIds(uniqueIds.begin(), uniqueIds.end());
        const bbp::sonata::EdgeStorage edgeStorage(synapseFilePath);

        // For external afferent projections we haven't had the chance to
        // get the connectivity.
        const bool haveGIDs = _externalSource.empty();
        if (!haveGIDs)
        {
            _allocate(_preGID, _size);
            _allocate(_postGID, _size);
        }

        _allocateAttributes(_size, _afferent);

        auto synapsePropFunc = [&](const std::string& popName) {
            const bbp::sonata::EdgePopulation edges(synapseFilePath, "",
                                                    popName);
            const bbp::sonata::Selection s = _afferent
                                                 ? edges.afferentEdges(nodeIds)
                                                 : edges.efferentEdges(nodeIds);

            const auto preGIDs = edges.sourceNodeIDs(s);
            const auto postGIDs = edges.targetNodeIDs(s);

            std::vector<float> delays;
            std::vector<float> decayTimes;
            std::vector<uint32_t> preSegmentIds;
            std::vector<uint32_t> preSectionIds;
            std::vector<uint32_t> postSegmentIds;
            std::vector<uint32_t> postSectionIds;
            std::vector<float> conductances;
            std::vector<float> facilitations;
            std::vector<float> depressions;
            TRY_GET_ATTRIBUTE(delays, edges, float, "delay", s)
            TRY_GET_ATTRIBUTE(decayTimes, edges, float, "decay_time", s)
            TRY_GET_ATTRIBUTE(preSegmentIds, edges, uint32_t,
                              "afferent_segment_id", s)
            TRY_GET_ATTRIBUTE(preSectionIds, edges, uint32_t,
                              "afferent_section_id", s)
            TRY_GET_ATTRIBUTE(postSegmentIds, edges, uint32_t,
                              "efferent_segment_id", s)
            TRY_GET_ATTRIBUTE(postSectionIds, edges, uint32_t,
                              "efferent_section_id", s)
            TRY_GET_ATTRIBUTE(conductances, edges, float, "conductance", s)
            TRY_GET_ATTRIBUTE(facilitations, edges, float, "facilitation_time",
                              s)
            TRY_GET_ATTRIBUTE(depressions, edges, float, "depression_time", s)

            for (size_t j = 0; j < preGIDs.size(); j++)
            {
                FILTER(static_cast<uint32_t>(preGIDs[j] + 1));
                if (!haveGIDs)
                {
                    if (preGIDs.size() > j)
                        _preGID.get()[j] =
                            static_cast<uint32_t>(preGIDs[j] + 1);
                    if (postGIDs.size() > j)
                        _postGID.get()[j] =
                            static_cast<uint32_t>(postGIDs[j] + 1);
                }

                ADD_TO_STORAGE(_delay, j, delays, j)
                ADD_TO_STORAGE(_decay, j, decayTimes, j)
                ADD_TO_STORAGE(_preSectionID, j, preSectionIds, j)
                ADD_TO_STORAGE(_preSegmentID, j, preSegmentIds, j)
                ADD_TO_STORAGE(_postSectionID, j, postSectionIds, j)
                ADD_TO_STORAGE(_postSegmentID, j, postSectionIds, j)
                ADD_TO_STORAGE(_conductance, j, conductances, j)
                ADD_TO_STORAGE(_facilitation, j, facilitations, j)
                ADD_TO_STORAGE(_depression, j, depressions, j)
            }
        };

        if (!synapsePop.empty())
            synapsePropFunc(synapsePop);
        else
            synapsePropFunc(*edgeStorage.populationNames().begin());

        if (!_afferent)
        {
            swap(_preGID, _postGID);
        }
    }

    void _loadPositions(const GIDSet* gids, const GIDSet* filterGIDs) const
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (!_externalSource.empty())
            BRAIN_THROW(
                "Synapse positions are not available for external projection "
                "synapses")

        if (_preCenterPositionX)
            return;

        std::string synapseFilePath = _circuit->getSynapseSource();

        std::set<uint64_t> uniqueIds;
        // Input ids must be decreased by 1
        for (uint32_t oldId : *gids)
        {
            if (oldId > 0)
                uniqueIds.insert(oldId - 1);
        }
        const std::vector<uint64_t> nodeIds(uniqueIds.begin(), uniqueIds.end());
        const bbp::sonata::EdgeStorage edgeStorage(synapseFilePath);

        _allocatePositions(_size);

        auto synapsePosFunc = [&](const std::string& name) {
            const bbp::sonata::EdgePopulation edges(synapseFilePath, "", name);
            const bbp::sonata::Selection s = _afferent
                                                 ? edges.afferentEdges(nodeIds)
                                                 : edges.efferentEdges(nodeIds);

            const auto preGids = edges.sourceNodeIDs(s);
            const auto postGids = edges.targetNodeIDs(s);

            std::vector<double> preCenterXs, preCenterYs, preCenterZs;
            std::vector<double> preSurfaceXs, preSurfaceYs, preSurfaceZs;
            std::vector<double> postCenterXs, postCenterYs, postCenterZs;
            std::vector<double> postSurfaceXs, postSurfaceYs, postSurfaceZs;

            TRY_GET_ATTRIBUTE(preCenterXs, edges, double, "afferent_center_x",
                              s)
            TRY_GET_ATTRIBUTE(preCenterYs, edges, double, "afferent_center_y",
                              s)
            TRY_GET_ATTRIBUTE(preCenterZs, edges, double, "afferent_center_z",
                              s)
            TRY_GET_ATTRIBUTE(preSurfaceXs, edges, double, "afferent_surface_x",
                              s)
            TRY_GET_ATTRIBUTE(preSurfaceYs, edges, double, "afferent_surface_y",
                              s)
            TRY_GET_ATTRIBUTE(preSurfaceZs, edges, double, "afferent_surface_z",
                              s)
            TRY_GET_ATTRIBUTE(postCenterXs, edges, double, "efferent_center_x",
                              s)
            TRY_GET_ATTRIBUTE(postCenterYs, edges, double, "efferent_center_y",
                              s)
            TRY_GET_ATTRIBUTE(postCenterZs, edges, double, "efferent_center_z",
                              s)
            TRY_GET_ATTRIBUTE(postSurfaceXs, edges, double,
                              "efferent_surface_x", s)
            TRY_GET_ATTRIBUTE(postSurfaceYs, edges, double,
                              "efferent_surface_y", s)
            TRY_GET_ATTRIBUTE(postSurfaceZs, edges, double,
                              "efferent_surface_z", s)

            for (size_t j = 0; j < preGids.size(); j++)
            {
                FILTER(static_cast<uint32_t>(preGids[j] + 1))

                ADD_TO_STORAGE(_preCenterPositionX, j, preCenterXs, j)
                ADD_TO_STORAGE(_preCenterPositionY, j, preCenterYs, j)
                ADD_TO_STORAGE(_preCenterPositionZ, j, preCenterZs, j)
                ADD_TO_STORAGE(_preSurfacePositionX, j, preSurfaceXs, j)
                ADD_TO_STORAGE(_preSurfacePositionY, j, preSurfaceYs, j)
                ADD_TO_STORAGE(_preSurfacePositionZ, j, preSurfaceZs, j)
                ADD_TO_STORAGE(_postCenterPositionX, j, postCenterXs, j)
                ADD_TO_STORAGE(_postCenterPositionY, j, postCenterYs, j)
                ADD_TO_STORAGE(_postCenterPositionZ, j, postCenterZs, j)
                ADD_TO_STORAGE(_postSurfacePositionX, j, postSurfaceXs, j)
                ADD_TO_STORAGE(_postSurfacePositionY, j, postSurfaceYs, j)
                ADD_TO_STORAGE(_postSurfacePositionZ, j, postSurfaceZs, j)
            }
        };

        const auto& synapsePop = _circuit->getSynapsePopulation();
        if (!synapsePop.empty())
            synapsePosFunc(synapsePop);
        else
            synapsePosFunc(*edgeStorage.populationNames().begin());

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

    void _ensureGIDs() const
    {
        if (_externalSource.empty())
            return;

        // Until C++17 call_once is using decay_copy
        std::call_once(_attributeFlag, &SonataImpl::_loadAttributes, this,
                       &_gids, &_filterGIDs);
    }

    void _ensureAttributes() const
    {
        // Until C++17 call_once is using decay_copy
        std::call_once(_attributeFlag, &SonataImpl::_loadAttributes, this,
                       &_gids, &_filterGIDs);
    }

    void _ensurePositions() const
    {
        // Until C++17 call_once is using decay_copy
        std::call_once(_positionFlag, &SonataImpl::_loadPositions, this, &_gids,
                       &_filterGIDs);
    }
};

Synapses::Synapses(const Circuit& circuit, const GIDSet& gids,
                   const GIDSet& filterGIDs, const bool afferent,
                   const SynapsePrefetch prefetch)
{
    if (circuit._impl->getSynapseSource().find("sonata") != std::string::npos)
        _impl = std::shared_ptr<Synapses::BaseImpl>(
            new SonataImpl(circuit, gids, filterGIDs, afferent, prefetch));
    else
        _impl = std::shared_ptr<Synapses::BaseImpl>(
            new Impl(circuit, gids, filterGIDs, afferent, prefetch));
}

Synapses::Synapses(const Circuit& circuit, const GIDSet& gids,
                   const std::string& source, const SynapsePrefetch prefetch)
{
    if (circuit._impl->getSynapseSource().find("sonata") != std::string::npos)
        _impl = std::shared_ptr<Synapses::BaseImpl>(
            new SonataImpl(circuit, gids, source, prefetch));
    else
        _impl = std::shared_ptr<Synapses::BaseImpl>(
            new Impl(circuit, gids, source, prefetch));
}

Synapses::~Synapses() {}

Synapses::Synapses(const SynapsesStream& stream)
{
    const auto synapseSource = stream._impl->_circuit._impl->getSynapseSource();
    // Do some AMAZING CHECKS to guess wether its SONATA or not
    if (stream._impl->_externalSource.empty())
    {
        if (synapseSource.find("sonata") != std::string::npos ||
            synapseSource.find("edges.h5") != std::string::npos)
            _impl = std::shared_ptr<Synapses::BaseImpl>(
                new SonataImpl(stream._impl->_circuit, stream._impl->_gids,
                               stream._impl->_filterGIDs,
                               stream._impl->_afferent,
                               stream._impl->_prefetch));
        else
            _impl = std::shared_ptr<Synapses::BaseImpl>(
                new Impl(stream._impl->_circuit, stream._impl->_gids,
                         stream._impl->_filterGIDs, stream._impl->_afferent,
                         stream._impl->_prefetch));
    }
    else
    {
        if (synapseSource.find("sonata") != std::string::npos ||
            synapseSource.find("edges.h5") != std::string::npos)
            _impl = std::shared_ptr<Synapses::BaseImpl>(
                new SonataImpl(stream._impl->_circuit, stream._impl->_gids,
                               stream._impl->_externalSource,
                               stream._impl->_prefetch));
        else
            _impl = std::shared_ptr<Synapses::BaseImpl>(
                new Impl(stream._impl->_circuit, stream._impl->_gids,
                         stream._impl->_externalSource,
                         stream._impl->_prefetch));
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
    return impl._getSize();
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
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    if (!impl._index)
        BRAIN_THROW("Synapse index not available")
    return impl._index.get();
}

const uint32_t* Synapses::preGIDs() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureGIDs();
    return impl._preGID.get();
}

const uint32_t* Synapses::preSectionIDs() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._preSectionID.get();
}

const uint32_t* Synapses::preSegmentIDs() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._preSegmentID.get();
}

const float* Synapses::preDistances() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._preDistance.get();
}

const float* Synapses::preSurfaceXPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preSurfacePositionX.get();
}

const float* Synapses::preSurfaceYPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preSurfacePositionY.get();
}

const float* Synapses::preSurfaceZPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preSurfacePositionZ.get();
}

const float* Synapses::preCenterXPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preCenterPositionX.get();
}

const float* Synapses::preCenterYPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preCenterPositionY.get();
}

const float* Synapses::preCenterZPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._preCenterPositionZ.get();
}

const uint32_t* Synapses::postGIDs() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureGIDs();
    return impl._postGID.get();
}

const uint32_t* Synapses::postSectionIDs() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._postSectionID.get();
}

const uint32_t* Synapses::postSegmentIDs() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._postSegmentID.get();
}

const float* Synapses::postDistances() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._postDistance.get();
}

const float* Synapses::postSurfaceXPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postSurfacePositionX.get();
}

const float* Synapses::postSurfaceYPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postSurfacePositionY.get();
}

const float* Synapses::postSurfaceZPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postSurfacePositionZ.get();
}

const float* Synapses::postCenterXPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postCenterPositionX.get();
}

const float* Synapses::postCenterYPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postCenterPositionY.get();
}

const float* Synapses::postCenterZPositions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensurePositions();
    return impl._postCenterPositionZ.get();
}

const float* Synapses::delays() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._delay.get();
}

const float* Synapses::conductances() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._conductance.get();
}

const float* Synapses::utilizations() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._utilization.get();
}

const float* Synapses::depressions() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._depression.get();
}

const float* Synapses::facilitations() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._facilitation.get();
}

const float* Synapses::decays() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._decay.get();
}

const int* Synapses::efficacies() const
{
    const Impl& impl = static_cast<const Impl&>(*_impl);
    impl._ensureAttributes();
    return impl._efficacy.get();
}
} // namespace brain
