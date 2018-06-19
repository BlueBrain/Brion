/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Juan Hernando <jhernando@fi.upm.es>
 *                          Adrien.Devresse@epfl.ch
 *                          Daniel.Nachbaur@epfl.ch
 *                          Stefan.Eilemann@epfl.ch
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

#include "targets.h"

#include <brain/neuron/morphology.h>
#include <brion/blueConfig.h>
#include <brion/circuit.h>
#include <brion/circuitConfig.h>
#include <brion/csvConfig.h>
#include <brion/detail/hdf5Mutex.h>
#include <brion/morphology.h>
#include <brion/nodes.h>
#include <brion/synapse.h>
#include <brion/synapseSummary.h>
#include <brion/target.h>

#include <lunchbox/debug.h>
#include <lunchbox/lockable.h>
#include <lunchbox/log.h>
#include <lunchbox/scopedMutex.h>

#ifdef BRAIN_USE_KEYV
#include <keyv/Map.h>
#endif

#ifdef BRAIN_USE_MVD3
#include <highfive/H5Utility.hpp>
#include <mvd/mvd3.hpp>
#include <mvd/mvd_generic.hpp>
#endif

#include <boost/filesystem.hpp>

#include <future>
#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>

namespace fs = boost::filesystem;
using boost::lexical_cast;

namespace brain
{
const std::string summaryFilename("/nrn_summary.h5");
const std::string afferentFilename("/nrn.h5");
const std::string externalAfferentFilename("/proj_nrn.h5");
const std::string efferentFilename("/nrn_efferent.h5");
const std::string afferentPositionsFilename("/nrn_positions.h5");
const std::string efferentPositionsFilename("/nrn_positions_efferent.h5");
const std::string extraFilename("/nrn_extra.h5");

namespace
{
#ifdef BRAIN_USE_MVD3
bool _isSequence(const GIDSet& gids)
{
    return (*gids.rbegin() - *gids.begin() + 1) == gids.size();
}

::MVD3::Range _getRange(const GIDSet& gids)
{
    const size_t offset = (*gids.begin());
    const size_t count = *gids.rbegin() - offset + 1;
    return ::MVD3::Range(offset - 1, count);
}

template <typename SrcArray, typename DstArray, typename AssignOp>
void _assign(const ::MVD3::Range& range, const GIDSet& gids, SrcArray& src,
             DstArray& dst, const AssignOp& assignOp)
{
    if (_isSequence(gids)) // OPT: no holes, no translation needed
    {
        std::transform(src.begin(), src.end(), dst.begin(), assignOp);
        return;
    }

    auto j = dst.begin();
    for (const auto gid : gids)
        *j++ = assignOp(src[gid - range.offset - 1]);
}

Vector3f _toVector3f(const ::MVD3::Positions::const_subarray<1>::type& subarray)
{
    return Vector3f(subarray[0], subarray[1], subarray[2]);
}

Quaternionf _toQuaternion(
    const ::MVD3::Rotations::const_subarray<1>::type& subarray)
{
    return Quaternionf(subarray[0], subarray[1], subarray[2], subarray[3]);
}

template <typename T>
const T& _nop(const T& x)
{
    return x;
}
#endif

template <typename T>
std::vector<T> _getAttribute(const brion::NodeGroup& nodeGroup,
                             const std::string& name, const size_t start,
                             const size_t end)
{
    return nodeGroup.getAttribute<T>(name, start, end);
}

// bool is not a proper H5 type, so we need a special function for this type
// of node attributes
template <>
std::vector<bool> _getAttribute<bool>(const brion::NodeGroup& nodeGroup,
                                      const std::string& name,
                                      const size_t start, const size_t end)
{
    const auto values = nodeGroup.getAttribute<uint8_t>(name, start, end);
    std::vector<bool> result;
    std::transform(values.begin(), values.end(), result.begin(),
                   [](const uint8_t x) { return x != 0; });
    return result;
}

template <typename T>
void _shuffle(T& container, const size_t* seed)
{
    std::random_device randomDevice;
    std::mt19937_64 randomEngine(randomDevice());
    const char* seedEnv = getenv("BRAIN_CIRCUIT_SEED");
    if (seed)
    {
        randomEngine.seed(*seed);
    }
    else if (seedEnv)
    {
        try
        {
            randomEngine.seed(std::stoul(seedEnv));
        }
        catch (const std::exception& exc)
        {
            LBWARN << "Could not set BRAIN_CIRCUIT_SEED to " << seedEnv << ": "
                   << exc.what() << std::endl;
        }
    }
    std::shuffle(container.begin(), container.end(), randomEngine);
}

using CachedMorphologies =
    std::unordered_map<std::string, neuron::MorphologyPtr>;
using CachedSynapses = std::unordered_map<std::string, brion::SynapseMatrix>;
} // namespace

#ifdef BRAIN_USE_KEYV
class SynapseCache
{
public:
    SynapseCache(const keyv::MapPtr& cache, const URI& synapseSource)
        : _cache(cache)
        , _synapsePath(fs::canonical(synapseSource.getPath()).generic_string())
    {
    }

    operator bool() const { return _cache != nullptr; }
    Strings createKeys(const GIDSet& gids, const bool afferent) const
    {
        Strings keys;
        keys.reserve(gids.size());

        std::string prefix = _synapsePath;
        prefix += afferent ? "_afferent" : "_efferent";

        for (const auto gid : gids)
            keys.emplace_back(
                servus::make_uint128(prefix + std::to_string(gid)).getString());

        return keys;
    }

    void savePositions(const uint32_t gid, const std::string& hash,
                       const brion::SynapseMatrix& value) const
    {
        if (!_cache)
            return;

        const size_t size = value.num_elements() * sizeof(float);
        if (!_cache->insert(hash, value.data(), size))
        {
            LBWARN << "Failed to insert synapse positions for GID " << gid
                   << " into cache; item size is " << float(size) / LB_1MB
                   << " MB" << std::endl;
        }
    }

    CachedSynapses loadPositions(const Strings& keys,
                                 const bool hasSurfacePositions) const
    {
        CachedSynapses loaded;
        if (!_cache)
            return loaded;
        LBDEBUG << "Using cache for synapses position loading" << std::endl;
        typedef std::future<std::pair<std::string, brion::SynapseMatrix>>
            Future;

        std::vector<Future> futures;
        futures.reserve(keys.size());

        // 3 columns for afferent, 3 for efferent (x2 if there are surface
        // positions) plus an extra one for a cell GID.
        const size_t numColumns = hasSurfacePositions
                                      ? int(brion::SYNAPSE_POSITION_ALL)
                                      : int(brion::SYNAPSE_OLD_POSITION_ALL);

        _cache->takeValues(keys, [&futures, numColumns](const std::string& key,
                                                        char* data,
                                                        const size_t size) {
            futures.push_back(std::async([key, data, size, numColumns] {
                // there is no constructor in multi_array which just accepts the
                // size in bytes (although there's a getter for it used in
                // saveSynapsePositionsToCache()), so we reconstruct the row and
                // column count here.
                const size_t numRows = size / sizeof(float) / numColumns;
                brion::SynapseMatrix values(
                    boost::extents[numRows][numColumns]);
                ::memmove(values.data(), data, size);
                std::free(data);
                return std::make_pair(key, values);
            }));
        });

        for (auto& future : futures)
            loaded.insert(future.get());

        LBDEBUG << "Loaded synapse positions for " << loaded.size()
                << " out of " << keys.size() << " neurons from cache"
                << std::endl;
        return loaded;
    }

private:
    keyv::MapPtr _cache;
    std::string _synapsePath;
};
#else
class SynapseCache
{
public:
    operator bool() const { return false; }
};
#endif

#ifdef BRAIN_USE_KEYV
class MorphologyCache
{
public:
    MorphologyCache(const keyv::MapPtr& cache, const URI& circuitSource)
        : _cache(cache)
        , _circuitPath(fs::canonical(circuitSource.getPath()).generic_string())
    {
    }

    operator bool() const { return _cache != nullptr; }
    Strings createKeys(const URIs& uris) const
    {
        Strings keys;
        keys.reserve(uris.size());
        for (const auto& uri : uris)
        {
            keys.emplace_back(
                servus::make_uint128(uri.getPath() + "v2").getString());
        }
        return keys;
    }

    Strings createKeys(const URIs& uris,
                       const std::vector<bool>& recenter) const
    {
        Strings keys;
        keys.reserve(uris.size());
        const char* const tags[] = {"v2", "v2c"};
        for (size_t i = 0; i != uris.size(); ++i)
        {
            const auto& uri = uris[i];
            const auto tag = tags[recenter[i]];
            keys.emplace_back(
                servus::make_uint128(uri.getPath() + tag).getString());
        }
        return keys;
    }

    Strings createKeys(const URIs& uris, const GIDSet& gids) const
    {
        assert(uris.size() == gids.size());
        Strings keys;
        keys.reserve(uris.size());
        auto gid = gids.begin();
        for (const auto& uri : uris)
        {
            auto key = uri.getPath() + _circuitPath + std::to_string(*gid);
            key = servus::make_uint128(key + "v2").getString();
            keys.emplace_back(std::move(key));
            ++gid;
        }
        return keys;
    }

    void save(const std::string& uri, const std::string& key,
              neuron::MorphologyPtr morphology)
    {
        if (!_cache)
            return;

        servus::Serializable::Data data = morphology->toBinary();
        if (!_cache->insert(key, data.ptr.get(), data.size))
        {
            LBWARN << "Failed to insert morphology " << uri
                   << " into cache; item size is " << float(data.size) / LB_1MB
                   << " MB" << std::endl;
        }
    }

    CachedMorphologies load(const std::set<std::string>& keySet)
    {
        CachedMorphologies loaded;
        if (!_cache)
            return loaded;

        LBDEBUG << "Using cache for morphology loading" << std::endl;
        typedef std::future<std::pair<std::string, neuron::MorphologyPtr>>
            Future;
        std::vector<Future> futures;

        Strings keys(keySet.begin(), keySet.end());
        futures.reserve(keys.size());

        _cache->takeValues(keys, [&futures](const std::string& key, char* data,
                                            const size_t size) {
            futures.push_back(std::async([key, data, size] {
                neuron::MorphologyPtr morphology(
                    new neuron::Morphology(data, size));
                std::free(data);
                return std::make_pair(key, morphology);
            }));
        });

        for (auto& future : futures)
            loaded.insert(future.get());

        LBINFO << "Loaded " << loaded.size() << " morphologies from cache, "
               << "loading " << keySet.size() - loaded.size()
               << " remaining from file" << std::endl;
        return loaded;
    }

private:
    keyv::MapPtr _cache;
    const std::string _circuitPath;
};
#else
class MorphologyCache
{
public:
    operator bool() const { return false; }
};
#endif

class Circuit::Impl
{
public:
    URI _source;

    virtual ~Impl() {}
    virtual size_t getNumNeurons() const = 0;
    virtual GIDSet getGIDs() const = 0;
    virtual GIDSet getGIDs(const std::string& target) const = 0;

    GIDSet getRandomGIDs(const float fraction, const std::string& target,
                         const size_t* seed = nullptr) const
    {
        if (fraction < 0.f || fraction > 1.f)
            LBTHROW(
                std::runtime_error("Fraction for getRandomGIDs() must be "
                                   "in the range [0,1]"));

        const GIDSet& gids = target.empty() ? getGIDs() : getGIDs(target);
        uint32_ts randomGids(gids.begin(), gids.end());
        _shuffle(randomGids, seed);
        randomGids.resize(size_t(std::ceil(randomGids.size() * fraction)));
        return GIDSet(randomGids.begin(), randomGids.end());
    }

    virtual Vector3fs getPositions(const GIDSet& gids) const = 0;
    virtual size_ts getMTypes(const GIDSet& gids) const = 0;
    virtual Strings getMorphologyTypeNames() const = 0;
    virtual size_ts getETypes(const GIDSet& gids) const = 0;
    virtual Strings getElectrophysiologyTypeNames() const = 0;
    virtual Quaternionfs getRotations(const GIDSet& gids) const = 0;
    virtual Strings getMorphologyNames(const GIDSet& gids) const = 0;
    virtual std::vector<bool> getRecenter(const GIDSet&) const
    {
        return std::vector<bool>();
    }

    URI getMorphologyURI(const std::string& name) const
    {
        if (boost::filesystem::path(name).is_absolute())
            return URI(name);

        URI uri(getMorphologySource());
        const auto h5 = uri.getPath() + "/" + name + ".h5";
        if (!fs::exists(fs::path(h5)))
        {
            const auto swc = uri.getPath() + "/" + name + ".swc";
            if (fs::exists(fs::path(swc)))
            {
                uri.setPath(swc);
                return uri;
            }
        }
        uri.setPath(h5);
        return uri;
    }

    virtual URI getMorphologySource() const = 0;

    virtual MorphologyCache* getMorphologyCache() const { return nullptr; }
    virtual SynapseCache* getSynapseCache() const { return nullptr; }
    virtual const brion::SynapseSummary& getSynapseSummary() const = 0;
    virtual const brion::Synapse& getSynapseAttributes(
        const bool afferent) const = 0;
    virtual const brion::Synapse& getAfferentProjectionAttributes(
        const std::string& name) const = 0;
    virtual const brion::Synapse* getSynapseExtra() const = 0;
    virtual const brion::Synapse& getSynapsePositions(
        const bool afferent) const = 0;
};

class SonataCircuit : public Circuit::Impl
{
public:
    explicit SonataCircuit(const URI& uri)
        : config(uri)
    {
        const auto& subnetworks = config.getNodes();
        const auto& node = subnetworks.front();
        nodes.reset(new brion::Nodes(URI(node.elements)));

        if (subnetworks.size() > 1)
        {
            LBWARN << "More than one subnetwork found, ignoring extra ones."
                   << std::endl;
        }

        const auto populations = nodes->getPopulationNames();
        if (populations.size() > 1)
        {
            LBWARN << "More than one population found, ignoring extra ones."
                   << std::endl;
        }

        population = populations.front();
        const auto nodeGroupIDs = nodes->getNodeGroupIDs(population);
        const auto nodeIDs = nodes->getNodeIDs(population);
        const auto nodeGroupIndices = nodes->getNodeGroupIndices(population);
        const auto originalNodeTypeIDs = nodes->getNodeTypes(population);
        const size_t numNodes = nodeIDs.size();

        for (size_t i = 0; i < numNodes; i++)
        {
            if (nodeGroupIDs[i] != 0)
            {
                LBWARN << "More than one group ID found, ignoring extra ones."
                       << std::endl;
                break;
            }
        }

        size_t expectedIdx = 0;
        for (size_t i = 0; i < numNodes; i++)
        {
            const auto groupID = nodeGroupIDs[i];
            const auto nodeGroupIndex = nodeGroupIndices[i];

            if (groupID != 0)
                continue;

            if (expectedIdx != nodeGroupIndex)
            {
                LBTHROW(
                    std::runtime_error("Expected node group index '" +
                                       std::to_string(expectedIdx) + "' got '" +
                                       std::to_string(nodeGroupIndex) + "'"));
            }
            expectedIdx++;
            nodeTypeIDs.push_back(originalNodeTypeIDs[i]);
        }

        numNeurons = expectedIdx;
        morphologySource = URI(config.getComponentPath("morphologies_dir"));
        nodeGroup = nodes->openGroup(population, 0);
        nodeTypesFile.reset(new brion::CsvConfig(URI(node.types)));

        for (const auto name : nodeGroup.getAttributeNames())
            nodeAttributes.insert(name);
        for (const auto name : nodeTypesFile->getProperties())
            nodeTypeProperties.insert(name);
    }
    virtual ~SonataCircuit() {}
    size_t getNumNeurons() const final { return numNeurons; }
    GIDSet getGIDs() const
    {
        brain::GIDSet gids;
        brain::GIDSet::const_iterator hint = gids.begin();
        for (uint32_t i = 0; i < getNumNeurons(); ++i)
            // Note that GIDs start at 0 in this case
            hint = gids.insert(hint, i);
        return gids;
    }

    GIDSet getGIDs(const std::string& /*target*/) const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }

    Vector3fs getPositions(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Vector3fs();

        const size_t startIdx = *gids.begin();
        const size_t endIdx = *gids.rbegin() + 1;

        const auto attributeX =
            nodeGroup.getAttribute<float>("x", startIdx, endIdx);
        const auto attributeY =
            nodeGroup.getAttribute<float>("y", startIdx, endIdx);
        const auto attributeZ =
            nodeGroup.getAttribute<float>("z", startIdx, endIdx);

        Vector3fs output;
        for (auto gid : gids)
        {
            const float x = attributeX[gid - startIdx];
            const float y = attributeY[gid - startIdx];
            const float z = attributeZ[gid - startIdx];

            output.push_back(Vector3f(x, y, z));
        }
        return output;
    }
    size_ts getMTypes(const GIDSet& /*gids*/) const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    Strings getMorphologyTypeNames() const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    size_ts getETypes(const GIDSet& /*gids*/) const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    Strings getElectrophysiologyTypeNames() const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    Quaternionfs getRotations(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Quaternionfs();

        const size_t startIdx = *gids.begin();
        const size_t endIdx = *gids.rbegin() + 1;

        auto getter = [this](const char* attr, size_t start, size_t end) {
            try
            {
                HighFive::SilenceHDF5 silence;
                return nodeGroup.getAttribute<float>(attr, start, end);
            }
            catch (const HighFive::Exception& e)
            {
                return std::vector<float>(end - start, 0);
            }
        };
        std::vector<float> rotationAngleX =
            getter("rotation_angle_xaxis", startIdx, endIdx);
        std::vector<float> rotationAngleY =
            getter("rotation_angle_yaxis", startIdx, endIdx);
        std::vector<float> rotationAngleZ =
            getter("rotation_angle_zaxis", startIdx, endIdx);

        Quaternionfs output;

        for (const auto gid : gids)
        {
            const float rX = rotationAngleX[gid - startIdx];
            const float rY = rotationAngleY[gid - startIdx];
            const float rZ = rotationAngleZ[gid - startIdx];

            // Special case for missing rotation angles.
            if (rX == 0 && rY == 0 && rZ == 0)
            {
                output.push_back(Quaternionf());
                continue;
            }

            const float cX = std::cos(rX), cY = std::cos(rY), cZ = std::cos(rZ);
            const float sX = std::sin(rX), sY = std::sin(rY), sZ = std::sin(rZ);

            // These are the values given by multiplying the rotation
            // matrices for R(X)R(Y)R(Z) i.e. extrinsic rotation around Z
            // then Y then X
            vmml::Matrix3f mtx;

            mtx(0, 0) = cY * cZ;
            mtx(0, 1) = -cY * sZ;
            mtx(0, 2) = sY;
            mtx(1, 0) = cZ * sX * sY + cX * sZ;
            mtx(1, 1) = cX * cZ - sX * sY * sZ;
            mtx(1, 2) = -cY * sX;
            mtx(2, 0) = sX * sZ - cX * cZ * sY;
            mtx(2, 1) = cZ * sX + cX * sY * sZ;
            mtx(2, 2) = cX * cY;

            output.push_back(Quaternionf(mtx));
        }

        return output;
    }
    Strings getMorphologyNames(const GIDSet& gids) const final
    {
        return getAttribute<std::string>("morphology", gids);
    }

    std::vector<bool> getRecenter(const GIDSet& gids) const final
    {
        if (nodeAttributes.find("recenter") != nodeAttributes.end() ||
            nodeTypeProperties.find("recenter") != nodeTypeProperties.end())
        {
            return getAttribute<bool>("recenter", gids);
        }
        return std::vector<bool>(gids.size(), true);
    }

    template <typename T>
    std::vector<T> getAttribute(const std::string& name,
                                const GIDSet& gids) const
    {
        std::vector<T> output;
        if (gids.empty())
            return output;

        const size_t startIdx = *gids.begin();
        const size_t endIdx = *gids.rbegin() + 1;

        if (nodeAttributes.find(name) != nodeAttributes.end())
        {
            // Looking in the node group
            const auto data =
                _getAttribute<T>(nodeGroup, name, startIdx, endIdx);
            for (const auto gid : gids)
                output.push_back(data[gid - startIdx]);
        }
        else if (nodeTypeProperties.find(name) != nodeTypeProperties.end())
        {
            // Looking in the node types
            for (const auto gid : gids)
            {
                const auto nodeTypeID = nodeTypeIDs[gid];
                const auto value = nodeTypesFile->getProperty(nodeTypeID, name);
                try
                {
                    output.push_back(lexical_cast<T>(value));
                }
                catch (const boost::bad_lexical_cast&)
                {
                    throw std::runtime_error("Error converting attribute " +
                                             name + " for node type ID " +
                                             std::to_string(nodeTypeID));
                }
            }
        }
        else
        {
            throw std::runtime_error("Invalid node attribute: " + name);
        }
        return output;
    }

    URI getMorphologySource() const final { return morphologySource; }
    const brion::SynapseSummary& getSynapseSummary() const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    const brion::Synapse& getSynapseAttributes(const bool) const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    const brion::Synapse& getAfferentProjectionAttributes(
        const std::string&) const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    const brion::Synapse* getSynapseExtra() const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }
    const brion::Synapse& getSynapsePositions(const bool) const final
    {
        LBTHROW(std::runtime_error("Unimplemented"));
    }

    brion::CircuitConfig config;

    std::unique_ptr<brion::Nodes> nodes;
    brion::NodeGroup nodeGroup;
    std::unique_ptr<brion::CsvConfig> nodeTypesFile;
    size_t numNeurons = 0;
    URI morphologySource;
    std::string population;

    // Name caches to speed up looking for attributes in the node group or
    // the node types CSV
    std::unordered_set<std::string> nodeAttributes;
    std::unordered_set<std::string> nodeTypeProperties;

    uint32_ts nodeTypeIDs; // GID to node type ID mapping

    // Unimplemented
    URI synapseSource;
    URI circuitSource;

    std::unique_ptr<brion::Synapse> synapse;
    std::unique_ptr<brion::SynapseSummary> synapseSummary;
};

class BBPCircuit : public Circuit::Impl
{
public:
    explicit BBPCircuit(const brion::BlueConfig& config)
        : _morphologySource(config.getMorphologySource())
        , _synapseSource(config.getSynapseSource())
        , _targets(config)
#ifdef BRAIN_USE_KEYV
        , _cache(keyv::Map::createCache())
        , _morphologyCache(_cache, config.getCircuitSource())
        , _synapseCache(_cache, _synapseSource)
#endif
    {
        for (auto&& projection :
             config.getSectionNames(brion::CONFIGSECTION_PROJECTION))
        {
            _afferentProjectionSources[projection] =
                config.getProjectionSource(projection);
        }
    }

    GIDSet getGIDs() const
    {
        brain::GIDSet gids;
        brain::GIDSet::const_iterator hint = gids.begin();
        for (uint32_t i = 0; i < getNumNeurons(); ++i)
            hint = gids.insert(hint, i + 1);
        return gids;
    }

    GIDSet getGIDs(const std::string& target) const final
    {
        return _targets.getGIDs(target);
    }

    URI getMorphologySource() const final { return _morphologySource; }
    MorphologyCache* getMorphologyCache() const final
    {
        if (_morphologyCache)
            return &_morphologyCache;
        return nullptr;
    }

    SynapseCache* getSynapseCache() const final
    {
        if (_synapseCache)
            return &_synapseCache;
        return nullptr;
    }

    const brion::SynapseSummary& getSynapseSummary() const final
    {
        lunchbox::ScopedWrite mutex(_synapseSummary);

        if (!(*_synapseSummary))
            _synapseSummary->reset(new brion::SynapseSummary(
                _synapseSource.getPath() + summaryFilename));
        return **_synapseSummary;
    }

    const brion::Synapse& getSynapseAttributes(const bool afferent) const final
    {
        const size_t i = afferent ? 0 : 1;
        lunchbox::ScopedWrite mutex(_synapseAttributes[i]);

        if (!(*_synapseAttributes[i]))
            _synapseAttributes[i]->reset(new brion::Synapse(
                _synapseSource.getPath() +
                (afferent ? afferentFilename : efferentFilename)));
        return **_synapseAttributes[i];
    }

    const brion::Synapse& getAfferentProjectionAttributes(
        const std::string& name) const final
    {
        auto& lockable = _externalAfferents[name];
        lunchbox::ScopedWrite mutex(lockable);
        auto& synapses = *lockable;
        if (!synapses)
        {
            auto&& source = _afferentProjectionSources.find(name);
            if (source == _afferentProjectionSources.end())
            {
                _externalAfferents.erase(name);
                LBTHROW(std::runtime_error(
                    "Afferent synaptic projection not found: " + name));
            }
            fs::path path(source->second.getPath() + externalAfferentFilename);
            if (fs::exists(path) && fs::is_regular_file(fs::canonical(path)))
                synapses.reset(new brion::Synapse(path.string()));
            else
                // Trying with the afferent synapses filename as a fallback
                synapses.reset(new brion::Synapse(source->second.getPath() +
                                                  afferentFilename));
        }
        return *synapses;
    }

    const brion::Synapse* getSynapseExtra() const final
    {
        lunchbox::ScopedWrite mutex(_synapseExtra);

        if (!(*_synapseExtra))
        {
            try
            {
                _synapseExtra->reset(new brion::Synapse(
                    _synapseSource.getPath() + extraFilename));
            }
            catch (...)
            {
                return nullptr;
            }
        }
        return _synapseExtra->get();
    }

    const brion::Synapse& getSynapsePositions(const bool afferent) const final
    {
        const size_t i = afferent ? 0 : 1;
        lunchbox::ScopedWrite mutex(_synapsePositions[i]);

        auto& positions = *_synapsePositions[i];
        if (!positions)
            positions.reset(
                new brion::Synapse(_synapseSource.getPath() +
                                   (afferent ? afferentPositionsFilename
                                             : efferentPositionsFilename)));
        return *positions;
    }

    const brion::URI _morphologySource;
    const brion::URI _synapseSource;
    std::unordered_map<std::string, brion::URI> _afferentProjectionSources;
    Targets _targets;
#ifdef BRAIN_USE_KEYV
    keyv::MapPtr _cache;
#endif
    mutable MorphologyCache _morphologyCache;
    mutable SynapseCache _synapseCache;

    template <typename T>
    using LockPtr = lunchbox::Lockable<std::unique_ptr<T>>;

    mutable LockPtr<brion::SynapseSummary> _synapseSummary;
    mutable LockPtr<brion::Synapse> _synapseAttributes[2];
    mutable LockPtr<brion::Synapse> _synapseExtra;
    mutable LockPtr<brion::Synapse> _synapsePositions[2];

    mutable std::unordered_map<std::string, LockPtr<brion::Synapse>>
        _externalAfferents;
};

class MVD2 : public BBPCircuit
{
public:
    MVD2(const brion::BlueConfig& config)
        : BBPCircuit(config)
        , _circuit(config.getCircuitSource().getPath())
    {
    }

    size_t getNumNeurons() const final { return _circuit.getNumNeurons(); }
    Vector3fs getPositions(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Vector3fs();

        const brion::NeuronMatrix& data =
            _circuit.get(gids, brion::NEURON_POSITION_X |
                                   brion::NEURON_POSITION_Y |
                                   brion::NEURON_POSITION_Z);

        Vector3fs positions(gids.size());
#pragma omp parallel for
        for (size_t i = 0; i < gids.size(); ++i)
        {
            try
            {
                positions[i] = brion::Vector3f(lexical_cast<float>(data[i][0]),
                                               lexical_cast<float>(data[i][1]),
                                               lexical_cast<float>(data[i][2]));
            }
            catch (const boost::bad_lexical_cast&)
            {
                GIDSet::const_iterator gid = gids.begin();
                std::advance(gid, i);
                LBWARN << "Error parsing circuit position for gid " << *gid
                       << std::endl;
            }
        }
        return positions;
    }

    size_ts getMTypes(const GIDSet& gids) const final
    {
        if (gids.empty())
            return size_ts();

        const brion::NeuronMatrix& matrix =
            _circuit.get(gids, brion::NEURON_MTYPE);
        size_ts result(matrix.shape()[0]);

        brion::NeuronMatrix::const_array_view<1>::type view =
            matrix[boost::indices[brion::NeuronMatrix::index_range()][0]];
        std::transform(view.begin(), view.end(), result.begin(),
                       [](const std::string& x) { return std::stoul(x); });
        return result;
    }

    Strings getMorphologyTypeNames() const final
    {
        return _circuit.getTypes(brion::NEURONCLASS_MTYPE);
    }

    size_ts getETypes(const GIDSet& gids) const final
    {
        if (gids.empty())
            return size_ts();

        const brion::NeuronMatrix& matrix =
            _circuit.get(gids, brion::NEURON_ETYPE);
        size_ts result(matrix.shape()[0]);

        brion::NeuronMatrix::const_array_view<1>::type view =
            matrix[boost::indices[brion::NeuronMatrix::index_range()][0]];
        std::transform(view.begin(), view.end(), result.begin(),
                       [](const std::string& x) { return std::stoul(x); });
        return result;
    }

    Strings getElectrophysiologyTypeNames() const final
    {
        return _circuit.getTypes(brion::NEURONCLASS_ETYPE);
    }

    Quaternionfs getRotations(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Quaternionfs();

        const float deg2rad = float(M_PI) / 180.f;
        const brion::NeuronMatrix& data =
            _circuit.get(gids, brion::NEURON_ROTATION);
        Quaternionfs rotations(gids.size());

#pragma omp parallel for
        for (size_t i = 0; i < gids.size(); ++i)
        {
            try
            {
                // transform rotation Y angle in degree into rotation quaternion
                const float angle = lexical_cast<float>(data[i][0]) * deg2rad;
                rotations[i] = Quaternionf(angle, Vector3f(0, 1, 0));
            }
            catch (const boost::bad_lexical_cast&)
            {
                GIDSet::const_iterator gid = gids.begin();
                std::advance(gid, i);
                LBWARN << "Error parsing circuit orientation for gid " << *gid
                       << std::endl;
            }
        }
        return rotations;
    }

    Strings getMorphologyNames(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Strings();

        const brion::NeuronMatrix& matrix =
            _circuit.get(gids, brion::NEURON_MORPHOLOGY_NAME);
        Strings result;
        result.reserve(matrix.shape()[0]);

        brion::NeuronMatrix::const_array_view<1>::type view =
            matrix[boost::indices[brion::NeuronMatrix::index_range()][0]];
        for (const auto& name : view)
            result.push_back(name);
        return result;
    }

private:
    brion::Circuit _circuit;
};

#ifdef BRAIN_USE_MVD3
struct MVD3 : public BBPCircuit
{
    MVD3(const brion::BlueConfig& config)
        : BBPCircuit(config)
        , _circuit(config.getCircuitSource().getPath())
    {
    }

    size_t getNumNeurons() const final { return _circuit.getNbNeuron(); }
    Vector3fs getPositions(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Vector3fs();

        Vector3fs results(gids.size());
        const ::MVD3::Range& range = _getRange(gids);
        try
        {
            std::lock_guard<std::mutex> lock(brion::detail::hdf5Mutex());
            HighFive::SilenceHDF5 silence;
            const ::MVD3::Positions& positions = _circuit.getPositions(range);
            _assign(range, gids, positions, results, _toVector3f);
            return results;
        }
        catch (const HighFive::Exception& e)
        {
            LBTHROW(std::runtime_error("Exception in getPositions(): " +
                                       std::string(e.what())));
        }
    }

    size_ts getMTypes(const GIDSet& gids) const final
    {
        if (gids.empty())
            return size_ts();

        size_ts results(gids.size());
        const ::MVD3::Range& range = _getRange(gids);
        try
        {
            std::lock_guard<std::mutex> lock(brion::detail::hdf5Mutex());
            HighFive::SilenceHDF5 silence;
            const size_ts& mtypes = _circuit.getIndexMtypes(range);
            _assign(range, gids, mtypes, results, _nop<size_t>);
            return results;
        }
        catch (const HighFive::Exception& e)
        {
            LBTHROW(std::runtime_error("Exception in getMTypes(): " +
                                       std::string(e.what())));
        }
    }

    Strings getMorphologyTypeNames() const final
    {
        std::lock_guard<std::mutex> lock(brion::detail::hdf5Mutex());
        return _circuit.listAllMtypes();
    }

    size_ts getETypes(const GIDSet& gids) const final
    {
        if (gids.empty())
            return size_ts();

        size_ts results(gids.size());
        const ::MVD3::Range& range = _getRange(gids);
        try
        {
            std::lock_guard<std::mutex> lock(brion::detail::hdf5Mutex());
            HighFive::SilenceHDF5 silence;
            const size_ts& etypes = _circuit.getIndexEtypes(range);
            _assign(range, gids, etypes, results, _nop<size_t>);
            return results;
        }
        catch (const HighFive::Exception& e)
        {
            LBTHROW(std::runtime_error("Exception in getETypes(): " +
                                       std::string(e.what())));
        }
    }

    Strings getElectrophysiologyTypeNames() const final
    {
        std::lock_guard<std::mutex> lock(brion::detail::hdf5Mutex());
        return _circuit.listAllEtypes();
    }

    Quaternionfs getRotations(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Quaternionfs();

        Quaternionfs results(gids.size());
        const ::MVD3::Range& range = _getRange(gids);
        try
        {
            std::lock_guard<std::mutex> lock(brion::detail::hdf5Mutex());
            HighFive::SilenceHDF5 silence;
            const ::MVD3::Rotations& rotations = _circuit.getRotations(range);
            _assign(range, gids, rotations, results, _toQuaternion);
            return results;
        }
        catch (const HighFive::Exception& e)
        {
            LBTHROW(std::runtime_error("Exception in getRotations(): " +
                                       std::string(e.what())));
        }
    }

    Strings getMorphologyNames(const GIDSet& gids) const final
    {
        if (gids.empty())
            return Strings();

        Strings results(gids.size());
        const ::MVD3::Range& range = _getRange(gids);
        try
        {
            std::lock_guard<std::mutex> lock(brion::detail::hdf5Mutex());
            HighFive::SilenceHDF5 silence;
            const Strings& morphos = _circuit.getMorphologies(range);
            _assign(range, gids, morphos, results, _nop<std::string>);
            return results;
        }
        catch (const HighFive::Exception& e)
        {
            LBTHROW(std::runtime_error("Exception in getMorphologyNames(): " +
                                       std::string(e.what())));
        }
    }

private:
    ::MVD3::MVD3File _circuit;
};
#endif
} // namespace brain
