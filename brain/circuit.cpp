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

#include "circuit.h"
#include "detail/circuit.h"

#include "synapsesStream.h"
#include <boost/algorithm/string.hpp>

namespace brain
{
namespace
{
URIs _getMorphologyURIs(const Circuit::Impl& circuit, const GIDSet& gids)
{
    const Strings& names = circuit.getMorphologyNames(gids);

    URIs uris;
    uris.reserve(names.size());
    for (const auto& name : names)
        uris.push_back(circuit.getMorphologyURI(name));
    return uris;
}

Matrix4fs _getTransforms(const Circuit::Impl& circuit, const GIDSet& gids)
{
    const Vector3fs& positions = circuit.getPositions(gids);
    const Quaternionfs& rotations = circuit.getRotations(gids);
    if (positions.size() != rotations.size())
        throw std::runtime_error(
            "Positions not equal rotations for given GIDs");

    Matrix4fs transforms(positions.size());

#pragma omp parallel for
    for (size_t i = 0; i < positions.size(); ++i)
        transforms[i] = Matrix4f(rotations[i], positions[i]);
    return transforms;
}

class MorphologyLoader
{
public:
    MorphologyLoader(const Circuit::Impl& impl)
        : _impl(impl)
        , _cache(_impl.getMorphologyCache())
    {
    }

    neuron::Morphologies load(const GIDSet& gids,
                              const Circuit::Coordinates coords)
    {
        _initRecentering(gids);

        const bool transform = coords == Circuit::Coordinates::global;

        URIs uris = _getMorphologyURIs(_impl, gids);
        for (auto& uri : uris)
        {
            // opt: don't stat abs file path
            if (uri.getScheme() == "file" && uri.getPath()[0] != '/')
                uri.setPath(fs::canonical(uri.getPath()).generic_string());
        }

        // The keys include the information about whether the morphology
        // needs recentering or not. This is only the case for local
        // morphologies.
        const Strings keys = _getKeys(gids, uris, coords);
        CachedMorphologies cached = _loadFromCache(keys);

        struct MorphologyUse
        {
            brion::MorphologyPtr original;
            brion::MorphologyPtr recentered;
            size_t numRecentered = 0;
            size_t numOriginal = 0;
        };
        std::unordered_map<std::string, MorphologyUse> loading;

        // Starting to load all those morphologies that weren't in cache
        for (size_t i = 0; i < uris.size(); ++i)
        {
            const auto& uri = uris[i];
            const auto& key = keys[i];

            if (!_cache || cached.find(key) == cached.end())
            {
                auto& morphAndCounts = loading[std::to_string(uri)];
                // Check if already loading
                if (morphAndCounts.numRecentered == 0 &&
                    morphAndCounts.numOriginal == 0)
                { // delay data access to next loop to allow async loading
                    morphAndCounts.original =
                        std::make_shared<brion::Morphology>(uri);
                }
                // Counting the cell as recentered morphology or original
                if (_recenterAll || (!_recenter.empty() && _recenter[i]))
                    ++morphAndCounts.numRecentered;
                else
                    ++morphAndCounts.numOriginal;
                // Creating a placeholder entry for this key in the cache
                cached.insert(std::make_pair(keys[i], nullptr));
            }
        }

        // Recentering loaded morphologies if necessary. Morphologies are
        // copied or recentered in place depending on the usage counters.
        for (auto& i : loading)
        {
            auto& morphologyUse = i.second;
            if (morphologyUse.numRecentered == 0)
                continue;

            auto& original = morphologyUse.original;
            auto& recentered = morphologyUse.recentered;
            if (morphologyUse.numOriginal)
                recentered = std::make_shared<brion::Morphology>(*original);
            else
                recentered = std::move(original);
            // Recentering
            _recenterMorphology(*recentered);
        }

        // load and transform missing and put them in GID-order into result
        neuron::Morphologies result;
        result.reserve(uris.size());
        const auto transforms =
            transform ? _getTransforms(_impl, gids) : Matrix4fs();

        for (size_t i = 0; i < uris.size(); ++i)
        {
            const auto& uri = uris[i];
            const auto& key = keys[i];

            auto& value = cached[key];
            if (value)
            {
                result.push_back(value);
                continue;
            }

            auto& morphAndCount = loading[std::to_string(uri)];
            const bool recenter =
                _recenterAll || (!_recenter.empty() && _recenter[i]);
            const size_t count = --(recenter ? morphAndCount.numRecentered
                                             : morphAndCount.numOriginal);
            auto& raw =
                recenter ? morphAndCount.recentered : morphAndCount.original;

            neuron::MorphologyPtr morphology;
            if (transform)
            {
                if (count == 0) // last usage, take existing instance
                    morphology.reset(
                        new neuron::Morphology(raw, transforms[i]));
                else // make a copy
                    morphology.reset(new neuron::Morphology(
                        std::make_shared<brion::Morphology>(*raw),
                        transforms[i]));
            }
            else
            {
                // share unmodified brion data
                morphology.reset(new neuron::Morphology(raw));
            }

            // Saving to the cache.
            if (_cache)
                _cache->save(uri.getPath(), key, morphology);
            // Assigning the morphology to its entry in the cache
            value = morphology;
            result.push_back(morphology);
        }

        return result;
    }

private:
    const Circuit::Impl& _impl;
    MorphologyCache* _cache;

    bool _recenterAll = false;
    std::vector<bool> _recenter;

    void _initRecentering(const GIDSet& gids)
    {
        _recenter = _impl.getRecenter(gids);
        _recenterAll = !_recenter.empty();
        if (!_recenter.empty())
        {
            for (auto i : _recenter)
                _recenterAll &= i;
        }
        if (_recenterAll)
            _recenter = std::vector<bool>();
    }

    Strings _getKeys(const GIDSet& gids, const URIs& uris,
                     const Circuit::Coordinates coords)
    {
        const bool transform = coords == Circuit::Coordinates::global;

        // < GID, key >
        Strings keys;
        if (_cache)
        {
            // In the case of recentering of morphologies is needed, the keys
            // already encode that.
            if (transform)
                keys = _cache->createKeys(uris, gids);
            else
            {
                const auto recenter = _recenterAll
                                          ? std::vector<bool>(uris.size(), true)
                                          : _recenter;
                if (!recenter.empty())
                    keys = _cache->createKeys(uris, recenter);
                else
                    keys = _cache->createKeys(uris);
            }
        }
        else
        {
            // No cache available, just use the uris as the keys.
            // A 'c' is appending to the end when some morphologies need
            // recentering and other don't
            keys.reserve(uris.size());
            if (!_recenterAll && !_recenter.empty())
            {
                size_t i = 0;
                for (const auto& uri : uris)
                {
                    auto key = uri.getPath();
                    if (_recenter[i])
                        key += "c";
                    keys.emplace_back(std::move(key));
                    ++i;
                }
            }
            else
            {
                for (const auto& uri : uris)
                    keys.emplace_back(uri.getPath());
            }
        }
        return keys;
    }

    /** Loads keys from cache and recenters morphologies when needed.
        The output morphologies are final. */
    CachedMorphologies _loadFromCache(const Strings& keys)
    {
        if (!_cache)
            return CachedMorphologies();

        std::set<std::string> keySet;
        for (const auto& key : keys)
            keySet.insert(key);
        return _cache->load(keySet);
    }

    void _recenterMorphology(brion::Morphology& morphology)
    {
        auto& points = morphology.getPoints();
        const auto& types = morphology.getSectionTypes();
        size_t somaID = 0;
        if (types[0] != brion::SECTION_SOMA)
        {
            // Looking for the soma
            for (; somaID != types.size(); ++somaID)
                if (types[somaID] == brion::SECTION_SOMA)
                    break;
            if (somaID == types.size())
                throw std::runtime_error(
                    "Error recentering morphology, no soma found");
        }

        // Finding the soma centroid
        const auto& sections = morphology.getSections();
        const size_t start = sections[somaID][0];
        const size_t end = somaID == sections.size() - 1
                               ? points.size()
                               : sections[somaID + 1][0];
        Vector3f centroid;
        for (size_t i = start; i != end; ++i)
            centroid += points[i].get_sub_vector<3, 0>();
        centroid /= float(end - start);

        // Translating all points
        if (centroid.length() < 1e-6)
            return; // Do not recenter is almost there
#pragma omp parallel for
        for (size_t i = 0; i < points.size(); ++i)
            points[i] -= centroid;
    }
};
}

Circuit::Impl* newImpl(const brion::BlueConfig& config)
{
    const std::string circuit = config.getCircuitSource().getPath();
    Circuit::Impl* out;
    if (boost::algorithm::ends_with(circuit, ".mvd2"))
        out = new MVD2(config);
    else
    {
#ifdef BRAIN_USE_MVD3

        out = new MVD3(config);
#else
        throw std::runtime_error("MVD3 not supported");
#endif
    }
    out->_source = brion::URI(config.getSource());
    return out;
}

Circuit::Impl* newImpl(const URI& source)
{
    // Check if sonata
    const auto path = source.getPath();
    Circuit::Impl* out;
    if (boost::algorithm::ends_with(path, ".json"))
        out = new SonataCircuit(source);
    else
        out = newImpl(brion::BlueConfig(source.getPath()));
    out->_source = source;
    return out;
}

Circuit::Circuit(const URI& source)
    : _impl(newImpl(source))
{
}

Circuit::Circuit(const brion::BlueConfig& config)
    : _impl(newImpl(config))
{
}

Circuit::Circuit(Circuit&& other)
    : _impl(std::move(other._impl))
{
}

Circuit::~Circuit() = default;

const URI& Circuit::getSource() const
{
    return _impl->_source;
}

GIDSet Circuit::getGIDs() const
{
    return _impl->getGIDs();
}

GIDSet Circuit::getGIDs(const std::string& target) const
{
    return _impl->getGIDs(target);
}

GIDSet Circuit::getRandomGIDs(const float fraction) const
{
    return _impl->getRandomGIDs(fraction, "");
}

GIDSet Circuit::getRandomGIDs(const float fraction,
                              const std::string& target) const
{
    return _impl->getRandomGIDs(fraction, target);
}

URIs Circuit::getMorphologyURIs(const GIDSet& gids) const
{
    return _getMorphologyURIs(*_impl, gids);
}

neuron::Morphologies Circuit::loadMorphologies(const GIDSet& gids,
                                               const Coordinates coords) const
{
    MorphologyLoader loader(*_impl);
    return loader.load(gids, coords);
}

Vector3fs Circuit::getPositions(const GIDSet& gids) const
{
    return _impl->getPositions(gids);
}

Strings Circuit::getMorphologyNames(const GIDSet& gids) const
{
    return _impl->getMorphologyNames(gids);
}

size_ts Circuit::getMorphologyTypes(const GIDSet& gids) const
{
    return _impl->getMTypes(gids);
}

Strings Circuit::getMorphologyTypeNames() const
{
    return _impl->getMorphologyTypeNames();
}

size_ts Circuit::getElectrophysiologyTypes(const GIDSet& gids) const
{
    return _impl->getETypes(gids);
}

Strings Circuit::getElectrophysiologyTypeNames() const
{
    return _impl->getElectrophysiologyTypeNames();
}

Matrix4fs Circuit::getTransforms(const GIDSet& gids) const
{
    return _getTransforms(*_impl, gids);
}

Quaternionfs Circuit::getRotations(const GIDSet& gids) const
{
    return _impl->getRotations(gids);
}

template <typename T>
std::vector<T> Circuit::getAttribute(const std::string& name,
                                     const GIDSet& gids) const
{
    // Virtual template functions are not supported by the language, since this
    // is supported only by SONATA anyway, will try a dynamic cast for
    // simplificty
    auto impl = dynamic_cast<const SonataCircuit*>(_impl.get());
    if (!impl)
        throw std::runtime_error(
            "Custom attributes are not supported by the this circuit type");
    return impl->getAttribute<T>(name, gids);
}

// Soma template instantiations
template std::vector<char> Circuit::getAttribute<char>(
    const std::string& name, const GIDSet& gids) const;

template std::vector<unsigned char> Circuit::getAttribute<unsigned char>(
    const std::string& name, const GIDSet& gids) const;

template std::vector<uint32_t> Circuit::getAttribute<uint32_t>(
    const std::string& name, const GIDSet& gids) const;
template std::vector<uint64_t> Circuit::getAttribute<uint64_t>(
    const std::string& name, const GIDSet& gids) const;
template std::vector<float> Circuit::getAttribute<float>(
    const std::string& name, const GIDSet& gids) const;
template std::vector<double> Circuit::getAttribute<double>(
    const std::string& name, const GIDSet& gids) const;
template std::vector<std::string> Circuit::getAttribute<std::string>(
    const std::string& name, const GIDSet& gids) const;

size_t Circuit::getNumNeurons() const
{
    return _impl->getNumNeurons();
}

SynapsesStream Circuit::getAfferentSynapses(
    const GIDSet& gids, const SynapsePrefetch prefetch) const
{
    return SynapsesStream(*this, gids, true, prefetch);
}

SynapsesStream Circuit::getExternalAfferentSynapses(
    const GIDSet& gids, const std::string& source,
    const SynapsePrefetch prefetch) const
{
    return SynapsesStream(*this, gids, source, prefetch);
}

SynapsesStream Circuit::getEfferentSynapses(
    const GIDSet& gids, const SynapsePrefetch prefetch) const
{
    return SynapsesStream(*this, gids, false, prefetch);
}

SynapsesStream Circuit::getProjectedSynapses(
    const GIDSet& preGIDs, const GIDSet& postGIDs,
    const SynapsePrefetch prefetch) const
{
    return SynapsesStream(*this, preGIDs, postGIDs, prefetch);
}
}
