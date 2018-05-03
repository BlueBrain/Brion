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
    const Strings& names = _impl->getMorphologyNames(gids);

    URIs uris;
    uris.reserve(names.size());
    for (Strings::const_iterator i = names.begin(); i < names.end(); ++i)
        uris.push_back(_impl->getMorphologyURI(*i));
    return uris;
}

neuron::Morphologies Circuit::loadMorphologies(const GIDSet& gids,
                                               const Coordinates coords) const
{
    URIs uris = getMorphologyURIs(gids);
    const bool transform = coords == Coordinates::global;

    for (auto& uri : uris)
    {
        // opt: don't stat abs file path
        if (uri.getScheme() == "file" && uri.getPath()[0] != '/')
            uri.setPath(fs::canonical(uri.getPath()).generic_string());
    }
    auto cache = _impl->getMorphologyCache();

    // < GID, key >
    Strings keys;
    if (cache)
    {
        keys =
            transform ? cache->createKeys(uris, gids) : cache->createKeys(uris);
    }
    else
    {
        // No cache available, just use the uris as the keys.
        keys.reserve(uris.size());
        for (const auto& uri : uris)
            keys.push_back(uri.getPath());
    }

    CachedMorphologies cached;
    if (cache)
    {
        std::set<std::string> keySet;
        for (const auto& key : keys)
            keySet.insert(key);
        cached = cache->load(keySet);
    }

    using MorphologyUse = std::pair<brion::MorphologyPtr, size_t>;
    std::unordered_map<std::string, MorphologyUse> loading;

    // resolve missing morphologies
    for (size_t i = 0; i < uris.size(); ++i)
    {
        const auto& uri = uris[i];
        const auto& key = keys[i];

        if (!cache || cached.find(key) == cached.end())
        {
            auto& morphAndCount = loading[std::to_string(uri)];
            if (morphAndCount.second == 0)
            { // delay data access to next loop to allow async loading
                morphAndCount.first = std::make_shared<brion::Morphology>(uri);
            }
            ++morphAndCount.second;
            cached.insert(std::make_pair(keys[i], nullptr));
        }
    }

    // load and transform missing and put them in GID-order into result
    neuron::Morphologies result;
    result.reserve(uris.size());
    const auto transforms = transform ? getTransforms(gids) : Matrix4fs();

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
        size_t count = --morphAndCount.second;
        auto& original = morphAndCount.first;

        neuron::MorphologyPtr morphology;
        if (transform)
        {
            if (count == 0) // last usage, take existing instance
                morphology.reset(
                    new neuron::Morphology(original, transforms[i]));
            else // make a copy
                morphology.reset(new neuron::Morphology(
                    std::make_shared<brion::Morphology>(*original),
                    transforms[i]));
        }
        else
            // share unmodified brion data
            morphology.reset(new neuron::Morphology(original));

        if (cache)
            cache->save(uri.getPath(), key, morphology);
        value = morphology;
        result.push_back(morphology);
    }

    return result;
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
    const Vector3fs& positions = _impl->getPositions(gids);
    const Quaternionfs& rotations = _impl->getRotations(gids);
    if (positions.size() != rotations.size())
        throw std::runtime_error(
            "Positions not equal rotations for given GIDs");

    Matrix4fs transforms(positions.size());

#pragma omp parallel for
    for (size_t i = 0; i < positions.size(); ++i)
        transforms[i] = Matrix4f(rotations[i], positions[i]);
    return transforms;
}

Quaternionfs Circuit::getRotations(const GIDSet& gids) const
{
    return _impl->getRotations(gids);
}

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
