/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Jonas Karlsson <jonas.karlsson@epfl.ch>
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

#include "circuitConfig.h"

#include "detail/utils.h"

namespace brion
{
namespace
{
std::map<std::string, std::string> _fillComponents(const nlohmann::json& json,
                                                   const PathResolver& resolver)
{
    const auto comps = json.at("components");
    std::map<std::string, std::string> output;

    for (auto it = comps.begin(); it != comps.end(); ++it)
        output[it.key()] = resolver.toAbsolute(it.value());

    return output;
}

std::vector<brion::CircuitConfig::SubnetworkFiles> _fillSubnetwork(
    nlohmann::json::reference networks, const std::string& prefix,
    const PathResolver& resolver)
{
    std::vector<brion::CircuitConfig::SubnetworkFiles> output;

    const std::string component = prefix + "s";
    const std::string elementsFile = prefix + "s_file";
    const std::string typesFile = prefix + "_types_file";

    auto iter = networks.find(component);
    if (iter == networks.end())
        return output;

    for (const auto& node : *iter)
        output.emplace_back(brion::CircuitConfig::SubnetworkFiles{
            resolver.toAbsolute(node.at(elementsFile)),
            resolver.toAbsolute(node.at(typesFile))});

    return output;
}
}

struct CircuitConfig::Impl
{
    Impl(const std::string& uri)
        : resolver(uri)
    {
        const auto json = parseSonataJson(uri);
        try
        {
            targetSimulator = json.at("target_simulator");
        }
        catch (nlohmann::detail::out_of_range&)
        {
        }

        auto networks = json.at("networks");
        componentDirs = _fillComponents(json, resolver);
        networkEdges = _fillSubnetwork(networks, "edge", resolver);
        networkNodes = _fillSubnetwork(networks, "node", resolver);
    }

    std::string targetSimulator;
    std::map<std::string, std::string> componentDirs;
    std::vector<CircuitConfig::SubnetworkFiles> networkNodes;
    std::vector<CircuitConfig::SubnetworkFiles> networkEdges;
    PathResolver resolver;
};

CircuitConfig::CircuitConfig(const URI& uri)
    : impl(new CircuitConfig::Impl(uri.getPath()))
{
}

CircuitConfig::~CircuitConfig() = default;
CircuitConfig::CircuitConfig(CircuitConfig&&) = default;

std::string CircuitConfig::getTargetSimulator() const
{
    return impl->targetSimulator;
}

std::string CircuitConfig::getComponentPath(const std::string& name) const
{
    const auto it = impl->componentDirs.find(name);
    if (it == impl->componentDirs.end())
        throw std::runtime_error("Could not find component '" + name + "'");

    return it->second;
}

const std::vector<CircuitConfig::SubnetworkFiles>& CircuitConfig::getNodes()
    const
{
    return impl->networkNodes;
}
const std::vector<CircuitConfig::SubnetworkFiles>& CircuitConfig::getEdges()
    const
{
    return impl->networkEdges;
}
}
