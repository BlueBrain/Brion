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
    const auto comps = json["components_dir"];
    std::map<std::string, std::string> output;

    for (auto it = comps.begin(); it != comps.end(); ++it)
        output[it.key()] = resolver.toAbsolute(it.value());

    return output;
}

std::vector<brion::CircuitConfig::SubnetworkFiles> _fillSubnetwork(
    const nlohmann::json& json, const std::string& networkType,
    const std::string& elementName, const std::string& typeName,
    const PathResolver& resolver)
{
    std::vector<brion::CircuitConfig::SubnetworkFiles> output;

    const auto nodes = json["networks"][networkType];

    for (const auto& node : nodes)
    {
        brion::CircuitConfig::SubnetworkFiles network;
        network.elements = resolver.toAbsolute(node[elementName]);
        network.types = resolver.toAbsolute(node[typeName]);
        output.push_back(network);
    }

    return output;
}
}

struct CircuitConfig::Impl
{
    Impl(const std::string& uri)
        : resolver(uri)
    {
        const auto json = parseSonataJson(uri);
        targetSimulator = json["target_simulator"];
        componentDirs = _fillComponents(json, resolver);
        networkEdges = _fillSubnetwork(json, "edges", "edges_file",
                                       "edge_types_file", resolver);
        networkNodes = _fillSubnetwork(json, "nodes", "nodes_file",
                                       "node_types_file", resolver);
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
