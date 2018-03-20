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

#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <streambuf>
#include <string>

#include "json.hpp"

namespace
{
nlohmann::json parseCircuitJson(const std::string& jsonStr)
{
    using nlohmann::json;

    auto jsonFlat = json::parse(jsonStr).flatten();
    std::map<std::string, std::string> variables;

    // Find variables
    for (auto it = jsonFlat.begin(); it != jsonFlat.end(); ++it)
    {
        const auto keyName = it.key();
        const auto name =
            keyName.substr(std::string(keyName).find_last_of('/') + 1);

        if ('$' == name[0])
        {
            if (variables.find(name) != variables.end())
                throw std::runtime_error("Duplicate variable `" + name + "`");

            variables[name] = it.value();
        }
    }

    { // Expand variables dependent on other variables
        bool anyChange = true;

        while (anyChange)
        {
            anyChange = false;
            auto variablesCopy = variables;

            for (const auto& vI : variables)
            {
                const auto& vIKey = vI.first;
                const auto& vIValue = vI.second;

                for (auto& vJ : variablesCopy)
                {
                    auto& vJValue = vJ.second;
                    auto startPos = vJValue.find(vIKey);

                    if (startPos != std::string::npos)
                    {
                        vJValue.replace(startPos, vIKey.length(), vIValue);
                        anyChange = true;
                    }
                }
            }

            variables = variablesCopy;
        }
    }

    // Expand variables in whole json
    for (auto it = jsonFlat.begin(); it != jsonFlat.end(); ++it)
    {
        if (!it.value().is_string())
            continue;

        auto valueStr = it.value().get<std::string>();
        auto& value = it.value();

        for (auto& var : variables)
        {
            auto& varName = var.first;
            auto& varValue = var.second;
            auto startPos = valueStr.find(varName);

            if (startPos != std::string::npos)
            {
                valueStr.replace(startPos, varName.length(), varValue);
                value = valueStr;
            }
        }
    }

    return jsonFlat.unflatten();
}

brion::CircuitConfig::Components fill_components(const nlohmann::json& json)
{
    brion::CircuitConfig::Components components;

    const auto comps = json["components_dir"];

    const auto tryGetValue = [&](const std::string& name) {
        return comps.find(name) != comps.end() ? comps[name] : "";
    };

    components.morphologies_dir = tryGetValue("morphologies_dir");
    components.synaptic_models_dir = tryGetValue("synaptic_models_dir");
    components.mechanisms_dir = tryGetValue("mechanisms_dir");
    components.biophysical_neuron_models_dir =
        tryGetValue("biophysical_neuron_models_dir");
    components.point_neuron_models_dir = tryGetValue("point_neuron_models_dir");
    return components;
}

std::vector<brion::CircuitConfig::NetworkNode> fill_nodes(
    const nlohmann::json& json)
{
    std::vector<brion::CircuitConfig::NetworkNode> output;

    const auto nodes = json["networks"]["nodes"];

    for (const auto& node : nodes)
    {
        brion::CircuitConfig::NetworkNode networkNode;
        networkNode.nodes_file = node["nodes_file"];
        networkNode.node_types_file = node["node_types_file"];
        output.push_back(networkNode);
    }

    return output;
}

std::vector<brion::CircuitConfig::NetworkEdge> fill_edges(
    const nlohmann::json& json)
{
    std::vector<brion::CircuitConfig::NetworkEdge> output;

    const auto edges = json["networks"]["edges"];

    for (const auto& edge : edges)
    {
        brion::CircuitConfig::NetworkEdge networkEdge;
        networkEdge.edges_file = edge["edges_file"];
        networkEdge.edge_types_file = edge["edge_types_file"];
        output.push_back(networkEdge);
    }

    return output;
}
}

namespace brion
{
struct CircuitConfig::Impl
{
    Impl(const std::string& uri)
    {
        std::ifstream file(uri);

        if (file.fail())
            throw std::runtime_error("Could not open file `" + uri + "`");

        std::string contents;

        file.seekg(0, std::ios::end);
        contents.reserve(file.tellg());
        file.seekg(0, std::ios::beg);

        contents.assign((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());

        const auto json = parseCircuitJson(contents);
        target_simulator = json["target_simulator"];
        components = fill_components(json);
        networkNodes = fill_nodes(json);
        networkEdges = fill_edges(json);
    }

    std::string target_simulator;
    CircuitConfig::Components components;
    std::vector<CircuitConfig::NetworkNode> networkNodes;
    std::vector<CircuitConfig::NetworkEdge> networkEdges;
};

CircuitConfig::CircuitConfig(const URI& uri)
    : impl(new CircuitConfig::Impl(uri.getPath()))
{
}

CircuitConfig::~CircuitConfig() = default;
CircuitConfig::CircuitConfig(CircuitConfig&&) = default;

std::string CircuitConfig::getTargetSimulator() const
{
    return impl->target_simulator;
}

CircuitConfig::Components CircuitConfig::getComponents() const
{
    return impl->components;
}

std::vector<CircuitConfig::NetworkNode> CircuitConfig::getNetworkNodes() const
{
    return impl->networkNodes;
}
std::vector<CircuitConfig::NetworkEdge> CircuitConfig::getNetworkEdges() const
{
    return impl->networkEdges;
}
}
