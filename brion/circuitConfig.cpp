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

std::map<std::string, std::string> fill_components(const nlohmann::json& json)
{
    const auto comps = json["components_dir"];
    std::map<std::string, std::string> output;

    for (auto it = comps.begin(); it != comps.end(); ++it)
        output[it.key()] = it.value();

    return output;
}

std::vector<brion::CircuitConfig::SubnetworkFiles> fill_subnetwork(
    const nlohmann::json& json, const std::string& network_type,
    const std::string& element_name, const std::string& type_name)
{
    std::vector<brion::CircuitConfig::SubnetworkFiles> output;

    const auto nodes = json["networks"][network_type];

    for (const auto& node : nodes)
    {
        brion::CircuitConfig::SubnetworkFiles network;
        network.elements = node[element_name];
        network.types = node[type_name];
        output.push_back(network);
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
        component_dirs = fill_components(json);
        networkEdges =
            fill_subnetwork(json, "edges", "edges_file", "edge_types_file");
        networkNodes =
            fill_subnetwork(json, "nodes", "nodes_file", "node_types_file");
    }

    std::string target_simulator;
    std::map<std::string, std::string> component_dirs;
    std::vector<CircuitConfig::SubnetworkFiles> networkNodes;
    std::vector<CircuitConfig::SubnetworkFiles> networkEdges;
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

std::string CircuitConfig::getComponentPath(const std::string& name) const
{
    const auto it = impl->component_dirs.find(name);
    if (it == impl->component_dirs.end())
        throw std::runtime_error("Could not find component '" + name + "'");

    return it->second;
}

std::vector<CircuitConfig::SubnetworkFiles> CircuitConfig::getNodes() const
{
    return impl->networkNodes;
}
std::vector<CircuitConfig::SubnetworkFiles> CircuitConfig::getEdges() const
{
    return impl->networkEdges;
}
}
