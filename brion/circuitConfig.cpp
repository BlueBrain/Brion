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
std::map<std::string, std::string> _readVariables(const nlohmann::json& json)
{
    auto manifest = json["manifest"];

    std::map<std::string, std::string> variables;

    const std::regex regexVariable("\\$[a-zA-Z0-9_]*");

    // Find variables in manifest section
    for (auto it = manifest.begin(); it != manifest.end(); ++it)
    {
        const auto name = it.key();

        if (std::regex_match(name, regexVariable))
        {
            if (variables.find(name) != variables.end())
                throw std::runtime_error("Duplicate variable `" + name + "`");

            variables[name] = it.value();
        }
        else
        {
            throw std::runtime_error("Invalid variable name `" + name + "`");
        }
    }

    return variables;
}

std::map<std::string, std::string> _resolveVariables(
    std::map<std::string, std::string> variables)
{
    bool anyChange = true;
    constexpr size_t maxIterations = 5;
    size_t iteration = 0;

    while (anyChange && iteration < maxIterations)
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
        iteration++;
    }

    if (iteration == maxIterations)
        throw std::runtime_error(
            "Reached maximum allowed iterations in variable expansion, "
            "possibly infinite recursion.");

    return variables;
}

nlohmann::json _expandVariables(
    const nlohmann::json& json,
    const std::map<std::string, std::string>& variables)
{
    auto jsonFlat = json.flatten();

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

nlohmann::json _parseCircuitJson(const std::string& jsonStr)
{
    const auto json = nlohmann::json::parse(jsonStr);

    auto variables = _readVariables(json);
    variables = _resolveVariables(variables);
    return _expandVariables(json, variables);
}

std::map<std::string, std::string> _fillComponents(const nlohmann::json& json)
{
    const auto comps = json["components_dir"];
    std::map<std::string, std::string> output;

    for (auto it = comps.begin(); it != comps.end(); ++it)
        output[it.key()] = it.value();

    return output;
}

std::vector<brion::CircuitConfig::SubnetworkFiles> _fillSubnetwork(
    const nlohmann::json& json, const std::string& networkType,
    const std::string& elementName, const std::string& typeName)
{
    std::vector<brion::CircuitConfig::SubnetworkFiles> output;

    const auto nodes = json["networks"][networkType];

    for (const auto& node : nodes)
    {
        brion::CircuitConfig::SubnetworkFiles network;
        network.elements = node[elementName];
        network.types = node[typeName];
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

        const auto json = _parseCircuitJson(contents);
        targetSimulator = json["target_simulator"];
        componentDirs = _fillComponents(json);
        networkEdges =
            _fillSubnetwork(json, "edges", "edges_file", "edge_types_file");
        networkNodes =
            _fillSubnetwork(json, "nodes", "nodes_file", "node_types_file");
    }

    std::string targetSimulator;
    std::map<std::string, std::string> componentDirs;
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
