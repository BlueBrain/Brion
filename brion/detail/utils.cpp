/* Copyright (c) 2018, EPFL/Blue Brain Project
 *                     Jonas Karlsson <jonas.karlsson@epfl.ch>
 *                     Juan Hernando <juan.hernando@epfl.ch>
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

#include "utils.h"

#include <boost/filesystem.hpp>
#include <boost/version.hpp>

#include <fstream>
#include <iostream>
#include <regex>
#include <streambuf>

namespace brion
{
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

std::map<std::string, std::string> _replaceVariables(
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

nlohmann::json _expandVariables(const nlohmann::json& json,
                                const std::map<std::string, std::string>& vars)
{
    auto jsonFlat = json.flatten();

    // Expand variables in whole json
    for (auto it = jsonFlat.begin(); it != jsonFlat.end(); ++it)
    {
        if (!it.value().is_string())
            continue;

        auto valueStr = it.value().get<std::string>();
        auto& value = it.value();

        for (auto& var : vars)
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
} // namespace

nlohmann::json parseSonataJson(const std::string& uri)
{
    // Reading the input file into a string
    std::ifstream file(uri);
    if (file.fail())
        throw std::runtime_error("Could not open file `" + uri + "`");

    // Parsing
    const auto json = nlohmann::json::parse(file);

    // Parsing manifest and expanding all variables
    const auto vars = _replaceVariables(_readVariables(json));
    return _expandVariables(json, vars);
}

PathResolver::PathResolver(const std::string& basePath)
    : _basePath(boost::filesystem::path(basePath).parent_path())
{
}

std::string PathResolver::toAbsolute(const std::string& pathStr) const
{
    const boost::filesystem::path path(pathStr);
    const auto absolute = path.is_absolute()
                              ? path
                              : boost::filesystem::absolute(path, _basePath);
#if BOOST_VERSION / 100 >= 1060
    return absolute.lexically_normal().string();
#else
    // https://stackoverflow.com/questions/1746136/how-do-i-normalize-a-pathname-using-boostfilesystem
    boost::filesystem::path result;
    for (boost::filesystem::path::iterator it = absolute.begin();
         it != absolute.end(); ++it)
    {
        if (*it == "..")
        {
            // /a/b/.. is not necessarily /a if b is a symbolic link
            if (boost::filesystem::is_symlink(result))
                result /= *it;
            // /a/b/../.. is not /a/b/.. under most circumstances
            // We can end up with ..s in our result because of symbolic links
            else if (result.filename() == "..")
                result /= *it;
            // Otherwise it should be safe to resolve the parent
            else
                result = result.parent_path();
        }
        else if (*it == ".")
        {
            // Ignore
        }
        else
        {
            // Just cat other path entries
            result /= *it;
        }
    }

    return result.string();
#endif
}
} // namespace brion
