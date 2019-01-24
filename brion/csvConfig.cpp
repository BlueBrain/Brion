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

#include "csvConfig.h"

#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <streambuf>
#include <string>
#include <unordered_map>

namespace
{
std::vector<std::string> explode_whitespace(const std::string& str)
{
    const boost::char_separator<char> sep{" "};
    const boost::tokenizer<boost::char_separator<char>> tok{str, sep};
    std::vector<std::string> output;
    for (const auto& t : tok)
        output.emplace_back(t);
    return output;
}
}

namespace brion
{
struct CsvConfig::Impl
{
    Impl(const std::string& uri)
    {
        std::ifstream file(uri);

        if (file.fail())
            throw std::runtime_error("Could not open file `" + uri + "`");

        // The following code only works with LF and CRLF line endings.
        for (std::string line; std::getline(file, line);)
        {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.find_first_not_of(' ') == std::string::npos)
                continue;
            table.push_back(explode_whitespace(line));
        }

        // Verify layout
        const size_t numColumns = table.front().size();
        for (const auto& row : table)
        {
            if (numColumns != row.size())
                throw std::runtime_error("Number of columns inconsistent in `" +
                                         uri + "`");
        }

        // Fill access maps
        size_t ctr = 0;
        for (const auto& columnName : table.front())
        {
            nameToColumnIndex[columnName] = ctr;
            ctr++;
        }

        const auto nodeTypeIdIt = nameToColumnIndex.find("node_type_id");

        if (nodeTypeIdIt == nameToColumnIndex.end())
            throw std::runtime_error("Could not find column 'node_type_id'");

        const auto pos = nodeTypeIdIt->second;

        for (size_t i = 1; i < table.size(); i++)
        {
            const auto& nodeTypeIdStr = table[i][pos];
            const size_t nodeTypeId = std::stoi(nodeTypeIdStr);
            nodeTypeIdToRowIndex[nodeTypeId] = i;
        }
    }

    std::vector<std::vector<std::string>> table;

    std::unordered_map<size_t, size_t> nodeTypeIdToRowIndex;
    std::unordered_map<std::string, size_t> nameToColumnIndex;
};

CsvConfig::CsvConfig(const URI& uri)
    : impl(new CsvConfig::Impl(uri.getPath()))
{
}

size_ts CsvConfig::getNodeTypeIds() const
{
    size_ts output;
    for (const auto& it : impl->nodeTypeIdToRowIndex)
        output.push_back(it.first);
    return output;
}

Strings CsvConfig::getProperties() const
{
    Strings output;
    for (const auto& it : impl->nameToColumnIndex)
        output.push_back(it.first);
    output.erase(std::remove(output.begin(), output.end(), "node_type_id"),
                 output.end());
    return output;
}

const std::string& CsvConfig::getProperty(const size_t nodeTypeId,
                                          const std::string& property) const
{
    auto itRow = impl->nodeTypeIdToRowIndex.find(nodeTypeId);
    auto itCol = impl->nameToColumnIndex.find(property);

    if (itRow == impl->nodeTypeIdToRowIndex.end())
        throw std::runtime_error("Node type id not found: `" +
                                 std::to_string(nodeTypeId) + "`");

    if (itCol == impl->nameToColumnIndex.end())
        throw std::runtime_error("Property not found: `" + property + "`");

    return impl->table[itRow->second][itCol->second];
}

CsvConfig::~CsvConfig() = default;
CsvConfig::CsvConfig(CsvConfig&&) = default;
}
