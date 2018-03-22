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

#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <streambuf>
#include <string>
#include <unordered_map>

namespace
{
// Taken from:
// https://stackoverflow.com/questions/890164/how-can-i-split-a-string-by-a-delimiter-into-an-array
std::vector<std::string> explode(const std::string& str, const char& ch)
{
    std::string next;
    std::vector<std::string> result;

    // For each character in the string
    for (auto it = str.begin(); it != str.end(); it++)
    {
        // If we've hit the terminal character
        if (*it == ch)
        {
            // If we have some characters accumulated
            if (!next.empty())
            {
                // Add them to the result vector
                result.push_back(next);
                next.clear();
            }
        }
        else
        {
            // Accumulate the next character into the sequence
            next += *it;
        }
    }
    if (!next.empty())
        result.push_back(next);
    return result;
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

        std::string contents;

        file.seekg(0, std::ios::end);
        contents.reserve(file.tellg());
        file.seekg(0, std::ios::beg);

        contents.assign((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());

        const auto lines = explode(contents, '\n');
        for (const auto& line : lines)
            table.push_back(explode(line, ' '));

        { // Verify layout
            const size_t numColumns = table.front().size();
            for (const auto& row : table)
            {
                if (numColumns != row.size())
                    throw std::runtime_error(
                        "Number of columns inconsistent in `" + uri + "`");
            }
        }

        { // Fill access maps
            size_t ctr = 0;
            for (const auto& columnName : table.front())
            {
                nameToColumnIndex[columnName] = ctr;
                ctr++;
            }

            for (size_t i = 1; i < table.size(); i++)
            {
                const auto& nodeTypeIdStr = table[i][0];
                const size_t nodeTypeId = std::stoi(nodeTypeIdStr);
                nodeTypeIdToRowIndex[nodeTypeId] = i;
            }
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

const std::string& CsvConfig::getProperty(const size_t nodeTypeId,
                                          const std::string& property) const
{
    const size_t rowIndex = impl->nodeTypeIdToRowIndex.at(nodeTypeId);
    const size_t columnIndex = impl->nameToColumnIndex.at(property);

    return impl->table[rowIndex][columnIndex];
}

CsvConfig::~CsvConfig() = default;
CsvConfig::CsvConfig(CsvConfig&&) = default;
}
