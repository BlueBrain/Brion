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

        const size_t num_columns = table.front().size();

        {
            size_t ctr = 0;
            for (const auto& column_name : table.front())
            {
                name_to_column_index[column_name] = ctr;
                ctr++;
            }
        }

        for (size_t i = 1; i < table.size(); i++)
        {
            const auto& node_type_id_str = table[i][0];
            const size_t node_type_id = std::stoi(node_type_id_str);
            node_type_id_to_row_index[node_type_id] = i;
        }

        for (const auto& row : table)
        {
            if (num_columns != row.size())
                throw std::runtime_error("Number of columns inconsistent in `" +
                                         uri + "`");
        }
    }

    std::vector<std::vector<std::string>> table;

    std::unordered_map<size_t, size_t> node_type_id_to_row_index;
    std::unordered_map<std::string, size_t> name_to_column_index;
};

CsvConfig::CsvConfig(const URI& uri)
    : impl(new CsvConfig::Impl(uri.getPath()))
{
}

const std::string& CsvConfig::get_property(const size_t node_type_id,
                                           const std::string& property) const
{
    const size_t row_index = impl->node_type_id_to_row_index.at(node_type_id);
    const size_t column_index = impl->name_to_column_index.at(property);

    return impl->table[row_index][column_index];
}

CsvConfig::~CsvConfig() = default;
CsvConfig::CsvConfig(CsvConfig&&) = default;
}
