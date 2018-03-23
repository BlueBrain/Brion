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
#pragma once

#include <brion/types.h>

#include <memory>

namespace brion
{
/** Read access to a csv config file.
 */
class CsvConfig
{
public:
    /** Open given filepath to a csv file for reading.
     *
     * @param uri filepath to circuit file
     *
     * @throw std::runtime_error if file could not be opened
     * @throw std::runtime_error if csv table is inconsistent
     * @throw std::runtime_error if 'node_type_id' column is missing
     */
    CsvConfig(const URI& uri);
    CsvConfig(CsvConfig&&);
    CsvConfig(const CsvConfig& other) = delete;
    ~CsvConfig();

    /** Returns the value of a property of the node_type_id
     *
     * @param nodeTypeId The id of the node type
     * @param property The name of the property
     *
     * @throw std::runtime_error if nodeTypeId or property not found
     */
    const std::string& getProperty(size_t nodeTypeId,
                                   const std::string& property) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
}
