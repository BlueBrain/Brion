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

#include "nodeGroup.h"

#include <brion/types.h>

#include <memory>

namespace brion
{
/** Provides access to data in a H5 file.
 *
 * Note that H5 exceptions may be thrown when calling class methods.
 */

class Nodes
{
public:
    /** Open a nodes h5 file.
        @throw runtime_error if the file couldn't be opened or it's not a node
        file.
    */
    Nodes(const URI& uri);
    ~Nodes();

    /** Return the name of all the children of the nodes group */
    Strings getPopulationNames() const;

    /** Return the total number of nodes of a population */
    size_t getNumberOfNodes(const std::string& population) const;

    /** Read and return the values in the node_id dataset of the requested
        population.
        May be empty if the dataset is not present */
    uint32_ts getNodeIDs(const std::string& population) const;

    /** Read and return the values in the node_group_id dataset of the requested
        population.
        @throw runtime_error if the dataset does not exist. */
    uint32_ts getNodeGroupIDs(const std::string& population) const;

    /** Read and return the values in the node_group_ind dataset of the
        requested population.
        @throw runtime_error if the dataset does not exist. */
    uint32_ts getNodeGroupIndices(const std::string& population) const;

    /** Read and return the values in the node_type_id dataset of the
        requested population.
        @throw runtime_error if the dataset does not exist. */
    uint32_ts getNodeTypes(const std::string& population) const;

    /** Returns a handle to a group
     *
     * @param population Name of the population
     * @param groupId Id of the group
     */
    NodeGroup openGroup(const std::string& population, uint32_t groupId);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
}
