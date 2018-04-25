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

namespace HighFive
{
class Group;
}

namespace brion
{
class Nodes;

/** A handle to a specific group of nodes in a H5 file.
 *
 * Note that H5 exceptions may be thrown when calling class methods.
 */

class NodeGroup
{
public:
    friend class Nodes;
    NodeGroup();
    NodeGroup(NodeGroup&&);
    ~NodeGroup();

    NodeGroup& operator=(NodeGroup&&);

    /** Return the names of all the datasets of the node group */
    Strings getAttributeNames() const;

    /** Return the names of all the datasets in the 'dynamics_params' group */
    Strings getDynamicParameterNames() const;

    /** Return the number of nodes in the node group */
    size_t getNumberOfNodes() const;

    /** Read and return the values of the requested dataset
     *
     * @param name Name of the dataset
     */
    template <typename T>
    std::vector<T> getAttribute(const std::string& name) const;

    /** Read and return the values of the requested dataset
     *
     * Reads from start to the index before end, i.e. in set builder notation:
     * '[start, end)'.
     *
     * @param name Name of the dataset
     * @param start Index of first element
     * @param end Index _after_ last element
     *
     * @throw std::runtime_error if start >= end
     */
    template <typename T>
    std::vector<T> getAttribute(const std::string& name, size_t start,
                                size_t end) const;

    /** Read and return the values of the requested dataset in the dynamic
     * parameters group.
     *
     * @param name Name of the dataset
     */
    template <typename T>
    std::vector<T> getDynamicParameter(const std::string& name) const;

    /** Read and return the values of the requested dataset in the dynamic
     * parameters group.
     *
     * Reads from start to the index before end, i.e. in set builder notation:
     * '[start, end)'.
     *
     * @param name Name of the dataset
     * @param start Index of first element
     * @param end Index _after_ last element
     *
     * @throw std::runtime_error if start >= end
     */
    template <typename T>
    std::vector<T> getDynamicParameter(const std::string& name, size_t start,
                                       size_t end) const;

private:
    NodeGroup(const NodeGroup& other) = delete;
    NodeGroup(const HighFive::Group& group);

    struct Impl;
    std::unique_ptr<Impl> impl;
};
}
