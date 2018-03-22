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
/** Read access to a Sonata circuit config file.
 */
class CircuitConfig
{
public:
    struct NetworkNode
    {
        std::string nodes_file;
        std::string node_types_file;
    };

    struct NetworkEdge
    {
        std::string edges_file;
        std::string edge_types_file;
    };

public:
    /** Open given filepath to a circuit file for reading.
     *
     * @param uri filepath to circuit file
     * @throw std::runtime_error if file is not found or invalid
     */
    CircuitConfig(const URI& uri);
    CircuitConfig(CircuitConfig&&);
    CircuitConfig(const CircuitConfig& other) = delete;
    ~CircuitConfig();

    /** Return the target simulator */
    std::string getTargetSimulator() const;

    /** Return the directory of a component in the components_dir given its name
    *
    * @param name component name
    * @throw std::runtime_error if component not found
    */
    std::string getComponentPath(const std::string& name) const;

    /** Return the list of network nodes */
    std::vector<NetworkNode> getNetworkNodes() const;

    /** Return the list of network edges */
    std::vector<NetworkEdge> getNetworkEdges() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
}
