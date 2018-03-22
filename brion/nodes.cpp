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

#include "nodes.h"

#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

#include <iostream>
#include <memory>

namespace
{
static brion::uint32_ts _readIntVector(const HighFive::File& file,
                                       const std::string& population,
                                       const std::string& dataset)
{
    brion::uint32_ts list;
    const HighFive::Group group = file.getGroup("/nodes/" + population);
    const HighFive::DataSet ds = group.getDataSet(dataset);
    ds.read(list);
    return list;
}
}

namespace brion
{
//////////////////////////////////////////////////////////////////////////////

struct Nodes::Impl
{
    Impl(const std::string& uri)
        : file(new HighFive::File(uri))
    {
    }

    std::unique_ptr<HighFive::File> file;
};

Nodes::Nodes(const URI& uri)
    : impl(new Nodes::Impl(uri.getPath()))
{
}

Nodes::~Nodes() = default;

Strings Nodes::getPopulationNames() const
{
    Strings populationNames;
    HighFive::Group group = impl->file->getGroup("/nodes");

    const size_t num_objects = group.getNumberObjects();
    populationNames.reserve(num_objects);

    for (size_t i = 0; i < num_objects; i++)
        populationNames.emplace_back(group.getObjectName(i));

    return populationNames;
}

size_t Nodes::getNumberOfNodes(const std::string population) const
{
    return impl->file->getGroup("/nodes/" + population)
        .getDataSet("node_type_id")
        .getSpace()
        .getDimensions()[0];
}

uint32_ts Nodes::getNodeIDs(const std::string population) const
{
    return _readIntVector(*impl->file, population, "node_id");
}

uint32_ts Nodes::getNodeGroupIDs(const std::string population) const
{
    return _readIntVector(*impl->file, population, "node_group_id");
}

uint32_ts Nodes::getNodeGroupIndices(const std::string population) const
{
    return _readIntVector(*impl->file, population, "node_group_index");
}

uint32_ts Nodes::getNodeTypes(const std::string population) const
{
    return _readIntVector(*impl->file, population, "node_type_id");
}

NodeGroup Nodes::openGroup(const std::string& population, uint32_t groupId)
{
    const HighFive::Group group = impl->file->getGroup("/nodes/" + population)
                                      .getGroup(std::to_string(groupId));
    return NodeGroup(group);
}
}
