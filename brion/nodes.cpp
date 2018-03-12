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

#include <HighFive/include/highfive/H5DataSet.hpp>
#include <HighFive/include/highfive/H5File.hpp>

#include <iostream>
#include <memory>

namespace brion
{
static uint32_ts get_id_list_helper(const HighFive::File& file,
                                    const std::string& population,
                                    const std::string& dataset)
{
    uint32_ts list;
    const HighFive::Group group = file.getGroup("/nodes/" + population);
    const HighFive::DataSet ds = group.getDataSet(dataset);
    ds.read(list);
    return list;
}
//////////////////////////////////////////////////////////////////////////////

struct Nodes::Impl
{
    std::unique_ptr<HighFive::File> file;
};

Nodes::Nodes(const URI& uri)
    : impl(new Nodes::Impl())
{
    // lunchbox::ScopedWrite mutex(detail::hdf5Lock());
    const std::string path = uri.getPath();

    try
    {
        impl->file.reset(new HighFive::File(path, HighFive::File::ReadOnly));
    }
    catch (const HighFive::FileException& exc)
    {
        throw std::runtime_error("Could not open morphology file " + path +
                                 ": " + exc.what());
    }
}

Nodes::~Nodes() = default;

Strings Nodes::getPopulationNames() const
{
    Strings population_names;
    HighFive::Group group = impl->file->getGroup("/nodes");

    const size_t num_objects = group.getNumberObjects();
    population_names.reserve(num_objects);

    for (size_t i = 0; i < num_objects; i++)
        population_names.emplace_back(group.getObjectName(i));

    return population_names;
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
    return get_id_list_helper(*impl->file.get(), population, "node_id");
}

uint32_ts Nodes::getNodeGroupIDs(const std::string population) const
{
    return get_id_list_helper(*impl->file.get(), population, "node_group_id");
}

uint32_ts Nodes::getNodeGroupIndices(const std::string population) const
{
    return get_id_list_helper(*impl->file.get(), population,
                              "node_group_index");
}

uint32_ts Nodes::getNodeTypes(const std::string population) const
{
    return get_id_list_helper(*impl->file.get(), population, "node_type_id");
}
}
