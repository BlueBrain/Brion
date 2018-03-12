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

#include <HighFive/include/highfive/H5File.hpp>

#include <iostream>
#include <memory>

namespace brion
{
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
}
