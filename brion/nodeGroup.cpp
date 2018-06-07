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

#include "nodeGroup.h"

#include <highfive/H5File.hpp>
#include <highfive/H5Group.hpp>

namespace
{
template <typename T>
T getAttributeHelper(const HighFive::DataSet set, size_t start = 0,
                     const size_t end = std::numeric_limits<size_t>::max())
{
    if (start >= end)
    {
        throw std::runtime_error("Start index " + std::to_string(start) +
                                 " is greater than or equal to end index " +
                                 std::to_string(end));
    }

    T output;

    if (start != 0 && end != std::numeric_limits<size_t>::max())
    {
        const auto selection = set.select({start}, {end - start});
        selection.read(output);
    }
    else
    {
        set.read(output);
    }

    return output;
}
}

namespace brion
{
struct NodeGroup::Impl
{
    HighFive::Group group;
};

NodeGroup::NodeGroup(const HighFive::Group& group)
    : impl(new NodeGroup::Impl())
{
    impl->group = group;
}
NodeGroup::NodeGroup()
    : impl(new NodeGroup::Impl()){};

NodeGroup::~NodeGroup() = default;
NodeGroup::NodeGroup(NodeGroup&&) = default;
NodeGroup& NodeGroup::operator=(NodeGroup&&) = default;

Strings NodeGroup::getAttributeNames() const
{
    Strings attributeNames;
    const size_t num_objects = impl->group.getNumberObjects();

    for (size_t i = 0; i < num_objects; i++)
        attributeNames.push_back(impl->group.getObjectName(i));

    return attributeNames;
}

Strings NodeGroup::getDynamicParameterNames() const
{
    Strings parameterNames;

    const HighFive::Group dynamics_params_group =
        impl->group.getGroup("dynamics_params");

    const size_t num_objects = dynamics_params_group.getNumberObjects();

    for (size_t i = 0; i < num_objects; i++)
        parameterNames.push_back(dynamics_params_group.getObjectName(i));

    return parameterNames;
}

size_t NodeGroup::getNumberOfNodes() const
{
    return impl->group.getNumberObjects();
}

template <typename T>
std::vector<T> NodeGroup::getAttribute(const std::string& name, size_t start,
                                       size_t end) const
{
    return getAttributeHelper<std::vector<T>>(impl->group.getDataSet(name),
                                              start, end);
}

template <typename T>
std::vector<T> NodeGroup::getAttribute(const std::string& name) const
{
    return getAttributeHelper<std::vector<T>>(impl->group.getDataSet(name));
}

template <typename T>
std::vector<T> NodeGroup::getDynamicParameter(const std::string& name) const
{
    return getAttributeHelper<std::vector<T>>(
        impl->group.getGroup("dynamics_params").getDataSet(name));
}

template <typename T>
std::vector<T> NodeGroup::getDynamicParameter(const std::string& name,
                                              size_t start, size_t end) const
{
    return getAttributeHelper<std::vector<T>>(
        impl->group.getGroup("dynamics_params").getDataSet(name), start, end);
}

//////////////////////////////////////////////////////////////////////////////

#define CREATE_GET_FUNCTIONS(list_type, scalar_type)                \
    template list_type NodeGroup::getAttribute<scalar_type>(        \
        const std::string&) const;                                  \
    template list_type NodeGroup::getAttribute<scalar_type>(        \
        const std::string&, size_t, size_t) const;                  \
    template list_type NodeGroup::getDynamicParameter<scalar_type>( \
        const std::string&) const;                                  \
    template list_type NodeGroup::getDynamicParameter<scalar_type>( \
        const std::string&, size_t, size_t) const;

CREATE_GET_FUNCTIONS(chars, char)
CREATE_GET_FUNCTIONS(uchars, unsigned char)
CREATE_GET_FUNCTIONS(int32_ts, int32_t)
CREATE_GET_FUNCTIONS(uint16_ts, uint16_t)
CREATE_GET_FUNCTIONS(uint32_ts, uint32_t)
CREATE_GET_FUNCTIONS(uint64_ts, uint64_t)
CREATE_GET_FUNCTIONS(floats, float)
CREATE_GET_FUNCTIONS(doubles, double)
CREATE_GET_FUNCTIONS(Strings, std::string)
}
