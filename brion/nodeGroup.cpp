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

#include <HighFive/include/highfive/H5File.hpp>
#include <HighFive/include/highfive/H5Group.hpp>

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
NodeGroup::~NodeGroup() = default;

Strings NodeGroup::getAttributeNames()
{
    Strings attributeNames;
    const size_t num_objects = impl->group.getNumberObjects();

    for (size_t i = 0; i < num_objects; i++)
        attributeNames.push_back(impl->group.getObjectName(i));

    return attributeNames;
}

Strings NodeGroup::getDynamicParameterNames()
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
T NodeGroup::getAttribute(const std::string& name, size_t start,
                          size_t end) const
{
    assert(end >= start);

    T list;
    const HighFive::DataSet attribute = impl->group.getDataSet(name);
    attribute.read(list);

    if (start != 0 || end != list.size())
    {
        T sublist;
        sublist.reserve(end - start);
        for (size_t i = start; i < end; i++)
            sublist.push_back(list[i]);
        list = std::move(sublist);
    }

    return list;
}

template <typename T>
T NodeGroup::getAttribute(const std::string& name) const
{
    T list;
    const HighFive::DataSet attribute = impl->group.getDataSet(name);
    attribute.read(list);
    return list;
}

//////////////////////////////////////////////////////////////////////////////

template int32_ts NodeGroup::getAttribute<int32_ts>(const std::string&) const;
template uint16_ts NodeGroup::getAttribute<uint16_ts>(const std::string&) const;
template uint32_ts NodeGroup::getAttribute<uint32_ts>(const std::string&) const;
template uint64_ts NodeGroup::getAttribute<uint64_ts>(const std::string&) const;
template floats NodeGroup::getAttribute<floats>(const std::string&) const;
template doubles NodeGroup::getAttribute<doubles>(const std::string&) const;
template Strings NodeGroup::getAttribute<Strings>(const std::string&) const;

//////////////////////////////////////////////////////////////////////////////

template int32_ts NodeGroup::getAttribute<int32_ts>(const std::string&, size_t,
                                                    size_t) const;
template uint16_ts NodeGroup::getAttribute<uint16_ts>(const std::string&,
                                                      size_t, size_t) const;
template uint32_ts NodeGroup::getAttribute<uint32_ts>(const std::string&,
                                                      size_t, size_t) const;
template uint64_ts NodeGroup::getAttribute<uint64_ts>(const std::string&,
                                                      size_t, size_t) const;
template floats NodeGroup::getAttribute<floats>(const std::string&, size_t,
                                                size_t) const;
template doubles NodeGroup::getAttribute<doubles>(const std::string&, size_t,
                                                  size_t) const;
template Strings NodeGroup::getAttribute<Strings>(const std::string&, size_t,
                                                  size_t) const;
}
