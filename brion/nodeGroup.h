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

namespace brion
{
class Nodes;

class NodeGroup
{
public:
    friend Nodes;
    NodeGroup() = delete;

    Strings getAttributeNames();

    Strings getDynamicParameterNames();

    size_t getNumberOfNodes() const;

    template <typename T>
    std::vector<T> getAttribute(const std::string& name) const;

    template <typename T>
    std::vector<T> getAttribute(const std::string& name, size_t start,
                                size_t end) const;

    template <typename T>
    std::vector<T> getDynamicParameter(const std::string& name) const;

    template <typename T>
    std::vector<T> getDynamicParameter(const std::string& name, size_t start,
                                       size_t end) const;

private:
    NodeGroup(...);
};
}
