/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Juan Hernando <jhernando@fi.upm.es>
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

#include <brion/blueConfig.h>
#include <brion/target.h>

#include <lunchbox/log.h>

namespace brain
{
class Targets
{
public:
    Targets(const brion::BlueConfig& config)
        : _targetSources(config.getTargetSources())
    {
    }

    GIDSet getGIDs(const std::string& target) const
    {
        _initParsers();
        return brion::Target::parse(_targetParsers, target);
    }

    Strings getTargetNames() const
    {
        _initParsers();
        Strings names;
        for (const auto& parser : _targetParsers)
        {
            const auto toAdd = parser.getTargetNames(brion::TARGET_CELL);
            names.reserve(names.size() + toAdd.size());
            for (auto& n : toAdd)
                names.emplace_back(std::move(n));
        }
        return names;
    }

private:
    const brion::URIs _targetSources;
    mutable brion::Targets _targetParsers;

    void _initParsers() const
    {
        if (_targetParsers.empty())
        {
            for (const URI& uri : _targetSources)
            {
                try
                {
                    _targetParsers.push_back(brion::Target(uri.getPath()));
                }
                catch (const std::runtime_error& exc)
                {
                    LBWARN << "Failed to load targets from " << uri.getPath()
                           << ": " << exc.what() << std::endl;
                }
            }
        }
    }
};
}
