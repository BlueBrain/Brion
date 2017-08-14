/* Copyright (c) 2013-2017, EPFL/Blue Brain Project
 *                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
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

#include "morphology.h"

#include "morphologyPlugin.h"

#include <lunchbox/plugin.h>
#include <lunchbox/pluginFactory.h>

namespace brion
{
class Morphology::Impl
{
public:
    typedef lunchbox::PluginFactory<MorphologyPlugin> MorphologyPluginFactory;

    explicit Impl(const MorphologyInitData& initData)
        : plugin(MorphologyPluginFactory::getInstance().create(initData))
    {
    }

    std::unique_ptr<MorphologyPlugin> plugin;
};

Morphology::Morphology(const std::string& source)
    : _impl(new Impl(MorphologyInitData(URI(source))))
{
}

Morphology::Morphology(Morphology&&) = default;
Morphology& Morphology::operator=(Morphology&&) = default;

Morphology::~Morphology()
{
}

CellFamily Morphology::getCellFamily() const
{
    return _impl->plugin->getCellFamily();
}

Vector4fsPtr Morphology::readPoints()
{
    return _impl->plugin->readPoints();
}

Vector2isPtr Morphology::readSections()
{
    return _impl->plugin->readSections();
}

SectionTypesPtr Morphology::readSectionTypes()
{
    return _impl->plugin->readSectionTypes();
}

floatsPtr Morphology::readPerimeters()
{
    return _impl->plugin->readPerimeters();
}

MorphologyVersion Morphology::getVersion() const
{
    return _impl->plugin->getVersion();
}

const MorphologyInitData& Morphology::getInitData() const
{
    return _impl->plugin->getInitData();
}
}
