/* Copyright (c) 2020, EPFL/Blue Brain Project
 *                     Nadir Román Guerrero <nadir.romanguerrero@epfl.ch>
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

#ifndef BRION_PLUGIN_MORPHOLOGYMORPHIO_H
#define BRION_PLUGIN_MORPHOLOGYMORPHIO_H

#include "../morphologyPlugin.h"

namespace brion
{
namespace plugin
{
class MorphologyMORPHIO : public MorphologyPlugin
{
public:
    explicit MorphologyMORPHIO(const MorphologyInitData& initData);

    /** Check if this plugin can handle the given uri. */
    static bool handles(const MorphologyInitData& initData);
    static std::string getDescription();

private:
    // Plugin API
    void load() final;
};
}
}

#endif // BRION_PLUGIN_MORPHOLOGYMORPHIO_H
