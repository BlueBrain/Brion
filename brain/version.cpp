/* Copyright (c) 2020, EPFL/Blue Brain Project
 *                          Nadir Román Guerrero <nadir.romanguerrero@epfl.ch>
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

#include <brain/version.h>

namespace brain
{

int Version::getMajor()
{
    return BRAIN_VERSION_MAJOR;
}

int Version::getMinor()
{
    return BRAIN_VERSION_MINOR;
}

int Version::getPatch()
{
    return BRAIN_VERSION_PATCH;
}

std::string Version::getString()
{
    return BRAIN_VERSION_STRING;
}

unsigned long long Version::getRevision()
{
    return BRAIN_VERSION_REVISION;
}

}
