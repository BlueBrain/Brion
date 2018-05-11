/* Copyright (c) 2018, EPFL/Blue Brain Project
 *                     Jonas Karlsson <jonas.karlsson@epfl.ch>
 *                     Juan Hernando <juan.hernando@epfl.ch>
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

#ifndef BRION_DETAIL_UTILS
#define BRION_DETAIL_UTILS

#include "json.hpp"

namespace brion
{
/** Open a json file, parse it and expand variables in the manifest section.
    @param uri The path to the file to open
    @return the json data
    @throw if file could not be read or parse error
*/
nlohmann::json parseSonataJson(const std::string& uri);
}
#endif
