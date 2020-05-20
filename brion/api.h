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

#ifndef BRION_API_H
#define BRION_API_H

#if defined(_MSC_VER) || defined(__declspec)
#  define BRION_DLLEXPORT __declspec(dllexport)
#  define BRION_DLLIMPORT __declspec(dllimport)
#else // _MSC_VER
#  define BRION_DLLEXPORT
#  define BRION_DLLIMPORT
#endif // _MSC_VER

#if defined(BRION_STATIC)
#  define BRION_API
#elif defined(BRION_SHARED)
#  define BRION_API BRION_DLLEXPORT
#else
#  define BRION_API BRION_DLLIMPORT
#endif

#if defined(BRION_SHARED_INL)
#  define BRION_INL BRION_DLLEXPORT
#else
#  define BRION_INL BRION_DLLIMPORT
#endif

#endif
