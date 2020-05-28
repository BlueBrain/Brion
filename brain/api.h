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

#ifndef BRAIN_API_H
#define BRAIN_API_H

#if defined(_MSC_VER) || defined(__declspec)
#  define BRAIN_DLLEXPORT __declspec(dllexport)
#  define BRAIN_DLLIMPORT __declspec(dllimport)
#else // _MSC_VER
#  define BRAIN_DLLEXPORT
#  define BRAIN_DLLIMPORT
#endif // _MSC_VER

#if defined(BRAIN_STATIC)
#  define BRAIN_API
#elif defined(BRAIN_SHARED)
#  define BRAIN_API BRAIN_DLLEXPORT
#else
#  define BRAIN_API BRAIN_DLLIMPORT
#endif

#if defined(BRAIN_SHARED_INL)
#  define BRAIN_INL BRAIN_DLLEXPORT
#else
#  define BRAIN_INL BRAIN_DLLIMPORT
#endif

#endif
