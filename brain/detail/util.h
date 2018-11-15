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

#ifndef BRAIN_DETAIL_UTIL
#define BRAIN_DETAIL_UTIL

#include <random>
#include <set>

namespace brain
{
namespace
{
template <typename T>
inline void _shuffle(T& container, const size_t* const seed)
{
    std::random_device randomDevice;
    std::mt19937_64 randomEngine(randomDevice());
    const char* seedEnv = getenv("BRAIN_CIRCUIT_SEED");
    if (seed)
    {
        randomEngine.seed(*seed);
    }
    else if (seedEnv)
    {
        try
        {
            randomEngine.seed(std::stoul(seedEnv));
        }
        catch (const std::exception& exc)
        {
            LBWARN << "Could not set BRAIN_CIRCUIT_SEED to " << seedEnv << ": "
                   << exc.what() << std::endl;
        }
    }
    std::shuffle(container.begin(), container.end(), randomEngine);
}
}

template <typename T>
inline std::set<T> randomSet(const std::set<T>& input, const float fraction,
                             const size_t* const seed)
{
    std::vector<T> output(input.begin(), input.end());
    _shuffle(output, seed);
    output.resize(size_t(std::ceil(input.size() * fraction)));
    return std::set<T>(output.begin(), output.end());
}
}
#endif
