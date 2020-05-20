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

#ifndef BRION_TYPES
#define BRION_TYPES

#include <brion/enums.h>
#include <brion/uint128_t.h>
#include <brion/uri.h>

#include <boost/multi_array.hpp>
#include <map>
#include <set>
#include <vector>

#define GLM_FORCE_CTOR_INIT
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>
#include <glm/gtx/io.hpp>

#ifdef __GNUC__
#define BRION_UNUSED __attribute__((unused))
#else
#define BRION_UNUSED
#endif

/** @namespace brion Blue Brain File IO classes */
namespace brion
{
using namespace enums;
class BlueConfig;
class Circuit;
class CompartmentReport;
class CompartmentReportPlugin;
class Mesh;
class Morphology;
class MorphologyInitData;
class SpikeReport;
class SpikeReportPlugin;
class Synapse;
class SynapseSummary;
class Target;

using chars         = std::vector<char>;
using uchars        = std::vector<unsigned char>;
using size_ts       = std::vector<size_t>;
using int32_ts      = std::vector<int32_t>;
using uint16_ts     = std::vector<uint16_t>;
using uint32_ts     = std::vector<uint32_t>;
using uint64_ts     = std::vector<uint64_t>;
using floats        = std::vector<float>;
using doubles       = std::vector<double>;
using Vector2is     = std::vector<glm::ivec2>;
using Vector3fs     = std::vector<glm::vec3>;
using Vector4fs     = std::vector<glm::vec4>;
using Vector3ds     = std::vector<glm::dvec3>;
using Vector4ds     = std::vector<glm::dvec4>;
using SectionTypes  = std::vector<SectionType>;
using Targets       = std::vector<Target>;

using int32_tsPtr   = std::shared_ptr<int32_ts>;
using uint16_tsPtr  = std::shared_ptr<uint16_ts>;
using uint32_tsPtr  = std::shared_ptr<uint32_ts>;
using floatsPtr      = std::shared_ptr<floats>;
using doublesPtr    = std::shared_ptr<doubles>;
using Vector3fsPtr  = std::shared_ptr<Vector3fs>;
using Vector3dsPtr  = std::shared_ptr<Vector3ds>;
using Vector4dsPtr  = std::shared_ptr<Vector4ds>;

struct AABB
{
    glm::vec3 _min {999999.f, 999999.f, 999999.f};
    glm::vec3 _max {-999999.f, -999999.f, -999999.f};

    void merge(const glm::vec3& p) noexcept
    {
        _min = glm::min(p, _min);
        _max = glm::max(p, _max);
    }

    void merge(const AABB& o) noexcept
    {
        _min = glm::min(o._min, _min);
        _max = glm::max(o._max, _max);
    }

    glm::vec3 getCenter() const noexcept
    {
        return (_max + _min) * 0.5f;
    }

    glm::vec3& getMin() noexcept { return _min; }
    const glm::vec3& getMin() const noexcept { return _min; }
    glm::vec3& getMax() noexcept { return _max; }
    const glm::vec3& getMax() const noexcept { return _max; }
};

using MorphologyPtr = std::shared_ptr<Morphology>;
using ConstMorphologyPtr = std::shared_ptr<const Morphology>;

/** Ordered set of GIDs of neurons. */
using GIDSet = std::set<uint32_t>;

using GIDSetCIter   = GIDSet::const_iterator;
using GIDSetIter    = GIDSet::iterator;

/** The offset for the voltage per section for each neuron, uin64_t max for
 *  sections with no compartments.
 */
using SectionOffsets = std::vector<uint64_ts>;

/** The number of compartments per section for each neuron. */
using CompartmentCounts = std::vector<uint16_ts>;

/** Data matrix storing NeuronAttributes for each neuron. */
using NeuronMatrix = boost::multi_array<std::string, 2>;

/** Data matrix storing SynapseAttributes for each neuron. */
using SynapseMatrix = boost::multi_array<float, 2>;

/** Data matrix storing GID, numEfferent, numAfferent for each neuron. */
using SynapseSummaryMatrix = boost::multi_array<uint32_t, 2>;

/** A spike */
using Spike     = std::pair<float, uint32_t>;
using Spikes    = std::vector<Spike>;

/** A list of Spikes events per cell gid, indexed by spikes times. */
using SpikeMap = std::multimap<float, uint32_t>;

struct Frame
{
    double timestamp;
    floatsPtr data;
};

struct Frames
{
    doublesPtr timeStamps;
    /** The data of multiple compartment frames in a flat array. The number
        of frames equals timeStamp->size(). All frames have the same size,
        this size and the mapping of values to entities is defined in the
        report mapping. */
    floatsPtr data;
};

/** A value for undefined timestamps */

const float UNDEFINED_TIMESTAMP BRION_UNUSED =
    std::numeric_limits<float>::max();
const float RESTING_VOLTAGE BRION_UNUSED = -67.; //!< Resting voltage in mV
/** Lowest voltage after hyperpolarisation */
const float MINIMUM_VOLTAGE BRION_UNUSED = -80.;

using Strings = std::vector<std::string>;

using URIs = std::vector<URI>;
}

// if you have a type T in namespace N, the operator << for T needs to be in
// namespace N too
namespace boost
{
template <typename T>
inline std::ostream& operator<<(std::ostream& os,
                                const boost::multi_array<T, 2>& data)
{
    for (size_t i = 0; i < data.shape()[0]; ++i)
    {
        for (size_t j = 0; j < data.shape()[1]; ++j)
            os << data[i][j] << " ";
        os << std::endl;
    }
    return os;
}
}

namespace std
{
template <class T, class U>
inline std::ostream& operator<<(std::ostream& os, const std::pair<T, U>& pair)
{
    return os << "[ " << pair.first << ", " << pair.second << " ]";
}
}

#endif
