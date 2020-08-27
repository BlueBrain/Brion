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

#include "compartmentReportCommon.h"

#include "../log.h"

namespace brion
{
namespace plugin
{
CompartmentReportCommon::CompartmentReportCommon()
{
}

void CompartmentReportCommon::_cacheNeuronCompartmentCounts()
{
    const CompartmentCounts& counts = getCompartmentCounts();
    _neuronCompartments.resize(counts.size());
    for (size_t i = 0; i < counts.size(); ++i)
        _neuronCompartments[i] =
            std::accumulate(counts[i].begin(), counts[i].end(), 0);
}

size_t CompartmentReportCommon::getNumCompartments(const size_t index) const
{
    return _neuronCompartments[index];
}

size_t CompartmentReportCommon::_getFrameNumber(double timestamp) const
{
    const auto startTime = getStartTime();
    const auto endTime = getEndTime();
    assert(endTime > startTime);
    const auto step = getTimestep();

    timestamp =
        std::max(std::min(timestamp, std::nextafter(endTime, -INFINITY)),
                 startTime) -
        startTime;

    return static_cast<size_t>(round(timestamp / step));
}

size_t CompartmentReportCommon::getFrameCount() const
{
    if (getStartTime() < getEndTime())
        return _getFrameNumber(getEndTime()) + 1;
    return 0;
}

floatsPtr CompartmentReportCommon::loadFrame(const double timestamp) const
{
    const size_t size = getFrameSize();
    floatsPtr buffer(new floats(size));
    if (size != 0)
        _loadFrame(_getFrameNumber(timestamp), buffer->data());
    return buffer;
}

Frames CompartmentReportCommon::loadFrames(double start, double end) const
{
    const auto startTime = getStartTime();
    if (start >= getEndTime() || end < startTime || end <= start)
        return Frames();

    const double timestep = getTimestep();
    const size_t startFrame = _getFrameNumber(start);
    end = std::nextafter(end, -INFINITY);
    const size_t count = _getFrameNumber(end) - startFrame + 1;

    Frames frames;

    frames.timeStamps.reset(new std::vector<double>);
    for (size_t i = 0; i < count; ++i)
        frames.timeStamps->push_back(startTime + (i + startFrame) * timestep);

    const auto frameSize = getFrameSize();
    frames.data.reset(new floats(frameSize * count));
    if (frameSize == 0)
        return frames;

    if (!_loadFrames(startFrame, count, frames.data->data()))
        return Frames();

    return frames;
}

bool CompartmentReportCommon::writeFrame(const GIDSet& gids,
                                         const float* values,
                                         const size_ts& sizes,
                                         const double timestamp)
{
    size_t offset = 0;
    size_t index = 0;
    for (const auto gid : gids)
    {
        if (!writeFrame(gid, values + offset, sizes[index], timestamp))
            return false;
        offset += sizes[index];
        ++index;
    }
    return true;
}

GIDSet CompartmentReportCommon::_computeIntersection(const GIDSet& all,
                                                     const GIDSet& subset)
{
    GIDSet intersection;
    std::set_intersection(subset.begin(), subset.end(), all.begin(), all.end(),
                          std::inserter(intersection, intersection.begin()));
    if (intersection != subset || intersection.empty())
    {
        BRION_WARN << "Requested " << subset.size() << " GIDs [" << *subset.begin()
                 << ":" << *subset.rbegin() << "] are not a subset of the "
                 << all.size() << " GIDs in the report [" << *all.begin() << ":"
                 << *all.rbegin();
        if (intersection.empty())
            BRION_WARN << " with no GIDs in common" << std::endl;
        else
            BRION_WARN << "], using intersection size " << intersection.size()
                     << " [" << *intersection.begin() << ":"
                     << *intersection.rbegin() << "]" << std::endl;
    }
    return intersection;
}

std::vector<uint32_t> CompartmentReportCommon::_computeSubsetIndices(
    const GIDSet& source, const GIDSet& target)
{
    GIDSet::iterator i = source.begin();
    std::vector<uint32_t> indices;
    indices.reserve(target.size());
    uint32_t sourceIndex = 0;
    for (const auto gid : target)
    {
        assert(i != source.end());
        while (*i != gid)
        {
            ++i;
            ++sourceIndex;
        }
        indices.push_back(sourceIndex);
    }
    return indices;
}

CompartmentReportCommon::MappingInfo CompartmentReportCommon::_reduceMapping(
    const MappingInfo& source, const std::vector<uint32_t>& indices)
{
    MappingInfo target;
    const size_t count = indices.size();
    target.offsets.resize(count);
    target.counts.resize(count);
    target.cellOffsets.reserve(count);
    target.cellSizes.reserve(count);

    size_t targetIndex = 0;
    size_t frameSize = 0;
    for (const auto sourceIndex : indices)
    {
        auto& offsets = target.offsets[targetIndex];
        offsets = source.offsets[sourceIndex];
        const auto shift = frameSize - source.cellOffsets[sourceIndex];
        for (auto& offset : offsets)
        {
            if (offset != std::numeric_limits<uint64_t>().max())
                offset += shift;
        }
        target.counts[targetIndex] = source.counts[sourceIndex];
        target.cellOffsets.push_back(frameSize);
        const auto size = source.cellSizes[sourceIndex];
        target.cellSizes.push_back(size);
        frameSize += size;
        ++targetIndex;
    }
    target.frameSize = frameSize;
    return target;
}

bool CompartmentReportCommon::_loadFrames(const size_t startFrame,
                                          const size_t count,
                                          float* buffer) const
{
    for (size_t i = 0; i != count; ++i, buffer += getFrameSize())
    {
        if (!_loadFrame(startFrame + i, buffer))
            return false;
    }
    return true;
}
}
}
