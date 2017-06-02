/* Copyright (c) 2017, EPFL/Blue Brain Project
 *                     Juan Hernando <juan.hernando@epfl.ch>
 *                     Mohamed-Ghaith Kaabi <mohamed.kaabi@epfl.ch>
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

#include "../compartmentReport.h"
#include "../compartmentReportMapping.h"
#include "brion/compartmentReport.h"
#include <lunchbox/threadPool.h>

namespace brain
{
namespace detail
{
struct CompartmentReportReader
{
    CompartmentReportReader(const brion::URI& uri_)
        : uri(uri_)
    {
        const brion::CompartmentReport report{uri, brion::MODE_READ};

        metaData.startTime = report.getStartTime();
        metaData.endTime = report.getEndTime();
        metaData.timeStep = report.getTimestep();
        metaData.timeUnit = report.getTimeUnit();
        metaData.dataUnit = report.getDataUnit();
    }

    const brion::URI uri;
    CompartmentReportMetaData metaData;
    lunchbox::ThreadPool threadPool;
};

struct CompartmentReportView
{
    CompartmentReportView(
        const std::shared_ptr<CompartmentReportReader>& readerImpl_,
        const brion::GIDSet& gids)
        : report(std::make_shared<brion::CompartmentReport>(readerImpl_->uri,
                                                            brion::MODE_READ,
                                                            gids))
        , readerImpl{readerImpl_}
    {
        _initIndices();
    }

    std::shared_ptr<brion::CompartmentReport> report;
    std::shared_ptr<CompartmentReportReader> readerImpl;
    brain::CompartmentReportMapping mapping{this};
    brain::CompartmentReportMapping::Index indices;

private:
    inline void _initIndices();
};

void CompartmentReportView::_initIndices()
{
    size_t indicesCount = 0;
    const auto& gids = report->getGIDs();

    const auto gidCount = gids.size();
    std::vector<size_t> indexPositions(gidCount);
    for (size_t i = 0; i < gids.size(); ++i)
    {
        indexPositions[i] = indicesCount;
        indicesCount += report->getOffsets()[i].size();
    }

    indices.resize(indicesCount);

    const std::vector<uint32_t> gidList(report->getGIDs().begin(),
                                        report->getGIDs().end());
    for (size_t i = 0; i < gids.size(); ++i)
    {
        const brion::uint16_ts& compartments =
            report->getCompartmentCounts()[i];
        const brion::uint64_ts& offsets = report->getOffsets()[i];
        uint16_t section = 0;
        auto pos = indexPositions[i];
        for (auto offset : offsets)
        {
            indices[pos + section] = {offset, gidList[i], section,
                                      compartments[section]};
            ++section;
        }
    }
}
}
} // namespaces
