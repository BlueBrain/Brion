/* Copyright (c) 2015-2017, EPFL/Blue Brain Project
 *                          Jonas Karlsson <jonas.karlsson@epfl.ch>
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

#include "spikeReportHDF.h"

#include <lunchbox/memoryMap.h>
#include <lunchbox/pluginRegisterer.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

#include <fstream>
#include <memory>

namespace brion
{
namespace plugin
{
namespace
{
lunchbox::PluginRegisterer<SpikeReportHDF> registerer;
constexpr char HDF_REPORT_FILE_EXT[] = ".h5";
}

struct SpikeReportHDF::Impl
{
    void initializeReport(const std::string& uri)
    {
        file.release();
        file.reset(new HighFive::File(uri));

        uint32_ts gids;
        floats timestamps;
        const HighFive::Group group = file->getGroup("/spikes");
        const HighFive::DataSet set_gids = group.getDataSet("gids");
        const HighFive::DataSet set_timestamps = group.getDataSet("timestamps");
        set_gids.read(gids);
        set_timestamps.read(timestamps);

        // assert timestamps and gids have the same size

        const size_t numElements = gids.size();

        for (size_t i = 0; i < numElements; i++)
            spikes.push_back(std::make_pair<>(timestamps[i], gids[i]));
        initialized = true;
    }

    std::unique_ptr<HighFive::File> file;
    bool initialized = false;
    Spikes spikes;
    Spikes::iterator lastReadPosition;
};

SpikeReportHDF::SpikeReportHDF(const SpikeReportInitData& initData)
    : SpikeReportPlugin(initData)
    , impl(new Impl())
{
    if (handles(initData))
        impl->initializeReport(initData.getURI().getPath());
    impl->lastReadPosition = impl->spikes.begin();
}

bool SpikeReportHDF::handles(const SpikeReportInitData& initData)
{
    const URI& uri = initData.getURI();
    if (!uri.getScheme().empty() && uri.getScheme() != "file")
        return false;

    const auto ext = boost::filesystem::path(uri.getPath()).extension();
    return ext == brion::plugin::HDF_REPORT_FILE_EXT;
}

std::string SpikeReportHDF::getDescription()
{
    return "Sonata spike reports: "
           "[file://]/path/to/report" +
           std::string(HDF_REPORT_FILE_EXT);
}

Spikes SpikeReportHDF::read(const float)
{
    // In file based reports, this function reads all remaining data.
    Spikes spikes;
    auto start = impl->lastReadPosition;
    impl->lastReadPosition = impl->spikes.end();
    _currentTime = UNDEFINED_TIMESTAMP;
    _state = State::ended;

    for (; start != impl->spikes.end(); ++start)
        pushBack(*start, spikes);

    return spikes;
}

Spikes SpikeReportHDF::readUntil(const float toTimeStamp)
{
    Spikes spikes;
    auto start = impl->lastReadPosition;

    impl->lastReadPosition =
        std::lower_bound(impl->lastReadPosition, impl->spikes.end(),
                         toTimeStamp, [](const Spike& spike, float val) {
                             return spike.first < val;
                         });

    if (impl->lastReadPosition != impl->spikes.end())
        _currentTime = impl->lastReadPosition->first;
    else
    {
        _currentTime = UNDEFINED_TIMESTAMP;
        _state = State::ended;
    }

    if (start != impl->spikes.end())
    {
        std::for_each(start, impl->lastReadPosition,
                      [&spikes, this](const Spike& spike) {
                          pushBack(spike, spikes);
                      });
    }
    return spikes;
}

void SpikeReportHDF::readSeek(const float toTimeStamp)
{
    if (impl->spikes.empty())
    {
        _currentTime = UNDEFINED_TIMESTAMP;
        _state = State::ended;
        return;
    }

    if (toTimeStamp < impl->spikes.begin()->first)
    {
        impl->lastReadPosition = impl->spikes.begin();
        _state = State::ok;
        _currentTime = toTimeStamp;
    }
    else if (toTimeStamp > impl->spikes.rbegin()->first)
    {
        impl->lastReadPosition = impl->spikes.end();
        _state = State::ended;
        _currentTime = brion::UNDEFINED_TIMESTAMP;
    }
    else
    {
        impl->lastReadPosition =
            std::lower_bound(impl->spikes.begin(), impl->spikes.end(),
                             toTimeStamp, [](const Spike& spike, float val) {
                                 return spike.first < val;
                             });
        _state = State::ok;
        _currentTime = toTimeStamp;
    }
}

void SpikeReportHDF::writeSeek(float toTimeStamp)
{
    if (toTimeStamp < _currentTime)
        LBTHROW(
            std::runtime_error("Backward seek not supported in write mode"));

    _currentTime = toTimeStamp;
}

void SpikeReportHDF::write(const Spike* /*spikes*/, const size_t /*size*/)
{
}
}
} // namespaces
