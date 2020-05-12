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

#include "spikeReportHDF5.h"

#include "../pluginLibrary.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <highfive/H5DataSet.hpp>
#include <highfive/H5Utility.hpp>

#include <memory>

namespace brion
{
namespace plugin
{
namespace
{

class PluginRegisterer
{
public:
    PluginRegisterer()
    {
        auto& pluginManager = PluginLibrary::instance().getManager<SpikeReportPlugin>();
        pluginManager.registerFactory<SpikeReportHDF5>();
    }
};

PluginRegisterer registerer;

constexpr char HDF_REPORT_FILE_EXT[] = ".h5";
} // namespace

SpikeReportHDF5::SpikeReportHDF5(const PluginInitData& initData)
    : SpikeReportPlugin(initData)
    , _file([initData]() {
        // For consistency with the other spike plugins, we convert
        // Highfive exceptions into std::runtime_error
        try
        {
            HighFive::SilenceHDF5 silence;
            return HighFive::File(initData.getURI().getPath());
        }
        catch (const HighFive::Exception& e)
        {
            throw std::runtime_error(e.what());
        }
    }())
{
    const auto group = _file.getGroup("/spikes");
    const auto allG = group.getGroup("All");
    const auto setGids = allG.getDataSet("node_ids");
    const auto setTimestamps = allG.getDataSet("timestamps");

    uint32_ts gids;
    floats timestamps;
    setGids.read(gids);
    setTimestamps.read(timestamps);

    const size_t numElements = gids.size();

    for (size_t i = 0; i < numElements; i++)
        _spikes.push_back(std::make_pair<>(timestamps[i], gids[i]));

    std::sort(_spikes.begin(), _spikes.end());
    _lastReadPosition = _spikes.begin();

    if (!_spikes.empty())
        _endTime = _spikes.rbegin()->first;
}

bool SpikeReportHDF5::handles(const PluginInitData& initData)
{
    const URI& uri = initData.getURI();
    if (!uri.getScheme().empty() && uri.getScheme() != "file")
        return false;

    const auto ext = boost::filesystem::path(uri.getPath()).extension();
    return ext == brion::plugin::HDF_REPORT_FILE_EXT;
}

std::string SpikeReportHDF5::getDescription()
{
    return "Sonata spike reports: "
           "[file://]/path/to/report" +
           std::string(HDF_REPORT_FILE_EXT);
}

Spikes SpikeReportHDF5::read(const float)
{
    // In file based reports, this function reads all remaining data.
    Spikes spikes;
    auto start = _lastReadPosition;
    _lastReadPosition = _spikes.end();
    _currentTime = UNDEFINED_TIMESTAMP;
    _state = State::ended;

    for (; start != _spikes.end(); ++start)
        pushBack(*start, spikes);

    return spikes;
}

Spikes SpikeReportHDF5::readUntil(const float toTimeStamp)
{
    Spikes spikes;

    const auto end = _spikes.end();

    for (; _lastReadPosition != end; ++_lastReadPosition)
    {
        if (_lastReadPosition->first >= toTimeStamp)
        {
            _currentTime = _lastReadPosition->first;
            break;
        }
        pushBack(*_lastReadPosition, spikes);
    }

    if (_lastReadPosition != _spikes.end())
        _currentTime = _lastReadPosition->first;
    else
    {
        _currentTime = UNDEFINED_TIMESTAMP;
        _state = State::ended;
    }

    return spikes;
}

void SpikeReportHDF5::readSeek(const float toTimeStamp)
{
    if (_spikes.empty())
    {
        _currentTime = UNDEFINED_TIMESTAMP;
        _state = State::ended;
        return;
    }

    if (toTimeStamp < _spikes.begin()->first)
    {
        _lastReadPosition = _spikes.begin();
        _state = State::ok;
        _currentTime = toTimeStamp;
    }
    else if (toTimeStamp > _spikes.rbegin()->first)
    {
        _lastReadPosition = _spikes.end();
        _state = State::ended;
        _currentTime = brion::UNDEFINED_TIMESTAMP;
    }
    else
    {
        _lastReadPosition =
            std::lower_bound(_spikes.begin(), _spikes.end(), toTimeStamp,
                             [](const Spike& spike, float val) {
                                 return spike.first < val;
                             });
        _state = State::ok;
        _currentTime = toTimeStamp;
    }
}
} // namespace plugin
} // namespace brion
