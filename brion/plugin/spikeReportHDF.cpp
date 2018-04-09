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

#include <fstream>

namespace brion
{
namespace plugin
{
namespace
{
lunchbox::PluginRegisterer<SpikeReportHDF> registerer;
constexpr char HDF_REPORT_FILE_EXT[] = ".h5";
}

class SpikeReportHDF::Impl
{
};

SpikeReportHDF::SpikeReportHDF(const SpikeReportInitData& initData)
    : SpikeReportPlugin(initData)
    , impl(new Impl())
{
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
    return "";
}

Spikes SpikeReportHDF::read(const float)
{
    Spikes spikes;

    return spikes;
}

Spikes SpikeReportHDF::readUntil(const float /*max*/)
{
    Spikes spikes;

    return spikes;
}

void SpikeReportHDF::readSeek(const float /*toTimeStamp*/)
{
}

void SpikeReportHDF::writeSeek(float /*toTimeStamp*/)
{
}

void SpikeReportHDF::write(const Spike* /*spikes*/, const size_t /*size*/)
{
}
}
} // namespaces
