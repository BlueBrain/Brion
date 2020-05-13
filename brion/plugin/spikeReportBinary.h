/* Copyright (c) 2015-2017, EPFL/Blue Brain Project
 *                          Stefan.Eilemann@epfl.ch
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

#ifndef BRION_PLUGIN_SPIKEREPORTBINARY_H
#define BRION_PLUGIN_SPIKEREPORTBINARY_H

#include <brion/spikeReportPlugin.h>
#include <brion/types.h>

namespace brion
{
namespace plugin
{
class BinaryReportMap;

/**
 * A Binary spike report reader.
 *
 * The format read by this plugin is:
 * - 4b integer: magic '0xf0a'
 * - 4b integer: version, currently '1'
 * - (4b float, 4b integer) pairs until end of file: spike time, neuron GID,
 *   sorted by time
 */
class SpikeReportBinary : public SpikeReportPlugin
{
public:
    explicit SpikeReportBinary(const PluginInitData& initData);

    static bool handles(const PluginInitData& initData);
    static std::string getDescription();

    void close() final {}
    Spikes read(float min) final;
    Spikes readUntil(float max) final;
    void readSeek(float toTimeStamp) final;
    void writeSeek(float toTimeStamp) final;
    void write(const Spike* spikes, size_t size) final;
    bool supportsBackwardSeek() const final { return true; }
private:
    std::unique_ptr<BinaryReportMap> _memFile;
    size_t _startIndex = 0;
};
}
}

#endif
