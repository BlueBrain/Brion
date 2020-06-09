/* Copyright (c) 2015-2017, EPFL/Blue Brain Project
 *                          Stefan.Eilemann@epfl.ch
 *                          Mohamed-Ghaith Kaabi <mohamed.kaabi@epfl.ch>
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

#include "spikeReportBinary.h"

#include "../log.h"
#include "../pluginLibrary.h"
#include "../uri.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <fstream>

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
        pluginManager.registerFactory<SpikeReportBinary>();
    }
};

PluginRegisterer registerer;

const char* const BINARY_REPORT_FILE_EXT = ".spikes";
} // namespace

namespace fs = boost::filesystem;

class Header
{
public:
    bool isValid() const { return _magic == 0xf0a && _version == 1; }
private:
    uint32_t _magic = 0xf0a;
    uint32_t _version = 1;
};

class BinaryReportMap
{
public:
    // Read-only mapping
    BinaryReportMap(const std::string& path)
        : _path(path)
        , _map(path, std::ios_base::in)
    {
        const size_t totalSize = _map.size();
        if (totalSize < sizeof(Header) || (totalSize % sizeof(uint32_t)) != 0)
            BRION_THROW("Incompatible binary report: " + path)

        if (!reinterpret_cast<const Header&>(*(_map.const_data())).isValid())
            BRION_THROW("Invalid binary spike report header: " + path)
    }

    // read-write mapping
    BinaryReportMap(const std::string& path, size_t nSpikes)
        : _path(path)
    {
        boost::iostreams::mapped_file_params fileParams;
        fileParams.path = path;
        fileParams.flags = boost::iostreams::mapped_file::readwrite;
        if(!boost::filesystem::exists(path))
            fileParams.new_file_size = sizeof(Header) + sizeof(Spike) * nSpikes;
        _map.open(fileParams);
        reinterpret_cast<Header&>(*(_map.data())) = Header();
    }

    ~BinaryReportMap()
    {
        _map.close();
    }

    void resize(const size_t nSpikes)
    {
        if(_map.flags() != boost::iostreams::mapped_file::readwrite)
            BRION_THROW("SpikeReportBinary: Cannot resize, file opened on read only mode")
        // Save whatever is already on the mapped memory
        std::vector<char> tempDataBuffer(_map.size());
        std::memcpy(tempDataBuffer.data(), _map.data(), _map.size() * sizeof(char));
        // Recreate the mapped file with the new size
        _map.close();
        boost::iostreams::mapped_file_params fileParams;
        fileParams.path = _path;
        fileParams.flags = boost::iostreams::mapped_file::readwrite;
        fileParams.new_file_size = sizeof(Header) + sizeof(Spike) * nSpikes;
        _map = boost::iostreams::mapped_file(fileParams);
        // Copy back the saed data
        std::memcpy(_map.data(), tempDataBuffer.data(), tempDataBuffer.size() * sizeof(char));
    }

    size_t getNumSpikes() const
    {
        return (_map.size() - sizeof(Header)) / sizeof(Spike);
    }

    const Spike* getReadableSpikes() const
    {
        return reinterpret_cast<const Spike*>(_map.const_data() + sizeof(Header));
    }

    Spike* getWritableSpikes()
    {
        return reinterpret_cast<Spike*>(_map.data() + sizeof(Header));
    }

private:
    std::string _path;
    boost::iostreams::mapped_file _map;
};

SpikeReportBinary::SpikeReportBinary(const PluginInitData& initData)
    : SpikeReportPlugin(initData)
{
    if (_accessMode == MODE_READ)
        _memFile.reset(new BinaryReportMap(getURI().getPath()));
    else
        _memFile.reset(new BinaryReportMap(getURI().getPath(), 0));

    const Spike* spikeArray = _memFile->getReadableSpikes();
    const size_t nElems = _memFile->getNumSpikes();
    if (nElems != 0)
        _endTime = spikeArray[nElems - 1].first;
}

bool SpikeReportBinary::handles(const PluginInitData& initData)
{
    const URI& uri = initData.getURI();
    if (!uri.getScheme().empty() && uri.getScheme() != "file")
        return false;

    const auto ext = boost::filesystem::path(uri.getPath()).extension();
    return ext == brion::plugin::BINARY_REPORT_FILE_EXT;
}

std::string SpikeReportBinary::getDescription()
{
    return "Blue Brain binary spike reports: "
           "[file://]/path/to/report" +
           std::string(BINARY_REPORT_FILE_EXT);
}

Spikes SpikeReportBinary::read(const float)
{
    // In file based reports, this function reads all remaining data.
    Spikes spikes;
    const Spike* spikeArray = _memFile->getReadableSpikes();
    const size_t nElems = _memFile->getNumSpikes();

    for (; _startIndex < nElems; ++_startIndex)
        pushBack(spikeArray[_startIndex], spikes);

    _currentTime = UNDEFINED_TIMESTAMP;
    _state = State::ended;
    return spikes;
}

Spikes SpikeReportBinary::readUntil(const float max)
{
    Spikes spikes;

    const Spike* spikeArray = _memFile->getReadableSpikes();
    const size_t nElems = _memFile->getNumSpikes();

    for (; _startIndex < nElems; ++_startIndex)
    {
        if (spikeArray[_startIndex].first >= max)
        {
            _currentTime = spikeArray[_startIndex].first;
            break;
        }
        pushBack(spikeArray[_startIndex], spikes);
    }

    if (_startIndex == nElems)
    {
        _currentTime = UNDEFINED_TIMESTAMP;
        _state = State::ended;
    }

    return spikes;
}

void SpikeReportBinary::readSeek(const float toTimeStamp)
{
    const Spike* spikeArray = _memFile->getReadableSpikes();
    const size_t nElems = _memFile->getNumSpikes();

    const Spike* position = nullptr;

    if (toTimeStamp < _currentTime)
    {
        position =
            std::lower_bound(spikeArray, spikeArray + _startIndex, toTimeStamp,
                             [](const Spike& spike, const float val) {
                                 return spike.first < val;
                             });
    }
    else
    {
        position = std::lower_bound(spikeArray + _startIndex,
                                    spikeArray + nElems, toTimeStamp,
                                    [](const Spike& spike, const float val) {
                                        return spike.first < val;
                                    });
    }

    if (position == (spikeArray + nElems)) // end
    {
        _startIndex = nElems;
        _state = State::ended;
        _currentTime = UNDEFINED_TIMESTAMP;
    }
    else
    {
        _state = State::ok;
        _startIndex = std::distance(spikeArray, position);
        _currentTime = toTimeStamp;
    }
}

void SpikeReportBinary::writeSeek(float toTimeStamp)
{
    readSeek(toTimeStamp);
}

void SpikeReportBinary::write(const Spike* spikes, const size_t size)
{
    size_t totalSpikes = _startIndex + size;

    if (size == 0)
        return;

    // create or resize the file
    if (_memFile->getNumSpikes() != totalSpikes)
        _memFile->resize(totalSpikes);

    Spike* spikeArray = _memFile->getWritableSpikes();
    for (size_t i = 0; i != size; ++i)
        spikeArray[_startIndex++] = spikes[i];

    const float lastTimestamp = spikes[size - 1].first;
    _currentTime =
        std::nextafter(lastTimestamp, std::numeric_limits<float>::max());
    _endTime = std::max(_endTime, lastTimestamp);
}
} // namespace plugin
} // namespace brion
