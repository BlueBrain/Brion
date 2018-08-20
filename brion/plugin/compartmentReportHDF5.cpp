/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Juan Hernando <juan.hernando@epfl.ch>
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

#include "compartmentReportHDF5.h"
#include "utilsHDF5.h"

#include "../detail/hdf5Mutex.h"
#include "../detail/utilsHDF5.h"

#include <brion/types.h>
#include <brion/version.h>

#include <boost/filesystem.hpp>
#include <boost/icl/interval.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/scoped_array.hpp>

#include <highfive/H5Utility.hpp>

#include <lunchbox/debug.h>
#include <lunchbox/pluginRegisterer.h>

namespace brion
{
namespace plugin
{
namespace
{
const uint32_t _sonataMagic = 0x0A7A;
const uint32_t _currentVersion[] = {0, 1};

const size_t _blockSize = 1024 * 1024;

std::vector<hsize_t> _computeChunkDims(const std::vector<uint32_t>& cellSizes,
                                       float cellsToFramesRatio,
                                       size_t blockSize = _blockSize)
{
    if (cellsToFramesRatio == 0)
        return {blockSize / 4, 1};
    else if (std::isinf(cellsToFramesRatio))
        return {1, blockSize / 4};

    std::vector<uint32_t> counts(cellSizes.begin(), cellSizes.end());
    std::nth_element(counts.begin(), counts.begin() + counts.size() / 2,
                     counts.end());
    const auto median = counts[counts.size() / 2];

    size_t frameSize = 0;
    for (const auto size : cellSizes)
        frameSize += size;
    const auto valuesPerBlock = blockSize / sizeof(float);

    auto frames = hsize_t(
        std::floor(std::sqrt(valuesPerBlock / median / cellsToFramesRatio)));
    // Exceptionally, if the number of frames choosen doesn't fill properly
    // a block for the total number of compartments, we increase the number
    // of frames to the maximum possible
    if ((frames * frameSize) < valuesPerBlock)
        return {hsize_t(std::floor(valuesPerBlock / frameSize)), frameSize};
    return {frames, valuesPerBlock / frames};
}

bool _verifyFile(const HighFive::File& file)
{
    try
    {
        uint32_t magic = 0;
        file.getAttribute("magic").read(magic);
        if (magic != _sonataMagic)
            return false;

        std::vector<uint32_t> version;
        file.getAttribute("version").read(version);
        if (version.size() != 2 || version[0] != _currentVersion[0] ||
            version[1] != _currentVersion[1])
            return false;
    }
    catch (HighFive::Exception& e)
    {
        return false;
    }
    return true;
}

size_t _parseCacheSizeOption(const URI& uri)
{
    const auto i = uri.findQuery("cache_size");
    if (i == uri.queryEnd())
        return 0;

    const auto& value = i->second;
    if (value == "auto")
        return std::numeric_limits<size_t>::max();

    std::size_t pos;
    size_t size = std::stoll(value, &pos);
    if (pos == value.size() - 1)
    {
        if (value[pos] == 'K')
            size *= 1024;
        else if (value[pos] == 'M')
            size *= 1024 * 1024;
        else
            size = 0;
    }
    if (size == 0 and pos != value.size())
    {
        std::cerr << "Warning: invalid value for cache_size H5 parameter. "
                     "Accepted values are auto or a positive number optionally "
                     "followed"
                     " by the letters M or K"
                  << std::endl;
    }
    return size;
}

lunchbox::PluginRegisterer<CompartmentReportHDF5> registerer;
}

CompartmentReportHDF5::CompartmentReportHDF5(
    const CompartmentReportInitData& initData)
    : _startTime(0)
    , _endTime(0)
    , _timestep(0)
    , _file(new HighFive::File(
          openFile(initData.getURI().getPath(), initData.getAccessMode())))
{
    HighFive::SilenceHDF5 silence;
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    const int accessMode = initData.getAccessMode();
    if (accessMode == MODE_READ)
    {
        _readMetaData();
        _reopenDataSet(_parseCacheSizeOption(initData.getURI()));
        if (initData.initMapping)
        {
            _updateMapping(initData.getGIDs());
        }
        else
        {
            _parseBasicCellInfo();
            _subset = false;
        }
        return;
    }
    _parseWriteOptions(initData.getURI());
}

CompartmentReportHDF5::~CompartmentReportHDF5()
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    _file.reset();
}

bool CompartmentReportHDF5::handles(const CompartmentReportInitData& initData)
{
    const auto& uri = initData.getURI();
    if (!isHDF5File(uri))
        return false;

    // Checking if the file is a SONATA report
    if ((initData.getAccessMode() & MODE_READ) == 0)
        return true;

    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    HighFive::SilenceHDF5 silence;
    return _verifyFile(openFile(initData.getURI().getPath(), MODE_READ, false));
}

std::string CompartmentReportHDF5::getDescription()
{
    return "SONATA HDF5 compartment reports:"
           "  "
           "[file://]/path/to/"
           "report.(h5|hdf5)[?[cache_size=(auto|num_bytes):][cell_to_frames="
           "ratio]]";
}

size_t CompartmentReportHDF5::getCellCount() const
{
    return _subset ? _gids.size() : _sourceGIDs.size();
}

const GIDSet& CompartmentReportHDF5::getGIDs() const
{
    return _subset ? _gids : _sourceGIDs;
}

const SectionOffsets& CompartmentReportHDF5::getOffsets() const
{
    return (_subset ? _targetMapping : _sourceMapping).offsets;
}

const CompartmentCounts& CompartmentReportHDF5::getCompartmentCounts() const
{
    return (_subset ? _targetMapping : _sourceMapping).counts;
}

size_t CompartmentReportHDF5::getNumCompartments(const size_t index) const
{
    return (_subset ? _targetMapping : _sourceMapping).cellSizes[index];
}

size_t CompartmentReportHDF5::getFrameSize() const
{
    return (_subset ? _targetMapping : _sourceMapping).frameSize;
}

void CompartmentReportHDF5::updateMapping(const GIDSet& gids)
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    HighFive::SilenceHDF5 silence;

    _updateMapping(gids);
}

void CompartmentReportHDF5::writeHeader(const double startTime,
                                        const double endTime,
                                        const double timestep,
                                        const std::string& dunit,
                                        const std::string& tunit)
{
    LBASSERTINFO(endTime - startTime >= timestep,
                 "Invalid report time " << startTime << ".." << endTime << "/"
                                        << timestep);
    if (timestep <= 0.f)
    {
        std::ostringstream msg;
        msg << "Timestep is not > 0.0, got " << timestep;
        throw std::invalid_argument(msg.str());
    }
    _startTime = startTime;
    _endTime = endTime;
    _timestep = timestep;
    _dunit = dunit;
    _tunit = tunit;
}

bool CompartmentReportHDF5::writeCompartments(const uint32_t gid,
                                              const uint16_ts& counts)
{
    // Storing the mapping data temporarily until the first frame is inserted
    _GIDlist.push_back(gid);
    uint32_t sectionID = 0;
    for (const auto count : counts)
    {
        for (size_t i = 0; i != count; ++i)
            _elementIDs.push_back(sectionID);
        ++sectionID;
    }
    _targetMapping.cellOffsets.push_back(_targetMapping.frameSize);
    _targetMapping.cellSizes.push_back(_elementIDs.size() -
                                       _targetMapping.frameSize);
    _targetMapping.frameSize = _elementIDs.size();

    return true;
}

bool CompartmentReportHDF5::writeFrame(const uint32_t gid, const float* values,
                                       const size_t /*size*/,
                                       const double timestamp)
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    if (!_data)
    {
        _writeMetadataAndMapping();
        _allocateDataSet();
    }

    try
    {
        const size_t frameNumber = _getFrameNumber(timestamp);
        const auto i = std::lower_bound(_GIDlist.begin(), _GIDlist.end(), gid);
        if (i == _GIDlist.end() || *i != gid)
            LBTHROW(std::runtime_error("Invalid GID for writing to report"));
        const size_t index = i - _GIDlist.begin();
        auto selection =
            _data->select({frameNumber, _targetMapping.cellOffsets[index]},
                          {1, _targetMapping.cellSizes[index]});
        // HighFive is not handling const correctly
        selection.write(const_cast<float*>(values));
    }
    catch (const HighFive::Exception& e)
    {
        LBERROR << "CompartmentReportHDF5: error writing frame: " << e.what()
                << std::endl;
        return false;
    }
    return true;
}

bool CompartmentReportHDF5::writeFrame(const GIDSet& gids, const float* values,
                                       const size_ts& sizes,
                                       const double timestamp)
{
    if (gids.empty())
        return true;

    // If the frame is complete we can write it directly to the output,
    // otherwise we call the base class method that goes cell by cell.
    size_t index = 0;
    for (const auto gid : gids)
    {
        if (gid != _GIDlist[index])
            return CompartmentReportCommon::writeFrame(gids, values, sizes,
                                                       timestamp);
        ++index;
    }

    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    if (!_data)
    {
        _writeMetadataAndMapping();
        _allocateDataSet();
    }
    try
    {
        const size_t frameNumber = _getFrameNumber(timestamp);
        auto selection =
            _data->select({frameNumber, 0}, {1, _targetMapping.frameSize});
        // HighFive is not handling const correctly
        selection.write(const_cast<float*>(values));
    }
    catch (const HighFive::Exception& e)
    {
        LBERROR << "CompartmentReportHDF5: error writing frame: " << e.what()
                << std::endl;
        return false;
    }
    return true;
}

bool CompartmentReportHDF5::flush()
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    _file->flush();
    return true;
}

bool CompartmentReportHDF5::_loadFrame(const size_t frameNumber,
                                       float* buffer) const
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    // Considering the case of full frames first
    if (!_subset)
    {
        _data->select({frameNumber, 0}, {1, _sourceMapping.frameSize})
            .read(buffer);
        return true;
    }

    // Computing the slices of data to be read
    boost::icl::interval_set<size_t> intervals;
    for (const auto index : _subsetIndices)
    {
        auto start = _sourceMapping.cellOffsets[index];
        auto end = start + _sourceMapping.cellSizes[index];
        auto interval = boost::icl::interval<size_t>::right_open(start, end);
        intervals.insert(interval);
    }

    // Reading slice by slice and processing all cells contained in a given
    // slice
    size_t targetOffset = 0;
    for (const auto interval : intervals)
    {
        const auto sourceOffset = boost::icl::lower(interval);
        const auto count = boost::icl::upper(interval) - sourceOffset;
        const auto& slice =
            _data->select({frameNumber, sourceOffset}, {1, count});
        slice.read(buffer + targetOffset);
        targetOffset += count;
    }
    return true;
}

bool CompartmentReportHDF5::_loadFrames(size_t frameNumber, size_t frameCount,
                                        float* buffer) const
{
    if (_subset && _gids.size() != 1)
    {
        return CompartmentReportCommon::_loadFrames(frameNumber, frameCount,
                                                    buffer);
    }

    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    const size_t offset = _gids.size() == 1 ?
        _sourceMapping.cellOffsets[_subsetIndices[0]] : 0;

    const auto& slice = _data->select({frameNumber, offset},
                                      {frameCount, _targetMapping.frameSize});
    slice.read(buffer);
    return true;
}

void CompartmentReportHDF5::_readMetaData()
{
    try
    {
        if (!_verifyFile(*_file))
        {
            LBTHROW(
                std::runtime_error("Error opening compartment report: not"
                                   " a SONATA compartment report"));
        }

        // Opening the dataset temporarily, it will be reopen later with
        // proper chunk cache configuration.
        _data.reset(new HighFive::DataSet(_file->getDataSet("data")));
        try
        {
            _data->getAttribute("units").read(_dunit);
        }
        catch (const HighFive::AttributeException&)
        {
            _dunit = "mV";
        }
        const auto& mapping = _file->getGroup("mapping");
        const auto& time = mapping.getDataSet("time");
        try
        {
            time.getAttribute("units").read(_tunit);
        }
        catch (const HighFive::AttributeException&)
        {
            _tunit = "ms";
        }

        std::vector<double> timeData;
        time.read(timeData);
        if (timeData.size() != 3)
        {
            LBTHROW(std::runtime_error(
                "Error opening compartment report: Bad time metadata"));
        }
        _startTime = timeData[0];
        _endTime = timeData[1];
        _timestep = timeData[2];

        // Finding the total compartment count
        const auto& dims = _data->getSpace().getDimensions();
        if (dims.size() != 2)
            LBTHROW(
                std::runtime_error("Bad report: data is not 2-dimensional"));
        _sourceMapping.frameSize = dims[1];
    }
    catch (std::exception& e)
    {
        throw std::runtime_error(
            std::string("Error opening compartment report: ") + e.what());
    }
}

void CompartmentReportHDF5::_parseBasicCellInfo()
{
    // This not only parses the GIDs, but also computes the cell offsets
    // and compartment counts per cell, with the proper reordering if needed.
    const auto& mapping = _file->getGroup("mapping");
    std::vector<uint32_t> gids;
    std::vector<size_t> offsets;
    const auto& gidsDataSet = mapping.getDataSet("gids");
    gidsDataSet.read(gids);
    mapping.getDataSet("index_pointer").read(offsets);
    if (gids.size() != offsets.size() || gids.empty())
        LBTHROW(std::runtime_error("Bad report metadata"));
    int8_t sorted = false;
    try
    {
        gidsDataSet.getAttribute("sorted").read(sorted);
    }
    catch (...)
    {
    }
    if (*offsets.rbegin() >= _sourceMapping.frameSize)
        LBTHROW(std::runtime_error("Bad report: inconsistent cell offsets"));

    auto cellCount = gids.size();

    // Computing the compartments per cell
    std::vector<uint32_t> cellSizes;
    cellSizes.reserve(cellCount);
    for (size_t i = 0; i != cellCount - 1; ++i)
        cellSizes.push_back(offsets[i + 1] - offsets[i]);
    cellSizes.push_back(_sourceMapping.frameSize - offsets[cellCount - 1]);

    if (sorted)
    {
        _sourceGIDs = GIDSet(gids.begin(), gids.end());
        _sourceMapping.cellOffsets = std::move(offsets);
        _sourceMapping.cellSizes = std::move(cellSizes);
    }
    else
    {
        // Finding the sorting indices
        std::vector<uint32_t> indices(gids.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
                  [gids](const uint32_t a, const uint32_t b) {
                      return gids[a] < gids[b];
                  });

        std::vector<uint32_t> sortedGIDs(cellCount);
        _sourceMapping.cellOffsets.resize(cellCount);
        _sourceMapping.cellSizes.resize(cellCount);
        for (size_t i = 0; i != cellCount; ++i)
        {
            const auto index = indices[i];
            sortedGIDs[i] = gids[index];
            _sourceMapping.cellOffsets[i] = offsets[index];
            _sourceMapping.cellSizes[i] = cellSizes[index];
        }
        _sourceGIDs = GIDSet(sortedGIDs.begin(), sortedGIDs.end());
    }
}

void CompartmentReportHDF5::_processMapping()
{
    const auto& mapping = _file->getGroup("mapping");
    std::vector<uint32_t> sectionIDs;
    mapping.getDataSet("element_id").read(sectionIDs);
    try
    {
        mapping.getDataSet("element_pos");
        LBWARN << "Unsupported mapping attribute in compartment"
                  " report: element_pos"
               << std::endl;
    }
    catch (...)
    {
    }

    const auto cellCount = _sourceGIDs.size();
    _sourceMapping.offsets.resize(cellCount);
    _sourceMapping.counts.resize(cellCount);

// With 8 threads there's some speed up, but very small. We stick with 6
// to avoid using more than 1 socket.
#pragma omp parallel for num_threads(6) schedule(dynamic)
    for (size_t idx = 0; idx < _sourceGIDs.size(); ++idx)
    {
        uint32_t current = LB_UNDEFINED_UINT16;
        uint32_t previous = LB_UNDEFINED_UINT16;
        uint16_t count = 0;

        // < sectionID, < frameIndex, numCompartments > >
        std::vector<uint64_t> offsets;
        std::vector<uint16_t> counts;
        offsets.reserve(32);
        counts.reserve(32);

        const auto offset = _sourceMapping.cellOffsets[idx];
        for (uint32_t j = 0; j < _sourceMapping.cellSizes[idx]; ++j)
        {
            previous = current;
            current = sectionIDs[offset + j];
            assert(current < 65536);

            // in case this is the start of a new section
            if (current != previous)
            {
                if (offsets.size() <= current)
                {
                    offsets.resize(current + 1, LB_UNDEFINED_UINT64);
                    counts.resize(current + 1, 0);
                }

                offsets[current] = offset + j;

                if (previous != LB_UNDEFINED_UINT16)
                    counts[previous] = count;

                count = 0;
            }
            ++count;
        }
        counts[current] = count;

        _sourceMapping.offsets[idx] = std::move(offsets);
        _sourceMapping.counts[idx] = std::move(counts);
    }
}

void CompartmentReportHDF5::_updateMapping(const GIDSet& gids)
{
    if (_sourceGIDs.empty())
        _parseBasicCellInfo();
    if (_sourceMapping.offsets.empty())
        _processMapping();

    _subset = !(gids.empty() || gids == _sourceGIDs);

    if (!_subset)
        return;

    const GIDSet intersection = _computeIntersection(_sourceGIDs, gids);
    if (intersection.empty())
    {
        LBTHROW(std::runtime_error(
            "CompartmentReportBinary::updateMapping: GIDs out of range"));
    }
    if (intersection != gids)
    {
        _updateMapping(intersection);
        return;
    }
    _gids = std::move(intersection);

    _subsetIndices = _computeSubsetIndices(_sourceGIDs, _gids);
    _targetMapping = _reduceMapping(_sourceMapping, _subsetIndices);
}

void CompartmentReportHDF5::_writeMetadataAndMapping()
{
    auto root = _file->getGroup("/");

    // Writing metadata

    auto version = _file->createAttribute<uint32_t>(
        "version", HighFive::DataSpace(std::vector<size_t>({2})));
    version.write(_currentVersion);

    auto magic = _file->createAttribute<uint32_t>(
        "magic", HighFive::DataSpace(std::vector<size_t>({1})));
    magic.write(_sonataMagic);

    detail::addStringAttribute(*_file, "creator", "Brion");
    detail::addStringAttribute(*_file, "software_version", BRION_REV_STRING);

    const time_t now = ::time(0);
#ifdef _WIN32
    char* gmtString = ::ctime(&now);
#else
    char gmtString[32];
    ::ctime_r(&now, gmtString);
#endif
    std::string creationTimeName = gmtString;
    // ctime_r ends with \n
    creationTimeName = creationTimeName.substr(0, creationTimeName.size() - 1);
    detail::addStringAttribute(*_file, "creation_time", creationTimeName);

    // Processing the mapping to be sorted by GID
    size_t cellCount = _GIDlist.size();
    std::vector<uint32_t> indices(cellCount);
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [this](const uint32_t a, const uint32_t b) {
                  return _GIDlist[a] < _GIDlist[b];
              });

    std::vector<uint32_t> gids;
    std::vector<size_t> cellOffsets;
    std::vector<uint32_t> cellSizes;
    std::vector<uint32_t> sortedMapping;
    gids.reserve(cellCount);
    cellSizes.reserve(cellCount);
    cellOffsets.reserve(cellCount);
    sortedMapping.reserve(_elementIDs.size());
    for (auto i : indices)
    {
        gids.push_back(_GIDlist[i]);
        const size_t size = _targetMapping.cellSizes[i];
        cellSizes.push_back(size);
        cellOffsets.push_back(sortedMapping.size());
        const size_t start = _targetMapping.cellOffsets[i];
        for (size_t j = start; j != start + size; ++j)
            sortedMapping.push_back(_elementIDs[j]);
    }
    _GIDlist = std::move(gids);
    _elementIDs = std::vector<uint32_t>();
    _targetMapping.cellSizes = std::move(cellSizes);
    _targetMapping.cellOffsets = std::move(cellOffsets);

    // Writing the mapping datasets
    auto mapping = _file->createGroup("mapping");

    auto time =
        mapping.createDataSet<double>("time", HighFive::DataSpace(
                                                  std::vector<size_t>{3}));
    time.write(std::vector<double>{_startTime, _endTime, _timestep});
    detail::addStringAttribute(time, "tunit", _tunit);

    auto gidsDataSet = mapping.createDataSet<uint32_t>(
        "gids", HighFive::DataSpace(std::vector<size_t>{cellCount}));
    gidsDataSet.write(_GIDlist);
    auto sorted = gidsDataSet.createAttribute<int8_t>(
        "sorted", HighFive::DataSpace(std::vector<size_t>({1})));
    sorted.write(1);

    auto elementDataSet =
        mapping.createDataSet<uint32_t>("element_id",
                                        HighFive::DataSpace(std::vector<size_t>{
                                            sortedMapping.size()}));
    elementDataSet.write(sortedMapping);

    auto offsetSet = mapping.createDataSet<uint64_t>(
        "index_pointer", HighFive::DataSpace(std::vector<size_t>{
                             _targetMapping.cellOffsets.size()}));
    offsetSet.write(_targetMapping.cellOffsets);
}

void CompartmentReportHDF5::_allocateDataSet()
{
    const double step = getTimestep();
    // Adding step / 2 to the window to avoid off by 1 errors during truncation
    // after the division.
    const size_t frames = (getEndTime() - getStartTime() + step * 0.5) / step;
    LBASSERT(frames > 0);

    HighFive::DataSpace dataspace(
        std::vector<size_t>{frames, _targetMapping.frameSize});

    HighFive::DataSetCreateProps chunking;
    auto chunkDims =
        _computeChunkDims(_targetMapping.cellSizes, _cellsToFramesRatio);
    if (chunkDims[0] >= frames)
        chunkDims[0] = frames;
    if (chunkDims[1] > _targetMapping.frameSize)
        chunkDims[1] = _targetMapping.frameSize;
    chunking.add(HighFive::Chunking(chunkDims));

    HighFive::DataSetAccessProps caching;
    size_t blocksPerFrame =
        (_targetMapping.frameSize + chunkDims[1] - 1) / chunkDims[1];
    caching.add(
        HighFive::Caching(blocksPerFrame * _blockSize, blocksPerFrame + 1));

    _data.reset(new HighFive::DataSet(
        _file->createDataSet<float>("data", dataspace, chunking, caching)));

    detail::addStringAttribute(*_data, "dunit", _dunit);
}

void CompartmentReportHDF5::_parseWriteOptions(const URI& uri)
{
    for (auto i = uri.queryBegin(); i != uri.queryEnd(); ++i)
    {
        const auto& key = i->first;
        const auto& value = i->second;
        if (key == "cells_to_frames")
        {
            if (value == "inf")
            {
                _cellsToFramesRatio = std::numeric_limits<float>::infinity();
            }
            else
            {
                std::string::size_type idx;
                const float ratio = std::stof(value, &idx);
                if (idx != value.size())
                {
                    std::cerr << "Warning: invalid value for "
                                 "cell_to_frames H5 report parameter"
                              << std::endl;
                }
                else
                {
                    _cellsToFramesRatio = ratio;
                }
            }
        }
    }
}

void CompartmentReportHDF5::_reopenDataSet(size_t cacheSizeHint)
{
    // Getting the chunk dims
    auto properties = H5Dget_create_plist(_data->getId());
    hsize_t chunkDims[2];
    H5Pget_chunk(properties, 2, chunkDims);
    H5Pclose(properties);
    // And the frame count
    const size_t frameCount = _data->getSpace().getDimensions()[0];

    if (chunkDims[0] == 0)
        return; // Nothing to configure for this case.

    // Need to close the dataset first. Otherwise internal H5 reference
    // couning prevents the caching parameters from taking effect.
    _data.reset();

    if (cacheSizeHint == 0)
    {
        // The default cache configuration from HDF5 gives very bad performance
        // because it can only hold 3 blocks and it causes block eviction and
        // re-reads all the time. The preferred behaviour if not hint is given
        // is to disable the cache.
        HighFive::DataSetAccessProps accessProps;
        accessProps.add(HighFive::Caching(0, 0));
        _data.reset(
            new HighFive::DataSet(_file->getDataSet("data", accessProps)));
        return;
    }

    // We pick the number of slots so it can at least hold a full trace or
    // a full frame. Note that the maximum cache size allowed by this number
    // and the actual maximum cache size are independent.
    // To avoid collisions reading traces we also need a number of slots s,
    // such as blocks[1] ∤ s. At the same time we want blocks[0] ∤ s.
    size_t blocks[2] = {(frameCount + chunkDims[0] - 1) / chunkDims[0],
                        (_sourceMapping.frameSize + chunkDims[1] - 1) /
                            chunkDims[1]};
    if (blocks[1] > blocks[0])
        std::swap(blocks[0], blocks[1]);

    if (cacheSizeHint == std::numeric_limits<size_t>::max())
    {
        // Auto adjusting cache size to fit the largest of a frame or trace
        // Ideally we should use the _targetMapping.frameSize to compute the
        // number of blocks needed by a frame, but with the current design,
        // sharing the dataset cache between different CompartmentReportViews
        // would become impossible. If needed, the user can always adjust the
        // cache manually.
        cacheSizeHint = blocks[0] * _blockSize;
    }

    size_t numSlots = blocks[0];
    if (blocks[0] != 1 && blocks[1] != 1)
    {
        // Making numSlots % blocks[0] == 1. Since numSlots == blocks[0], we
        // only have to add 1
        ++numSlots;
        // Now making numSlots not divisible by blocks[1]
        if (numSlots % blocks[1])
            ++numSlots;
    }

    HighFive::DataSetAccessProps accessProps;
    accessProps.add(HighFive::Caching(cacheSizeHint, numSlots));
    _data.reset(new HighFive::DataSet(_file->getDataSet("data", accessProps)));
}
}
}
