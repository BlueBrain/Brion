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
    std::vector<uint32_t> counts(cellSizes.begin(), cellSizes.end());
    std::nth_element(counts.begin(), counts.begin() + counts.size() / 2,
                     counts.end());
    const auto median = counts[counts.size() / 2];

    size_t frameSize = 0;
    for (const auto size : cellSizes)
        frameSize += size;
    const auto valuesPerBlock = std::min(frameSize, blockSize / 4);

    const auto evenSplit = std::sqrt(valuesPerBlock / median);
    const auto frames =
        hsize_t(std::max(1., std::floor(evenSplit / cellsToFramesRatio)));
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

lunchbox::PluginRegisterer<CompartmentReportHDF5> registerer;
}

CompartmentReportHDF5::CompartmentReportHDF5(
    const CompartmentReportInitData& initData)
    : _startTime(0)
    , _endTime(0)
    , _timestep(0)
    , _file(new HighFive::File(detail::openFile(initData.getURI().getPath(),
                                                initData.getAccessMode())))
{
    HighFive::SilenceHDF5 silence;
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    const int accessMode = initData.getAccessMode();
    if (accessMode == MODE_READ)
    {
        _readMetaData();
        if (initData.initMapping)
        {
            _updateMapping(initData.getGIDs());
        }
        else
        {
            _parseBasicCellInfo();
            _subset = false;
        }
    }
}

CompartmentReportHDF5::~CompartmentReportHDF5()
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    _file.reset();
}

bool CompartmentReportHDF5::handles(const CompartmentReportInitData& initData)
{
    const auto& uri = initData.getURI();
    if (!detail::isHDF5File(uri))
        return false;

    // Checking if the file is a SONATA report
    if ((initData.getAccessMode() & MODE_READ) == 0)
        return true;

    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    HighFive::SilenceHDF5 silence;
    return _verifyFile(
        detail::openFile(initData.getURI().getPath(), MODE_READ, false));
}

std::string CompartmentReportHDF5::getDescription()
{
    return "SONATA HDF5 compartment reports:"
           "  [file://]/path/to/report.(h5|hdf5)";
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
    if (_gids.size() != 1)
    {
        return CompartmentReportCommon::_loadFrames(frameNumber, frameCount,
                                                    buffer);
    }

    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    size_t offset = _sourceMapping.cellOffsets[_subsetIndices[0]];
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
    auto chunkDims = _computeChunkDims(_targetMapping.cellSizes, 5);
    if (chunkDims[0] >= frames)
        // Only apply chunking if the dataset is large enough. Otherwise HDF5
        // gives an error.
        chunking.add(HighFive::Chunking(chunkDims));

    _data.reset(new HighFive::DataSet(
        _file->createDataSet<float>("data", dataspace, chunking)));

    detail::addStringAttribute(*_data, "dunit", _dunit);
}
}
}
