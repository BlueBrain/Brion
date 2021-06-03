/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Juan Hernando <juan.hernando@epfl.ch>
 *                          Nadir Román Guerrero <nadir.romanguerrero@epfl.ch>
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

#include "compartmentReportHDF5Sonata.h"
#include "utilsHDF5.h"

#include "../detail/hdf5Mutex.h"
#include "../detail/utilsHDF5.h"

#include "../log.h"
#include "../pluginLibrary.h"

#include <brion/types.h>
#include <brion/version.h>

#include <boost/filesystem.hpp>
#include <boost/icl/interval.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/scoped_array.hpp>

#include <highfive/H5DataType.hpp>

#include <limits>

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
        auto& pluginManager =
            PluginLibrary::instance().getManager<CompartmentReportPlugin>();
        pluginManager.registerFactory<CompartmentReportHDF5Sonata>();
    }
};

PluginRegisterer registerer;

// constexpr uint32_t _sonataMagic = 0x0A7A;
// constexpr uint32_t _currentVersion[] = {0, 1};

constexpr size_t _autoCacheSize = std::numeric_limits<size_t>::max();

std::vector<hsize_t> _computeChunkDims(const std::vector<uint32_t>& cellSizes,
                                       const float cellsToFramesRatio,
                                       const size_t chunkSize)
{
    if (cellsToFramesRatio == 0.f)
        return {chunkSize / 4, 1};
    else if (std::isinf(cellsToFramesRatio))
        return {1, chunkSize / 4};

    std::vector<uint32_t> counts(cellSizes.begin(), cellSizes.end());
    std::nth_element(counts.begin(), counts.begin() + counts.size() / 2,
                     counts.end());
    const auto median = counts[counts.size() / 2];

    size_t frameSize = 0;
    for (const auto size : cellSizes)
        frameSize += size;
    const auto valuesPerChunk = chunkSize / sizeof(float);

    if (frameSize == 0)
        throw std::runtime_error("Frame size is 0");

    auto frames = hsize_t(
        std::floor(std::sqrt(valuesPerChunk / median / cellsToFramesRatio)));
    // Exceptionally, if the number of frames choosen doesn't fill properly
    // a chunk for the total number of compartments, we increase the number
    // of frames to the maximum possible
    if ((frames * frameSize) < valuesPerChunk)
        return {hsize_t(std::floor(valuesPerChunk / frameSize)), frameSize};
    return {frames, valuesPerChunk / frames};
}

size_t _parseSizeOption(const std::string& value, const std::string& name)
{
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
        std::cerr << "Warning: invalid value for " << name << "  H5 parameter. "
                  << std::endl;
    }
    return size;
}

size_t _parseCacheSizeOption(const URI& uri)
{
    const auto keyValueIter = uri.findQuery("cache_size");
    if (keyValueIter == uri.queryEnd())
        return 0;

    const auto& value = keyValueIter->second;
    if (value == "auto")
        return _autoCacheSize;

    return _parseSizeOption(value, "cache_size");
}

GIDSet gidsToBase0(const GIDSet& src)
{
    GIDSet result;
    std::transform(src.begin(), src.end(), std::inserter(result, result.end()),
                   [](uint32_t gid) -> int {
                       if (gid == 0)
                           throw std::runtime_error(
                               "Tried to substract 1 to 0 based GID");
                       return gid - 1;
                   });

    return result;
}

GIDSet gidsToBase1(const GIDSet& src)
{
    GIDSet result;
    std::transform(src.begin(), src.end(), std::inserter(result, result.end()),
                   [](uint32_t gid) -> int { return gid + 1; });

    return result;
}

} // namespace

CompartmentReportHDF5Sonata::CompartmentReportHDF5Sonata(
    const CompartmentReportInitData& initData)
    : _startTime(0)
    , _endTime(0)
    , _timestep(0)
    , _file(new HighFive::File(
          openFile(initData.getURI().getPath(), initData.getAccessMode())))
{
    BRION_WARN << "The SONATA format support is experimental and not "
                  "officially supported. "
               << "It is encouraged to use libsonata instead" << std::endl;

    HighFive::SilenceHDF5 silence;
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    const int accessMode = initData.getAccessMode();
    if (accessMode == MODE_READ)
    {
        _readMetaData();
        _reopenDataSet(_parseCacheSizeOption(initData.getURI()));
        if (initData.initMapping)
        {
            _updateMapping(gidsToBase0(initData.getGIDs()));
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

CompartmentReportHDF5Sonata::~CompartmentReportHDF5Sonata()
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    _file.reset();
}

bool CompartmentReportHDF5Sonata::handles(
    const CompartmentReportInitData& initData)
{
    const auto& uri = initData.getURI();
    if (!isHDF5File(uri))
        return false;

    // Checking if the file is a SONATA report
    if ((initData.getAccessMode() & MODE_READ) == 0)
        return true;

    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    HighFive::SilenceHDF5 silence;
    std::unique_ptr<HighFive::File> temp(new HighFive::File(
        openFile(initData.getURI().getPath(), MODE_READ, false)));
    return temp->exist("report");
}

std::string CompartmentReportHDF5Sonata::getDescription()
{
    return "SONATA HDF5 compartment reports:  "
           "[file://]/path/to/report.(h5|hdf5)"
           "[?[cache_size=(auto|num_bytes)&][cells_to_frames=(inf|ratio)&]"
           "[chunk_size=bytes]]\n"
           "    Byte counts can by suffixed by K or M.\n"
           "    The cache is disabled by default, auto will reserve space for"
           "a whole frame or trace, whatever is bigger. The actual size depends"
           " on the chunk dimensions. For files with row or column layouts the"
           " auto cache size is limited to 1GB.";
}

size_t CompartmentReportHDF5Sonata::getCellCount() const
{
    return _subset ? _gids.size() : _sourceGIDs.size();
}

const GIDSet& CompartmentReportHDF5Sonata::getGIDs() const
{
    return _subset ? _gids1based : _sourceGIDs1based;
}

const SectionOffsets& CompartmentReportHDF5Sonata::getOffsets() const
{
    return (_subset ? _targetMapping : _sourceMapping).offsets;
}

const CompartmentCounts& CompartmentReportHDF5Sonata::getCompartmentCounts()
    const
{
    return (_subset ? _targetMapping : _sourceMapping).counts;
}

size_t CompartmentReportHDF5Sonata::getNumCompartments(const size_t index) const
{
    return (_subset ? _targetMapping : _sourceMapping).cellSizes[index];
}

size_t CompartmentReportHDF5Sonata::getFrameSize() const
{
    return (_subset ? _targetMapping : _sourceMapping).frameSize;
}

void CompartmentReportHDF5Sonata::updateMapping(const GIDSet& gids)
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    HighFive::SilenceHDF5 silence;

    _updateMapping(gidsToBase0(gids));
}

void CompartmentReportHDF5Sonata::writeHeader(const double startTime,
                                              const double endTime,
                                              const double timestep,
                                              const std::string& dunit,
                                              const std::string& tunit)
{
    std::string assertMessage(
        "Invalid report time " + std::to_string(startTime) + ".." +
        std::to_string(endTime) + "/" + std::to_string(timestep));
    BRION_ASSERT_INFO(endTime - startTime >= timestep, assertMessage.c_str())

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

bool CompartmentReportHDF5Sonata::writeCompartments(const uint32_t gid,
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

bool CompartmentReportHDF5Sonata::writeFrame(const uint32_t gid,
                                             const float* values,
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
            BRION_THROW("Invalid GID for writing to report")
        const size_t index = i - _GIDlist.begin();
        auto selection =
            _data->select({frameNumber, _targetMapping.cellOffsets[index]},
                          {1, _targetMapping.cellSizes[index]});
        // HighFive is not handling const correctly
        selection.write(const_cast<float*>(values));
    }
    catch (const HighFive::Exception& e)
    {
        BRION_ERROR << "CompartmentReportHDF5Sonata: error writing frame: "
                    << e.what() << std::endl;
        return false;
    }
    return true;
}

bool CompartmentReportHDF5Sonata::writeFrame(const GIDSet& gids,
                                             const float* values,
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
        BRION_ERROR << "CompartmentReportHDF5Sonata: error writing frame: "
                    << e.what() << std::endl;
        return false;
    }
    return true;
}

bool CompartmentReportHDF5Sonata::flush()
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());
    _file->flush();
    return true;
}

bool CompartmentReportHDF5Sonata::_loadFrame(const size_t frameNumber,
                                             float* buffer) const
{
    std::lock_guard<std::mutex> mutex(detail::hdf5Mutex());

    std::vector<std::pair<size_t, size_t>> intervals;
    if (!_subset)
    {
        intervals.reserve(_sourceMapping.cellOffsets.size());
        for (size_t i = 0; i < _sourceMapping.cellOffsets.size(); ++i)
        {
            const auto start = _sourceMapping.cellOffsets[i];
            intervals.emplace_back(start, start + _sourceMapping.cellSizes[i]);
        }
    }
    else
    {
        intervals.reserve(_subsetIndices.size());
        for (const auto index : _subsetIndices)
        {
            const auto start = _sourceMapping.cellOffsets[index];
            const auto end = start + _sourceMapping.cellSizes[index];
            intervals.emplace_back(start, end);
        }
    }

    // Reading slice by slice and processing all cells contained in a given
    // slice
    size_t targetOffset = 0;
    for (const auto interval : intervals)
    {
        const auto sourceOffset = interval.first;
        const auto count = interval.second - sourceOffset;
        const auto& slice =
            _data->select({frameNumber, sourceOffset}, {1, count});
        slice.read(buffer + targetOffset);
        targetOffset += count;
    }
    return true;
}

void CompartmentReportHDF5Sonata::_readMetaData()
{
    try
    {
        if (!_file->exist("report"))
            BRION_THROW(
                "Error opening compartment report: No \"report\" group found")

        const HighFive::Group reportGroup = _file->getGroup("report");

        auto objectNameList = reportGroup.listObjectNames();
        if (objectNameList.empty())
        {
            BRION_THROW(
                "Error opening compartment report: "
                "No population found within report group")
        }
        auto firstPopulation = *objectNameList.begin();

        const HighFive::Group allGroup = reportGroup.getGroup(firstPopulation);

        // Opening the dataset temporarily, it will be reopen later with
        // proper chunk cache configuration.
        _data.reset(new HighFive::DataSet(allGroup.getDataSet("data")));
        try
        {
            _data->getAttribute("units").read(_dunit);
        }
        catch (const HighFive::AttributeException&)
        {
            _dunit = "mV";
        }
        const auto& mapping = allGroup.getGroup("mapping");
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
            BRION_THROW("Error opening compartment report: Bad time metadata")

        _startTime = timeData[0];
        _endTime = timeData[1];
        _timestep = timeData[2];

        // Finding the total compartment count
        const auto& dims = _data->getSpace().getDimensions();
        if (dims.size() != 2)
            BRION_THROW("Bad report: data is not 2-dimensional")

        _sourceMapping.frameSize = dims[1];
    }
    catch (std::exception& e)
    {
        BRION_THROW(std::string("Error opening compartment report: ") +
                    std::string(e.what()));
    }
}

void CompartmentReportHDF5Sonata::_parseBasicCellInfo()
{
    const HighFive::Group reportGroup = _file->getGroup("report");
    auto objectNameList = reportGroup.listObjectNames();
    if (objectNameList.empty())
    {
        BRION_THROW(
            "Error opening compartment report: "
            "No population found within report group")
    }
    auto firstPopulation = *objectNameList.begin();

    const HighFive::Group allGroup = reportGroup.getGroup(firstPopulation);

    // This not only parses the GIDs, but also computes the cell offsets
    // and compartment counts per cell, with the proper reordering if needed.
    const auto& mapping = allGroup.getGroup("mapping");
    std::vector<uint32_t> gids;
    std::vector<size_t> offsets;
    const auto& gidsDataSet = mapping.getDataSet("node_ids");
    gidsDataSet.read(gids);
    mapping.getDataSet("index_pointers").read(offsets);
    if ((gids.size() != offsets.size() && gids.size() + 1 != offsets.size()) ||
        gids.empty())
        BRION_THROW("Bad report metadata")
    int8_t sorted = false;
    try
    {
        gidsDataSet.getAttribute("sorted").read(sorted);
    }
    catch (...)
    {
    }
    if ((offsets.size() == gids.size() &&
         offsets.back() >= _sourceMapping.frameSize) ||
        // An extra offset with the total size is required:
        // https://github.com/AllenInstitute/sonata/issues/62
        // The code will support having a missing offset for already existing
        // reports.
        (offsets.size() == gids.size() + 1 &&
         offsets.back() != _sourceMapping.frameSize))
        BRION_THROW("Bad report: inconsistent cell offsets")

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
    _sourceGIDs1based = gidsToBase1(_sourceGIDs);
}

void CompartmentReportHDF5Sonata::_processMapping()
{
    const HighFive::Group reportGroup = _file->getGroup("report");
    auto objectNameList = reportGroup.listObjectNames();
    if (objectNameList.empty())
    {
        BRION_THROW(
            "Error opening compartment report: "
            "No population found within report group")
    }
    auto firstPopulation = *objectNameList.begin();

    const HighFive::Group allGroup = reportGroup.getGroup(firstPopulation);
    const auto& mapping = allGroup.getGroup("mapping");
    std::vector<uint32_t> sectionIDs;
    mapping.getDataSet("element_ids").read(sectionIDs);
    try
    {
        mapping.getDataSet("element_pos");
        BRION_WARN << "Unsupported mapping attribute in compartment "
                   << "report: element_pos" << std::endl;
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
        uint32_t current = std::numeric_limits<uint16_t>().max();
        uint32_t previous = std::numeric_limits<uint16_t>().max();
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
                    offsets.resize(current + 1,
                                   std::numeric_limits<uint64_t>().max());
                    counts.resize(current + 1, 0);
                }

                offsets[current] = offset + j;

                if (previous != std::numeric_limits<uint16_t>().max())
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

void CompartmentReportHDF5Sonata::_updateMapping(const GIDSet& gids)
{
    if (_sourceGIDs.empty())
        _parseBasicCellInfo();
    if (_sourceMapping.offsets.empty())
        _processMapping();

    _subset = !(gids.empty() || gids == _sourceGIDs);

    if (!_subset)
        return;

    const GIDSet intersection = _computeIntersection(_sourceGIDs, gids);
    if (intersection != gids)
    {
        _updateMapping(intersection);
        return;
    }
    _gids = std::move(intersection);
    _gids1based = gidsToBase1(_gids);

    _subsetIndices = _computeSubsetIndices(_sourceGIDs, _gids);
    _targetMapping = _reduceMapping(_sourceMapping, _subsetIndices);
}

void CompartmentReportHDF5Sonata::_writeMetadataAndMapping()
{
    auto root = _file->getGroup("/");
    HighFive::Group reportG = _file->createGroup("report");
    HighFive::Group allG = reportG.createGroup("All");

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
    auto mapping = allG.createGroup("mapping");

    auto time =
        mapping.createDataSet<double>("time", HighFive::DataSpace(
                                                  std::vector<size_t>{3}));
    time.write(std::vector<double>{_startTime, _endTime, _timestep});
    detail::addStringAttribute(time, "units", _tunit);

    auto gidsDataSet = mapping.createDataSet<uint32_t>(
        "node_ids", HighFive::DataSpace(std::vector<size_t>{cellCount}));
    gidsDataSet.write(_GIDlist);
    auto sorted = gidsDataSet.createAttribute<int8_t>(
        "sorted", HighFive::DataSpace(std::vector<size_t>({1})));
    sorted.write(1);

    auto elementDataSet =
        mapping.createDataSet<uint32_t>("element_ids",
                                        HighFive::DataSpace(std::vector<size_t>{
                                            sortedMapping.size()}));
    elementDataSet.write(sortedMapping);

    auto offsetSet = mapping.createDataSet<uint64_t>(
        "index_pointers", HighFive::DataSpace(std::vector<size_t>{
                              _targetMapping.cellOffsets.size()}));
    offsetSet.write(_targetMapping.cellOffsets);
}

void CompartmentReportHDF5Sonata::_allocateDataSet()
{
    HighFive::Group reportG = _file->getGroup("report");
    auto objectNameList = reportG.listObjectNames();
    if (objectNameList.empty())
    {
        BRION_THROW(
            "Error opening compartment report: "
            "No population found within report group")
    }
    auto firstPopulation = *objectNameList.begin();

    HighFive::Group allGroup = reportG.getGroup(firstPopulation);

    const double step = getTimestep();
    // Adding step / 2 to the window to avoid off by 1 errors during truncation
    // after the division.
    const size_t frames = (getEndTime() - getStartTime() + step * 0.5) / step;
    BRION_ASSERT(frames > 0);

    HighFive::DataSpace dataspace(
        std::vector<size_t>{frames, _targetMapping.frameSize});

    HighFive::DataSetCreateProps chunking;
    auto chunkDims = _computeChunkDims(_targetMapping.cellSizes,
                                       _cellsToFramesRatio, _chunkSize);
    if (chunkDims[0] >= frames)
        chunkDims[0] = frames;
    if (chunkDims[1] > _targetMapping.frameSize)
        chunkDims[1] = _targetMapping.frameSize;
    chunking.add(HighFive::Chunking(chunkDims));

    HighFive::DataSetAccessProps caching;
    size_t chunksPerFrame =
        (_targetMapping.frameSize + chunkDims[1] - 1) / chunkDims[1];
    caching.add(
        HighFive::Caching(chunksPerFrame + 1, chunksPerFrame * _chunkSize));

    _data.reset(new HighFive::DataSet(
        allGroup.createDataSet<float>("data", dataspace, chunking, caching)));

    detail::addStringAttribute(*_data, "units", _dunit);
}

void CompartmentReportHDF5Sonata::_parseWriteOptions(const URI& uri)
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
                if (idx != value.size() || ratio < 0)
                {
                    std::cerr << "Warning: invalid value for "
                                 "cells_to_frames H5 report parameter"
                              << std::endl;
                }
                else
                {
                    _cellsToFramesRatio = ratio;
                }
            }
        }
        if (key == "chunk_size")
        {
            const size_t chunkSize = _parseSizeOption(value, "chunk_size");
            if (chunkSize != 0)
                _chunkSize = chunkSize;
        }
    }
}

void CompartmentReportHDF5Sonata::_reopenDataSet(size_t cacheSizeHint)
{
    // Getting the chunk dims
    auto properties = H5Dget_create_plist(_data->getId());
    H5Pget_chunk(properties, 2, _chunkDims);
    H5Pclose(properties);
    // And the frame count
    const size_t frameCount = _data->getSpace().getDimensions()[0];

    if (_chunkDims[0] == 0)
        return; // Nothing to configure for this case.

    // Need to close the dataset first. Otherwise internal H5 reference
    // couning prevents the caching parameters from taking effect.
    _data.reset();

    if (cacheSizeHint == 0)
    {
        // The default cache configuration from HDF5 gives very bad performance
        // because it can only hold 3 chunks and it causes chunk eviction and
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
    // such as chunks[1] ∤ s. At the same time we want chunks[0] ∤ s.
    size_t chunks[2] = {(frameCount + _chunkDims[0] - 1) / _chunkDims[0],
                        (_sourceMapping.frameSize + _chunkDims[1] - 1) /
                            _chunkDims[1]};
    if (chunks[1] > chunks[0])
        std::swap(chunks[0], chunks[1]);
    size_t numSlots = chunks[0];
    if (chunks[0] != 1 && chunks[1] != 1)
    {
        // Making numSlots % chunks[0] == 1. Since numSlots == chunks[0], we
        // only have to add 1
        ++numSlots;
        // Now making numSlots not divisible by chunks[1]
        if (numSlots % chunks[1])
            ++numSlots;
    }

    if (cacheSizeHint == _autoCacheSize)
    {
        // Auto adjusting cache size to fit the largest of a frame or trace.
        const size_t chunkSize = _chunkDims[0] * _chunkDims[1] * 4;
        if (_chunkDims[0] == 1 || _chunkDims[1])
        {
            // For column and row layouts we limits the cache to the largest
            // amount of chunks that fit in 1 GiB. Otherwise the cache would
            // be as large as the dataset.
            cacheSizeHint = (1 << 20 / chunkSize) * chunkSize;
        }
        else
        {
            // Ideally we should use the _targetMapping.frameSize to compute the
            // number of chunks needed by a frame, but with the current design,
            // sharing the dataset cache between different
            // CompartmentReportViews would become impossible. If needed, the
            // user can always adjust the cache manually.
            cacheSizeHint = chunks[0] * chunkSize;
        }
    }

    HighFive::DataSetAccessProps accessProps;
    accessProps.add(HighFive::Caching(numSlots, cacheSizeHint));
    HighFive::Group reportG = _file->getGroup("report");
    auto objectNameList = reportG.listObjectNames();
    if (objectNameList.empty())
    {
        BRION_THROW(
            "Error opening compartment report: "
            "No population found within report group")
    }
    auto firstPopulation = *objectNameList.begin();
    HighFive::Group allG = reportG.getGroup(firstPopulation);
    _data.reset(new HighFive::DataSet(allG.getDataSet("data", accessProps)));
}

} // namespace plugin
} // namespace brion
