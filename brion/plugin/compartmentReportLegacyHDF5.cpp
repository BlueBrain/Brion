/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
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

#include "compartmentReportLegacyHDF5.h"
#include "utilsHDF5.h"

#include "../detail/hdf5Mutex.h"
#include "../detail/utilsHDF5.h"

#include "../log.h"
#include "../pluginLibrary.h"

#include <brion/version.h>

#include <boost/filesystem.hpp>
#include <boost/scoped_array.hpp>

#include <highfive/H5Utility.hpp>

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
        auto& pluginManager = PluginLibrary::instance().getManager<CompartmentReportPlugin>();
        pluginManager.registerFactory<CompartmentReportLegacyHDF5>();
    }
};

PluginRegisterer registerer;
}

// Computes compartment counts for a section within a cell inside an ordered
// offset based compartment mapping (see CompartmentReportMapping).
uint16_t calcCompartmentCounts(SectionOffsets::const_iterator cell,
                               uint64_ts::const_iterator section,
                               const SectionOffsets& mapping,
                               const size_t totalCompartments)
{
    const uint64_t firstIndex = *section;
    if (firstIndex == std::numeric_limits<uint64_t>().max()) // Detecting empty sections
        return 0;

    // Skipping sections that don't appear in the report. Right now, this is not
    // very bad since we will only have to skip sections when we ask for the
    // second axon section, due to the remaining axon sections not being
    // reported.
    uint64_t lastIndex = std::numeric_limits<uint64_t>().max();
    for (++section; section != cell->end() && lastIndex == std::numeric_limits<uint64_t>().max();
         ++section)
    {
        lastIndex = *section;
    }

    if (lastIndex == std::numeric_limits<uint64_t>().max())
    {
        // We reached the end of the neuron without finding a section with valid
        // offset. We need to search for the first offset of the next neuron (if
        // any). We cannot assume that the first section always has a defined
        // offset because that's not the case for the "synapse" reports.
        while (++cell != mapping.end() && cell->empty())
        {
        }

        if (cell != mapping.end())
        {
            for (auto i : *cell)
            {
                if (i != std::numeric_limits<uint64_t>().max())
                {
                    lastIndex = i;
                    break;
                }
            }
            BRION_ASSERT(lastIndex != std::numeric_limits<uint64_t>().max());
        }
        else
            // No cell with mapping found after the initial one.
            lastIndex = totalCompartments;
    }

    BRION_ASSERT(lastIndex - firstIndex > 0 &&
               lastIndex - firstIndex <= std::numeric_limits<size_t>::max());
    return uint16_t(lastIndex - firstIndex);
}

const std::string mappingDatasetName("mapping");
const std::string dataDatasetName("data");

const std::string mappingAttributes[] = {"type", "sections", "soma",
                                         "axon", "basal",    "apic"};
const std::string dataAttributes[] = {"rank", "tstart", "tstop",
                                      "Dt",   "dunit",  "tunit"};

CompartmentReportLegacyHDF5::CompartmentReportLegacyHDF5(
    const CompartmentReportInitData& initData)
    : _startTime(0)
    , _endTime(0)
    , _timestep(0)
    , _comps(0)
    , _path(initData.getURI().getPath())
    , _reportName(boost::filesystem::basename(_path))
    , _file(new HighFive::File(
          openFile(initData.getURI().getPath(), initData.getAccessMode())))
{
    const int accessMode = initData.getAccessMode();

    std::lock_guard<std::mutex> lock(detail::hdf5Mutex());
    HighFive::SilenceHDF5 silence;

    if (accessMode == MODE_READ)
        _readMetaData(*_file);

    if (initData.initMapping)
        _updateMapping(initData.getGIDs());
}

CompartmentReportLegacyHDF5::~CompartmentReportLegacyHDF5()
{
    std::lock_guard<std::mutex> lock(detail::hdf5Mutex());
    _file.reset();
    _datas.clear();
}

bool CompartmentReportLegacyHDF5::handles(
    const CompartmentReportInitData& initData)
{
    const auto& uri = initData.getURI();
    if (!isHDF5File(uri))
        return false;

    if (initData.getAccessMode() != MODE_READ)
        // Reject using the old h5 reports for writing.
        return false;

    // Checking if the file is an old-style h5 report
    try
    {
        // Making sure the mapping is not initialized
        CompartmentReportLegacyHDF5(
            CompartmentReportInitData(initData.getURI()));
    }
    catch (const std::runtime_error&)
    {
        return false;
    }
    return true;
}

std::string CompartmentReportLegacyHDF5::getDescription()
{
    return "Blue Brain HDF5 compartment reports:"
           "  [file://]/path/to/report.(h5|hdf5)";
}

size_t CompartmentReportLegacyHDF5::getCellCount() const
{
    if (_gids.empty())
        return _file->getNumberObjects();
    return _gids.size();
}

const GIDSet& CompartmentReportLegacyHDF5::getGIDs() const
{
    if (_gids.empty())
        _readGIDs();
    return _gids;
}

const SectionOffsets& CompartmentReportLegacyHDF5::getOffsets() const
{
    return _offsets;
}

const CompartmentCounts& CompartmentReportLegacyHDF5::getCompartmentCounts()
    const
{
    return _counts;
}

size_t CompartmentReportLegacyHDF5::getFrameSize() const
{
    return _comps;
}

bool CompartmentReportLegacyHDF5::_loadFrame(const size_t frameNumber,
                                             float* buffer) const
{
    std::lock_guard<std::mutex> lock(detail::hdf5Mutex());

    size_t cellIndex = 0;
    size_t destOffset = 0;
    for (auto cellID : getGIDs())
    {
        const auto& dataset = _datas.find(cellID)->second;
        const size_t compartments = getNumCompartments(cellIndex);
        const auto& selection =
            dataset.select({frameNumber, 0}, {1, compartments});

        // Deceiving HighFive into believing this is a two dimensional buffer
        float* ptr = buffer + destOffset;
        selection.read(ptr);

        ++cellIndex;
        destOffset += compartments;
    }
    return true;
}

void CompartmentReportLegacyHDF5::updateMapping(const GIDSet& gids)
{
    std::lock_guard<std::mutex> lock(detail::hdf5Mutex());
    _updateMapping(gids);
}

void CompartmentReportLegacyHDF5::writeHeader(const double startTime,
                                              const double endTime,
                                              const double timestep,
                                              const std::string& dunit,
                                              const std::string& tunit)
{
    BRION_ASSERT_INFO(endTime - startTime >= timestep,
                    "Invalid report time " + std::to_string(startTime) + ".." 
                    + std::to_string(endTime) + "/" + std::to_string(timestep));
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

    _createMetaData();
}

bool CompartmentReportLegacyHDF5::writeCompartments(const uint32_t gid,
                                                    const uint16_ts& counts)
{
    std::lock_guard<std::mutex> lock(detail::hdf5Mutex());

    try
    {
        const size_t compCount =
            std::accumulate(counts.begin(), counts.end(), 0);
        BRION_ASSERT(!counts.empty());
        BRION_ASSERT_INFO((compCount > 0), "No compartments for GID " + std::to_string(gid));
        HighFive::DataSet dataset = _createDataset(gid, compCount);

        const size_t sections = counts.size();
        BRION_ASSERT(sections > 0);
        dataset.getAttribute(mappingAttributes[1]).write(sections);

        boost::multi_array<float, 2> mapping(boost::extents[1][compCount]);
        size_t i = 0;
        for (size_t j = 0; j < counts.size(); ++j)
            for (size_t k = 0; k < counts[j]; ++k)
                mapping[0][i++] = j;

        dataset.write(mapping.data());
        return true;
    }
    catch (const HighFive::Exception& e)
    {
        BRION_ERROR << "CompartmentReportLegacyHDF5: error writing mapping: "
                  << e.what() << std::endl;
    }
    return false;
}

bool CompartmentReportLegacyHDF5::writeFrame(const uint32_t gid,
                                             const float* values,
                                             const size_t /*size*/,
                                             const double timestamp)
{
    std::lock_guard<std::mutex> lock(detail::hdf5Mutex());

    try
    {
        auto& dataset = _getDataset(gid);
        auto dims = dataset.getSpace().getDimensions();
        const size_t frameNumber = _getFrameNumber(timestamp);
        auto selection = dataset.select({frameNumber, 0}, {1, dims[1]});
        // HighFive is not handling const correctly
        selection.write(const_cast<float*>(values));
        return true;
    }
    catch (const HighFive::Exception& e)
    {
        BRION_ERROR << "CompartmentReportLegacyHDF5: error writing frame: "
                  << e.what() << std::endl;
    }
    return false;
}

bool CompartmentReportLegacyHDF5::flush()
{
    std::lock_guard<std::mutex> lock(detail::hdf5Mutex());
    _file->flush();
    return true;
}

HighFive::DataSet CompartmentReportLegacyHDF5::_openDataset(
    const HighFive::File& file, const uint32_t cellID)
{
    std::stringstream cellName;
    cellName << "a" << cellID;
    const std::string datasetName =
        "/" + cellName.str() + "/" + _reportName + "/" + dataDatasetName;
    HighFive::DataSet dataset = [file, datasetName]() {
        try
        {
            return file.getDataSet(datasetName);
        }
        catch (const HighFive::DataSetException&)
        {
            BRION_THROW("CompartmentReportLegacyHDF5: Dataset " +
                      datasetName + " not found in file: " +
                      file.getName())
        }
    }();

    if (dataset.getSpace().getNumberDimensions() != 2)
    {
        BRION_THROW("CompartmentReportLegacyHDF5: Error, not 2 dimensional array on " 
                  + datasetName)
    }

    return dataset;
}

void CompartmentReportLegacyHDF5::_updateMapping(const GIDSet& gids)
{
    _gids = gids;
    _datas.clear();

    if (_gids.empty())
        _readGIDs();

    _offsets.resize(_gids.size());

    size_t nextCompartmentIndex = 0;
    size_t cellIndex = 0;
    for (auto cellID : _gids)
    {
        std::stringstream cellName;
        cellName << "a" << cellID;
        const std::string datasetName =
            "/" + cellName.str() + "/" + _reportName + "/" + mappingDatasetName;
        HighFive::DataSet dataset = [this, datasetName]() {
            try
            {
                return _file->getDataSet(datasetName);
            }
            catch (HighFive::DataSetException&)
            {
                BRION_THROW("CompartmentReportLegacyHDF5: Dataset " + datasetName + " not found")
            }
        }();

        auto dims = dataset.getSpace().getDimensions();
        if (dims.size() != 2)
        {
            BRION_THROW("CompartmentReportLegacyHDF5: Error, not 2 dimensional array on " 
                      + datasetName)
        }

        boost::scoped_array<float> buffer(new float[dims[1]]);
        dataset.read(buffer.get());

        // Getting the last section id;
        size_t largestSectionID = 0;
        for (size_t i = 0; i < dims[1]; ++i)
        {
            if (buffer[i] > largestSectionID)
                largestSectionID = buffer[i];
        }

        uint64_ts& offsets = _offsets[cellIndex];
        offsets.resize(largestSectionID + 1, std::numeric_limits<uint64_t>().max());

        size_t lastSection = std::numeric_limits<uint16_t>().max();
        for (size_t i = 0; i < dims[1]; ++i, ++nextCompartmentIndex)
        {
            const size_t section = buffer[i];
            if (lastSection != section)
            {
                lastSection = section;
                // Storing the start index of a new section with at
                // least 1 compartment
                offsets[section] = nextCompartmentIndex;
            }
        }

        ++cellIndex;
    }

    _comps = nextCompartmentIndex;
    _counts.resize(_offsets.size());

    SectionOffsets::const_iterator offsetsIt;
    CompartmentCounts::iterator countsIt;
    for (offsetsIt = _offsets.begin(), countsIt = _counts.begin();
         offsetsIt != _offsets.end(); ++offsetsIt, ++countsIt)
    {
        *countsIt = uint16_ts();
        countsIt->reserve(offsetsIt->size());

        uint64_ts::const_iterator sectionOffsetsIt;
        for (sectionOffsetsIt = offsetsIt->begin();
             sectionOffsetsIt != offsetsIt->end(); ++sectionOffsetsIt)
        {
            countsIt->push_back(
                calcCompartmentCounts(offsetsIt, sectionOffsetsIt, _offsets,
                                      nextCompartmentIndex));
        }
    }

    for (auto cellID : _gids)
    {
        auto dataset = _openDataset(*_file, cellID);
        _datas.emplace(std::make_pair(cellID, std::move(dataset)));
    }

    _cacheNeuronCompartmentCounts();
}

HighFive::DataSet CompartmentReportLegacyHDF5::_createDataset(
    const uint32_t gid, const size_t compCount)
{
    BRION_ASSERT(compCount > 0);
    BRION_ASSERT(!_reportName.empty());

    std::ostringstream neuronName;
    neuronName << "a" << gid;

    HighFive::Group neuronGroup = _file->createGroup(neuronName.str().c_str());
    HighFive::Group reportGroup = neuronGroup.createGroup(_reportName);

    const double step = getTimestep();
    // Adding step / 2 to the window to avoid off by 1 errors during truncation
    // after the division.
    const size_t numSteps = (getEndTime() - getStartTime() + step * 0.5) / step;
    BRION_ASSERT(numSteps > 0);

    HighFive::DataSet mappingDataset =
        reportGroup.createDataSet<float>(mappingDatasetName,
                                         HighFive::DataSpace({1, compCount}));
    HighFive::DataSet dataDataset = reportGroup.createDataSet<float>(
        dataDatasetName,
        HighFive::DataSpace(std::vector<size_t>{numSteps, compCount}));

    _datas.emplace(std::make_pair(gid, std::move(dataDataset)));

    _createMappingAttributes(mappingDataset);
    _createDataAttributes(dataDataset);

    return mappingDataset;
}

HighFive::DataSet& CompartmentReportLegacyHDF5::_getDataset(const uint32_t gid)
{
    Datasets::iterator it = _datas.find(gid);
    if (it == _datas.end())
        throw std::runtime_error("Need mapping before writing data");
    return it->second;
}

void CompartmentReportLegacyHDF5::_readMetaData(const HighFive::File& file)
{
    try
    {
        HighFive::SilenceHDF5 silence;
        const std::string& datasetName = file.getObjectName(0);
        const auto& reportGroup = file.getGroup(datasetName);
        _reportName = reportGroup.getObjectName(0);
        std::stringstream tmp;
        tmp << datasetName.substr(1);
        uint32_t gid;
        tmp >> gid;

        auto dataset = _openDataset(*_file, gid);
        dataset.getAttribute(dataAttributes[1]).read(_startTime);
        dataset.getAttribute(dataAttributes[2]).read(_endTime);
        dataset.getAttribute(dataAttributes[3]).read(_timestep);
        try
        {
            dataset.getAttribute(dataAttributes[4]).read(_dunit);
        }
        catch (const HighFive::AttributeException&)
        {
            _dunit = "mV";
        }
        try
        {
            dataset.getAttribute(dataAttributes[5]).read(_tunit);
        }
        catch (const HighFive::AttributeException&)
        {
            _tunit = "ms";
        }
    }
    catch (const HighFive::Exception&)
    {
        throw std::runtime_error(_path.string() +
                                 " not a valid H5 compartment report file");
    }
}

void CompartmentReportLegacyHDF5::_readGIDs() const
{
    auto callback = [this](const char* name) {
        if (name[1] != '\0')
            _gids.insert(std::stoi(&name[1]));
    };
    auto wrapper = [](hid_t, const char* name, const H5L_info_t*,
                      void* data) -> herr_t {
        (*(static_cast<decltype(callback)*>(data)))(name);
        return 0;
    };
    // Asking iterate in alpha numerical order implies that if there're no
    // index, it will have to be created. I also tried H5_INDEX_CRT_ORDER, but
    // that causes an error.
    // In any case the performance seems faster that using H5Gget_objname_by_idx
    // which most probably has quadratic complexity.
    // Why there's not way to simply iterate the links in the order they
    // appear in the file?
    H5Literate(_file->getId(), H5_INDEX_NAME, H5_ITER_NATIVE, nullptr, wrapper,
               &callback);
}

void CompartmentReportLegacyHDF5::_createMetaData()
{
    auto root = _file->getGroup("/");

    detail::addStringAttribute(root, "creator", "Brion");
    detail::addStringAttribute(root, "software_version", BRION_REV_STRING);

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
    detail::addStringAttribute(root, "creation_time", creationTimeName);
}

void CompartmentReportLegacyHDF5::_createMappingAttributes(
    HighFive::DataSet& dataset)
{
    const std::string type = "compartment";
    detail::addStringAttribute(dataset, mappingAttributes[0], type);
    for (int i = 1; i < 6; ++i)
        dataset.createAttribute<int>(mappingAttributes[i],
                                     HighFive::DataSpace(
                                         std::vector<size_t>({1})));
}

void CompartmentReportLegacyHDF5::_createDataAttributes(
    HighFive::DataSet& dataset)
{
    HighFive::DataSpace scalar(std::vector<size_t>({1}));

    auto attribute =
        dataset.createAttribute<int32_t>(dataAttributes[0], scalar);
    attribute.write(0); // rank

    attribute = dataset.createAttribute<double>(dataAttributes[1], scalar);
    attribute.write(getStartTime());

    attribute = dataset.createAttribute<double>(dataAttributes[2], scalar);
    attribute.write(getEndTime());

    attribute = dataset.createAttribute<double>(dataAttributes[3], scalar);
    attribute.write(getTimestep());

    detail::addStringAttribute(dataset, dataAttributes[4], _dunit);
    detail::addStringAttribute(dataset, dataAttributes[5], _tunit);
}

} // namespace plugin
} // namespace brion
