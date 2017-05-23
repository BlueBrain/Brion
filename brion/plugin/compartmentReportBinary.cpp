/* Copyright (c) 2013-2017, EPFL/Blue Brain Project
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

#include "compartmentReportBinary.h"

#include <lunchbox/debug.h>
#include <lunchbox/intervalSet.h>
#include <lunchbox/log.h>
#include <lunchbox/memoryMap.h>
#include <lunchbox/pluginRegisterer.h>

#include <boost/filesystem/path.hpp>
#include <boost/foreach.hpp>

#include <map>

#if defined __linux__ || defined __APPLE__
#define HAS_AIO
#endif

#ifdef HAS_AIO
#include <aio.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace
{
// Offsets of the header information in the file.
enum HeaderPositions
{
    //! position of the double value identifying the byte order of the file
    IDENTIFIER = 0,
    //! offset of header information (int32_t) past the architecture identifier
    HEADER_SIZE = IDENTIFIER + sizeof(double),
    //! version of the reader library? (char *)
    LIBRARY_VERSION = 16,
    //! version of the simulator used in the simulation (char *)
    SIMULATOR_VERSION = 32,
    //! number of cells in each frame of the report (int32_t)
    TOTAL_NUMBER_OF_CELLS = 48,
    //! number of compartments in a frame (int32_t)
    TOTAL_NUMBER_OF_COMPARTMENTS = 52,
    //! number of frames in the report (int32_t)
    NUMBER_OF_STEPS = 64,
    //! time where the report starts in specified time unit (double)
    TIME_START = 72,
    //! time where the report ends in specified time unit (double)
    TIME_END = 80,
    //! timestep between two report frames (double)
    DT_TIME = 88,
    //! unit of the report data (char *)
    D_UNIT = 96,
    //! time unit of the report (char *)
    T_UNIT = 112,
    //! size of the mapping (int32_t)
    MAPPING_SIZE = 128,
    //! name of the mapping (char *)
    MAPPING_NAME = 144,
    //! size of the extra mapping (int32_t)
    EXTRA_MAPPING_SIZE = 160,
    //! name of the extra mapping (char *)
    EXTRA_MAPPING_NAME = 176,
    //! name of the report (char *)
    REPORT_NAME = 192,
    //! length of the header (int32_t)
    HEADER_LENGTH = 1024
};

// Offsets of data tokens inside a cell info block inside the file header.
enum CellInfoHeaderPositions
{
    // Cell_GID (int32_t)
    NUMBER_OF_CELL = 0,
    // number of compartments of this cell (int32_t)
    NUMBER_OF_COMPARTMENTS = 8,
    // data info offset (int32_t)
    DATA_INFO = 16,
    // extra mapping info offset (int32_t)
    EXTRA_MAPPING_INFO = 24,
    // mapping info offset (int32_t)
    MAPPING_INFO = 32,
    // size of the cellinfo block (int32_t)
    SIZE_CELL_INFO_LENGTH = 64
};

struct CellInfo
{
    int32_t gid;
    int32_t numCompartments;
    uint64_t mappingOffset;
    uint64_t extraMappingOffset;
    uint64_t dataOffset;

    bool operator<(const CellInfo& rhs) const { return gid < rhs.gid; }
};

typedef std::vector<CellInfo> CellInfos;

// If identifier read at position 0 matches ARCHITECTURE_IDENTIFIER, then the
// file was writting from native architecture
const double ARCHITECTURE_IDENTIFIER = 1.001;

template <typename T>
T get(const uint8_t* buffer, const size_t offset)
{
    return *reinterpret_cast<const T*>(buffer + offset);
}

template <typename T>
const T* getPtr(const uint8_t* buffer, const size_t offset)
{
    return reinterpret_cast<const T*>(buffer + offset);
}

#ifdef HAS_AIO

const size_t _maxAIOops = 4096;

std::string getErrorString(int errnum)
{
    char buffer[1024];
    buffer[1023] = '\0';
#if (_POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600) && !_GNU_SOURCE
    if (strerror_r(errnum, buffer, 1023) == -1)
        return "Uknown error";
    return buffer;
#else
    return strerror_r(errnum, buffer, 1023);
#endif
}

struct AIOReadData
{
    int fileDescriptor;
    void* buffer;
    size_t size;
    size_t offset;
};

void _initAIOControlBlock(aiocb& block, const AIOReadData& readData)
{
    bzero(&block, sizeof(aiocb));
    block.aio_buf = readData.buffer;
    block.aio_fildes = readData.fileDescriptor;
    block.aio_nbytes = readData.size;
    block.aio_offset = readData.offset;
    block.aio_lio_opcode = LIO_READ;
}

void _readAsync(aiocb** operations, size_t length)
{
    if (lio_listio(LIO_WAIT, operations, length, nullptr) == -1)
        throw std::runtime_error("Error in AIO operation setup" +
                                 getErrorString(errno));
    for (size_t i = 0; i != length; ++i)
    {
        const size_t bytesRead = aio_return(operations[i]);
        if (bytesRead != operations[i]->aio_nbytes)
            throw std::runtime_error("AIO read failed");
    }
}

void _readAsync(const std::vector<AIOReadData>& readData)
{
    std::vector<aiocb> controlBlocks(readData.size());
    std::vector<aiocb*> pointers(readData.size());
    size_t i = 0;
    for (auto& opData : readData)
    {
        auto& block = controlBlocks[i];
        _initAIOControlBlock(block, opData);
        pointers[i++] = &block;
    }
    auto** operations = pointers.data();

    size_t remaining = readData.size();
    while (remaining)
    {
        const size_t batchSize = std::min(remaining, _maxAIOops);
        _readAsync(operations, batchSize);
        remaining -= batchSize;
        operations += batchSize;
    }
}

#else
// If AIO is not available always use memory mapped files
const bool _useMemoryMap = true;
#endif
}

namespace lunchbox
{
template <>
inline void byteswap(CellInfo& value)
{
    byteswap(value.gid);
    byteswap(value.numCompartments);
    byteswap(value.mappingOffset);
    byteswap(value.extraMappingOffset);
    byteswap(value.dataOffset);
}
}

namespace brion
{
namespace plugin
{
namespace
{
lunchbox::PluginRegisterer<CompartmentReportBinary> registerer;
}

CompartmentReportBinary::CompartmentReportBinary(
    const CompartmentReportInitData& initData)
    : _startTime(0)
    , _endTime(0)
    , _timestep(0)
    , _file()
    , _header()
    , _subNumCompartments(0)
    , _subtarget(false)
#ifdef HAS_AIO
    , _ioAPI(IOapi::posix_aio)
#else
    , _ioAPI(IOapi::mmap)
#endif
{
#ifdef HAS_AIO
    if (getenv("BRION_USE_MEM_MAP") != nullptr)
        _ioAPI = IOapi::mmap;
#endif

    if (initData.getAccessMode() != MODE_READ)
        LBTHROW(
            std::runtime_error("Writing of binary compartments not "
                               "implemented"));

#ifdef HAS_AIO
    int flags = O_RDONLY;
    _fileDescriptor = open(initData.getURI().getPath().c_str(), flags);
    if (_fileDescriptor < 0)
        throw std::runtime_error("Failed to open " +
                                 initData.getURI().getPath());
#endif
    _file.map(initData.getURI().getPath());

    if (!_parseHeader())
        LBTHROW(std::runtime_error("Parsing header failed"));

    if (!_parseMapping())
        LBTHROW(std::runtime_error("Parsing mapping failed"));

    _cacheNeuronCompartmentCounts(initData.getGids());
}

CompartmentReportBinary::~CompartmentReportBinary()
{
    close(_fileDescriptor);
}

bool CompartmentReportBinary::handles(const CompartmentReportInitData& initData)
{
    if (initData.getAccessMode() != MODE_READ)
        return false;

    const URI& uri = initData.getURI();
    if (!uri.getScheme().empty() && uri.getScheme() != "file")
        return false;

    const boost::filesystem::path ext =
        boost::filesystem::path(uri.getPath()).extension();
    return ext == ".bin" || ext == ".rep" || ext == ".bbp";
}

std::string CompartmentReportBinary::getDescription()
{
    return "Blue Brain binary compartment reports:"
           "  [file://]/path/to/report.(bin|rep|bbp)";
}

const GIDSet& CompartmentReportBinary::getGIDs() const
{
    return _gids;
}

const SectionOffsets& CompartmentReportBinary::getOffsets() const
{
    return _perSectionOffsets[_subtarget];
}

const CompartmentCounts& CompartmentReportBinary::getCompartmentCounts() const
{
    return _perSectionCounts[_subtarget];
}

size_t CompartmentReportBinary::getFrameSize() const
{
    return _subtarget ? _subNumCompartments : _header.numCompartments;
}

bool CompartmentReportBinary::_loadFrame(const size_t frameNumber,
                                         float* buffer) const
{
    if (_ioAPI == IOapi::mmap)
        return _loadFrameMemMap(frameNumber, buffer);
    else
    {
        _loadFramesAIO(frameNumber, 1, buffer);
        return true;
    }
}

bool CompartmentReportBinary::_loadFrames(const size_t startFrame,
                                          const size_t count,
                                          float* buffer) const
{
    if (_ioAPI == IOapi::mmap)
        return CompartmentReportCommon::_loadFrames(startFrame, count, buffer);
    else
    {
        _loadFramesAIO(startFrame, count, buffer);
        return true;
    }
}

bool CompartmentReportBinary::_loadFrameMemMap(const size_t frameNumber,
                                               float* buffer) const
{
    const uint8_t* ptr = _file.getAddress<const uint8_t>();
    if (!ptr)
        return false;

    const size_t frameOffset =
        _header.dataBlockOffset +
        _header.numCompartments * sizeof(float) * frameNumber;

    if (!_subtarget)
    {
        memcpy(buffer, ptr + frameOffset,
               _header.numCompartments * sizeof(float));

        if (_header.byteswap)
        {
#pragma omp parallel for
            for (int32_t i = 0; i < _header.numCompartments; ++i)
                lunchbox::byteswap(buffer[i]);
        }
        return true;
    }

    // Empty frames are detected in CompartmentReportCommon
    assert(_subNumCompartments != 0);

    const float* const source = (const float*)(ptr + frameOffset);
    const auto& sourceOffsets = _perCellOffsets[0];
    const auto& targetOffsets = _perCellOffsets[1];

    for (size_t i = 0; i < _gids.size(); ++i)
    {
        const uint32_t originalIndex = _subOriginalIndices[i];
        const uint16_t numCompartments = _perCellCounts[originalIndex];
        const uint64_t sourceOffset = sourceOffsets[originalIndex];
        const uint64_t targetOffset = targetOffsets[i];
        for (uint16_t k = 0; k < numCompartments; ++k)
            buffer[targetOffset + k] = source[sourceOffset + k];
    }

    if (_header.byteswap)
    {
#pragma omp parallel for
        for (ssize_t i = 0; i < ssize_t(_subNumCompartments); ++i)
            lunchbox::byteswap(buffer[i]);
    }
    return true;
}

#ifdef HAS_AIO
void CompartmentReportBinary::_loadFramesAIO(const size_t frameNumber,
                                             const size_t count,
                                             float* buffer) const
{
    const size_t originalFrameSize = _header.numCompartments * sizeof(float);
    size_t frameOffset =
        _header.dataBlockOffset + originalFrameSize * frameNumber;
    float* targetFrame = buffer;
    std::vector<AIOReadData> readData;
    readData.reserve(count * (_subtarget ? _gids.size() : 1));

    for (size_t n = 0; n < count; ++n)
    {
        if (_subtarget)
        {
            // Empty frames are detected in CompartmentReportCommon
            assert(_subNumCompartments != 0);

            const auto& sourceOffsets = _perCellOffsets[0];
            const auto& targetOffsets = _perCellOffsets[1];

            for (size_t i = 0; i < _gids.size(); ++i)
            {
                const uint32_t originalIndex = _subOriginalIndices[i];
                const size_t readSize =
                    _perCellCounts[originalIndex] * sizeof(float);
                const uint64_t sourceOffset =
                    frameOffset + sourceOffsets[originalIndex] * sizeof(float);
                void* targetBuffer = targetFrame + targetOffsets[i];
                readData.push_back(
                    {_fileDescriptor, targetBuffer, readSize, sourceOffset});
            }
            targetFrame += _subNumCompartments;
        }
        else
        {
            readData.push_back({_fileDescriptor, targetFrame,
                                _header.numCompartments * sizeof(float),
                                frameOffset});
            targetFrame += _header.numCompartments;
        }
        frameOffset += originalFrameSize;
    }
    const size_t readCount =
        (_subtarget ? _subNumCompartments : _header.numCompartments) * count;

    _readAsync(readData);

    if (_header.byteswap)
    {
#pragma omp parallel for
        for (size_t i = 0; i < readCount; ++i)
            lunchbox::byteswap(buffer[i]);
    }
}
#else
void CompartmentReportBinary::_loadFramesAIO(const size_t, const size_t,
                                             float*) const
{
}
#endif

floatsPtr CompartmentReportBinary::loadNeuron(const uint32_t gid) const
{
    const uint8_t* const bytePtr = _file.getAddress<const uint8_t>();
    if (!bytePtr || _perSectionOffsets[_subtarget].empty())
        return floatsPtr();

    const size_t index = getIndex(gid);
    const float* const ptr = (const float*)(bytePtr + _header.dataBlockOffset);

    const size_t frameSize = _header.numCompartments;
    const size_t nFrames = (_endTime - _startTime) / _timestep;
    const size_t nCompartments = getNumCompartments(index);
    const size_t nValues = nFrames * nCompartments;
    floatsPtr buffer(new floats(nValues));

    const SectionOffsets& offsets = _perSectionOffsets[0];
    const CompartmentCounts& compCounts = getCompartmentCounts();
    for (size_t i = 0; i < nFrames; ++i)
    {
        const size_t frameOffset = i * frameSize;
        size_t dstOffset = i * nCompartments;
        for (size_t j = 0; j < offsets[index].size(); ++j)
        {
            const uint16_t numCompartments = compCounts[index][j];
            const uint64_t sourceOffset = offsets[index][j];

            ::memcpy(buffer->data() + dstOffset,
                     ptr + frameOffset + sourceOffset,
                     numCompartments * sizeof(float));
            dstOffset += numCompartments;
        }
    }

    if (_header.byteswap)
    {
#pragma omp parallel for
        for (ssize_t i = 0; i < ssize_t(nValues); ++i)
            lunchbox::byteswap((*buffer)[i]);
    }

    return buffer;
}

void CompartmentReportBinary::updateMapping(const GIDSet& gids)
{
    _gids = gids.empty() ? _originalGIDs : gids;
    _subtarget = (_gids != _originalGIDs);

    if (!_subtarget)
        return;

    const GIDSet intersection = _computeIntersection(_originalGIDs, _gids);
    if (intersection.empty())
    {
        LBTHROW(
            std::runtime_error("CompartmentReportBinary::updateMapping:"
                               " GIDs out of range"));
    }
    if (intersection != _gids)
    {
        updateMapping(intersection);
        return;
    }

    // build gid to mapping index lookup table
    std::unordered_map<uint32_t, size_t> gidIndex;
    size_t c = 0;
    for (const auto gid : _originalGIDs)
        gidIndex[gid] = c++;

    _perSectionCounts[1].resize(gids.size());
    _perSectionOffsets[1].resize(gids.size());
    _perCellOffsets[1].resize(gids.size());
    _subOriginalIndices.resize(gids.size());

    _subNumCompartments = 0;
    size_t index = 0;
    for (const auto gid : _gids)
    {
        const size_t originalIndex = gidIndex[gid];
        _subOriginalIndices[index] = originalIndex;
        auto& offsets = _perSectionOffsets[1][index];
        offsets = _perSectionOffsets[0][originalIndex];
        const ssize_t offsetShift =
            _subNumCompartments - _perCellOffsets[0][originalIndex];
        for (auto& offset : offsets)
            offset += offsetShift;
        _perSectionCounts[1][index] = _perSectionCounts[0][originalIndex];
        _perCellOffsets[1][index] = _subNumCompartments;
        _subNumCompartments += _perCellCounts[originalIndex];
        ++index;
    }
}

void CompartmentReportBinary::writeHeader(const float /*startTime*/,
                                          const float /*endTime*/,
                                          const float /*timestep*/,
                                          const std::string& /*dunit*/,
                                          const std::string& /*tunit*/)
{
    LBUNIMPLEMENTED;
}

bool CompartmentReportBinary::writeCompartments(const uint32_t /*gid*/,
                                                const uint16_ts& /*counts*/)
{
    LBUNIMPLEMENTED;
    return false;
}

bool CompartmentReportBinary::writeFrame(const uint32_t /*gid*/,
                                         const float* /*values*/,
                                         const size_t /*size*/,
                                         const float /*timestamp*/)
{
    LBUNIMPLEMENTED;
    return false;
}

bool CompartmentReportBinary::flush()
{
    LBUNIMPLEMENTED;
    return false;
}

bool CompartmentReportBinary::_parseHeader()
{
    const uint8_t* ptr = _file.getAddress<const uint8_t>();
    if (!ptr)
        return false;

    _header.identifier = get<double>(ptr, IDENTIFIER);
    _header.headerSize = get<int32_t>(ptr, HEADER_SIZE);
    _header.numCells = get<int32_t>(ptr, TOTAL_NUMBER_OF_CELLS);
    _header.numCompartments = get<int32_t>(ptr, TOTAL_NUMBER_OF_COMPARTMENTS);
    _header.libVersion = getPtr<char>(ptr, LIBRARY_VERSION);
    _header.simVersion = getPtr<char>(ptr, SIMULATOR_VERSION);
    _header.numFrames = get<int32_t>(ptr, NUMBER_OF_STEPS);
    _startTime = get<double>(ptr, TIME_START);
    _endTime = get<double>(ptr, TIME_END);
    _timestep = get<double>(ptr, DT_TIME);
    _dunit = getPtr<char>(ptr, D_UNIT);
    _tunit = getPtr<char>(ptr, T_UNIT);
    _header.mappingSize = get<int32_t>(ptr, MAPPING_SIZE);
    _header.mappingName = getPtr<char>(ptr, MAPPING_NAME);
    _header.extraMappingSize = get<int32_t>(ptr, EXTRA_MAPPING_SIZE);
    _header.extraMappingName = getPtr<char>(ptr, EXTRA_MAPPING_NAME);
    _header.reportName = getPtr<char>(ptr, REPORT_NAME);

    _header.byteswap = _header.identifier != ARCHITECTURE_IDENTIFIER;

    if (_header.byteswap)
    {
        lunchbox::byteswap(_header);
        lunchbox::byteswap(_startTime);
        lunchbox::byteswap(_endTime);
        lunchbox::byteswap(_timestep);

        if (_header.identifier != ARCHITECTURE_IDENTIFIER)
        {
            LBERROR << "File is corrupt or originated from an unknown "
                    << "architecture." << std::endl;
            return false;
        }
    }

    if (_dunit.empty() || _dunit == "mv")
        _dunit = "mV";
    if (_tunit.empty())
        _tunit = "ms";

    return true;
}

bool CompartmentReportBinary::_parseMapping()
{
    const uint8_t* ptr = _file.getAddress<const uint8_t>();
    if (!ptr)
        return false;

    size_t offset = _header.headerSize;

    CellInfos cells(_header.numCells);
    for (int32_t i = 0; i < _header.numCells; ++i)
    {
        CellInfo& cell = cells[i];

        cell.gid = get<int32_t>(ptr, NUMBER_OF_CELL + offset);
        cell.numCompartments =
            get<int32_t>(ptr, NUMBER_OF_COMPARTMENTS + offset);
        cell.mappingOffset = get<uint64_t>(ptr, MAPPING_INFO + offset);
        cell.extraMappingOffset =
            get<uint64_t>(ptr, EXTRA_MAPPING_INFO + offset);
        cell.dataOffset = get<uint64_t>(ptr, DATA_INFO + offset);
        offset += SIZE_CELL_INFO_LENGTH;

        if (_header.byteswap)
            lunchbox::byteswap(cell);
    }

    _header.dataBlockOffset = cells[0].dataOffset;

    std::sort(cells.begin(), cells.end());
    SectionOffsets& offsetMapping = _perSectionOffsets[0];
    offsetMapping.resize(cells.size());
    CompartmentCounts& perSectionCounts = _perSectionCounts[0];
    perSectionCounts.resize(cells.size());
    std::vector<size_t>& perCellOffsets = _perCellOffsets[0];
    perCellOffsets.resize(cells.size());
    _perCellCounts.resize(cells.size());

    // According to Garik Suess, all compartments of a cell in a frame are next
    // to each other, and all compartments of a section are next to each other.
    // However, the sections are not necessarily sorted by their index in the
    // frame. It could be that for cell with GID 50 while all data is
    // contiguous, the sections are out of order so compartments for section 3 6
    // 22 8 could be next to each other, while the compartments inside these
    // sections are in order.
    size_t idx = 0;
    size_t totalCompartments = 0;
    for (const CellInfo& info : cells)
    {
        uint16_t current = LB_UNDEFINED_UINT16;
        uint16_t previous = LB_UNDEFINED_UINT16;
        uint16_t count = 0;

        // < sectionID, < frameIndex, numCompartments > >
        typedef std::map<uint16_t, std::pair<uint64_t, uint16_t>>
            SectionMapping;
        SectionMapping sectionsMapping;

        _perCellCounts[idx] = info.numCompartments;
        perCellOffsets[idx] = totalCompartments;
        totalCompartments += info.numCompartments;

        for (int32_t j = 0; j < info.numCompartments; ++j)
        {
            previous = current;
            const size_t pos(info.mappingOffset +
                             j * sizeof(float) * _header.mappingSize);
            float value = get<float>(ptr, pos);
            if (_header.byteswap)
                lunchbox::byteswap(value);
            current = value;

            // in case this is the start of a new section
            if (current != previous)
            {
                const uint64_t frameIndex =
                    j + ((info.dataOffset - _header.dataBlockOffset) /
                         sizeof(float));

                sectionsMapping.insert(
                    std::make_pair(current, std::make_pair(frameIndex, 0)));

                if (previous != LB_UNDEFINED_UINT16)
                    sectionsMapping[previous].second = count;

                count = 0;
            }
            ++count;
        }
        sectionsMapping[current].second = count;

        // now convert the maps into the desired mapping format
        uint64_ts& sectionOffsets = offsetMapping[idx];
        uint16_ts& sectionCompartmentCounts = perSectionCounts[idx];
        ++idx;

        // get maximum section id
        const uint16_t maxID = sectionsMapping.rbegin()->first;
        sectionOffsets.resize(maxID + 1, LB_UNDEFINED_UINT64);
        sectionCompartmentCounts.resize(maxID + 1, 0);

        for (auto k : sectionsMapping)
        {
            sectionOffsets[k.first] = k.second.first;
            sectionCompartmentCounts[k.first] = k.second.second;
        }

        _originalGIDs.insert(info.gid);
    }

    return true;
}
}
}
