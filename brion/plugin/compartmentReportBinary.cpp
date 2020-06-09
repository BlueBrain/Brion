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

#include "../log.h"
#include "../pluginLibrary.h"

#include <boost/filesystem/path.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <map>

#include <cstdio>

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
const size_t _mappingItemSize = 4;

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
    uint64_t accumCompartments;
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
#if _GNU_SOURCE
    return strerror_r(errnum, buffer, 1023);
#else
    if (strerror_r(errnum, buffer, 1023) == -1)
        return "Uknown error";
    return buffer;
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

void _readAsync(aiocb** operations, const size_t length)
{
    if (lio_listio(LIO_WAIT, operations, length, nullptr) == -1)
        BRION_THROW("Error in AIO setup: " + getErrorString(errno))
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
} // namespace

inline void byteswap(CellInfo& value)
{
    byteswap(value.gid);
    byteswap(value.numCompartments);
    byteswap(value.mappingOffset);
    byteswap(value.extraMappingOffset);
    byteswap(value.dataOffset);
}

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
        pluginManager.registerFactory<CompartmentReportBinary>();
    }
};

PluginRegisterer registerer;
} // namespace

CompartmentReportBinary::CompartmentReportBinary(
    const CompartmentReportInitData& initData)
    : _path(initData.getURI().getPath())
    , _startTime(0)
    , _endTime(0)
    , _timestep(0)
    , _header()
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
        BRION_THROW("Writing of binary compartments not implemented")

    if (_ioAPI == IOapi::posix_aio)
    {
#ifdef HAS_AIO
        _fileDescriptor = open(_path.c_str(), O_RDONLY);
        if (_fileDescriptor < 0)
            throw std::runtime_error("Failed to open " + _path);
        _fileHandle = fdopen(_fileDescriptor, "r");
        // We need to parse the header, plus the data offset for the first
        // cell to know where the mapping ends.
        _remapFile(HEADER_LENGTH + DATA_INFO + sizeof(int32_t));
#endif
    }
    else
        _file.open(_path);

    if (!_parseHeader())
        BRION_THROW("Parsing header failed")

    // Remapping the file until the end of the cell mapping if necessary.
    if (_ioAPI == IOapi::posix_aio)
    {
        if (!_remapFile(_dataOffset))
            BRION_THROW("Failed to memory map file")
    }

    if (initData.initMapping)
    {
        if (!_parseMapping())
            BRION_THROW("Parsing mapping failed")
        updateMapping(initData.getGIDs());
    }
    else
        _parseGIDs();
}

CompartmentReportBinary::~CompartmentReportBinary()
{
    if (_ioAPI == IOapi::posix_aio)
    {
        close(_fileDescriptor);
    }
    _file.close();
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

bool CompartmentReportBinary::_remapFile(const size_t size)
{
    if (_file.is_open())
        _file.close();
    _file.open(_path, std::ios_base::in, size);
    return _file.is_open();
}

size_t CompartmentReportBinary::getCellCount() const
{
    if (_gids.empty())
        return _header.numCells;
    return _gids.size();
}

const GIDSet& CompartmentReportBinary::getGIDs() const
{
    if (_gids.empty())
        return _originalGIDs;
    return _gids;
}

const SectionOffsets& CompartmentReportBinary::getOffsets() const
{
    return (_subtarget ? _targetMapping : _sourceMapping).offsets;
}

const CompartmentCounts& CompartmentReportBinary::getCompartmentCounts() const
{
    return (_subtarget ? _targetMapping : _sourceMapping).counts;
}

size_t CompartmentReportBinary::getNumCompartments(const size_t index) const
{
    return (_subtarget ? _targetMapping : _sourceMapping).cellSizes[index];
}

size_t CompartmentReportBinary::getFrameSize() const
{
    return (_subtarget ? _targetMapping : _sourceMapping).frameSize;
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
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(_file.const_data());
    if (!ptr)
        return false;

    const size_t frameOffset =
        _dataOffset + _sourceMapping.frameSize * sizeof(float) * frameNumber;

    if (!_subtarget)
    {
        memcpy(buffer, ptr + frameOffset,
               _sourceMapping.frameSize * sizeof(float));

        if (_header.byteswap)
        {
#pragma omp parallel for
            for (auto i = 0u; i < _sourceMapping.frameSize; ++i)
                byteswap(buffer[i]);
        }
        return true;
    }

    // Empty frames are detected in CompartmentReportCommon
    assert(_targetMapping.frameSize != 0);

    const auto* const source = (const float*)(ptr + frameOffset);

    for (size_t i = 0; i < _gids.size(); ++i)
    {
        const auto originalIndex = _subsetIndices[i];
        const auto numCompartments = _targetMapping.cellSizes[i];
        const auto sourceOffset = _sourceMapping.cellOffsets[originalIndex];
        const auto targetOffset = _targetMapping.cellOffsets[i];
        for (auto k = 0u; k < numCompartments; ++k)
            buffer[targetOffset + k] = source[sourceOffset + k];
    }

    if (_header.byteswap)
    {
#pragma omp parallel for
        for (ssize_t i = 0; i < ssize_t(_targetMapping.frameSize); ++i)
            byteswap(buffer[i]);
    }
    return true;
}

#ifdef HAS_AIO
void CompartmentReportBinary::_loadFramesAIO(const size_t frameNumber,
                                             const size_t count,
                                             float* buffer) const
{
    const size_t originalFrameSize = _sourceMapping.frameSize * sizeof(float);
    size_t frameOffset = _dataOffset + originalFrameSize * frameNumber;
    float* targetFrame = buffer;
    std::vector<AIOReadData> readData;
    readData.reserve(count * (_subtarget ? _gids.size() : 1));

    for (size_t n = 0; n < count; ++n)
    {
        if (_subtarget)
        {
            // Empty frames are detected in CompartmentReportCommon
            assert(_targetMapping.frameSize != 0);

            const auto& sourceOffsets = _sourceMapping.cellOffsets;
            const auto& targetOffsets = _targetMapping.cellOffsets;

            for (size_t i = 0; i < _gids.size(); ++i)
            {
                const auto originalIndex = _subsetIndices[i];
                const auto readSize =
                    _targetMapping.cellSizes[i] * sizeof(float);
                const auto sourceOffset =
                    frameOffset + sourceOffsets[originalIndex] * sizeof(float);
                void* targetBuffer = targetFrame + targetOffsets[i];
                readData.push_back(
                    {_fileDescriptor, targetBuffer, readSize, sourceOffset});
            }
            targetFrame += _targetMapping.frameSize;
        }
        else
        {
            readData.push_back({_fileDescriptor, targetFrame,
                                _sourceMapping.frameSize * sizeof(float),
                                frameOffset});
            targetFrame += _sourceMapping.frameSize;
        }
        frameOffset += originalFrameSize;
    }
    const size_t readCount =
        (_subtarget ? _targetMapping : _sourceMapping).frameSize * count;

    _readAsync(readData);

    if (_header.byteswap)
    {
#pragma omp parallel for
        for (size_t i = 0; i < readCount; ++i)
            byteswap(buffer[i]);
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
    const uint8_t* const bytePtr =
        reinterpret_cast<const uint8_t*>(_file.const_data());
    if (!bytePtr ||
        (_subtarget ? _targetMapping : _sourceMapping).offsets.empty())
    {
        return floatsPtr();
    }

    const size_t index = getIndex(gid);

    const size_t frameSize = _sourceMapping.frameSize;
    const size_t nFrames = (_endTime - _startTime) / _timestep;
    const size_t nCompartments = getNumCompartments(index);
    const size_t nValues = nFrames * nCompartments;
    floatsPtr buffer(new floats(nValues));

    const SectionOffsets& offsets = _sourceMapping.offsets;
    const CompartmentCounts& compCounts = getCompartmentCounts();
    for (size_t i = 0; i < nFrames; ++i)
    {
        const size_t frameOffset = i * frameSize;
        size_t dstOffset = i * nCompartments;
        for (size_t j = 0; j < offsets[index].size(); ++j)
        {
            const uint32_t numCompartments = compCounts[index][j];
            const uint64_t sourceOffset = offsets[index][j];

            if (numCompartments)
            {
                const size_t nBytes = numCompartments * sizeof(float);
                void* dest = buffer->data() + dstOffset;
                const size_t srcOffset =
                    _dataOffset + (frameOffset + sourceOffset) * sizeof(float);

                if (_ioAPI == IOapi::mmap)
                {
                    ::memcpy(dest, bytePtr + srcOffset, nBytes);
                }
                else
                {
                    std::fseek(_fileHandle, srcOffset, SEEK_SET);
                    if (std::fread(dest, 1, nBytes, _fileHandle) != nBytes)
                    {
                        throw std::runtime_error("Failed to read data");
                    }
                }
            }

            dstOffset += numCompartments;
        }
    }

    if (_header.byteswap)
    {
#pragma omp parallel for
        for (ssize_t i = 0; i < ssize_t(nValues); ++i)
            byteswap((*buffer)[i]);
    }

    return buffer;
}

void CompartmentReportBinary::updateMapping(const GIDSet& gids)
{
    if (_sourceMapping.frameSize == 0 && !_parseMapping())
        BRION_THROW("Parsing mapping failed")

    if (gids.empty())
    {
        // Copy _originalGIDs to _gids only if the target current target
        // is a subtarget.
        if (_subtarget || _gids.empty())
            _gids = _originalGIDs;
        _subtarget = false;
    }
    else
    {
        _gids = gids;
        _subtarget = _gids != _originalGIDs;
    }

    if (!_subtarget)
        return;

    const GIDSet intersection = _computeIntersection(_originalGIDs, _gids);
    if (intersection.empty())
    {
        BRION_THROW("CompartmentReportBinary::updateMapping: GIDs out of range")
    }
    if (intersection != _gids)
    {
        updateMapping(intersection);
        return;
    }

    _subsetIndices = _computeSubsetIndices(_originalGIDs, _gids);
    _targetMapping = _reduceMapping(_sourceMapping, _subsetIndices);
}

void CompartmentReportBinary::writeHeader(const double /*startTime*/,
                                          const double /*endTime*/,
                                          const double /*timestep*/,
                                          const std::string& /*dunit*/,
                                          const std::string& /*tunit*/)
{
    BRION_THROW("CompartmentReportBinary::writeHeader not implemented")
}

bool CompartmentReportBinary::writeCompartments(const uint32_t /*gid*/,
                                                const uint16_ts& /*counts*/)
{
    BRION_THROW("CompartmentReportBinary::writeCompartments not implemented")
    return false;
}

bool CompartmentReportBinary::writeFrame(const uint32_t /*gid*/,
                                         const float* /*values*/,
                                         const size_t /*size*/,
                                         const double /*timestamp*/)
{
    BRION_THROW("CompartmentReportBinary::writeFrame not implemented")
    return false;
}

bool CompartmentReportBinary::flush()
{
    BRION_THROW("CompartmentReportBinary::flush not implemented")
    return false;
}

bool CompartmentReportBinary::_parseHeader()
{
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(_file.const_data());
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
        byteswap(_header);
        byteswap(_startTime);
        byteswap(_endTime);
        byteswap(_timestep);
        if (_header.identifier != ARCHITECTURE_IDENTIFIER)
        {
            BRION_ERROR << "File is corrupt or originated from an unknown "
                      << "architecture." << std::endl;
            return false;
        }
    }

    if (_dunit.empty() || _dunit == "mv")
        _dunit = "mV";
    if (_tunit.empty())
        _tunit = "ms";

    // After parsing the header we figure out the end of the mapping by reading
    // the data offset of the first cell.
    // This is needed for remapping the file before reading the GIDs.
    _dataOffset = get<uint64_t>(ptr, DATA_INFO + _header.headerSize);
    if (_header.byteswap)
        byteswap(_dataOffset);

    return true;
}

void CompartmentReportBinary::_parseGIDs()
{
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(_file.const_data());
    ptr += _header.headerSize + NUMBER_OF_CELL;
    for (int32_t i = 0; i < _header.numCells; ++i)
    {
        auto gid = get<int32_t>(ptr, i * SIZE_CELL_INFO_LENGTH);
        if (_header.byteswap)
            byteswap(gid);
        _originalGIDs.insert(gid);
    }
}

bool CompartmentReportBinary::_parseMapping()
{
    _sourceMapping.frameSize = _header.numCompartments;
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(_file.const_data());
    size_t offset = _header.headerSize;
    std::unique_ptr<uint8_t[]> buffer(new uint8_t[_dataOffset]);
    std::copy(ptr, ptr + _dataOffset, buffer.get());
    ptr = buffer.get();

    // According to Garik Suess, all compartments of a cell in a frame are next
    // to each other, and all compartments of a section are next to each other.
    // However, the sections are not necessarily sorted by their index in the
    // frame. It could be that for cell with GID 50 while all data is
    // contiguous, the sections are out of order so compartments for section 3 6
    // 22 8 could be next to each other, while the compartments inside these
    // sections are in order.

    uint64_t totalCompartments = 0;
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
        if (cell.dataOffset < _dataOffset)
        {
            BRION_ERROR << "Bad offset in report mapping" << std::endl;
            return false;
        }
        offset += SIZE_CELL_INFO_LENGTH;

        if (_header.byteswap)
            byteswap(cell);

        cell.accumCompartments = totalCompartments;
        totalCompartments += cell.numCompartments;
        _originalGIDs.insert(cell.gid);
    }

    std::sort(cells.begin(), cells.end());
    auto& offsets = _sourceMapping.offsets;
    offsets.resize(cells.size());
    auto& counts = _sourceMapping.counts;
    counts.resize(cells.size());
    auto& cellOffsets = _sourceMapping.cellOffsets;
    cellOffsets.resize(cells.size());
    auto& cellSizes = _sourceMapping.cellSizes;
    cellSizes.resize(cells.size());

// With 8 threads there's some speed up, but very small. We stick with 6
// to avoid using more than 1 socket.
#pragma omp parallel for num_threads(6) schedule(dynamic)
    for (size_t idx = 0; idx < cells.size(); ++idx)
    {
        const CellInfo& info = cells[idx];
        uint16_t current = std::numeric_limits<uint16_t>().max();
        uint16_t previous = std::numeric_limits<uint16_t>().max();
        uint16_t count = 0;

        // < sectionID, < frameIndex, numCompartments > >
        std::vector<std::pair<uint16_t, std::pair<uint64_t, uint16_t>>>
            sectionsMapping;
        sectionsMapping.reserve(info.numCompartments);

        cellSizes[idx] = info.numCompartments;
        cellOffsets[idx] = info.accumCompartments;

        for (int32_t j = 0; j < info.numCompartments; ++j)
        {
            previous = current;
            const size_t pos(info.mappingOffset +
                             j * _mappingItemSize * _header.mappingSize);
            float value = get<float>(ptr, pos);
            if (_header.byteswap)
                byteswap(value);
            assert(value < 65536);
            current = value;

            // in case this is the start of a new section
            if (current != previous)
            {
                const uint64_t frameIndex =
                    j + ((info.dataOffset - _dataOffset) / _mappingItemSize);

                sectionsMapping.push_back(
                    std::make_pair(current, std::make_pair(frameIndex, 0)));

                if (previous != std::numeric_limits<uint16_t>().max())
                    sectionsMapping[sectionsMapping.size() - 2].second.second =
                        count;

                count = 0;
            }
            ++count;
        }
        sectionsMapping.back().second.second = count;
        std::sort(sectionsMapping.begin(), sectionsMapping.end());

        // now convert the maps into the desired mapping format
        auto& sectionOffsets = offsets[idx];
        auto& sectionCompartmentCounts = counts[idx];

        // get maximum section id
        const uint16_t maxID = sectionsMapping.rbegin()->first;
        sectionOffsets.resize(maxID + 1, std::numeric_limits<uint64_t>().max());
        sectionCompartmentCounts.resize(maxID + 1, 0);

        for (auto sectionInfo : sectionsMapping)
        {
            const auto id = sectionInfo.first;
            sectionOffsets[id] = sectionInfo.second.first;
            sectionCompartmentCounts[id] = sectionInfo.second.second;
        }
    }

    return true;
}

} // namespace plugin
} // namespace brion
