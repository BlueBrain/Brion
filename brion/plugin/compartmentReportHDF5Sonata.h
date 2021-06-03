/* Copyright (c) 2013-2017, EPFL/Blue Brain Project
 *                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
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

#ifndef BRION_PLUGIN_COMPARTMENTREPORTHDF5SONATA
#define BRION_PLUGIN_COMPARTMENTREPORTHDF5SONATA

#include "compartmentReportCommon.h"

#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

#include <boost/filesystem/path.hpp>

#include <unordered_map>

namespace brion
{
namespace plugin
{
/** Sonata H5 compartment report parser */
class CompartmentReportHDF5Sonata : public CompartmentReportCommon
{
public:
    explicit CompartmentReportHDF5Sonata(
        const CompartmentReportInitData& initData);
    virtual ~CompartmentReportHDF5Sonata();

    static bool handles(const CompartmentReportInitData& initData);
    static std::string getDescription();

    double getStartTime() const final { return _startTime; }
    double getEndTime() const final { return _endTime; }
    double getTimestep() const final { return _timestep; }
    const std::string& getDataUnit() const final { return _dunit; }
    const std::string& getTimeUnit() const final { return _tunit; }
    size_t getCellCount() const final;
    const GIDSet& getGIDs() const final;
    const SectionOffsets& getOffsets() const final;
    const CompartmentCounts& getCompartmentCounts() const final;
    size_t getNumCompartments(size_t index) const final;
    size_t getFrameSize() const final;

    void updateMapping(const GIDSet& gids) final;

    void writeHeader(double startTime, double endTime, double timestep,
                     const std::string& dunit, const std::string& tunit) final;
    bool writeCompartments(uint32_t gid, const uint16_ts& counts) final;
    bool writeFrame(uint32_t gid, const float* values, size_t size,
                    double timestamp) final;
    bool writeFrame(const GIDSet& gids, const float* values,
                    const size_ts& sizes, double timestamp) final;
    bool flush() final;

private:
    double _startTime;
    double _endTime;
    double _timestep;
    std::string _dunit;
    std::string _tunit;

    std::unique_ptr<HighFive::File> _file;
    std::unique_ptr<HighFive::DataSet> _data;

    // Read API attributes
    GIDSet _gids;
    GIDSet _sourceGIDs;
    GIDSet _gids1based;
    GIDSet _sourceGIDs1based;
    bool _subset = false;
    std::vector<uint32_t> _subsetIndices;
    hsize_t _chunkDims[2] = {0, 0};

    MappingInfo _sourceMapping;

    // Write API temporary attributes
    std::vector<uint32_t> _GIDlist;
    std::vector<uint32_t> _elementIDs;
    float _cellsToFramesRatio = 0.125;
    size_t _chunkSize = 1024 * 1024; // 1 MiB

    // Read/Write API attribute
    MappingInfo _targetMapping;

    bool _loadFrame(size_t timestamp, float* buffer) const final;

    void _updateMapping(const GIDSet& gids);

    void _readMetaData();
    void _reopenDataSet(size_t cacheSizeHint);
    /** Parses the GIDs and offsets and derives per cell compartment counts.
        The data from the H5 file is resorted if needed. */
    void _parseBasicCellInfo();
    void _processMapping();

    void _writeMetadataAndMapping();
    void _allocateDataSet();

    void _parseWriteOptions(const URI& uri);
};
} // namespace plugin
} // namespace brion

#endif
