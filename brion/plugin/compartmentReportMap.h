/* Copyright (c) 2014-2017, EPFL/Blue Brain Project
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

#ifndef BRION_PLUGIN_COMPARTMENTREPORTMAP
#define BRION_PLUGIN_COMPARTMENTREPORTMAP

#include "compartmentReportCommon.h"
#include <keyv/Map.h>
#include <unordered_map>

namespace brion
{
namespace plugin
{
/**
 * A read/write report using a keyv::Map as backend
 *
 * Stores the report as a set of fine-grained key-value pairs. A namespace for
 * the underlying keyv::Map is enforced and serves as the "filename" for the
 * report. The private _getFOOKey() methods define the key names. The header,
 * gids, dunit and tunit KV pairs contain the corresponding global information
 * about the report. Each counts key stores the mapping for the corresponding
 * neuron. Consequently, the ctor and updateMapping() reads four header KV pairs
 * and gids.size() mapping KV pairs. The value KV pair contains all values for
 * one neuron at one time step. loadFrame() reads gids.size() KV-pairs, and
 * loadNeuron reads nTimesteps KV pairs. These two, and reading the mapping, use
 * the asynchronous bulk operation keyv::Map::takeValues() for performance.
 */
class CompartmentReportMap : public CompartmentReportCommon
{
public:
    explicit CompartmentReportMap(const CompartmentReportInitData& initData);
    virtual ~CompartmentReportMap();

    static bool handles(const CompartmentReportInitData& initData);
    static std::string getDescription();

    float getStartTime() const final { return _header.startTime; }
    float getEndTime() const final { return _header.endTime; }
    float getTimestep() const final { return _header.timestep; }
    const std::string& getDataUnit() const final { return _dunit; }
    const std::string& getTimeUnit() const final { return _tunit; }
    const brion::GIDSet& getGIDs() const final { return _gids; }
    const SectionOffsets& getOffsets() const final { return _offsets; }
    size_t getFrameSize() const final { return _totalCompartments; }
    const CompartmentCounts& getCompartmentCounts() const final
    {
        return _counts;
    }

    floatsPtr loadNeuron(uint32_t gid) const final;

    void updateMapping(const GIDSet& gids) final;

    void writeHeader(float startTime, float endTime, float timestep,
                     const std::string& dunit, const std::string& tunit) final;
    bool writeCompartments(uint32_t gid, const uint16_ts& counts) final;
    bool writeFrame(uint32_t gid, const float* values, size_t size,
                    float timestamp) final;
    bool flush() final;
    bool erase() final;

    struct Header
    {
        Header();
        uint32_t magic;
        uint32_t version;
        uint32_t nGIDs; // redundant, but opt for fetching the GIDSet
        float startTime;
        float endTime;
        float timestep;
    };

private:
    std::vector<keyv::Map> _stores;

    Header _header;

    std::string _dunit;
    std::string _tunit;

    brion::GIDSet _gids;

    // index to get value from voltage buffer for all sections (ordered by
    // morphological description) for each cell. Cells are indexed by _gidIndex
    // (depends on the selected cells, given by updateMapping(), stored in
    // CompartmentReport::_gids
    SectionOffsets _offsets;

    // num compartments for all sections (ordered by morphological description,
    // 0s possible) for each cell. Cells are indexed by _gidIndex (depends on
    // the selected cells, given by updateMapping(), stored in
    // CompartmentReport::_gids
    brion::CompartmentCounts _counts;

    // total number of compartments (equals the size of the voltage buffer)
    uint64_t _totalCompartments;

    // <GID, num compartments per section>
    typedef std::map<uint32_t, brion::uint16_ts> CellCompartments;
    CellCompartments _cellCounts;

    bool _readable;

    using OffsetMap = std::unordered_map<std::string, size_t>;

    void _clear();
    bool _loadHeader();
    bool _flushHeader();
    bool _load(float* buffer, const Strings& keys,
               const OffsetMap& offsets) const;
    bool _loadFrame(size_t frameNumber, float* buffer) const final;

    std::string _getHeaderKey() const { return "header"; }
    std::string _getGidsKey() const { return "gids"; }
    std::string _getTunitKey() const { return "tunit"; }
    std::string _getDunitKey() const { return "dunit"; }
    std::string _getCountsKey(const uint32_t gid) const
    {
        return "counts_" + std::to_string(gid);
    }
    std::string _getValueKey(const uint32_t gid, const size_t frame) const
    {
        return std::to_string(gid) + "_" + std::to_string(frame);
    }
};
}
}

#endif
