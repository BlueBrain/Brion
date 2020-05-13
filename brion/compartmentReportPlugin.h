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

#pragma once

#include <brion/api.h>
#include <brion/pluginInitData.h>
#include <brion/types.h>

#include <boost/noncopyable.hpp>

#include <future>

namespace brion
{
/**
 * Basic plugin init data for CompartmentReportPlugin.
 * @version 1.5
 */
class CompartmentReportInitData : public PluginInitData
{
public:
    /** Create a CompartmentReportInitData object given a URI, access mode
     *  and GIDs.
     *
     * @param uri URI to compartment report. The report type is deduced from
     *        here.
     * @param accessMode the brion::AccessMode bitmask
     * @param gids the neurons of interest in READ_MODE
     * @version 3.0
     */
    CompartmentReportInitData(const URI& uri, const int accessMode,
                              const GIDSet& gids = GIDSet())
        : PluginInitData(uri, accessMode)
        , initMapping(true)
        , _gids(gids)
    {
    }

    /** Create a CompartmentReportInitData object given a URI, read access mode
     *  and no mapping requested at report construction.
     *
     * @param uri URI to compartment report. The report type is deduced from
     *        here.
     * @version 3.0
     */
    explicit CompartmentReportInitData(const URI& uri)
        : PluginInitData(uri, MODE_READ)
        , initMapping(false)
    {
    }

    /** Hint to the implementation to make the data mapping available at
        construction or not.
        @version 3.0 */
    const bool initMapping;

    /** @return Returns the gids. @version 1.4 */
    const GIDSet& getGIDs() const { return _gids; }
private:
    const GIDSet _gids;
};

/**
 * Base interface for compartment report readers and writers.
 */
class CompartmentReportPlugin : public boost::noncopyable
{
public:
    using DataT = CompartmentReportInitData;

    /** @internal */
    virtual ~CompartmentReportPlugin() {}
    /** @name Abstract interface */
    //@{
    /** @copydoc brion::CompartmentReport::getStartTime */
    virtual double getStartTime() const = 0;

    /** @copydoc brion::CompartmentReport::getEndTime */
    virtual double getEndTime() const = 0;

    /** @copydoc brion::CompartmentReport::getTimestep */
    virtual double getTimestep() const = 0;

    /** @copydoc brion::CompartmentReport::getDataUnit */
    virtual const std::string& getDataUnit() const = 0;

    /** @copydoc brion::CompartmentReport::getTimeUnit */
    virtual const std::string& getTimeUnit() const = 0;

    /** @copydoc brion::CompartmentReport::getCellCount */
    virtual size_t getCellCount() const { return getGIDs().size(); }
    /** @copydoc brion::CompartmentReport::getGIDs */
    virtual const GIDSet& getGIDs() const = 0;

    /** @copydoc brion::CompartmentReport::getOffsets */
    virtual const SectionOffsets& getOffsets() const = 0;

    /** @copydoc brion::CompartmentReport::getNumCompartments */
    virtual size_t getNumCompartments(size_t index) const = 0;

    /** @copydoc brion::CompartmentReport::getCompartmentCounts */
    virtual const CompartmentCounts& getCompartmentCounts() const = 0;

    /** @copydoc brion::CompartmentReport::getFrameSize */
    virtual size_t getFrameSize() const = 0;

    /** @copydoc brion::CompartmentReport::getFrameCount */
    virtual size_t getFrameCount() const = 0;

    /** @copydoc brion::CompartmentReport::loadFrame */
    virtual floatsPtr loadFrame(double timestamp) const = 0;

    /** @copydoc brion::CompartmentReport::loadFrames */
    virtual Frames loadFrames(double start, double end) const = 0;

    /** @sa brion::CompartmentReport::loadNeuron */
    virtual floatsPtr loadNeuron(uint32_t gid BRION_UNUSED) const
    {
        throw std::runtime_error("loadNeuron() not implemented");
    }

    /** @copydoc brion::CompartmentReport::updateMapping */
    virtual void updateMapping(const GIDSet& gids) = 0;

    /** @copydoc brion::CompartmentReport::setBufferSize */
    virtual void setBufferSize(size_t size) { /* To keep doxygen happy */ (void)size; }

    /** @copydoc brion::CompartmentReport::clearBuffer */
    virtual void clearBuffer() {}
    /** @copydoc brion::CompartmentReport::getBufferSize */
    virtual size_t getBufferSize() const { return 0; }
    /** @copydoc brion::CompartmentReport::writeHeader */
    virtual void writeHeader(double startTime, double endTime, double timestep,
                             const std::string& dunit,
                             const std::string& tunit) = 0;

    /** @copydoc brion::CompartmentReport::writeCompartments */
    virtual bool writeCompartments(uint32_t gid, const uint16_ts& counts) = 0;

    // clang-format off
    /** @copydoc brion::CompartmentReport::writeFrame(uint32_t gid, const float*
     * values, size_t size, double timestamp) */
    virtual bool writeFrame(uint32_t gid, const float* values, size_t size,
                            double timestamp) = 0;

    /** @copydoc brion::CompartmentReport::writeFrame(const GIDSet& gid, const
     * float* values, const size_t& sizes, double timestamp) */
    virtual bool writeFrame(const GIDSet& gid, const float* values,
                            const size_ts& sizes, double timestamp) = 0;
    // clang-format on

    /** @copydoc brion::CompartmentReport::flush */
    virtual bool flush() = 0;

    /** @copydoc brion::CompartmentReport::erase */
    virtual bool erase() { return false; }
    //@}

    /** @copydoc brion::CompartmentReport::getIndex */
    size_t getIndex(const uint32_t gid) const
    {
        const auto& gids = getGIDs();
        const size_t index = std::distance(gids.begin(), gids.find(gid));
        if (index >= gids.size())
            throw std::runtime_error("Gid " + std::to_string(gid) +
                                     " not in report");
        return index;
    }
};
}

namespace std
{
inline string to_string(const brion::CompartmentReportInitData& data)
{
    return to_string(data.getURI());
}
}
