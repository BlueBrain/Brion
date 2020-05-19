/* Copyright (c) 2013-2016, EPFL/Blue Brain Project
 *                          Raphael Dumusc <raphael.dumusc@epfl.ch>
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

#ifndef SPIKEREPORTPLUGIN_H
#define SPIKEREPORTPLUGIN_H

#include <brion/api.h>
#include <brion/enums.h>
#include <brion/pluginInitData.h>
#include <brion/spikeReport.h>
#include <brion/types.h>

#include <boost/noncopyable.hpp>

#include <functional>

namespace brion
{
/** Base interface for spike report readers plugins.
 *
 * @version 2.0
 */
class SpikeReportPlugin : public boost::noncopyable
{
public:
    using DataT = PluginInitData;

    using State = SpikeReport::State;

    SpikeReportPlugin(const PluginInitData& initData)
        : _uri(initData.getURI())
        , _accessMode(initData.getAccessMode())
    {
    }

    /** @internal */
    virtual ~SpikeReportPlugin() {}
    /** @copydoc brion::SpikeReport::close */
    virtual void close()
    {
        throw std::runtime_error(
            "Operation not supported in spike report plugin");
    }

    /** @sa brion::SpikeReport::read */
    virtual Spikes read(float min BRION_UNUSED)
    {
        throw std::runtime_error(
            "Operation not supported in spike report plugin");
    }

    /** @sa brion::SpikeReport::readUntil */
    virtual Spikes readUntil(float max BRION_UNUSED)
    {
        throw std::runtime_error(
            "Operation not supported in spike report plugin");
    }

    /** @sa brion::SpikeReport::seek */
    virtual void readSeek(float toTimeStamp BRION_UNUSED)
    {
        throw std::runtime_error(
            "Operation not supported in spike report plugin");
    }

    /** @sa brion::SpikeReport::seek */
    virtual void writeSeek(float toTimeStamp BRION_UNUSED)
    {
        throw std::runtime_error(
            "Operation not supported in spike report plugin");
    }

    /** @sa brion::SpikeReport::write */
    virtual void write(const Spike* spikes BRION_UNUSED,
                       size_t size BRION_UNUSED)
    {
        throw std::runtime_error(
            "Operation not supported in spike report plugin");
    }

    /** @copydoc brion::SpikeReport::supportsBackwardSeek */
    virtual bool supportsBackwardSeek() const = 0;

    void setFilter(const GIDSet& ids)
    {
        _idsSubset = ids;
        if (!_idsSubset.empty())
            _pushBackFunction = &SpikeReportPlugin::_pushBackFiltered;
        else
            _pushBackFunction = &SpikeReportPlugin::_pushBack;
    }

    virtual const URI& getURI() const { return _uri; }
    State getState() const { return _state; }
    int getAccessMode() const { return _accessMode; }
    virtual float getCurrentTime() const { return _currentTime; }
    virtual float getEndTime() const { return _endTime; }
    bool isClosed() const { return _closed; }
    bool isInInterruptedState() const { return _interrupted; }
protected:
    typedef void (SpikeReportPlugin::*SpikeInsertFunction)(const Spike&,
                                                           Spikes&) const;

    URI _uri;
    brion::GIDSet _idsSubset;
    int _accessMode = brion::MODE_READ;
    float _currentTime = 0;
    float _endTime = 0;
    State _state = State::ok;

    void pushBack(const Spike& spike, Spikes& spikes) const
    {
        (this->*_pushBackFunction)(spike, spikes);
    }

    void checkNotInterrupted()
    {
        if (isInInterruptedState())
        {
            throw std::runtime_error("Interrupted");
        }
    }

private:
    friend class ::brion::SpikeReport;

    SpikeInsertFunction _pushBackFunction = &SpikeReportPlugin::_pushBack;
    bool _closed = false;
    bool _interrupted = false;

    // Spike filtering.
    void _pushBack(const Spike& spike, Spikes& spikes) const
    {
        spikes.push_back(spike);
    }

    void _pushBackFiltered(const Spike& spike, Spikes& spikes) const
    {
        if (_idsSubset.find(spike.second) != _idsSubset.end())
        {
            spikes.push_back(spike);
        }
    }

    // Used in SpikeReport.
    void _setClosed() { _closed = true; }
    void _checkCanRead()
    {
        if (_accessMode != MODE_READ)
        {
            throw std::runtime_error("Can't read: Not open in read mode");
        }
    }
    void _checkCanWrite()
    {
        if (_accessMode != MODE_WRITE)
        {
            throw std::runtime_error("Can't write: Not open in write mode");
        }
    }

    void _checkStateOk()
    {
        switch (_state)
        {
        case State::ended:
            throw std::logic_error("State is ENDED");
        case State::failed:
            throw std::logic_error("State is FAILED");
        default:
            break;
        }
    }

    void _checkNotClosed()
    {
        if (_closed)
        {
            throw std::runtime_error("Report closed");
        }
    }

    void _setInterrupted(const bool i) { _interrupted = i; }
};
}

namespace std
{
inline string to_string(const brion::PluginInitData& data)
{
    return to_string(data.getURI());
}
}
#endif // SPIKEREPORTPLUGIN_H
