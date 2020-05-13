/* Copyright (c) 2017, EPFL/Blue Brain Project
 *                     Mohamed-Ghaith Kaabi <mohamed.kaabi@epfl.ch>
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

#include "spikeReport.h"

#include "log.h"
#include "pluginInitData.h"
#include "pluginLibrary.h"
#include "spikeReportPlugin.h"
#include "threadPool.h"

#include <brion/version.h>

#include <memory>
#include <queue>

namespace
{
const std::string spikePluginDSONamePattern("Brion.*SpikeReport");

template<typename R>
bool is_ready(std::future<R> const& f)
{
    return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
}

using SpikeReportInitData = ::brion::PluginInitData;

namespace brion
{
namespace detail
{
class SpikeReport
{
public:
    explicit SpikeReport(const SpikeReportInitData& initData)
     : plugin(PluginLibrary::instance().create<SpikeReportPlugin>(initData))
    {
    }

    std::unique_ptr<SpikeReportPlugin> plugin;
    ThreadPool threadPool{1};
    bool busy {false};
};
}

struct BusyGuard
{
    BusyGuard(bool& busy_)
        : busy(busy_)
    {
    }
    ~BusyGuard() { busy = false; }
    bool& busy;
};
}

namespace brion
{
SpikeReport::SpikeReport(const URI& uri, int mode)
    : _impl(new detail::SpikeReport(SpikeReportInitData(uri, mode)))
{
    switch (mode)
    {
    case MODE_READ:
    case MODE_WRITE:
        break;
    default:
        BRION_THROW("Unhandled open mode")
    }
}

SpikeReport::SpikeReport(const URI& uri, const GIDSet& ids)
    : SpikeReport(uri, MODE_READ)
{
    _impl->plugin->setFilter(ids);
}

SpikeReport::~SpikeReport()
{
    if (_impl)
        close();
}

SpikeReport::SpikeReport(SpikeReport&& from)
    : _impl(std::move(from._impl))
{
}

SpikeReport& SpikeReport::operator=(SpikeReport&& from)
{
    if (this != &from)
        _impl = std::move(from._impl);
    return *this;
}

std::string SpikeReport::getDescriptions()
{
    return PluginLibrary::instance().getManager<SpikeReportPlugin>().getDescriptions();
}

void SpikeReport::close()
{
    if (_impl->plugin->isClosed())
        return;

    if (_impl->threadPool.hasPendingJobs())
    {
        // interrupt the jobs
        _impl->plugin->_setInterrupted(true);
        // blocks until all the pending jobs are done
        _impl->threadPool.post([] {}).get();
    }

    _impl->plugin->close();
    _impl->plugin->_setClosed();
}

const URI& SpikeReport::getURI() const
{
    return _impl->plugin->getURI();
}

float SpikeReport::getCurrentTime() const
{
    return _impl->plugin->getCurrentTime();
}

float SpikeReport::getEndTime() const
{
    return _impl->plugin->getEndTime();
}

SpikeReport::State SpikeReport::getState() const
{
    return _impl->plugin->getState();
}

bool SpikeReport::isClosed() const
{
    return _impl->plugin->isClosed();
}

void SpikeReport::interrupt()
{
    _impl->plugin->_setInterrupted(true);
    // blocks until all the pending jobs are done
    _impl->threadPool.post([] {}).get();
    _impl->busy = false;
    _impl->plugin->_setInterrupted(false);
}

std::future<Spikes> SpikeReport::read(float min)
{
    _impl->plugin->_checkNotClosed();
    _impl->plugin->_checkCanRead();
    _impl->plugin->_checkStateOk();

    if (_impl->busy)
        BRION_THROW("Can't read: Pending read operation")

    _impl->busy = true;
    return _impl->threadPool.post([&, min] {
        BusyGuard guard(_impl->busy);
        return _impl->plugin->read(min);
    });
}

std::future<Spikes> SpikeReport::readUntil(const float max)
{
    _impl->plugin->_checkNotClosed();
    _impl->plugin->_checkCanRead();
    _impl->plugin->_checkStateOk();

    if (max <= getCurrentTime())
    {
        BRION_THROW_IMPL(std::logic_error("Can't read to " + std::to_string(max) +
                                 " with current time " +
                                 std::to_string(getCurrentTime())));
    }

    if (_impl->busy)
        BRION_THROW("Can't read: Pending read operation")

    _impl->busy = true;
    return _impl->threadPool.post([&, max] {
        BusyGuard guard(_impl->busy);
        return _impl->plugin->readUntil(max);
    });

}

std::future<void> SpikeReport::seek(const float toTimeStamp)
{
    _impl->plugin->_checkNotClosed();
    if (_impl->plugin->getAccessMode() == MODE_READ)
    {
        if (_impl->busy)
            BRION_THROW("Can't seek: Pending read operation")

        _impl->busy = true;
        return _impl->threadPool.post([&, toTimeStamp] {
            BusyGuard guard(_impl->busy);
            _impl->plugin->readSeek(toTimeStamp);
        });
    }

    return _impl->threadPool.post(
            [&, toTimeStamp] { return _impl->plugin->writeSeek(toTimeStamp); });
}

void SpikeReport::write(const Spikes& spikes)
{
    write(spikes.data(), spikes.size());
}

void SpikeReport::write(const Spike* spikes, const size_t size)
{
    _impl->plugin->_checkCanWrite();
    _impl->plugin->_checkNotClosed();

    if (_impl->busy)
        BRION_THROW("Can't write spikes: Pending seek operation")

    if (size != 0 && spikes[0].first < getCurrentTime())
    {
        BRION_THROW_IMPL(std::logic_error("Can't write spikes: first spike at " + std::to_string(spikes[0].first) +
                  " time inferior to current time " + std::to_string(getCurrentTime())))
    }

    if (size != 0 && !std::is_sorted(spikes, spikes + size,
                                     [](const Spike& x, const Spike& y) {
                                         return x.first < y.first;
                                     }))
    {
        BRION_THROW_IMPL(std::logic_error("Can't write spikes: Expecting a sorted spikes"))
    }

    _impl->plugin->write(spikes, size);
}

bool SpikeReport::supportsBackwardSeek() const
{
    return _impl->plugin->supportsBackwardSeek();
}
}
