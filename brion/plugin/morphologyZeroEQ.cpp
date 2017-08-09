/* Copyright (c) 2017, EPFL/Blue Brain Project
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

#include "morphologyZeroEQ.h"

#include "../constants.h"

#include <lunchbox/pluginRegisterer.h>
#include <lunchbox/threadPool.h>
#include <zeroeq/client.h>
#include <zeroeq/uri.h>

#include <mutex>

namespace brion
{
namespace plugin
{
namespace
{
lunchbox::PluginRegisterer<MorphologyZeroEQ> registerer;
}

const std::string SERVER_SESSION("morphologyServer");
namespace
{
lunchbox::ThreadPool& _getWorkers()
{
    static lunchbox::ThreadPool workers;
    return workers;
}
}

class MorphologyZeroEQ::Client // adds thread-safety to zeroeq::Client
{
public:
    Client()
        : _client(getenv(zeroeq::ENV_REP_SESSION.c_str())
                      ? zeroeq::DEFAULT_SESSION
                      : SERVER_SESSION)
    {
    }

    Client(const zeroeq::URI& uri)
        : _client({uri})
    {
    }

    bool request(const uint128_t& r, const void* data, size_t size,
                 const zeroeq::ReplyFunc& func)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _client.request(r, data, size, func);
    }

    bool receive()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _client.receive(0);
    }

    void lock() { _mutex.lock(); }
    void unlock() { _mutex.unlock(); }

private:
    std::mutex _mutex;
    zeroeq::Client _client;
};

MorphologyZeroEQ::MorphologyZeroEQ(const MorphologyInitData& initData)
    : MorphologyPlugin(initData)
    , _client(_getClient())
{
    ClientPtr client = _client; // keep ref for thread-safety
    const std::string path = initData.getURI().getPath();
    const auto handler = [&](const uint128_t& id, const void* data,
                             const size_t size) {
        if (id == 0)
            LBWARN << "Server could not load morphology" << std::endl;
        if (data && size)
        {
            _client->unlock();
            _data.fromBinary(data, size);
            _client->lock();
        }
        _client.reset();
    };

    if (!client->request(ZEROEQ_GET_MORPHOLOGY, path.data(), path.size(),
                         handler))
    {
        LBTHROW(std::runtime_error("Failed to request morphology data"));
    }

    _loadFuture = _getWorkers().post([&] {
        ClientPtr client_ = _client; // keep ref for thread-safety
        while (_client)
            client_->receive();
        return _data.hasData();
    });
}

MorphologyZeroEQ::~MorphologyZeroEQ()
{
    if (_loadFuture.valid())
        _loadFuture.get();
}

void MorphologyZeroEQ::_finishLoading()
{
    if (_loadFuture.valid() && !_loadFuture.get())
        LBTHROW(std::runtime_error("Failed to load morphology from server"));
}

bool MorphologyZeroEQ::handles(const MorphologyInitData& initData)
{
    return initData.getURI().getScheme() == ZEROEQ_SCHEME;
}

std::string MorphologyZeroEQ::getDescription()
{
    return "Morphology data server:\n"
           "  zeroeq://[server:port]/path/to/morphology";
}

Vector4fsPtr MorphologyZeroEQ::readPoints()
{
    _finishLoading();
    return _data.getPoints();
}

Vector2isPtr MorphologyZeroEQ::readSections()
{
    _finishLoading();
    return _data.getSections();
}

SectionTypesPtr MorphologyZeroEQ::readSectionTypes()
{
    _finishLoading();
    return _data.getSectionTypes();
}

Vector2isPtr MorphologyZeroEQ::readApicals()
{
    _finishLoading();
    return _data.getApicals();
}

floatsPtr MorphologyZeroEQ::readPerimeters()
{
    _finishLoading();
    return _data.getPerimeters();
}

void MorphologyZeroEQ::writePoints(const Vector4fs&)
{
    LBUNIMPLEMENTED;
}

void MorphologyZeroEQ::writeSections(const Vector2is&)
{
    LBUNIMPLEMENTED;
}

void MorphologyZeroEQ::writeSectionTypes(const SectionTypes&)
{
    LBUNIMPLEMENTED;
}

void MorphologyZeroEQ::writeApicals(const Vector2is&)
{
    LBUNIMPLEMENTED;
}

void MorphologyZeroEQ::writePerimeters(const floats&)
{
    LBUNIMPLEMENTED;
}

void MorphologyZeroEQ::flush()
{
    LBUNIMPLEMENTED;
}

MorphologyZeroEQ::ClientPtr MorphologyZeroEQ::_getClient()
{
    // static factory reusing Clients for all instances of the same URI
    using ClientMap = std::unordered_map<std::string, std::weak_ptr<Client>>;
    static ClientMap clients;

    // OPT: keep the last client alive, otherwise single-threaded
    // constructions would recreate client for each morphology
    static ClientPtr client;

    const auto& uri = getInitData().getURI();
    const std::string address =
        uri.getHost() + ":" + std::to_string(int(uri.getPort()));

    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    client = clients[address].lock();
    if (!client)
    {
        if (address == ":0")
            client.reset(new Client); // use zeroconf
        else
            client.reset(new Client(zeroeq::URI(address)));
        clients[address] = client;
    }

    return client;
}
}
}
