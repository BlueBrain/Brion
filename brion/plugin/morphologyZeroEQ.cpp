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
#include "../detail/skipWhiteSpace.h"

#include <lunchbox/pluginRegisterer.h>
#ifdef BRION_USE_ZEROEQ
#include <zeroeq/client.h>
#include <zeroeq/uri.h>
#endif

#include <mutex>

namespace brion
{
namespace plugin
{
namespace
{
lunchbox::PluginRegisterer<MorphologyZeroEQ> registerer;

template <typename T>
void _serializeArray(uint8_t*& dst, const std::shared_ptr<std::vector<T>>& src)
{
    const uint64_t arraySize = src->size();
    *reinterpret_cast<uint64_t*>(dst) = arraySize;
    dst += sizeof(uint64_t);
    memcpy(dst, src->data(), sizeof(T) * src->size());
    dst += sizeof(T) * src->size();
}

template <typename T>
bool _deserializeArray(std::shared_ptr<std::vector<T>>& dst,
                       const uint8_t*& src, const uint8_t* end)
{
    if (src + sizeof(uint64_t) > end)
        return false;
    const uint64_t arraySize = *reinterpret_cast<const uint64_t*>(src);
    src += sizeof(uint64_t);

    if (src + sizeof(T) * arraySize > end)
        return false;
    const T* dstPtr = reinterpret_cast<const T*>(src);
    dst.reset(new std::vector<T>(dstPtr, dstPtr + arraySize));
    src += sizeof(T) * arraySize;
    return true;
}
}

#ifdef BRION_USE_ZEROEQ
const std::string SERVER_SESSION("morphologyServer");

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

    bool request(const servus::Serializable& r, const zeroeq::ReplyFunc& func)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _client.request(r, func);
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
        return _client.receive(10);
    }

private:
    std::mutex _mutex;
    zeroeq::Client _client;
};
#endif

MorphologyZeroEQ::MorphologyZeroEQ(const MorphologyInitData& initData)
    : MorphologyPlugin(initData)
    , _points(new Vector4fs)
    , _sections(new Vector2is)
    , _types(new SectionTypes)
    , _apicals(new Vector2is)
    , _perimeters(new floats)
{
#ifdef BRION_USE_ZEROEQ
    const std::string path = initData.getURI().getPath();
    ClientPtr client = _getClient();
    bool received = false;
    bool success = false;
    const auto handler = [&](const uint128_t& id, const void* data,
                             const size_t size) {
        if (id == 0)
            LBWARN << "Server could not load morphology" << std::endl;
        if (data && size)
            success = _fromBinary(data, size);
        received = true;
    };

    if (!client->request(ZEROEQ_GET_MORPHOLOGY, path.data(), path.size(),
                         handler))
    {
        LBTHROW(std::runtime_error("Failed to request morphology data"));
    }

    while (!received)
        client->receive();

    if (!success)
        LBTHROW(std::runtime_error(
            "Failed to construct morphology from binary data"));
#else
    LBTHROW(std::runtime_error("Missing ZeroEQ support"));
#endif
}

MorphologyZeroEQ::MorphologyZeroEQ(const void* data, const size_t size)
    : MorphologyPlugin(MorphologyInitData({}))
{
    if (!_fromBinary(data, size))
        LBTHROW(std::runtime_error(
            "Failed to construct morphology from binary data"));
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

void MorphologyZeroEQ::writePoints(const Vector4fs& points)
{
    _points.reset(new Vector4fs(points));
}

void MorphologyZeroEQ::writeSections(const Vector2is& sections)
{
    _sections.reset(new Vector2is(sections));
}

void MorphologyZeroEQ::writeSectionTypes(const SectionTypes& types)
{
    _types.reset(new SectionTypes(types));
}

void MorphologyZeroEQ::writeApicals(const Vector2is& apicals)
{
    _apicals.reset(new Vector2is(apicals));
}

void MorphologyZeroEQ::writePerimeters(const floats& perimeters)
{
    _perimeters.reset(new floats(perimeters));
}

void MorphologyZeroEQ::flush()
{
    LBUNIMPLEMENTED; // implement set request
}

#ifdef BRION_USE_ZEROEQ
MorphologyZeroEQ::ClientPtr MorphologyZeroEQ::_getClient()
{
    // static factory reusing Clients for all instances of the same URI
    using ClientMap = std::unordered_map<std::string, std::weak_ptr<Client>>;
    static ClientMap clients;

    // OPT: keep the last client alive, otherwise single-threaded constructions
    // would recreate client for each morphology
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
#endif

servus::Serializable::Data MorphologyZeroEQ::_toBinary() const
{
    servus::Serializable::Data data;

    data.size = sizeof(MorphologyVersion) + sizeof(CellFamily) +
                sizeof(uint64_t) + sizeof(brion::Vector4f) * _points->size() +
                sizeof(uint64_t) + sizeof(brion::Vector2i) * _sections->size() +
                sizeof(uint64_t) + sizeof(uint32_t) * _types->size() +
                sizeof(uint64_t) + sizeof(brion::Vector2i) * _apicals->size() +
                sizeof(uint64_t) + sizeof(float) * _perimeters->size();

    uint8_t* ptr = new uint8_t[data.size];
    data.ptr.reset(ptr, std::default_delete<uint8_t[]>());

    *reinterpret_cast<MorphologyVersion*>(ptr) = getVersion();
    ptr += sizeof(MorphologyVersion);

    *reinterpret_cast<CellFamily*>(ptr) = getCellFamily();
    ptr += sizeof(CellFamily);

    _serializeArray(ptr, _points);
    _serializeArray(ptr, _sections);
    _serializeArray(ptr, _types);
    _serializeArray(ptr, _apicals);
    _serializeArray(ptr, _perimeters);
    return data;
}

bool MorphologyZeroEQ::_fromBinary(const void* data, const size_t size)
{
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(data);
    const uint8_t* const end = ptr + size;
    if (size < sizeof(MorphologyVersion) + sizeof(CellFamily))
        return false;

    _data.version = *reinterpret_cast<const MorphologyVersion*>(ptr);
    ptr += sizeof(MorphologyVersion);

    _data.family = *reinterpret_cast<const CellFamily*>(ptr);
    ptr += sizeof(CellFamily);

    return _deserializeArray(_points, ptr, end) &&
           _deserializeArray(_sections, ptr, end) &&
           _deserializeArray(_types, ptr, end) &&
           _deserializeArray(_apicals, ptr, end) &&
           _deserializeArray(_perimeters, ptr, end);
}
}
}
