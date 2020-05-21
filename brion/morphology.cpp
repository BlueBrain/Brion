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

#include "morphology.h"

#include "log.h"
#include "morphologyPlugin.h"
#include "pluginLibrary.h"
#include "serializable.h"
#include "threadPool.h"

#include <future>
#include <mutex>

namespace brion
{
namespace
{
/**
 * "Plugin" for copied and deserialized morphologies.
 *
 * Does not actually load any data, but holds the data gathered from the copy or
 * deserialization.
 */
class BinaryMorphology : public MorphologyPlugin
{
public:
    BinaryMorphology(const Morphology& from)
        : MorphologyPlugin(from.getInitData())
    {
        _points = from.getPoints();
        _sections = from.getSections();
        _sectionTypes = from.getSectionTypes();
        _perimeters = from.getPerimeters();
    }

    BinaryMorphology(const void* data, size_t size)
        : MorphologyPlugin(MorphologyInitData({}))
    {
        if (!fromBinary(data, size))
            BRION_THROW("Failed to construct morphology from binary data")
    }

    void load() final { /*NOP*/}
};
}

class Morphology::Impl
{
public:
    explicit Impl(const MorphologyInitData& initData)
        : plugin(PluginLibrary::instance().create<MorphologyPlugin>(initData))
    {
        loadFuture = ThreadPool::getInstance().post([&] {
            plugin->load();
            if (plugin->getPoints().empty())
                BRION_THROW("Failed to load morphology "
                          + std::to_string(plugin->getInitData().getURI()))
        });
    }

    Impl(const Morphology& from)
        : plugin(new BinaryMorphology(from))
    {
    }

    Impl(const void* data, size_t size)
        : plugin(new BinaryMorphology(data, size))
    {
    }

    ~Impl()
    {
        try
        {
            finishLoad();
        }
        catch (const std::exception& e)
        {
            BRION_ERROR << e.what() << std::endl;
        }
        catch (...)
        {
            BRION_ERROR << "Unknown exception during morphology load" << std::endl;
        }
    }

    void finishLoad() const
    {
        // This call outside call_once is intended to work around the gcc
        // problem of deadlocking on call_once when calling it again after a
        // first call has thrown. This is not bullet-proof if multiple threads
        // try to access the morphology, but at least works for the case of
        // a single thread calling the destuctor after an accessor failed.
        if (!loadFuture.valid())
            return;

        std::call_once(loadFlag, [this]() {
            if (!loadFuture.valid())
                return;

            loadFuture.get();
        });
    }

    std::unique_ptr<MorphologyPlugin> plugin;
    mutable std::once_flag loadFlag;
    mutable std::future<void> loadFuture; // fulfilled by worker
                                          // thread pool
};

Morphology::Morphology(const URI& source)
    : _impl(new Impl(MorphologyInitData(source)))
{
}

Morphology::Morphology(const void* data, size_t size)
    : _impl(new Impl(data, size))
{
}

Morphology::Morphology(const Morphology& from)
    : _impl(new Impl(from))
{
}

Morphology& Morphology::operator=(const Morphology& from)
{
    if (this != &from)
        _impl.reset(new Impl(from));
    return *this;
}

Morphology::Morphology(Morphology&&) = default;
Morphology& Morphology::operator=(Morphology&&) = default;

Morphology::~Morphology()
{
}

CellFamily Morphology::getCellFamily() const
{
    _impl->finishLoad();
    return _impl->plugin->getCellFamily();
}

Vector4fs& Morphology::getPoints()
{
    _impl->finishLoad();
    return _impl->plugin->getPoints();
}

const Vector4fs& Morphology::getPoints() const
{
    _impl->finishLoad();
    return _impl->plugin->getPoints();
}

Vector2is& Morphology::getSections()
{
    _impl->finishLoad();
    return _impl->plugin->getSections();
}

const Vector2is& Morphology::getSections() const
{
    _impl->finishLoad();
    return _impl->plugin->getSections();
}

SectionTypes& Morphology::getSectionTypes()
{
    _impl->finishLoad();
    return _impl->plugin->getSectionTypes();
}

const SectionTypes& Morphology::getSectionTypes() const
{
    _impl->finishLoad();
    return _impl->plugin->getSectionTypes();
}

floats& Morphology::getPerimeters()
{
    _impl->finishLoad();
    return _impl->plugin->getPerimeters();
}

const floats& Morphology::getPerimeters() const
{
    _impl->finishLoad();
    return _impl->plugin->getPerimeters();
}

MorphologyVersion Morphology::getVersion() const
{
    _impl->finishLoad();
    return _impl->plugin->getVersion();
}

const MorphologyInitData& Morphology::getInitData() const
{
    _impl->finishLoad();
    return _impl->plugin->getInitData();
}

Serializable::Data Morphology::toBinary() const
{
    _impl->finishLoad();
    return _impl->plugin->toBinary();
}
}
