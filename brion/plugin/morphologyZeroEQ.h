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

#pragma once

#include "../detail/serializableMorphology.h" // member
#include "../morphologyPlugin.h"              // base class

#include <future>

namespace brion
{
namespace plugin
{
class MorphologyZeroEQ : public MorphologyPlugin
{
public:
    explicit MorphologyZeroEQ(const MorphologyInitData& initData);
    virtual ~MorphologyZeroEQ();

    /** Check if this plugin can handle the given uri. */
    static bool handles(const MorphologyInitData& initData);
    static std::string getDescription();

    Vector4fsPtr readPoints() final;
    Vector2isPtr readSections() final;
    SectionTypesPtr readSectionTypes() final;
    Vector2isPtr readApicals() final;
    floatsPtr readPerimeters() final;

    void writePoints(const Vector4fs& points) final;
    void writeSections(const Vector2is& sections) final;
    void writeSectionTypes(const SectionTypes& types) final;
    void writeApicals(const Vector2is& apicals) final;
    void writePerimeters(const floats& perimeters) final;
    void flush() final;

private:
    // ZeroEQ API
    class Client;
    using ClientPtr = std::shared_ptr<Client>;
    ClientPtr _getClient();
    void _load(); // throws

    detail::SerializableMorphology _data;
    ClientPtr _client;         // during pending load requests
    std::future<bool> _loader; // fulfilled by client loader thread pool
};
}
}
