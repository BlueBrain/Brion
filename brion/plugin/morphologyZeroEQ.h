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

#include "../constants.h"        // used inline
#include "../morphologyPlugin.h" // base class

#include <servus/serializable.h> // base class

#include <future>

namespace brion
{
namespace plugin
{
class MorphologyZeroEQ : public MorphologyPlugin, public servus::Serializable
{
public:
    explicit MorphologyZeroEQ(const MorphologyInitData& initData);
    MorphologyZeroEQ(const void* data, size_t size);
    virtual ~MorphologyZeroEQ();

    // Needs to be inline, otherwise we would need to link circulary to Brion
    explicit MorphologyZeroEQ(const brion::Morphology& m)
        : MorphologyPlugin(m.getInitData())
        , _points(m.readPoints())
        , _sections(m.readSections())
        , _types(m.readSectionTypes())
        , _apicals(m.readApicals())
        , _perimeters(m.readPerimeters())
    {
    }

    /** Check if this plugin can handle the given uri. */
    static bool handles(const MorphologyInitData& initData);
    static std::string getDescription();

    Vector4fsPtr readPoints() const final;
    Vector2isPtr readSections() const final;
    SectionTypesPtr readSectionTypes() const final;
    Vector2isPtr readApicals() const final;
    floatsPtr readPerimeters() const final;

    void writePoints(const Vector4fs& points) final;
    void writeSections(const Vector2is& sections) final;
    void writeSectionTypes(const SectionTypes& types) final;
    void writeApicals(const Vector2is& apicals) final;
    void writePerimeters(const floats& perimeters) final;
    void flush() final;

private:
    void _load() const; // throws

    // Serializable API
    std::string getTypeName() const final
    {
        return "brion::plugin::MorphologyZeroEQ";
    }
    bool _fromBinary(const void* data, const size_t size) final;
    servus::Serializable::Data _toBinary() const final;

    // ZeroEQ API
    class Client;
    using ClientPtr = std::shared_ptr<Client>;
    ClientPtr _getClient();

    Vector4fsPtr _points;
    Vector2isPtr _sections;
    SectionTypesPtr _types;
    Vector2isPtr _apicals;
    floatsPtr _perimeters;
    ClientPtr _client;                 // during pending load requests
    mutable std::future<void> _loader; // fulfilled by client loader thread pool
};
}
}
