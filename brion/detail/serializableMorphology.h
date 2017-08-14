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

#include <brion/types.h>

#include <servus/serializable.h> // base class

namespace brion
{
namespace detail
{
class SerializableMorphology : public servus::Serializable
{
public:
    SerializableMorphology() = default;
    SerializableMorphology(const void* data, size_t size);
    explicit SerializableMorphology(brion::Morphology& m);
    virtual ~SerializableMorphology();

    MorphologyVersion getVersion() const { return _version; }
    CellFamily getFamily() const { return _family; }
    Vector4fsPtr getPoints() { return _points; }
    Vector2isPtr getSections() { return _sections; }
    SectionTypesPtr getSectionTypes() { return _types; }
    floatsPtr getPerimeters() { return _perimeters; }

    ConstVector4fsPtr getPoints() const { return _points; }
    ConstVector2isPtr getSections() const { return _sections; }
    ConstSectionTypesPtr getSectionTypes() const { return _types; }

    bool hasData() const
    {
        return _points || _sections || _types || _perimeters;
    }

private:
    // Serializable API
    std::string getTypeName() const final
    {
        return "brion::SerializableMorphology::v2";
    }
    bool _fromBinary(const void* data, const size_t size) final;
    servus::Serializable::Data _toBinary() const final;

    MorphologyVersion _version;
    CellFamily _family;
    Vector4fsPtr _points;
    Vector2isPtr _sections;
    SectionTypesPtr _types;
    floatsPtr _perimeters;
};
}
}

// Inline implementation so that users (BrionPlugins) don't need to link Brion
#include "serializableMorphology.ipp"
