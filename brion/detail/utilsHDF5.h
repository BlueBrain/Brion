/* Copyright (c) 2016-2017, EPFL/Blue Brain Project
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

#ifndef BRION_DETAIL_UTILSHDF5
#define BRION_DETAIL_UTILSHDF5

#include <brion/types.h>

#include <highfive/H5Attribute.hpp>
#include <highfive/H5DataType.hpp>

namespace HighFive
{
class File;

namespace details
{
// ARRAY SIZES
// -------------------------------------------------------
template <typename T>
struct array_dims<std::vector<glm::tvec1<T>>>
{
    static const size_t value = 2;
};

template <typename T>
struct array_dims<std::vector<glm::tvec2<T>>>
{
    static const size_t value = 2;
};

template <typename T>
struct array_dims<std::vector<glm::tvec3<T>>>
{
    static const size_t value = 2;
};

template <typename T>
struct array_dims<std::vector<glm::tvec4<T>>>
{
    static const size_t value = 2;
};

// ARRAY TYPES
// -------------------------------------------------------
template <typename T>
struct type_of_array<std::vector<glm::tvec1<T>>>
{
    typedef T type;
};

template <typename T>
struct type_of_array<std::vector<glm::tvec2<T>>>
{
    typedef T type;
};

template <typename T>
struct type_of_array<std::vector<glm::tvec3<T>>>
{
    typedef T type;
};

template <typename T>
struct type_of_array<std::vector<glm::tvec4<T>>>
{
    typedef T type;
};

template <typename T>
struct type_of_array<glm::tvec1<T>>
{
    typedef T type;
};

template <typename T>
struct type_of_array<glm::tvec2<T>>
{
    typedef T type;
};

template <typename T>
struct type_of_array<glm::tvec3<T>>
{
    typedef T type;
};

template <typename T>
struct type_of_array<glm::tvec4<T>>
{
    typedef T type;
};

// DATA CONVERTERS
// -------------------------------------------------------
template <typename T>
struct data_converter<std::vector<glm::tvec1<T>>>
{
    inline data_converter(std::vector<glm::tvec1<T>>&, DataSpace&) {}
    inline T* transform_read(std::vector<glm::tvec1<T>>& vector)
    {
        _buf = std::vector<T>(vector.size(), T());
        return reinterpret_cast<T*>(_buf.data());
    }

    inline T* transform_write(std::vector<glm::tvec1<T>>& vector)
    {
        _buf = std::vector<T>(vector.size(), T());
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
            _buf[i] = vector[i].x;

        return reinterpret_cast<T*>(vector.data());
    }

    inline void process_result(std::vector<glm::tvec1<T>>& vector)
    {
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
            vector[i] = glm::tvec1<T>(_buf[i]);
    }

    std::vector<T> _buf;
};

template <typename T>
struct data_converter<std::vector<glm::tvec2<T>>>
{
    inline data_converter(std::vector<glm::tvec2<T>>&, DataSpace&) {}
    inline T* transform_read(std::vector<glm::tvec2<T>>& vector)
    {
        _buf = std::vector<T>(vector.size() * 2, T());
        return reinterpret_cast<T*>(_buf.data());
    }

    inline T* transform_write(std::vector<glm::tvec2<T>>& vector)
    {
        _buf = std::vector<T>(vector.size() * 2, T());
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
        {
            const auto idx = i * 2;
            _buf[idx] = vector[i].x;
            _buf[idx+1] = vector[i].y;
        }
        return reinterpret_cast<T*>(vector.data());
    }

    inline void process_result(std::vector<glm::tvec2<T>>& vector)
    {
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
        {
            const auto idx = i * 2;
            vector[i] = glm::tvec2<T>(_buf[idx], _buf[idx+1]);
        }
    }

    std::vector<T> _buf;
};

template <typename T>
struct data_converter<std::vector<glm::tvec3<T>>>
{
    inline data_converter(std::vector<glm::tvec3<T>>&, DataSpace&) {}
    inline T* transform_read(std::vector<glm::tvec3<T>>& vector)
    {
        _buf = std::vector<T>(vector.size() * 3, T());
        return reinterpret_cast<T*>(_buf.data());
    }

    inline T* transform_write(std::vector<glm::tvec3<T>>& vector)
    {
        _buf = std::vector<T>(vector.size() * 3, T());
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
        {
            const auto idx = i * 3;
            _buf[idx] = vector[i].x;
            _buf[idx+1] = vector[i].y;
            _buf[idx+2] = vector[i].z;
        }
        return reinterpret_cast<T*>(vector.data());
    }

    inline void process_result(std::vector<glm::tvec3<T>>& vector)
    {
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
        {
            const auto idx = i * 3;
            vector[i] = glm::tvec3<T>(_buf[idx], _buf[idx+1], _buf[idx+2]);
        }
    }

    std::vector<T> _buf;
};

template <typename T>
struct data_converter<std::vector<glm::tvec4<T>>>
{
    inline data_converter(std::vector<glm::tvec4<T>>&, DataSpace&) {}
    inline T* transform_read(std::vector<glm::tvec4<T>>& vector)
    {
        _buf = std::vector<T>(vector.size() * 4, T());
        return reinterpret_cast<T*>(_buf.data());
    }

    inline T* transform_write(std::vector<glm::tvec4<T>>& vector)
    {
        _buf = std::vector<T>(vector.size() * 4, T());
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
        {
            const auto idx = i * 4;
            _buf[idx] = vector[i].x;
            _buf[idx+1] = vector[i].y;
            _buf[idx+2] = vector[i].z;
            _buf[idx+3] = vector[i].w;
        }
        return reinterpret_cast<T*>(vector.data());
    }

    inline void process_result(std::vector<glm::tvec4<T>>& vector)
    {
        //#pragma omp parallel for
        for(size_t i = 0; i < vector.size(); ++i)
        {
            const auto idx = i * 4;
            vector[i] = glm::tvec4<T>(_buf[idx], _buf[idx+1],
                                      _buf[idx+2], _buf[idx+3]);
        }
    }

    std::vector<T> _buf;
};

}// namespace details

template <>
inline AtomicType<brion::SectionType>::AtomicType()
{
    _hid = H5Tcopy(H5T_NATIVE_INT);
}

template <>
inline AtomicType<brion::MorphologyVersion>::AtomicType()
{
    _hid = H5Tcopy(H5T_NATIVE_INT);
}


template<>
inline AtomicType<glm::ivec2>::AtomicType()
{
    _hid = H5Tcopy(H5T_NATIVE_INT);
}

template<>
inline AtomicType<glm::vec2>::AtomicType()
{
    _hid = H5Tcopy(H5T_NATIVE_INT);
}

template<>
inline AtomicType<glm::vec3>::AtomicType()
{
    _hid = H5Tcopy(H5T_NATIVE_INT);
}

template<>
inline AtomicType<glm::vec4>::AtomicType()
{
    _hid = H5Tcopy(H5T_NATIVE_INT);
}

}

namespace brion
{
namespace detail
{
/** Add a string attribute to the given H5 object. */
template <typename T>
inline void addStringAttribute(HighFive::AnnotateTraits<T>& object,
                               const std::string& name,
                               const std::string& value)
{
    auto dataspace = HighFive::DataSpace::From(value);
    auto attr = object.createAttribute(name, dataspace,
                                       HighFive::AtomicType<std::string>());
    attr.write(value);
}
}
}
#endif