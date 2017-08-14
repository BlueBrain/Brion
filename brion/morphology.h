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

#ifndef BRION_MORPHOLOGY
#define BRION_MORPHOLOGY

#include <brion/api.h>
#include <brion/types.h>
#include <vmmlib/vector.hpp> // return value

namespace brion
{
/** Read & write access to a Morphology file.
 *
 * Following RAII, this class is ready to use after the creation and will ensure
 * release of resources upon destruction.
 */
class Morphology
{
public:
    /** Close morphology file. @version 1.0 */
    BRION_API ~Morphology();

    BRION_API Morphology(Morphology&&);
    BRION_API Morphology& operator=(Morphology&&);

    /**
     * @name Read API
     *
     * Data returned by the read methods may or may not be cached by the
     * implementation. If the returned vectors are modified this may be
     * reflected in subsequent reads of the same data. Furthermore, calling read
     * multiple times might be expensive. It is therefore recommended to call
     * each read method at most once per instantiation.
     */
    //@{
    /** Open the given source to a morphology file for reading.
     *
     * @param source filepath to morphology file
     * @throw std::runtime_error if file is not a valid morphology file
     * @version 1.0
     */
    BRION_API explicit Morphology(const std::string& source);

    /** @return the cell family of that morphology. @version 1.8 */
    BRION_API CellFamily getCellFamily() const;

    /** Read points of morphology, representing x,y,z coordinates + diameter.
     *
     * @return x,y,z coords + diameter of all points of the morphology
     * @version 1.0
     */
    BRION_API Vector4fsPtr readPoints();

    /** Read sections of morphology, representing section start index and index
     *  of the parent section.
     *
     * @return First point and parent indices of all the sections of the
     *         morphology.
     * @version 1.0
     */
    BRION_API Vector2isPtr readSections();

    /** Read section types of morphology.
     *
     * @return type of all sections of the morphology
     * @version 1.0
     */
    BRION_API SectionTypesPtr readSectionTypes();

    /**
     * @return perimeters of the cross sections for each point of the morphology
     *         in micrometers.
     * @throw std::runtime_error if empty for FAMILY_GLIA
     * @version 1.8
     */
    BRION_API floatsPtr readPerimeters();

    /** @internal */
    BRION_API MorphologyVersion getVersion() const;

    /** @internal */
    const MorphologyInitData& getInitData() const;
    //@}

private:
    Morphology(const Morphology&) = delete;
    Morphology& operator=(const Morphology&) = delete;

    class Impl;
    std::unique_ptr<Impl> _impl;
};
}
#endif
