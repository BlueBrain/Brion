/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Juan Hernando <jhernando@fi.upm.es>
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

#include "soma.h"
#include "morphologyImpl.h"
#include "section.h"

namespace brain
{
namespace neuron
{
namespace
{
glm::vec3 _computeCentroid(const Vector4fs& points)
{
    glm::vec3 centroid;
    for (const auto& point : points)
        centroid += glm::vec3(point);
    centroid /= static_cast<float>(points.size());
    return centroid;
}
}

Soma::Soma(Morphology::ImplPtr morphology)
    : _morphology(morphology)
{
}

Soma::Soma(const Soma& soma)
    : _morphology(soma._morphology)
{
}

Soma& Soma::operator=(const Soma& soma)
{
    if (&soma == this)
        return *this;
    _morphology = soma._morphology;
    return *this;
}

Vector4fs Soma::getProfilePoints() const
{
    return _morphology->getSectionSamples(_morphology->somaSection);
}

float Soma::getMeanRadius() const
{
    const auto points = getProfilePoints();
    if (points.size() == 1)
        // Assume a (point, radius) soma
        return points[0][3];

    const auto centroid = _computeCentroid(points);
    float radius = 0;
    for (const auto& point : points)
        radius += glm::length(glm::vec3(point) - centroid);
    return radius /= float(points.size());
}

float Soma::getMaxRadius() const
{
    const auto& points = getProfilePoints();
    if (points.size() == 1)
        // Assume a (point, radius) soma
        return points[0][3];

    const auto& centroid = _computeCentroid(points);
    float max = 0;
    for (const auto& point : points)
    {
        auto radius = glm::length(glm::vec3(point) - centroid);
        max = std::max(max, radius);
    }
    return max;
}

glm::vec3 Soma::getCentroid() const
{
    return _computeCentroid(getProfilePoints());
}

Sections Soma::getChildren() const
{
    const uint32_ts& children =
        _morphology->getChildren(_morphology->somaSection);
    Sections result;
    for (const uint32_t id : children)
        result.push_back(Section(id, _morphology));
    return result;
}
}
}
