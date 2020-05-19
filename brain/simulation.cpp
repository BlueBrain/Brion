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
#include "simulation.h"

#include "detail/simulation.h"

#include "compartmentReport.h"
#include "log.h"
#include "spikeReportReader.h"

namespace brain
{
namespace
{
Simulation::Impl* newImpl(const URI& source)
{
    if (boost::algorithm::ends_with(source.getPath(), ".json"))
        return new SonataConfig{source};
    else
        return new BlueConfig{source};
}
}

Simulation::Simulation(const URI& source)
    : _impl{newImpl(source)}
{
}

Simulation::~Simulation()
{
}

Circuit Simulation::openCircuit() const
{
    return Circuit{_impl->getCircuitSource()};
}

SpikeReportReader Simulation::openSpikeReport() const
{
    return SpikeReportReader{_impl->getSpikeReportSource()};
}

CompartmentReport Simulation::openCompartmentReport(
    const std::string& name) const
{
    const auto path = _impl->getCompartmentReportSource(name);
    if (path == brain::URI{})
        BRAIN_THROW("Invalid or missing report: " + name)
    return CompartmentReport{path};
}

Strings Simulation::getCompartmentReportNames() const
{
    return _impl->getCompartmentReportNames();
}

GIDSet Simulation::getGIDs() const
{
    return _impl->getGIDs("", 1.0, nullptr);
}

GIDSet Simulation::getGIDs(const std::string& name) const
{
    return _impl->getGIDs(name, 1.0, nullptr);
}

GIDSet Simulation::getGIDs(const float fraction) const
{
    return _impl->getGIDs("", fraction, nullptr);
}

GIDSet Simulation::getGIDs(const std::string& name, const float fraction) const
{
    return _impl->getGIDs(name, fraction, nullptr);
}

GIDSet Simulation::getGIDs(const float fraction, const size_t seed) const
{
    return _impl->getGIDs("", fraction, &seed);
}

GIDSet Simulation::getGIDs(const std::string& name, const float fraction,
                           const size_t seed) const
{
    return _impl->getGIDs(name, fraction, &seed);
}

Strings Simulation::getTargetNames() const
{
    return _impl->getTargetNames();
}
}
