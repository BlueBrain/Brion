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

#include "circuit.h"
#include "compartmentReport.h"
#include "detail/targets.h"
#include "spikeReportReader.h"

#include <brion/blueConfig.h>

#include <lunchbox/debug.h>

namespace brain
{
namespace
{
char const* const _defaultSpikeFile = "/out.dat";
}

struct Simulation::Impl
{
    Impl(const URI& source)
        : _config(source.getPath())
        , _targets(_config)
    {
        const brion::Strings& runs =
            _config.getSectionNames(brion::CONFIGSECTION_RUN);
        if (runs.size() != 1)
            LBTHROW(
                std::runtime_error("Not exactly one Run section found "
                                   "in BlueConfig"));
        _label = runs[0];

        _outputPath =
            _config.get(brion::CONFIGSECTION_RUN, _label, "OutputRoot");
    }

    brion::BlueConfig _config;
    Targets _targets;
    std::string _label;
    std::string _outputPath;
};

Simulation::Simulation(const URI& source)
    : _impl{new Impl{source}}
{
}

Simulation::~Simulation()
{
}

Circuit Simulation::openCircuit() const
{
    return Circuit{_impl->_config};
}

SpikeReportReader Simulation::openSpikeReport() const
{
    const auto& source = _impl->_config.get(brion::CONFIGSECTION_RUN,
                                            _impl->_label, "SpikesPath");
    return SpikeReportReader(brion::URI(
        source.empty() ? _impl->_outputPath + _defaultSpikeFile : source));
}

CompartmentReport Simulation::openCompartmentReport(
    const std::string& name) const
{
    const auto path = _impl->_config.getReportSource(name);
    if (path == brain::URI{})
        throw std::runtime_error("Invalid or missing report: " + name);
    return CompartmentReport(path);
}

Strings Simulation::getCompartmentReportNames() const
{
    return _impl->_config.getSectionNames(brion::CONFIGSECTION_REPORT);
}

GIDSet Simulation::getGIDs() const
{
    const auto target = _impl->_config.get(brion::CONFIGSECTION_RUN,
                                           _impl->_label, "CircuitTarget");
    if (target.empty())
    {
        const auto circuit = Circuit{_impl->_config};
        return circuit.getGIDs();
    }
    return getGIDs(target);
}

GIDSet Simulation::getGIDs(const std::string& name) const
{
    return _impl->_targets.getGIDs(name);
}

Strings Simulation::getTargetNames() const
{
    return _impl->_targets.getTargetNames();
}
}
