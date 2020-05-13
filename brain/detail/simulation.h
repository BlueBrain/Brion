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

#include "targets.h"
#include "util.h"

#include "../log.h"

#include <brion/blueConfig.h>
#include <brion/simulationConfig.h>

#include <brain/circuit.h>
#include <brain/simulation.h>

#include <boost/algorithm/string.hpp>

namespace brain
{
namespace
{
char const* const _defaultNeuronSpikeFile = "/out.dat";
}

struct Simulation::Impl
{
    virtual ~Impl() {}
    virtual brion::URI getCircuitSource() const = 0;
    virtual Strings getTargetNames() const = 0;
    virtual GIDSet getGIDs(const std::string& label, const float fraction,
                           const size_t* const seed) const = 0;
    virtual Strings getCompartmentReportNames() const = 0;
    virtual brion::URI getSpikeReportSource() const = 0;
    virtual brion::URI getCompartmentReportSource(
        const std::string& name) const = 0;
};

class BlueConfig : public Simulation::Impl
{
public:
    BlueConfig(const URI& source)
        : _config(source.getPath())
        , _targets(_config)
    {
        const brion::Strings& runs =
            _config.getSectionNames(brion::CONFIGSECTION_RUN);
        if (runs.size() != 1)
            BRAIN_THROW("Not exactly one Run section found in BlueConfig")
        _label = runs[0];

        _outputPath =
            _config.get(brion::CONFIGSECTION_RUN, _label, "OutputRoot");
    }

    brion::URI getCircuitSource() const final
    {
        return brion::URI{_config.getSource()};
    }

    Strings getTargetNames() const final { return _targets.getTargetNames(); }
    GIDSet getGIDs(const std::string& target, const float fraction,
                   const size_t* const seed) const final
    {
        GIDSet gids = target.empty() ? _getGIDs() : _targets.getGIDs(target);
        if (fraction == 1)
            return gids;
        else
            return randomSet(gids, fraction, seed);
    }

    Strings getCompartmentReportNames() const final
    {
        return _config.getSectionNames(brion::CONFIGSECTION_REPORT);
    }

    brion::URI getSpikeReportSource() const final
    {
        const auto& source =
            _config.get(brion::CONFIGSECTION_RUN, _label, "SpikesPath");
        return brion::URI{source.empty() ? _outputPath + _defaultNeuronSpikeFile
                                         : source};
    }

    brion::URI getCompartmentReportSource(const std::string& name) const final
    {
        return _config.getReportSource(name);
    }

private:
    brion::BlueConfig _config;
    Targets _targets;
    std::string _label;
    std::string _outputPath;

    GIDSet _getGIDs() const
    {
        const auto target =
            _config.get(brion::CONFIGSECTION_RUN, _label, "CircuitTarget");
        if (target.empty())
        {
            const auto circuit = Circuit{_config};
            return circuit.getGIDs();
        }
        return getGIDs(target, 1.0, nullptr);
    }
};

class SonataConfig : public Simulation::Impl
{
public:
    SonataConfig(const URI& source)
        : _config(source.getPath())
    {
    }

    brion::URI getCircuitSource() const final
    {
        return brion::URI{_config.getNetworkConfig()};
    }

    Strings getTargetNames() const final
    {
        BRAIN_WARN << "Node set processing not supported yet in SONATA";
        return Strings();
    }
    GIDSet getGIDs(const std::string& target, const float fraction,
                   const size_t* const seed) const final
    {
        if (target.empty())
        {
            const auto circuit = Circuit{getCircuitSource()};
            if (fraction == 1)
                return circuit.getGIDs();
            else
                return randomSet(circuit.getGIDs(), fraction, seed);
        }
        BRAIN_THROW("Target translation not implemented in SONATA")
    }

    Strings getCompartmentReportNames() const final
    {
        return _config.getCompartmentReportNames();
    }

    brion::URI getSpikeReportSource() const final
    {
        return brion::URI(_config.getSpikesFilepath());
    }

    brion::URI getCompartmentReportSource(const std::string& name) const final
    {
        return brion::URI(_config.getCompartmentReportFilepath(name));
    }

private:
    brion::SimulationConfig _config;
};
}
