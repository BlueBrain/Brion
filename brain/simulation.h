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
#pragma once

#include <brain/api.h>
#include <brain/types.h>

namespace brain
{
/**
   Simulation configuration parser and entry point.

   This object parses a simulation configuration and gives access to its
   circuit, reports and cell sets.
 */
class Simulation
{
public:
    /**
     * Open a simulation configuration
     *
     * @param source the URI to the BlueConfig file.
     */
    BRAIN_API explicit Simulation(const URI& source);

    BRAIN_API ~Simulation();

    /** @return the gids of the default circuit target. */
    BRAIN_API GIDSet getGIDs() const;

    /** @return the gids of a cell set given its name. */
    BRAIN_API GIDSet getGIDs(const std::string& name) const;

    /** @return the names of the reports listed in the config. */
    BRAIN_API Strings getCompartmentReportNames() const;

    /** @return the names of all cell sets referred by this simulation. */
    BRAIN_API Strings getTargetNames() const;

    /** Open the circuit associated to this simulation. */
    BRAIN_API Circuit openCircuit() const;

    /** Open the main spike report of the simulation */
    BRAIN_API SpikeReportReader openSpikeReport() const;

    /** Open a compartment report with the given name.
        @param name the report name as it appears in the configuration.
    */
    BRAIN_API CompartmentReport
        openCompartmentReport(const std::string& name) const;

private:
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    struct Impl;
    std::unique_ptr<const Impl> _impl;
};
}
