/* Copyright (c) 2018, EPFL/Blue Brain Project
 *                     Juan Hernando <juan.hernando@epfl.ch>
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

#ifndef BRION_SIMULATIONCONFIG
#define BRION_SIMULATIONCONFIG

#include <brion/api.h>
#include <brion/types.h>

namespace brion
{
/** Read access to a SONATA simulation config file.
 */
class SimulationConfig
{
public:
    BRION_API ~SimulationConfig();

    /** Open a SONATA simulation config json */
    BRION_API SimulationConfig(const std::string& source);

    BRION_API SimulationConfig(SimulationConfig&&);
    BRION_API SimulationConfig& operator=(SimulationConfig&&);

    /** @return the path to the circuit configuration json. */
    std::string getNetworkConfig() const;

    /** @return the path to the node set file. */
    std::string getNodeSetFilepath() const;

    /** @return the path to spikes .h5 file. */
    std::string getSpikesFilepath() const;

    /** @return the names of the compartment reports with membrame_report as
        the module name. */
    Strings getCompartmentReportNames() const;

    /** @return the file path to a copartment report by name.
     *  @throw std::runtime_error is the report doesn't exist *
     */
    std::string getCompartmentReportFilepath(const std::string& name) const;

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
}
#endif
