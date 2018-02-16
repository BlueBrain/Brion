/* Copyright (c) 2006-2018, Juan Hernando <juan.hernando@epfl.ch>
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

#include <boost/python.hpp>

#include "arrayHelpers.h"
#include "docstrings.h"
#include "helpers.h"
#include "types.h"

#include <brain/circuit.h>
#include <brain/compartmentReport.h>
#include <brain/simulation.h>
#include <brain/spikeReportReader.h>

namespace bp = boost::python;

namespace brain
{
namespace
{
SimulationPtr Simulation_initFromURI(const std::string& uri)
{
    return SimulationPtr(new Simulation(URI(uri)));
}

CircuitPtr Simulation_openCircuit(const Simulation& simulation)
{
    return CircuitPtr(new Circuit{simulation.openCircuit()});
}

SpikeReportReaderPtr Simulation_openSpikeReport(const Simulation& simulation)
{
    return SpikeReportReaderPtr(
        new SpikeReportReader{simulation.openSpikeReport()});
}

CompartmentReportPtr Simulation_openCompartmentReport(
    const Simulation& simulation, const std::string& name)
{
    return CompartmentReportPtr(
        new CompartmentReport{simulation.openCompartmentReport(name)});
}

bp::object Simulation_getGIDs1(const Simulation& simulation)
{
    return brain_python::toNumpy(brain_python::toVector(simulation.getGIDs()));
}

bp::object Simulation_getGIDs2(const Simulation& simulation,
                               const std::string& name)
{
    return brain_python::toNumpy(
        brain_python::toVector(simulation.getGIDs(name)));
}

bp::object Simulation_getCompartmentReportNames(const Simulation& simulation)
{
    const auto names = simulation.getCompartmentReportNames();
    bp::list output;
    for (const auto& name : names)
        output.append(name);
    return output;
}

bp::object Simulation_getTargetNames(const Simulation& simulation)
{
    const auto names = simulation.getTargetNames();
    bp::list output;
    for (const auto& name : names)
        output.append(name);
    return output;
}
}

void export_Simulation()
// clang-format off
{

const auto self = bp::arg("self");
bp::class_<Simulation, boost::noncopyable, SimulationPtr>(
    "Simulation", DOXY_CLASS(brain::Simulation), bp::no_init)
    .def("__init__", bp::make_constructor(Simulation_initFromURI),
         DOXY_FN(brain::Simulation::Simulation(const URI&)))
    .def("open_circuit", Simulation_openCircuit,
         DOXY_FN(brain::Simulation::openCircuit))
    .def("open_spike_report", Simulation_openSpikeReport,
         DOXY_FN(brain::Simulation::openCircuit))
    .def("open_compartment_report", Simulation_openCompartmentReport,
         DOXY_FN(brain::Simulation::openCompartmentReport))
    .def("gids", Simulation_getGIDs1,
         DOXY_FN(brain::Simulation::getGIDs() const))
    .def("gids", Simulation_getGIDs2, (self, bp::arg("target")),
         DOXY_FN(brain::Simulation::getGIDs(const std::string&) const))
    .def("compartment_report_names", Simulation_getCompartmentReportNames,
         DOXY_FN(brain::Simulation::getCompartmentReportNames))
    .def("target_names", Simulation_getTargetNames,
         DOXY_FN(brain::Simulation::getTargetNames));
}
// clang-format on
}
