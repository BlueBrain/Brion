/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Juan Hernando <juan.hernando@epfl.ch>
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
 * This file is part of Brion <https://github.com/BlueBrain/Brion>
 */

#include <brion/brion.h>
#include <tests/paths.h>

#define BOOST_TEST_MODULE SimulationConfig
#include <boost/filesystem/path.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/version.hpp>

namespace fs = boost::filesystem;

BOOST_AUTO_TEST_CASE(invalid_open)
{
    BOOST_CHECK_THROW(brion::SimulationConfig("bla"), std::runtime_error);

    fs::path path(BRION_TESTDATA);
    path /= "local/README";
    BOOST_CHECK_THROW(brion::SimulationConfig(path.string()),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(verify_data)
{
    fs::path path(BRION_TESTDATA);
    path /= "sonata";

    brion::SimulationConfig config((path / "simulation.json").string());

    BOOST_CHECK_EQUAL(config.getNetworkConfig(),
                      (path / "simple_network.json").string());
    // The extra ./ is necessary because we need at least boost 1.60 to remove
    // . and .. for paths that do not resolve to an existing file.
    BOOST_CHECK_EQUAL(config.getNodeSetFilepath(),
#if BOOST_VERSION / 100 > 1060
                      (path / "node_sets.json").string());
#else
                      (path / "./node_sets.json").string());
#endif
    BOOST_CHECK_EQUAL(config.getSpikesFilepath(),
                      (path / "the_spikes.h5").string());

    const auto expected = {"all_compartments", "soma"};
    const auto names = config.getCompartmentReportNames();

    BOOST_CHECK_EQUAL_COLLECTIONS(names.begin(), names.end(), expected.begin(),
                                  expected.end());
    BOOST_CHECK_EQUAL(config.getCompartmentReportFilepath("soma"),
                      (path / "soma.h5").string());
    BOOST_CHECK_EQUAL(config.getCompartmentReportFilepath("all_compartments"),
                      (path / "report.h5").string());
}
