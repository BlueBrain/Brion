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
 */

#include <BBP/TestDatasets.h>
#include <brain/brain.h>
#include <brion/brion.h>

#define BOOST_TEST_MODULE Simulation
#include <boost/filesystem/path.hpp>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(test_invalid_open)
{
    BOOST_CHECK_THROW(brain::Simulation{brain::URI{"/xxx"}},
                      std::runtime_error);

    boost::filesystem::path path(BBP_TESTDATA);
    path /= "local/README";
    BOOST_CHECK_THROW(brain::Simulation{brain::URI{path.string()}},
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(test_open)
{
    brain::Simulation simulation{brain::URI{bbp::test::getBlueconfig()}};
}

BOOST_AUTO_TEST_CASE(test_get_gids)
{
    brain::Simulation simulation{brain::URI{bbp::test::getBlueconfig()}};
    auto t1 = simulation.getGIDs();
    BOOST_CHECK_EQUAL(*t1.begin(), 1);
    BOOST_CHECK_EQUAL(*t1.rbegin(), 600);
    BOOST_CHECK_EQUAL(t1.size(), 600);

    auto t2 = simulation.getGIDs("Layer1");
    BOOST_CHECK_EQUAL(t2.size(), 16);
    auto t3 = simulation.getGIDs("Layer1", 0.5);
    BOOST_CHECK_EQUAL(t3.size(), 8);
    auto t4 = simulation.getGIDs("Layer1", 0.5, 0);
    BOOST_CHECK_EQUAL(t4.size(), 8);
    auto t5 = simulation.getGIDs("Layer1", 0.5, 1);
    BOOST_CHECK_EQUAL(t5.size(), 8);
    BOOST_CHECK(t4 != t5);
}

BOOST_AUTO_TEST_CASE(test_open_circuit)
{
    brain::Simulation simulation{brain::URI{bbp::test::getBlueconfig()}};
    auto circuit = simulation.openCircuit();
}

BOOST_AUTO_TEST_CASE(test_open_spike_report)
{
    brain::Simulation simulation{brain::URI{bbp::test::getBlueconfig()}};
    auto report = simulation.openSpikeReport();
}

BOOST_AUTO_TEST_CASE(test_open_compartment_report)
{
    brain::Simulation simulation{brain::URI{bbp::test::getBlueconfig()}};
    std::cout << bbp::test::getBlueconfig() << std::endl;
    const auto names = simulation.getCompartmentReportNames();
    brain::Strings expected{"voltage", "allCompartments"};
    BOOST_CHECK_EQUAL_COLLECTIONS(names.begin(), names.end(), expected.begin(),
                                  expected.end());
    for (const auto& name : names)
        auto report = simulation.openCompartmentReport(name);
}

BOOST_AUTO_TEST_CASE(test_open_invalid_compartment_report)
{
    brain::Simulation simulation{brain::URI{bbp::test::getBlueconfig()}};
    BOOST_CHECK_THROW(simulation.openCompartmentReport("foo"),
                      std::runtime_error);
}
