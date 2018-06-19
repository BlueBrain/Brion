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

#include <tests/paths.h>

#include <brain/brain.h>

#define BOOST_TEST_MODULE Sonata_simulation
#include <boost/test/unit_test.hpp>

const brion::URI TEST_SONATA_SIMULATION_URI(std::string("file://") +
                                            BRION_TESTDATA +
                                            "/sonata/simulation.json");

BOOST_AUTO_TEST_CASE(constructors)
{
    brain::Simulation simulation(TEST_SONATA_SIMULATION_URI);
    BOOST_CHECK_THROW(brain::Simulation(brion::URI("nonexistentfile.json")),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(openCircuit)
{
    brain::Simulation simulation(TEST_SONATA_SIMULATION_URI);
    auto circuit = simulation.openCircuit();

    brion::GIDSet ids = {0, 2, 3, 7};
    const auto postitions = circuit.getPositions(ids);
    BOOST_CHECK_EQUAL(postitions[0], vmml::Vector3f(0, 0, 0.5));
    BOOST_CHECK_EQUAL(postitions[1], vmml::Vector3f(2, 0, 0.5));
    BOOST_CHECK_EQUAL(postitions[2], vmml::Vector3f(3, 1, 0.5));
    BOOST_CHECK_EQUAL(postitions[3], vmml::Vector3f(1, 1, -0.5));
}

BOOST_AUTO_TEST_CASE(targets)
{
    brain::Simulation simulation(TEST_SONATA_SIMULATION_URI);
    // Node
    BOOST_CHECK(simulation.getTargetNames() == brion::Strings());

    brion::GIDSet gids{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    BOOST_CHECK(simulation.getGIDs() == gids);

    BOOST_CHECK_THROW(simulation.getGIDs("foo"), std::runtime_error);
}
