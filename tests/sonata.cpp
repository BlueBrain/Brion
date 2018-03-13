/* Copyright (c) 2013-2017, EPFL/Blue Brain Project
 *                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
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

#include <brain/brain.h>
#include <brion/brion.h>
#include <tests/paths.h>

#include <brion/nodes.h>

#define BOOST_TEST_MODULE Sonata
#include <boost/test/unit_test.hpp>
#include <cstdarg>

// typedef for brevity
typedef brion::Vector4f V4f;
typedef brion::Vector3f V3f;

const brion::URI TEST_MORPHOLOGY_URI(std::string("file://") + BRION_TESTDATA +
                                     "/sonata/simple_nodes.h5");

constexpr char POPULATION_NAME[] = "simple";

BOOST_AUTO_TEST_CASE(sonata_constructors)
{
    brion::Nodes nodes0(TEST_MORPHOLOGY_URI);
    brion::Nodes nodes1(TEST_MORPHOLOGY_URI);
}

BOOST_AUTO_TEST_CASE(sonata_getPopulationNames)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto pn = nodes.getPopulationNames();
    BOOST_CHECK_EQUAL(pn.size(), 1);
    BOOST_CHECK_EQUAL(pn[0], POPULATION_NAME);
}

BOOST_AUTO_TEST_CASE(sonata_getNumberOfNodes)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto num_nodes = nodes.getNumberOfNodes(POPULATION_NAME);
    BOOST_CHECK_EQUAL(num_nodes, 20);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeIDs)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto node_ids = nodes.getNodeIDs(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 25);
    BOOST_CHECK_EQUAL(node_ids[10], 69);
    BOOST_CHECK_EQUAL(node_ids[19], 92);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeGroupIDs)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto node_ids = nodes.getNodeGroupIDs(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 0);
    BOOST_CHECK_EQUAL(node_ids[10], 0);
    BOOST_CHECK_EQUAL(node_ids[19], 1);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeGroupIndices)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto node_ids = nodes.getNodeGroupIndices(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 9);
    BOOST_CHECK_EQUAL(node_ids[10], 9);
    BOOST_CHECK_EQUAL(node_ids[19], 4);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeTypes)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto node_ids = nodes.getNodeTypes(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 0);
    BOOST_CHECK_EQUAL(node_ids[10], 2);
    BOOST_CHECK_EQUAL(node_ids[19], 1);
}
