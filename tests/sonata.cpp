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
                                     "/sonnet/simple_nodes.h5");

BOOST_AUTO_TEST_CASE(sonata_constructors)
{
    brion::Nodes nodes0(TEST_MORPHOLOGY_URI);
    brion::Nodes nodes1(TEST_MORPHOLOGY_URI);
}

BOOST_AUTO_TEST_CASE(sonata_population_names)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto pn = nodes.getPopulationNames();
    BOOST_CHECK_EQUAL(pn.size(), 1);
    BOOST_CHECK_EQUAL(pn[0], "simple");
}

BOOST_AUTO_TEST_CASE(sonata_node_ids)
{
    brion::Nodes nodes(TEST_MORPHOLOGY_URI);
    const auto node_ids = nodes.getNodeIDs("simple");
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 25);
    BOOST_CHECK_EQUAL(node_ids[10], 69);
    BOOST_CHECK_EQUAL(node_ids[19], 92);
}
