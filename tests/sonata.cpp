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

#include <brion/circuitConfig.h>
#include <brion/csvConfig.h>
#include <brion/nodes.h>
#include <brion/types.h>

#include <highfive/H5Utility.hpp>

#define BOOST_TEST_MODULE Sonata
#include <boost/test/unit_test.hpp>
#include <cstdarg>

// typedef for brevity
typedef brion::Vector4f V4f;
typedef brion::Vector3f V3f;

const brion::URI TEST_SONATA_SIMPLE_NODES_URI(std::string("file://") +
                                              BRION_TESTDATA +
                                              "/sonata/simple_nodes.h5");

const brion::URI TEST_SONATA_SIMPLE_NETWORK_URI(std::string("file://") +
                                                BRION_TESTDATA +
                                                "/sonata/simple_network.json");

const brion::URI TEST_SONATA_NODE_TYPES_URI(std::string("file://") +
                                            BRION_TESTDATA +
                                            "/sonata/node_types.csv");

constexpr char POPULATION_NAME[] = "simple";

BOOST_AUTO_TEST_CASE(sonata_constructors)
{
    brion::Nodes nodes0(TEST_SONATA_SIMPLE_NODES_URI);

    HighFive::SilenceHDF5 silence;
    BOOST_CHECK_THROW(brion::Nodes(brion::URI("void")), std::exception);
}

BOOST_AUTO_TEST_CASE(sonata_getPopulationNames)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    const auto pn = nodes.getPopulationNames();
    BOOST_CHECK_EQUAL(pn.size(), 1);
    BOOST_CHECK_EQUAL(pn[0], POPULATION_NAME);
}

BOOST_AUTO_TEST_CASE(sonata_getNumberOfNodes)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    const auto num_nodes = nodes.getNumberOfNodes(POPULATION_NAME);
    BOOST_CHECK_EQUAL(num_nodes, 20);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeIDs)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    const auto node_ids = nodes.getNodeIDs(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 25);
    BOOST_CHECK_EQUAL(node_ids[10], 69);
    BOOST_CHECK_EQUAL(node_ids[19], 92);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeGroupIDs)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    const auto node_ids = nodes.getNodeGroupIDs(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 0);
    BOOST_CHECK_EQUAL(node_ids[10], 0);
    BOOST_CHECK_EQUAL(node_ids[19], 1);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeGroupIndices)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    const auto node_ids = nodes.getNodeGroupIndices(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 0);
    BOOST_CHECK_EQUAL(node_ids[10], 5);
    BOOST_CHECK_EQUAL(node_ids[19], 0);
}

BOOST_AUTO_TEST_CASE(sonata_getNodeTypes)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    const auto node_ids = nodes.getNodeTypes(POPULATION_NAME);
    BOOST_CHECK_EQUAL(node_ids.size(), 20);
    BOOST_CHECK_EQUAL(node_ids[0], 0);
    BOOST_CHECK_EQUAL(node_ids[10], 2);
    BOOST_CHECK_EQUAL(node_ids[19], 1);
}

/////////////////////////////////////////////////////////////////////////////

BOOST_AUTO_TEST_CASE(sonata_nodeGroup_getAttributeNames)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    auto group = nodes.openGroup(POPULATION_NAME, 0);
    const auto names = group.getAttributeNames();
    BOOST_CHECK_EQUAL(names.size(), 8);
    BOOST_CHECK_EQUAL(names[1], "morphology_file");
    BOOST_CHECK_EQUAL(names[2], "rotation_angle_x");
    BOOST_CHECK_EQUAL(names[3], "rotation_angle_y");
}

BOOST_AUTO_TEST_CASE(sonata_nodeGroup_getDynamicParameterNames)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    auto group = nodes.openGroup(POPULATION_NAME, 0);
    const auto names = group.getDynamicParameterNames();
    BOOST_CHECK_EQUAL(names.size(), 1);
    BOOST_CHECK_EQUAL(names[0], "my_dataset");
}

BOOST_AUTO_TEST_CASE(sonata_nodeGroup_getDynamicParameter)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    auto group = nodes.openGroup(POPULATION_NAME, 0);

    const auto my_dataset = group.getDynamicParameter<float>("my_dataset");
    BOOST_CHECK_EQUAL(my_dataset.size(), 10);
    BOOST_CHECK_EQUAL(my_dataset[0], 0.1f);
}

BOOST_AUTO_TEST_CASE(sonata_nodeGroup_getNumberOfNodes)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    auto group = nodes.openGroup(POPULATION_NAME, 0);
    const auto num_nodes = group.getNumberOfNodes();
    BOOST_CHECK_EQUAL(num_nodes, 8);
}

BOOST_AUTO_TEST_CASE(sonata_nodeGroup_getAttribute)
{
    brion::Nodes nodes(TEST_SONATA_SIMPLE_NODES_URI);
    auto group = nodes.openGroup(POPULATION_NAME, 0);

    const auto rotation_angle_x = group.getAttribute<float>("rotation_angle_x");
    const auto rotation_angle_y = group.getAttribute<float>("rotation_angle_y");
    const auto rotation_angle_z = group.getAttribute<float>("rotation_angle_z");
    const auto x = group.getAttribute<float>("x", 2, 12);
    const auto y = group.getAttribute<float>("y", 3, 12);
    const auto z = group.getAttribute<float>("z", 6, 10);

    BOOST_CHECK_EQUAL(rotation_angle_x.size(), 10);
    BOOST_CHECK_EQUAL(rotation_angle_x[0], 0.1f);

    BOOST_CHECK_EQUAL(rotation_angle_y.size(), 10);
    BOOST_CHECK_EQUAL(rotation_angle_y[0], 0.2f);

    BOOST_CHECK_EQUAL(rotation_angle_z.size(), 10);
    BOOST_CHECK_EQUAL(rotation_angle_z[0], 0.3f);

    BOOST_CHECK_EQUAL(x.size(), 10);
    BOOST_CHECK_EQUAL(x[0], 2.0f);

    BOOST_CHECK_EQUAL(y.size(), 9);
    BOOST_CHECK_EQUAL(y[0], 1.0f);

    BOOST_CHECK_EQUAL(z.size(), 4);
    BOOST_CHECK_EQUAL(z[0], -0.5f);
}

/////////////////////////////////////////////////////////////////////////////

BOOST_AUTO_TEST_CASE(circuit_config_constructors)
{
    auto config = brion::CircuitConfig(TEST_SONATA_SIMPLE_NETWORK_URI);
}

BOOST_AUTO_TEST_CASE(circuit_config_getTargetSimulator)
{
    auto config = brion::CircuitConfig(TEST_SONATA_SIMPLE_NETWORK_URI);
    BOOST_CHECK_EQUAL(config.getTargetSimulator(), "none");
}

BOOST_AUTO_TEST_CASE(circuit_config_getComponents)
{
    auto config = brion::CircuitConfig(TEST_SONATA_SIMPLE_NETWORK_URI);
    BOOST_CHECK_EQUAL(config.getComponentPath("morphologies_dir"),
                      "./morphologies");
}

BOOST_AUTO_TEST_CASE(circuit_config_getNetworkNodes)
{
    auto config = brion::CircuitConfig(TEST_SONATA_SIMPLE_NETWORK_URI);
    const auto nodes = config.getNodes();
    BOOST_CHECK_EQUAL(nodes[0].elements, "simple_nodes.h5");
    BOOST_CHECK_EQUAL(nodes[0].types, "node_types.csv");
    BOOST_CHECK_EQUAL(nodes[1].elements, "./simple_nodes.h5");
    BOOST_CHECK_EQUAL(nodes[1].types, "./node_types.csv");
}

BOOST_AUTO_TEST_CASE(circuit_config_getNetworkEdges)
{
    auto config = brion::CircuitConfig(TEST_SONATA_SIMPLE_NETWORK_URI);
    const auto edges = config.getEdges();
    BOOST_CHECK_EQUAL(edges[0].elements, "./simple_edges.h5");
    BOOST_CHECK_EQUAL(edges[0].types, "./edge_types.csv");
}

////////////////////////////////////////////////////////////////////////////

BOOST_AUTO_TEST_CASE(csv_config_constructors)
{
    auto csv = brion::CsvConfig(TEST_SONATA_NODE_TYPES_URI);
}

BOOST_AUTO_TEST_CASE(csv_config_get_property)
{
    auto csv = brion::CsvConfig(TEST_SONATA_NODE_TYPES_URI);
    BOOST_CHECK_EQUAL(csv.getProperty(1, "population"), "simple");
    BOOST_CHECK_EQUAL(csv.getProperty(0, "mtype"), "pyramidal");
    BOOST_CHECK_EQUAL(csv.getProperty(2, "etype"), "fast");
    BOOST_CHECK_EQUAL(csv.getProperty(1, "model_type"), "virtual");

    BOOST_CHECK_THROW(csv.getProperty(99, "population"), std::runtime_error);
    BOOST_CHECK_THROW(csv.getProperty(0, "void"), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(csv_config_getNodeTypeIds)
{
    auto csv = brion::CsvConfig(TEST_SONATA_NODE_TYPES_URI);
    auto nodeTypeIds = csv.getNodeTypeIds();
    std::sort(nodeTypeIds.begin(), nodeTypeIds.end());

    BOOST_CHECK_EQUAL(nodeTypeIds[0], 0);
    BOOST_CHECK_EQUAL(nodeTypeIds[1], 1);
    BOOST_CHECK_EQUAL(nodeTypeIds[2], 2);
}

BOOST_AUTO_TEST_CASE(csv_config_getProperties)
{
    auto csv = brion::CsvConfig(TEST_SONATA_NODE_TYPES_URI);
    auto properties = csv.getProperties();
    std::sort(properties.begin(), properties.end());

    BOOST_CHECK_EQUAL(properties[0], "etype");
    BOOST_CHECK_EQUAL(properties[1], "model_type");
    BOOST_CHECK_EQUAL(properties[2], "mtype");
    BOOST_CHECK_EQUAL(properties[3], "population");
}
/////////////////////////////////////////////////////////////////////////////

BOOST_AUTO_TEST_CASE(sonata_SonataConfig_constructors)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
}

BOOST_AUTO_TEST_CASE(sonata_SonataConfig_getPositions)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
    std::cout << TEST_SONATA_SIMPLE_NETWORK_URI << std::endl;
    brion::GIDSet ids = {0, 2, 3, 7};
    const auto postitions = circuit.getPositions(ids);

    BOOST_CHECK_EQUAL(postitions[0].x(), 0);
    BOOST_CHECK_EQUAL(postitions[1].y(), 0);
    BOOST_CHECK_EQUAL(postitions[2].z(), 0.5);
    BOOST_CHECK_EQUAL(postitions[3].x(), 1);
}
