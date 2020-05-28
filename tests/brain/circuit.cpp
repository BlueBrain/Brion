/* Copyright (c) 2013-2018, EPFL/Blue Brain Project
 *                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
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

#include <iostream>

#define BOOST_TEST_MODULE Circuit
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/test/unit_test.hpp>

namespace
{
inline int32_t getIndexOfLastBit(uint32_t v) noexcept
{
    int32_t count = -1;
    while (v)
    {
        ++count;
        v >>= 1;
    }
    return count;
}

std::string getValue(const brion::NeuronMatrix& data, const size_t idx,
                     const uint32_t attr)
{
    return data[idx][getIndexOfLastBit(attr)];
}

inline bool EqualMatrices4(const glm::mat4& a, const glm::mat4& b,
                           const float epsilon=std::numeric_limits<float>().epsilon())
{
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            if(std::abs(a[i][j] - b[i][j]) > epsilon)
                return false;
    return true;
}
}

BOOST_AUTO_TEST_CASE(test_invalid_open)
{
    BOOST_CHECK_THROW(brain::Circuit{brain::URI{"/bla"}}, std::runtime_error);
    BOOST_CHECK_THROW(brain::Circuit{brain::URI{"bla"}}, std::runtime_error);

    boost::filesystem::path path(BBP_TESTDATA);
    path /= "local/README";
    BOOST_CHECK_THROW(brain::Circuit{brain::URI{path.string()}},
                      std::runtime_error);

    path = BBP_TESTDATA;
    path /= "local/simulations/may17_2011/Control/voltage.h5";
    BOOST_CHECK_THROW(brain::Circuit{brain::URI{path.string()}},
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(test_all_attributes)
{
    boost::filesystem::path path(BBP_TESTDATA);
    path /= "local/circuits/circuit.mvd2";

    brion::Circuit circuit(path.string());
    BOOST_CHECK_EQUAL(circuit.getNumNeurons(), 10);

    const brion::NeuronMatrix& data =
        circuit.get(brion::GIDSet(), brion::NEURON_ALL_ATTRIBUTES);

    BOOST_CHECK_EQUAL(data.shape()[0], 10); // 10 neurons
    BOOST_CHECK_EQUAL(data.shape()[1], brion::NEURON_ALL);
    BOOST_CHECK_EQUAL(getValue(data, 0, brion::NEURON_MORPHOLOGY_NAME),
                      "R-BJM141005C2_B_cor");
    BOOST_CHECK_EQUAL(getValue(data, 1, brion::NEURON_COLUMN_GID), "0");
    BOOST_CHECK_EQUAL(getValue(data, 6, brion::NEURON_MTYPE), "17");
    BOOST_CHECK_EQUAL(getValue(data, 7, brion::NEURON_POSITION_Y),
                      "399.305168");
}

BOOST_AUTO_TEST_CASE(test_some_attributes)
{
    boost::filesystem::path path(BBP_TESTDATA);
    path /= "local/circuits/circuit.mvd2";

    brion::Circuit circuit(path.string());
    BOOST_CHECK_EQUAL(circuit.getNumNeurons(), 10);

    brion::GIDSet gids;
    gids.insert(4);
    gids.insert(6);
    const brion::NeuronMatrix& data =
        circuit.get(gids, brion::NEURON_ETYPE | brion::NEURON_MORPHOLOGY_NAME);

    BOOST_CHECK_EQUAL(data.shape()[0], 2); // 2 neurons
    BOOST_CHECK_EQUAL(data.shape()[1], 2); // 2 attributes
    BOOST_CHECK_EQUAL(data[0][0], "L2PC32_2");
    BOOST_CHECK_EQUAL(data[0][1], "0");
    BOOST_CHECK_EQUAL(data[1][0], "R-C010600A2");
    BOOST_CHECK_EQUAL(data[1][1], "3");
}

BOOST_AUTO_TEST_CASE(test_types)
{
    boost::filesystem::path path(BBP_TESTDATA);
    path /= "local/circuits/18.10.10_600cell/circuit.mvd2";

    brion::Circuit circuit(path.string());
    BOOST_CHECK_EQUAL(circuit.getNumNeurons(), 600);

    const brion::Strings& mtypes = circuit.getTypes(brion::NEURONCLASS_MTYPE);
    BOOST_CHECK_EQUAL(mtypes.size(), 22);
    BOOST_CHECK_EQUAL(mtypes[0], "AHC");
    BOOST_CHECK_EQUAL(mtypes[1], "NGC");
    BOOST_CHECK_EQUAL(mtypes[2], "ADC");
    BOOST_CHECK_EQUAL(mtypes[15], "L4SP");
    BOOST_CHECK_EQUAL(mtypes[21], "L6FFPC");

    const brion::Strings& mclasses =
        circuit.getTypes(brion::NEURONCLASS_MORPHOLOGY_CLASS);
    BOOST_CHECK_EQUAL(mclasses.size(), 22);
    BOOST_CHECK_EQUAL(mclasses[0], "INT");
    BOOST_CHECK_EQUAL(mclasses[1], "INT");
    BOOST_CHECK_EQUAL(mclasses[4], "PYR");
    BOOST_CHECK_EQUAL(mclasses[21], "PYR");

    const brion::Strings& fclasses =
        circuit.getTypes(brion::NEURONCLASS_FUNCTION_CLASS);
    BOOST_CHECK_EQUAL(fclasses.size(), 22);
    BOOST_CHECK_EQUAL(fclasses[0], "INH");
    BOOST_CHECK_EQUAL(fclasses[1], "INH");
    BOOST_CHECK_EQUAL(fclasses[4], "EXC");
    BOOST_CHECK_EQUAL(fclasses[21], "EXC");

    const brion::Strings& etypes = circuit.getTypes(brion::NEURONCLASS_ETYPE);
    BOOST_CHECK_EQUAL(etypes.size(), 8);
    BOOST_CHECK_EQUAL(etypes[0], "cADint");
    BOOST_CHECK_EQUAL(etypes[1], "cFS");
    BOOST_CHECK_EQUAL(etypes[2], "dFS");
    BOOST_CHECK_EQUAL(etypes[3], "cNA");
    BOOST_CHECK_EQUAL(etypes[4], "cADpyr");
    BOOST_CHECK_EQUAL(etypes[5], "bNA");
    BOOST_CHECK_EQUAL(etypes[6], "bAD");
    BOOST_CHECK_EQUAL(etypes[7], "cST");
}

BOOST_AUTO_TEST_CASE(brain_circuit_constructor)
{
    const auto path = bbp::test::getBlueconfig();
    brain::Circuit circuit{brain::URI{path}};
    BOOST_CHECK_EQUAL(circuit.getSource().getPath(), path);
    brain::Circuit circuit2((brion::BlueConfig(path)));
    BOOST_CHECK_EQUAL(circuit2.getSource().getPath(), path);
    BOOST_CHECK_THROW(brain::Circuit{brain::URI{"pluto"}}, std::runtime_error);
}

BOOST_AUTO_TEST_CASE(brain_circuit_target)
{
    const brain::Circuit circuit{brain::URI{bbp::test::getBlueconfig()}};
    const brion::BlueConfig config{bbp::test::getBlueconfig()};

    brion::GIDSet first = circuit.getGIDs();
    brion::GIDSet second = config.parseTarget("Column");
    BOOST_CHECK_EQUAL_COLLECTIONS(first.begin(), first.end(), second.begin(),
                                  second.end());

    first = circuit.getGIDs("Column");
    second = config.parseTarget("Column");
    BOOST_CHECK_EQUAL_COLLECTIONS(first.begin(), first.end(), second.begin(),
                                  second.end());

    first = circuit.getGIDs("Layer1");
    second = config.parseTarget("Layer1");
    BOOST_CHECK_EQUAL_COLLECTIONS(first.begin(), first.end(), second.begin(),
                                  second.end());

    BOOST_CHECK_THROW(circuit.getGIDs("!ThisIsAnInvalidTarget!"),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(brain_circuit_positions)
{
    const brain::Circuit circuit{brain::URI{bbp::test::getBlueconfig()}};

    brion::GIDSet gids;
    gids.insert(1);
    gids.insert(2);
    // This call also tests brain::Circuit::getMorphologyURIs
    const brain::Vector3fs positions = circuit.getPositions(gids);
    BOOST_CHECK_EQUAL(positions.size(), gids.size());

    typedef glm::vec3 V3;
    BOOST_CHECK_SMALL(
        glm::length(positions[0] - V3(54.410675, 1427.669280, 124.882234)),
        0.000001f);
    BOOST_CHECK_SMALL(
        glm::length(positions[1] - V3(28.758332, 1393.556264, 98.258210)),
        0.000001f);
}

BOOST_AUTO_TEST_CASE(brain_empty_gids_lists)
{
    const brain::Circuit mvd2{brain::URI{bbp::test::getBlueconfig()}};
    brion::GIDSet nil;
    BOOST_CHECK(mvd2.getPositions(nil).empty());
    BOOST_CHECK(mvd2.getTransforms(nil).empty());
    BOOST_CHECK(mvd2.getRotations(nil).empty());
    BOOST_CHECK(mvd2.getMorphologyURIs(nil).empty());
    BOOST_CHECK(mvd2.getMorphologyTypes(nil).empty());
    BOOST_CHECK(mvd2.getElectrophysiologyTypes(nil).empty());

    const brain::Circuit mvd3{brain::URI{BBP_TEST_BLUECONFIG3}};
    BOOST_CHECK(mvd3.getPositions(nil).empty());
    BOOST_CHECK(mvd3.getTransforms(nil).empty());
    BOOST_CHECK(mvd3.getRotations(nil).empty());
    BOOST_CHECK(mvd3.getMorphologyURIs(nil).empty());
    BOOST_CHECK(mvd3.getMorphologyTypes(nil).empty());
    BOOST_CHECK(mvd3.getElectrophysiologyTypes(nil).empty());
}

namespace
{
void _checkMorphology(const brain::neuron::Morphology& morphology,
                      const std::string& other)
{
    const brion::Morphology reference{brain::URI{
        BBP_TESTDATA + ("/local/morphologies/01.07.08/h5/" + other)}};
    BOOST_CHECK(morphology.getPoints() == reference.getPoints());
}
void _checkMorphology(const brain::neuron::Morphology& morphology,
                      const std::string& other,
                      const glm::mat4& transform)
{
    const brain::neuron::Morphology reference{
        brain::URI{BBP_TESTDATA + ("/local/morphologies/01.07.08/h5/" + other)},
        transform};
    const auto& p = morphology.getPoints();
    const auto& q = reference.getPoints();
    BOOST_REQUIRE(reference.getTransformation() == transform);

    BOOST_REQUIRE(p.size() == q.size());
    for (size_t i = 0; i != p.size(); ++i)
        BOOST_CHECK_SMALL(glm::length(p[i] - q[i]), 0.0002f);
}
}

BOOST_AUTO_TEST_CASE(test_gid_out_of_range)
{
    typedef boost::shared_ptr<const brain::Circuit> CircuitPtr;
    std::vector<CircuitPtr> circuits;
    circuits.push_back(
        CircuitPtr(new brain::Circuit{brain::URI{BBP_TEST_BLUECONFIG}}));
    circuits.push_back(
        CircuitPtr(new brain::Circuit{brain::URI{BBP_TEST_BLUECONFIG3}}));

    brion::GIDSet gids;
    gids.insert(10000000);
    for (const CircuitPtr& circuit : circuits)
    {
        BOOST_CHECK_THROW(circuit->getPositions(gids), std::runtime_error);
        BOOST_CHECK_THROW(circuit->getMorphologyTypes(gids),
                          std::runtime_error);
        BOOST_CHECK_THROW(circuit->getElectrophysiologyTypes(gids),
                          std::runtime_error);
        BOOST_CHECK_THROW(circuit->getRotations(gids), std::runtime_error);
        BOOST_CHECK_THROW(
            circuit->loadMorphologies(gids, brain::Circuit::Coordinates::local),
            std::runtime_error);
    }
}

BOOST_AUTO_TEST_CASE(load_local_morphologies)
{
    const brain::Circuit circuit{brain::URI{bbp::test::getBlueconfig()}};

    brion::GIDSet gids;
    for (uint32_t gid = 1; gid < 500; gid += 75)
        gids.insert(gid);
    // This call also tests brain::Circuit::getMorphologyURIs
    const brain::neuron::Morphologies morphologies =
        circuit.loadMorphologies(gids, brain::Circuit::Coordinates::local);
    BOOST_CHECK_EQUAL(morphologies.size(), gids.size());

    // Checking the first morphology
    _checkMorphology(*morphologies[0], "R-C010306G.h5");

    // Checking shared morphologies
    gids.clear();
    gids.insert(2);
    gids.insert(4);
    gids.insert(6);
    const brain::neuron::Morphologies repeated =
        circuit.loadMorphologies(gids, brain::Circuit::Coordinates::local);

    BOOST_CHECK_EQUAL(repeated.size(), gids.size());
    BOOST_CHECK_EQUAL(repeated[0].get(), repeated[2].get());
    BOOST_CHECK(repeated[0].get() != repeated[1].get());
}

BOOST_AUTO_TEST_CASE(load_global_morphologies)
{
    const brain::Circuit circuit{brain::URI{bbp::test::getBlueconfig()}};

    brion::GIDSet gids;
    for (uint32_t gid = 1; gid < 10; ++gid)
        gids.insert(gid);
    const brain::neuron::Morphologies morphologies =
        circuit.loadMorphologies(gids, brain::Circuit::Coordinates::global);
    BOOST_CHECK_EQUAL(morphologies.size(), gids.size());

    // Checking the first morphology
    glm::mat4 matrix = glm::rotate(glm::mat4(1.f), 
                                   152.927388f * static_cast<float>(M_PI) / 180.0f, 
                                   glm::vec3(0.f, 1.f, 0.f));
    matrix[3] = glm::vec4(28.758332f, 1393.556264f, 98.258210f, matrix[3].w);
    _checkMorphology(*morphologies[1], "R-C270106C.h5", matrix);

    // Checking another cell with the same morphology
    glm::mat4 matrix2 = glm::rotate(glm::mat4(1.f), 
                                    76.380744f * static_cast<float>(M_PI) / 180.0f, 
                                    glm::vec3(0.f, 1.f, 0.f));
    matrix2[3] = glm::vec4(46.656769f, 1437.640777f, -11.603118f, matrix2[3].w);
    _checkMorphology(*morphologies[5], "R-C270106C.h5", matrix2);
}

BOOST_AUTO_TEST_CASE(all_mvd3)
{
    brion::BlueConfig config(BBP_TEST_BLUECONFIG3);
    brain::Circuit circuit(config);
    const size_t numNeurons = circuit.getNumNeurons();
    BOOST_CHECK_EQUAL(circuit.getGIDs().size(), numNeurons);

    brain::Vector3fs positions = circuit.getPositions(circuit.getGIDs());
    brain::Matrix4fs transforms = circuit.getTransforms(circuit.getGIDs());
    BOOST_CHECK_EQUAL(positions.size(), numNeurons);
    BOOST_CHECK_EQUAL(transforms.size(), numNeurons);

    BOOST_CHECK_SMALL(glm::length(positions[20] - glm::vec3(30.1277100000f,
                                                 1794.1259110000f,
                                                 19.8605870000f)),
                      0.000001f);
    BOOST_CHECK_SMALL(glm::length(positions[100] - glm::vec3(48.7579240000f,
                                                        1824.4589930000f,
                                                        15.3025840000f)),
                      0.000001f);

    auto m1check = glm::mat4_cast(glm::quat (0.383102f, 0.f, 0.923706f, 0.f));
    m1check[3] = glm::vec4(30.12771f, 1794.125911f, 19.860587f, 1.0);
    BOOST_CHECK(EqualMatrices4(transforms[20], m1check, 0.00001f));

    auto m2check = glm::mat4_cast(glm::quat(0.120884f, 0.f, -0.992667f, 0.f));
    m2check[3] = glm::vec4(48.757924f, 1824.458993f, 15.302584f, 1.0);
    BOOST_CHECK(EqualMatrices4(transforms[100], m2check, 0.00001f));
}

BOOST_AUTO_TEST_CASE(partial_mvd3)
{
    brion::BlueConfig config(BBP_TEST_BLUECONFIG3);
    brain::Circuit circuit(config);

    brion::GIDSet gids;
    gids.insert(6);
    gids.insert(21);
    gids.insert(101);
    gids.insert(501);

    const brain::Vector3fs& positions = circuit.getPositions(gids);
    const brain::Matrix4fs& transforms = circuit.getTransforms(gids);
    BOOST_CHECK_EQUAL(positions.size(), 4);
    BOOST_CHECK_EQUAL(transforms.size(), 4);

    BOOST_CHECK_SMALL(glm::length(positions[1] - glm::vec3(30.1277100000f,
                                                      1794.1259110000f,
                                                      19.8605870000f)),
                      0.000001f);
    BOOST_CHECK_SMALL(glm::length(positions[2] - glm::vec3(48.7579240000,
                                                      1824.4589930000,
                                                      15.3025840000)),
                      0.000001f);

    glm::quat quat1check (0.383102f, 0.f, 0.923706f, 0.f);
    glm::mat4 check1 = glm::mat4_cast(quat1check);
    check1[3] = glm::vec4(30.12771f, 1794.125911f, 19.860587f, check1[3].w);
    BOOST_CHECK(EqualMatrices4(transforms[1], check1, 0.00001f));

    glm::quat quat2check (0.120884f, 0.f, -0.992667f, 0.f);
    glm::mat4 check2 = glm::mat4_cast(quat2check);
    check2[3] = glm::vec4(48.757924, 1824.458993, 15.302584, check2[3].w);
    BOOST_CHECK(EqualMatrices4(transforms[2], check2, 0.00001f));
}

BOOST_AUTO_TEST_CASE(morphology_names_mvd3)
{
    brion::BlueConfig config(BBP_TEST_BLUECONFIG3);
    brain::Circuit circuit(config);

    brion::GIDSet gids;
    gids.insert(21);
    gids.insert(501);

    const brain::URIs& names = circuit.getMorphologyURIs(gids);
    BOOST_REQUIRE_EQUAL(names.size(), 2);
    BOOST_CHECK(boost::algorithm::ends_with(
        std::to_string(names[0]),
        "dend-C280998A-P3_axon-sm110131a1-3_INT_idA.h5"));
    BOOST_CHECK(
        boost::algorithm::ends_with(std::to_string(names[1]),
                                    "dend-ch160801B_axon-Fluo55_low.h5"));
}

BOOST_AUTO_TEST_CASE(compare_mvd2_mvd3)
{
    brion::BlueConfig config2(BBP_TEST_BLUECONFIG);
    brain::Circuit circuit2(config2);

    brion::BlueConfig config3(BBP_TEST_BLUECONFIG3);
    brain::Circuit circuit3(config3);

    brion::GIDSet gids;
    gids.insert(21);
    gids.insert(501);

    const brain::size_ts& mtypes2 = circuit2.getMorphologyTypes(gids);
    const brain::size_ts& etypes2 = circuit2.getElectrophysiologyTypes(gids);
    const brain::Strings& allMTypes2 = circuit2.getMorphologyTypeNames();
    const brain::Strings& allETypes2 = circuit2.getElectrophysiologyTypeNames();
    const brain::URIs& names2 = circuit2.getMorphologyURIs(gids);

    const brain::size_ts& mtypes3 = circuit3.getMorphologyTypes(gids);
    const brain::size_ts& etypes3 = circuit3.getElectrophysiologyTypes(gids);
    const brain::Strings& allMTypes3 = circuit3.getMorphologyTypeNames();
    const brain::Strings& allETypes3 = circuit3.getElectrophysiologyTypeNames();
    const brain::URIs& names3 = circuit3.getMorphologyURIs(gids);

    BOOST_CHECK_EQUAL_COLLECTIONS(mtypes2.begin(), mtypes2.end(),
                                  mtypes3.begin(), mtypes3.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(etypes2.begin(), etypes2.end(),
                                  etypes3.begin(), etypes3.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(allMTypes2.begin(), allMTypes2.end(),
                                  allMTypes3.begin(), allMTypes3.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(allETypes2.begin(), allETypes2.end(),
                                  allETypes3.begin(), allETypes3.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(names2.begin(), names2.end(), names3.begin(),
                                  names3.end());
}

BOOST_AUTO_TEST_CASE(brain_circuit_random_gids)
{
    const brain::Circuit circuit{brain::URI{BBP_TEST_BLUECONFIG3}};
    const brain::GIDSet& gids = circuit.getRandomGIDs(0.1f);
    BOOST_CHECK_EQUAL(gids.size(), 100);

    const brain::GIDSet& gids2 = circuit.getRandomGIDs(0.1f);
    BOOST_CHECK_EQUAL(gids2.size(), 100);

    bool notEqual = true;
    brain::GIDSet::const_iterator it1 = gids.begin();
    brain::GIDSet::const_iterator it2 = gids2.begin();
    for (; it1 != gids.end(); ++it1, ++it2)
    {
        if (*it1 != *it2)
        {
            notEqual = true;
            break;
        }
    }
    BOOST_CHECK(notEqual);

    const brain::GIDSet& gids3 = circuit.getRandomGIDs(0.5f, "Layer1");
    BOOST_CHECK_EQUAL(gids3.size(), 10);

    BOOST_CHECK_THROW(circuit.getRandomGIDs(-5.f), std::runtime_error);
    BOOST_CHECK_THROW(circuit.getRandomGIDs(1.1f), std::runtime_error);
}
