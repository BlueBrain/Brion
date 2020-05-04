/* Copyright (c) 2018, EPFL/Blue Brain Project
 *                     Jonas Karlsson <jonas.karlsson@epfl.ch>
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

#define BOOST_TEST_MODULE Sonata_circuit
#include <boost/test/unit_test.hpp>

const std::string TEST_SONATA_PATH = std::string(BRION_TESTDATA) + "/sonata";
const brion::URI TEST_SONATA_SIMPLE_NETWORK_URI(std::string("file://") +
                                                BRION_TESTDATA +
                                                "/sonata/simple_network.json");

BOOST_AUTO_TEST_CASE(constructors)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
    BOOST_CHECK_THROW(brain::Circuit(brion::URI("file://nonexistentfile.json")),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(getPositions)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
    brion::GIDSet ids = {0, 2, 3, 7};
    const auto postitions = circuit.getPositions(ids);

    BOOST_CHECK_EQUAL(postitions[0], glm::vec3(0.f, 0.f, 0.5f));
    BOOST_CHECK_EQUAL(postitions[1], glm::vec3(2.f, 0.f, 0.5f));
    BOOST_CHECK_EQUAL(postitions[2], glm::vec3(3.f, 1.f, 0.5f));
    BOOST_CHECK_EQUAL(postitions[3], glm::vec3(1.f, 1.f, -0.5f));
}

BOOST_AUTO_TEST_CASE(getRotations)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
    brion::GIDSet ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const auto rotations = circuit.getRotations(ids);

    const auto x_axis = glm::vec4(1.f, 0.f, 0.f, 1.f);
    const auto y_axis = glm::vec4(0.f, 1.f, 0.f, 1.f);
    const auto z_axis = glm::vec4(0.f, 0.f, 1.f, 1.f);

    // Rotate 180 degrees around X
    const auto rot0 = glm::mat4_cast(rotations[0]);
    // Rotate 180 degrees around Y
    const auto rot1 = glm::mat4_cast(rotations[1]);
    // Rotate 180 degrees around Z
    const auto rot2 = glm::mat4_cast(rotations[2]);

    // Rotate 90 degrees around X
    const auto rot3 = glm::mat4_cast(rotations[3]);
    // Rotate 90 degrees around Y
    const auto rot4 = glm::mat4_cast(rotations[4]);
    // Rotate 90 degrees around Z
    const auto rot5 = glm::mat4_cast(rotations[5]);

    // Rotate 360 degrees around X
    const auto rot6 = glm::mat4_cast(rotations[6]);
    // Rotate 360 degrees around Y
    const auto rot7 = glm::mat4_cast(rotations[7]);
    // Rotate 360 degrees around Z
    const auto rot8 = glm::mat4_cast(rotations[8]);

    // Rotate 90 degrees around X,Y and Z
    const auto rot9 = glm::mat4_cast(rotations[9]);

    BOOST_CHECK_EQUAL((rot0 * x_axis).x, 1.f);
    BOOST_CHECK_EQUAL((rot1 * y_axis).y, 1.f);
    BOOST_CHECK_EQUAL((rot2 * z_axis).z, 1.f);

    BOOST_CHECK_EQUAL((rot0 * y_axis).y, -1.f);
    BOOST_CHECK_EQUAL((rot1 * x_axis).x, -1.f);
    BOOST_CHECK_EQUAL((rot2 * x_axis).x, -1.f);

    BOOST_CHECK_CLOSE((rot3 * y_axis).z, 1.f, 0.0001f);
    BOOST_CHECK_CLOSE((rot4 * x_axis).z, -1.f, 0.0001f);
    BOOST_CHECK_CLOSE((rot5 * x_axis).y, 1.f, 0.0001f);

    BOOST_CHECK_EQUAL((rot6 * x_axis).x, 1.f);
    BOOST_CHECK_EQUAL((rot7 * y_axis).y, 1.f);
    BOOST_CHECK_EQUAL((rot8 * z_axis).z, 1.f);

    BOOST_CHECK_CLOSE((rot9 * x_axis).z, 1.f, 0.0001f);
    BOOST_CHECK_CLOSE((rot9 * y_axis).y, -1.f, 0.0001f);
    BOOST_CHECK_CLOSE((rot9 * z_axis).x, 1.f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(numNeurons)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
    BOOST_CHECK_EQUAL(circuit.getNumNeurons(), 10);

    brion::GIDSet gids{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    BOOST_CHECK(circuit.getGIDs() == gids);
}

BOOST_AUTO_TEST_CASE(getMorphologyURIs)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
    brion::GIDSet ids = {0, 4, 9};

    const auto morphologyURIs = circuit.getMorphologyURIs(ids);
    BOOST_CHECK_EQUAL(morphologyURIs[0].getPath(),
                      TEST_SONATA_PATH + "/morphologies/morph_A.swc");
    BOOST_CHECK_EQUAL(morphologyURIs[1].getPath(),
                      TEST_SONATA_PATH + "/morphologies/morph_B.h5");
    BOOST_CHECK_EQUAL(morphologyURIs[2].getPath(),
                      TEST_SONATA_PATH + "/morphologies/morph_C.h5");
}

BOOST_AUTO_TEST_CASE(recentering)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);

    const auto zero = glm::vec3(0.f, 0.f, 0.f);
    const auto ten = glm::vec3(10.f, 10.f, 10.f);

    auto morphologies =
        circuit.loadMorphologies({0, 2}, brain::Circuit::Coordinates::local);
    BOOST_CHECK_EQUAL(morphologies[0]->getSoma().getCentroid(), ten);
    BOOST_CHECK_EQUAL(morphologies[1]->getSoma().getCentroid(), zero);
    morphologies =
        circuit.loadMorphologies({2, 7}, brain::Circuit::Coordinates::local);
    BOOST_CHECK_EQUAL(morphologies[0]->getSoma().getCentroid(), zero);
    BOOST_CHECK_EQUAL(morphologies[1]->getSoma().getCentroid(), ten);

    const auto transform = [](const glm::mat4& m,
                              const glm::vec3& v) {
        const auto t = m * glm::vec4(v.x, v.y, v.z, 1.f);
        return glm::vec3(t);
    };
    auto transforms = circuit.getTransforms({0, 2});
    morphologies =
        circuit.loadMorphologies({0, 2}, brain::Circuit::Coordinates::global);
    BOOST_CHECK_EQUAL(morphologies[0]->getSoma().getCentroid(),
                      transform(transforms[0], ten));
    BOOST_CHECK_EQUAL(morphologies[1]->getSoma().getCentroid(),
                      transform(transforms[1], zero));

    transforms = circuit.getTransforms({2, 7});
    morphologies =
        circuit.loadMorphologies({2, 7}, brain::Circuit::Coordinates::global);
    BOOST_CHECK_EQUAL(morphologies[0]->getSoma().getCentroid(),
                      transform(transforms[0], zero));
    BOOST_CHECK_EQUAL(morphologies[1]->getSoma().getCentroid(),
                      transform(transforms[1], ten));
}
