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

    BOOST_CHECK_EQUAL(postitions[0], vmml::Vector3f(0, 0, 0.5));
    BOOST_CHECK_EQUAL(postitions[1], vmml::Vector3f(2, 0, 0.5));
    BOOST_CHECK_EQUAL(postitions[2], vmml::Vector3f(3, 1, 0.5));
    BOOST_CHECK_EQUAL(postitions[3], vmml::Vector3f(1, 1, -0.5));
}

BOOST_AUTO_TEST_CASE(getRotations)
{
    brain::Circuit circuit(TEST_SONATA_SIMPLE_NETWORK_URI);
    brion::GIDSet ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const auto rotations = circuit.getRotations(ids);

    const auto x_axis = vmml::Vector3f(1, 0, 0);
    const auto y_axis = vmml::Vector3f(0, 1, 0);
    const auto z_axis = vmml::Vector3f(0, 0, 1);

    // Rotate 180 degrees around X
    const auto rot0 = rotations[0].getRotationMatrix();
    // Rotate 180 degrees around Y
    const auto rot1 = rotations[1].getRotationMatrix();
    // Rotate 180 degrees around Z
    const auto rot2 = rotations[2].getRotationMatrix();

    // Rotate 90 degrees around X
    const auto rot3 = rotations[3].getRotationMatrix();
    // Rotate 90 degrees around Y
    const auto rot4 = rotations[4].getRotationMatrix();
    // Rotate 90 degrees around Z
    const auto rot5 = rotations[5].getRotationMatrix();

    // Rotate 360 degrees around X
    const auto rot6 = rotations[6].getRotationMatrix();
    // Rotate 360 degrees around Y
    const auto rot7 = rotations[7].getRotationMatrix();
    // Rotate 360 degrees around Z
    const auto rot8 = rotations[8].getRotationMatrix();

    // Rotate 90 degrees around X,Y and Z
    const auto rot9 = rotations[9].getRotationMatrix();

    BOOST_CHECK_EQUAL((rot0 * x_axis).x(), 1);
    BOOST_CHECK_EQUAL((rot1 * y_axis).y(), 1);
    BOOST_CHECK_EQUAL((rot2 * z_axis).z(), 1);

    BOOST_CHECK_EQUAL((rot0 * y_axis).y(), -1);
    BOOST_CHECK_EQUAL((rot1 * x_axis).x(), -1);
    BOOST_CHECK_EQUAL((rot2 * x_axis).x(), -1);

    BOOST_CHECK_CLOSE((rot3 * y_axis).z(), 1, 0.0001);
    BOOST_CHECK_CLOSE((rot4 * x_axis).z(), -1, 0.0001);
    BOOST_CHECK_CLOSE((rot5 * x_axis).y(), 1, 0.0001);

    BOOST_CHECK_EQUAL((rot6 * x_axis).x(), 1);
    BOOST_CHECK_EQUAL((rot7 * y_axis).y(), 1);
    BOOST_CHECK_EQUAL((rot8 * z_axis).z(), 1);

    BOOST_CHECK_CLOSE((rot9 * x_axis).z(), 1, 0.0001);
    BOOST_CHECK_CLOSE((rot9 * y_axis).y(), -1, 0.0001);
    BOOST_CHECK_CLOSE((rot9 * z_axis).x(), 1, 0.0001);
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

    const auto zero = brion::Vector3f(0, 0, 0);
    const auto ten = brion::Vector3f(10, 10, 10);

    auto morphologies =
        circuit.loadMorphologies({0, 2}, brain::Circuit::Coordinates::local);
    BOOST_CHECK_EQUAL(morphologies[0]->getSoma().getCentroid(), ten);
    BOOST_CHECK_EQUAL(morphologies[1]->getSoma().getCentroid(), zero);
    morphologies =
        circuit.loadMorphologies({2, 7}, brain::Circuit::Coordinates::local);
    BOOST_CHECK_EQUAL(morphologies[0]->getSoma().getCentroid(), zero);
    BOOST_CHECK_EQUAL(morphologies[1]->getSoma().getCentroid(), ten);

    const auto transform = [](const brain::Matrix4f& m,
                              const brain::Vector3f& v) {
        const auto t = m * brion::Vector4f(v[0], v[1], v[2], 1);
        return t.get_sub_vector<3, 0>();
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
