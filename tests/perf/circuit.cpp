#include <BBP/TestDatasets.h>
#include <brain/brain.h>

#include <chrono>
#include <iostream>

#define BOOST_TEST_MODULE CircuitPerf
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(load_morphologies)
{
    const brain::Circuit circuit((brion::URI(bbp::test::getBlueconfig())));
    auto gids = circuit.getGIDs();
    if (gids.size() > 10000)
    {
        brion::GIDSet subset;
        auto i = gids.begin();
        while (subset.size() < 10000)
            subset.insert(*(i++));
        subset.swap(gids);
    }

    std::chrono::high_resolution_clock::time_point p1 = std::chrono::high_resolution_clock::now();
    {
        const auto morphologies =
            circuit.loadMorphologies(gids, brain::Circuit::Coordinates::local);
        BOOST_CHECK_EQUAL(morphologies.size(), gids.size());
    }
    std::chrono::high_resolution_clock::time_point p2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> span =
            std::chrono::duration_cast<std::chrono::duration<float>>(p2 - p1);
    std::cout << "Loaded " << gids.size() / span.count() * 1000.f
              << " morphologies/s" << std::endl;
}
