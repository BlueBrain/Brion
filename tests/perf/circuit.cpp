#include <BBP/TestDatasets.h>
#include <brain/brain.h>
#include <lunchbox/clock.h>

#define BOOST_TEST_MODULE CircuitPerf
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(load_morphologies)
{
    const brain::Circuit circuit(
        (brion::URI("/home/eilemann/stable/Brion/BlueConfig")));
    auto gids = circuit.getGIDs();
    if (gids.size() > 50000)
    {
        brion::GIDSet subset;
        auto i = gids.begin();
        while (subset.size() < 50000)
            subset.insert(*(i++));
        subset.swap(gids);
    }

    lunchbox::Clock clock;
    {
        const auto morphologies =
            circuit.loadMorphologies(gids, brain::Circuit::Coordinates::local);
        BOOST_CHECK_EQUAL(morphologies.size(), gids.size());
    }
    std::cout << "Loaded " << gids.size() / clock.getTimef() * 1000.f
              << " morphologies/s" << std::endl;
}
