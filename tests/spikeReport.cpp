/* Copyright (c) 2014-2017, EPFL/Blue Brain Project
 *                          Raphael Dumusc <raphael.dumusc@epfl.ch>
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
 * This file is part of Brion <https://github.com/BlueBrain/Brion>
 */

#include <BBP/TestDatasets.h>
#include <brion/brion.h>
#include <tests/paths.h>

#include <iostream>

#define BOOST_TEST_MODULE SpikeReport
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/test/unit_test.hpp>

constexpr auto BLURON_SPIKE_FILE = "spikes/spikes.dat";
constexpr auto NEST_SPIKE_FILE = "spikes/spikes-*.gdf";
constexpr auto BINARY_SPIKE_FILE = "spikes/binary.spikes";
constexpr auto HDF5_SPIKE_FILE0 = "spikes/spikes_unsorted.h5";
constexpr auto HDF5_SPIKE_FILE1 = "spikes/spikes_by_gid.h5";
constexpr auto HDF5_SPIKE_FILE2 = "spikes/spikes_by_time.h5";
constexpr auto ALL_FILES = {BLURON_SPIKE_FILE, NEST_SPIKE_FILE,
                            BINARY_SPIKE_FILE, HDF5_SPIKE_FILE0,
                            HDF5_SPIKE_FILE1,  HDF5_SPIKE_FILE2};

constexpr auto SPIKES_END_TIME = 9.955f;

inline void debugSpikes(const brion::Spikes& v)
{
    for (auto& spike : v)
    {
        std::cout << spike.first << " -- " << spike.second << std::endl;
    }
}

class TemporaryData
{
public:
    brion::Spikes spikes;
    std::string tmpFileName;

    explicit TemporaryData(const std::string& reportType)
    {
        // Try to gather CI jobs temp directory
        const auto tmpDirVar = std::getenv("TMPDIR");
        const auto tmpDir = std::string(tmpDirVar != nullptr? tmpDirVar : "/tmp");
        tmpFileName = tmpDir+ "/" + brion::make_UUID().getString() + "." +
                      reportType;

        spikes.push_back({0.1f, 20});
        spikes.push_back({0.2f, 22});
        spikes.push_back({0.2f, 23});
        spikes.push_back({0.3f, 24});
        spikes.push_back({0.4f, 25});
    }

    ~TemporaryData()
    {
        if (boost::filesystem::exists(tmpFileName))
            boost::filesystem::remove(tmpFileName);
    }
};

// uri
BOOST_AUTO_TEST_CASE(spikes_uri)
{
    TemporaryData data("dat");

    const brion::URI uri{data.tmpFileName};
    brion::SpikeReport report{uri, brion::MODE_WRITE};

    BOOST_CHECK_EQUAL(uri, report.getURI());
}

// invalid_extension
BOOST_AUTO_TEST_CASE(invalid_open_unknown_extension)
{
    BOOST_CHECK_THROW(brion::SpikeReport report0(brion::URI("./bla"),
                                                 brion::MODE_READ),
                      std::runtime_error);

    boost::filesystem::path path(BRION_TESTDATA);
    path /= "local/README";
    BOOST_CHECK_THROW(brion::SpikeReport report1(brion::URI(path.string()),
                                                 brion::MODE_READ),
                      std::runtime_error);
}

// invalid_open::file_notfound

BOOST_AUTO_TEST_CASE(invalid_open_file_notfound_binary)
{
    BOOST_CHECK_THROW(brion::SpikeReport report(brion::URI("/path/file.spikes"),
                                                brion::MODE_READ),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(invalid_open_file_notfound_bluron)
{
    BOOST_CHECK_THROW(brion::SpikeReport report(brion::URI("/path/file.dat"),
                                                brion::MODE_READ),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(invalid_open_file_notfound_nest)
{
    BOOST_CHECK_THROW(brion::SpikeReport report(brion::URI("/path/file.gdf"),
                                                brion::MODE_READ),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(invalid_open_file_notfound_h5)
{
    BOOST_CHECK_THROW(brion::SpikeReport report(brion::URI("/path/file.h5"),
                                                brion::MODE_READ),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(bluron_invalid_report_information)
{
    boost::filesystem::path path(BRION_TESTDATA);
    path /= BLURON_SPIKE_FILE;

    BOOST_CHECK_THROW(const brion::SpikeReport report(
                          brion::URI(path.string() + ";" + path.string()),
                          brion::MODE_READ),
                      std::runtime_error);
}

// invoke_invalid_method

BOOST_AUTO_TEST_CASE(invoke_invalid_method_binary)
{
    boost::filesystem::path path(BRION_TESTDATA);
    path /= BINARY_SPIKE_FILE;

    brion::SpikeReport report(brion::URI(path.string()), brion::MODE_READ);
    BOOST_CHECK_THROW(report.write(brion::Spikes{}), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(invoke_invalid_method_bluron)
{
    boost::filesystem::path path(BRION_TESTDATA);
    path /= BLURON_SPIKE_FILE;

    brion::SpikeReport report(brion::URI(path.string()), brion::MODE_READ);
    BOOST_CHECK_THROW(report.write(brion::Spikes{}), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(invoke_invalid_method_nest)
{
    boost::filesystem::path path(BRION_TESTDATA);
    brion::SpikeReport report(brion::URI((path / NEST_SPIKE_FILE).string()),
                              brion::MODE_READ);
    BOOST_CHECK_THROW(report.write(brion::Spikes{}), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(end_time)
{
    auto test = [](const char* suffix) {
        boost::filesystem::path path(BRION_TESTDATA);
        brion::SpikeReport report(brion::URI((path / suffix).string()),
                                  brion::MODE_READ);
        const auto t = report.getEndTime();
        BOOST_CHECK_MESSAGE(t == SPIKES_END_TIME, suffix << ", " << t
                                                         << " instead of "
                                                         << SPIKES_END_TIME);
    };

    for (auto&& file : ALL_FILES)
        test(file);
}

BOOST_AUTO_TEST_CASE(simple_read)
{
    auto test = [](const char* suffix) {
        boost::filesystem::path path(BRION_TESTDATA);
        brion::SpikeReport report(brion::URI((path / suffix).string()),
                                  brion::MODE_READ);

        // All the reports to be tested are file based, so they read until
        // the end.
        const auto spikes = report.read(0.3).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 500, suffix << " bad spike count "
                                                         << spikes.size());
        const auto t = report.getCurrentTime();
        BOOST_CHECK_MESSAGE(t == brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad current time " << t);
        const auto state = report.getState();
        BOOST_CHECK_MESSAGE(state == brion::SpikeReport::State::ended,
                            suffix << " bad state " << state);

        auto spike = *spikes.cbegin();
        BOOST_CHECK_MESSAGE(spike.first == 0.005f,
                            suffix << " bad first spike time " << spike.first);
        BOOST_CHECK_MESSAGE(spike.second == 162,
                            suffix << " bad first spike gid " << spike.second);

        spike = *spikes.crbegin();
        BOOST_CHECK_MESSAGE(spike.first == SPIKES_END_TIME,
                            suffix << " bad last spike time " << spike.first);
        BOOST_CHECK_MESSAGE(spike.second == 225,
                            suffix << " bad last spike gid " << spike.second);

        BOOST_CHECK_THROW(report.read(report.getCurrentTime()),
                          std::logic_error);
    };

    for (auto&& file : ALL_FILES)
        test(file);
}

BOOST_AUTO_TEST_CASE(filtered_read)
{
    auto test = [](const char* suffix) {
        boost::filesystem::path path(BRION_TESTDATA);
        brion::SpikeReport report(brion::URI((path / suffix).string()),
                                  brion::GIDSet{100, 101});

        // All the reports to be tested are file based, so they read until
        // the end.
        const auto spikes = report.read(0.3).get();

        BOOST_CHECK_MESSAGE(spikes.size() == 5, suffix << " bad spike count "
                                                       << spikes.size());
        const auto t = report.getCurrentTime();
        BOOST_CHECK_MESSAGE(t == brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad current time " << t);
        const auto state = report.getState();
        BOOST_CHECK_MESSAGE(state == brion::SpikeReport::State::ended,
                            suffix << " bad state " << state);

        auto spike = *spikes.cbegin();
        BOOST_CHECK_MESSAGE(spike.first == 2.05f,
                            suffix << " bad first spike time " << spike.first);
        BOOST_CHECK_MESSAGE(spike.second == 100,
                            suffix << " bad first spike gid " << spike.second);

        spike = *spikes.crbegin();
        BOOST_CHECK_MESSAGE(spike.first == 9.38f,
                            suffix << " bad last spike time " << spike.first);
        BOOST_CHECK_MESSAGE(spike.second == 101,
                            suffix << " bad last spike gid " << spike.second);

        BOOST_CHECK_THROW(report.read(report.getCurrentTime()),
                          std::logic_error);
    };

    for (auto&& file : ALL_FILES)
        test(file);
}

BOOST_AUTO_TEST_CASE(read_until)
{
    auto test = [](const char* suffix) {
        boost::filesystem::path path(BRION_TESTDATA);
        brion::SpikeReport report(brion::URI((path / suffix).string()),
                                  brion::MODE_READ);

        auto spikes = report.readUntil(1.0).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 49, suffix << " spike count "
                                                        << spikes.size());
        BOOST_CHECK_MESSAGE(report.getCurrentTime() >= 1.0,
                            suffix << " bad current time");
        BOOST_CHECK_MESSAGE(spikes.rbegin()->first < 1.0,
                            suffix << " bad last spike time");
        BOOST_CHECK_MESSAGE(report.getState() == brion::SpikeReport::State::ok,
                            suffix << " bad state");

        spikes = report.read(brion::UNDEFINED_TIMESTAMP).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 451, suffix << " spike count "
                                                         << spikes.size());
        BOOST_CHECK_MESSAGE(report.getCurrentTime() ==
                                brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad final current time");
        BOOST_CHECK_MESSAGE(report.getState() ==
                                brion::SpikeReport::State::ended,
                            suffix << " bad final state");
        ;
    };

    for (auto&& file : ALL_FILES)
        test(file);
}

BOOST_AUTO_TEST_CASE(read_until_filtered)
{
    auto test = [](const char* suffix) {
        boost::filesystem::path path(BRION_TESTDATA);
        brion::SpikeReport report(brion::URI((path / suffix).string()),
                                  brion::GIDSet{100, 101});

        auto spikes = report.readUntil(5.0).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 2, suffix << " spike count "
                                                       << spikes.size());
        BOOST_CHECK_MESSAGE(report.getCurrentTime() >= 5.0,
                            suffix << " bad current time");
        BOOST_CHECK_MESSAGE(spikes.rbegin()->first < 5.0,
                            suffix << " bad last spike time");
        BOOST_CHECK_MESSAGE(report.getState() == brion::SpikeReport::State::ok,
                            suffix << " bad state");

        spikes = report.read(brion::UNDEFINED_TIMESTAMP).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 3, suffix << " spike count "
                                                       << spikes.size());
        BOOST_CHECK_MESSAGE(report.getCurrentTime() ==
                                brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad final current time");
        BOOST_CHECK_MESSAGE(report.getState() ==
                                brion::SpikeReport::State::ended,
                            suffix << " bad final state");
        ;
    };

    for (auto&& file : ALL_FILES)
        test(file);
}

BOOST_AUTO_TEST_CASE(read_seek)
{
    auto test = [](const char* suffix) {
        boost::filesystem::path path(BRION_TESTDATA);
        brion::SpikeReport report(brion::URI((path / suffix).string()),
                                  brion::MODE_READ);

        report.seek(1.0f).get();
        BOOST_CHECK_MESSAGE(report.getCurrentTime() == 1.0,
                            suffix << " bad current time");
        BOOST_CHECK_MESSAGE(report.getState() == brion::SpikeReport::State::ok,
                            suffix << " bad state");
        ;

        auto spikes = report.read(brion::UNDEFINED_TIMESTAMP).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 451, suffix << " spike count "
                                                         << spikes.size());
        BOOST_CHECK_MESSAGE(report.getCurrentTime() ==
                                brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad final current time");
        BOOST_CHECK_MESSAGE(report.getState() ==
                                brion::SpikeReport::State::ended,
                            suffix << " bad final state");
        ;

        report.seek(5.0f).get();
        BOOST_CHECK_MESSAGE(report.getCurrentTime() == 5.0f,
                            suffix << " bad current time");
        BOOST_CHECK_MESSAGE(report.getState() == brion::SpikeReport::State::ok,
                            suffix << " bad state");
        ;

        spikes = report.read(brion::UNDEFINED_TIMESTAMP).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 257, suffix << " spike count "
                                                         << spikes.size());
        BOOST_CHECK_MESSAGE(report.getCurrentTime() ==
                                brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad final current time");
        BOOST_CHECK_MESSAGE(report.getState() ==
                                brion::SpikeReport::State::ended,
                            suffix << " bad final state");
        ;

        report.seek(-2.f).get();
        BOOST_CHECK_MESSAGE(report.getCurrentTime() == -2.f,
                            suffix << " bad current time");
        BOOST_CHECK_MESSAGE(report.getState() == brion::SpikeReport::State::ok,
                            suffix << " bad state");
        ;

        spikes = report.read(brion::UNDEFINED_TIMESTAMP).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 500, suffix << " spike count "
                                                         << spikes.size());
        BOOST_CHECK_MESSAGE(report.getCurrentTime() ==
                                brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad final current time");
        BOOST_CHECK_MESSAGE(report.getState() ==
                                brion::SpikeReport::State::ended,
                            suffix << " bad final state");
        ;

        report.seek(10.f).get();
        BOOST_CHECK_MESSAGE(report.getCurrentTime() ==
                                brion::UNDEFINED_TIMESTAMP,
                            suffix << " bad current time");
        BOOST_CHECK_MESSAGE(report.getState() ==
                                brion::SpikeReport::State::ended,
                            suffix << " bad state");
    };
    for (auto&& file : ALL_FILES)
        test(file);
}

BOOST_AUTO_TEST_CASE(invalid_read_binary)
{
    auto test = [](const char* suffix) {
        boost::filesystem::path path(BRION_TESTDATA);
        brion::SpikeReport report(brion::URI((path / suffix).string()),
                                  brion::MODE_READ);
        report.readUntil(0.3).get();
        BOOST_CHECK_NO_THROW(report.read(0.1));
        BOOST_CHECK_THROW(report.readUntil(0.1), std::logic_error);
    };

    for (auto&& file : ALL_FILES)
        test(file);
}

// write

inline void testWrite(const char* format)
{
    TemporaryData data{format};
    {
        brion::SpikeReport report{brion::URI(data.tmpFileName),
                                  brion::MODE_WRITE};
        report.write(data.spikes);
        report.close();
    }

    brion::SpikeReport report{brion::URI(data.tmpFileName), brion::MODE_READ};
    brion::Spikes spikes = report.read(brion::UNDEFINED_TIMESTAMP).get();
    BOOST_CHECK_EQUAL_COLLECTIONS(data.spikes.begin(), data.spikes.end(),
                                  spikes.begin(), spikes.end());

    // test writing multiple chunks
    report =
        brion::SpikeReport{brion::URI(data.tmpFileName), brion::MODE_WRITE};
    report.write(brion::Spikes{data.spikes.begin(), data.spikes.begin() + 3});
    report.write(brion::Spikes{data.spikes.begin() + 3, data.spikes.end()});

    report = brion::SpikeReport{brion::URI(data.tmpFileName), brion::MODE_READ};
    spikes = report.read(brion::UNDEFINED_TIMESTAMP).get();
    BOOST_CHECK_EQUAL_COLLECTIONS(data.spikes.begin(), data.spikes.end(),
                                  spikes.begin(), spikes.end());
}

BOOST_AUTO_TEST_CASE(write_data_binary)
{
    testWrite("spikes");
}

BOOST_AUTO_TEST_CASE(write_data_nest)
{
    testWrite("gdf");
}

BOOST_AUTO_TEST_CASE(write_data_bluron)
{
    testWrite("dat");
}

BOOST_AUTO_TEST_CASE(invalid_write)
{
    auto test = [](const char* format) {
        TemporaryData data{format};
        brion::SpikeReport report(brion::URI(data.tmpFileName),
                                  brion::MODE_WRITE);
        report.write(data.spikes);

        BOOST_CHECK_THROW(report.write({{0.0, 0}}), std::logic_error);

        BOOST_CHECK_THROW(report.write(
                              {{10.0, 0}, {10.0, 1}, {11.0, 0}, {0.5, 1}}),
                          std::logic_error);

        brion::SpikeReport reportRead{brion::URI(data.tmpFileName),
                                      brion::MODE_READ};
        BOOST_CHECK_THROW(reportRead.write({{100.0, 0}}), std::runtime_error);
    };

    for (auto&& format : {"spikes", "gdf", "dat"})
        test(format);
}

BOOST_AUTO_TEST_CASE(write_incremental)
{
    auto test = [](const char* format) {
        TemporaryData data{format};
        brion::SpikeReport reportWrite(brion::URI(data.tmpFileName),
                                       brion::MODE_WRITE);
        reportWrite.write({{0.1, 1}});
        reportWrite.write({{0.2, 1}});
        reportWrite.write({{0.3, 1}});
        reportWrite.close();

        brion::SpikeReport reportRead(brion::URI(data.tmpFileName),
                                      brion::MODE_READ);

        brion::Spikes spikes =
            reportRead.read(brion::UNDEFINED_TIMESTAMP).get();
        BOOST_CHECK_MESSAGE(spikes.size() == 3, format);
    };

    for (auto&& format : {"spikes", "gdf", "dat"})
        test(format);
}

// seek and write

inline void testSeekAndWrite(const char* format)
{
    TemporaryData data{format};
    brion::SpikeReport reportWrite(brion::URI(data.tmpFileName),
                                   brion::MODE_WRITE);

    reportWrite.write({{0.1, 1}});
    reportWrite.write({{0.2, 1}});
    reportWrite.write({{0.3, 1}});

    reportWrite.seek(0.2).get();

    reportWrite.write({{0.4, 1}});
    reportWrite.write({{0.8, 1}});

    reportWrite.close();

    brion::SpikeReport reportRead(brion::URI(data.tmpFileName),
                                  brion::MODE_READ);

    brion::Spikes spikes = reportRead.read(brion::UNDEFINED_TIMESTAMP).get();

    static const brion::Spikes expected = {{0.1, 1}, {0.4, 1}, {0.8, 1}};

    BOOST_CHECK_EQUAL_COLLECTIONS(spikes.begin(), spikes.end(),
                                  expected.begin(), expected.end());
}

BOOST_AUTO_TEST_CASE(seek_and_write_binary)
{
    testSeekAndWrite("spikes");
}
