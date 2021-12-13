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

#include "simulationConfig.h"
#include "detail/utils.h"
#include "log.h"

#include <boost/filesystem/path.hpp>

namespace brion
{
namespace fs = boost::filesystem;

namespace
{
const std::string _defaultSpikesFileName("spikes.h5");
}

struct SimulationConfig::Impl
{
    std::string networkConfig;
    std::string nodeSets;
    fs::path outputDir;
    std::string spikesFile;
    std::map<std::string, std::string> reportFilepaths;

    Impl(const std::string& uri)
        : _resolver(uri)
    {
        const auto json = parseSonataJson(uri);

        try
        {
            networkConfig = _resolver.toAbsolute(json.at("network"));
        }
        catch (nlohmann::detail::exception& e)
        {
            // Check if this configuration is a circuit configuration as well.
            // Otherwise report an error about the missing network field.
            if (json.find("networks") == json.end())
                throw std::runtime_error(
                    "Error parsing simulation config: network not specified");
            networkConfig = uri;
        }

        if (json.find("node_sets_file") != json.end())
            nodeSets = _resolver.toAbsolute(json["node_sets_file"]);

        try
        {
            const auto& output = json.at("output");
            outputDir = fs::path(_resolver.toAbsolute(output.at("output_dir")));

            auto reference = output.find("spikes_file");
            if (reference == output.end())
                spikesFile = _resolver.toAbsolute(
                    (outputDir / _defaultSpikesFileName).string());
            else
            {
                const std::string filename = *reference;
                spikesFile =
                    _resolver.toAbsolute((outputDir / filename).string());
            }

            const auto reports = json.find("reports");
            // Can't use range-based for
            if (reports != json.end())
            {
                for (auto report = reports->begin(); report != reports->end();
                     ++report)
                {
                    // Consider only reports with module "membrane_report"
                    reference = report->find("module");
                    if (reference == report->end() ||
                        *reference != "membrane_report")
                    {
                        continue;
                    }

                    const auto& name = report.key();
                    reference = report->find("file_name");
                    if (reference != report->end())
                    {
                        const std::string filename = *reference;
                        reportFilepaths[name] = _resolver.toAbsolute(
                            (outputDir / filename).string());
                    }
                    else
                    {
                        reportFilepaths[name] = _resolver.toAbsolute(
                            (outputDir / fs::path(name + ".h5")).string());
                    }
                }
            }
        }
        catch (nlohmann::detail::exception& e)
        {
            throw std::runtime_error(
                (std::string("Error parsing simulation config: ") + e.what())
                    .c_str());
        }
    }

private:
    PathResolver _resolver;
};

SimulationConfig::SimulationConfig(const std::string& source)
    : _impl(new SimulationConfig::Impl(source))
{
}

SimulationConfig::~SimulationConfig() = default;

SimulationConfig::SimulationConfig(SimulationConfig&&) = default;
SimulationConfig& SimulationConfig::operator=(SimulationConfig&&) = default;

std::string SimulationConfig::getNetworkConfig() const
{
    return _impl->networkConfig;
}

std::string SimulationConfig::getNodeSetFilepath() const
{
    return _impl->nodeSets;
}

std::string SimulationConfig::getSpikesFilepath() const
{
    return _impl->spikesFile;
}

Strings SimulationConfig::getCompartmentReportNames() const
{
    Strings names;
    for (const auto& item : _impl->reportFilepaths)
        names.push_back(item.first);
    return names;
}

std::string SimulationConfig::getCompartmentReportFilepath(
    const std::string& name) const
{
    const auto i = _impl->reportFilepaths.find(name);
    if (i == _impl->reportFilepaths.end())
        BRION_THROW("Unknown report: " + name)
    return i->second;
}
} // namespace brion
