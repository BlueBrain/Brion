/* Copyright (c) 2013-2017, EPFL/Blue Brain Project
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

#include "blueConfig.h"

#include "constants.h"
#include "log.h"
#include "target.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <fstream>

#include <unordered_map>

namespace fs = boost::filesystem;
namespace boost
{
template <>
inline brion::BlueConfigSection lexical_cast(const std::string& s)
{
    if (s == "Run")
        return brion::CONFIGSECTION_RUN;
    if (s == "Connection")
        return brion::CONFIGSECTION_CONNECTION;
    if (s == "Projection")
        return brion::CONFIGSECTION_PROJECTION;
    if (s == "Report")
        return brion::CONFIGSECTION_REPORT;
    if (s == "Stimulus")
        return brion::CONFIGSECTION_STIMULUS;
    if (s == "StimulusInject")
        return brion::CONFIGSECTION_STIMULUSINJECT;
    return brion::CONFIGSECTION_UNKNOWN;
}

template <>
inline std::string lexical_cast(const brion::BlueConfigSection& b)
{
    switch (b)
    {
    case brion::CONFIGSECTION_RUN:
        return "Run";
    case brion::CONFIGSECTION_CONNECTION:
        return "Connection";
    case brion::CONFIGSECTION_PROJECTION:
        return "Projection";
    case brion::CONFIGSECTION_REPORT:
        return "Report";
    case brion::CONFIGSECTION_STIMULUS:
        return "Stimulus";
    case brion::CONFIGSECTION_STIMULUSINJECT:
        return "StimulusInject";
    default:
        return "UNKNOWN";
    }
    throw boost::bad_lexical_cast();
}
} // namespace boost

inline std::string adjust_path(const std::string& blueConfigPath,
                               const std::string& currentDir,
                               const std::string& targetPath)
{
    // In the transition to SONATA, some paths include population names in their paths
    // with the format:
    //              <entry name> <path>:<population name>

    const std::string cleanPath = targetPath.find(":") != std::string::npos
                                        ? targetPath.substr(0, targetPath.find(":"))
                                        : targetPath;

    if(*cleanPath.begin() == '/' && fs::exists(cleanPath))
        return cleanPath;
    else
    {
        std::string fullPath = blueConfigPath + "/" + cleanPath;
        if(fs::exists(fullPath))
            return fullPath;

        fullPath = currentDir + "/" + cleanPath;
        if(fs::exists(fullPath))
            return fullPath;
    }

    return std::string(); // empty path
}

namespace brion
{
typedef std::unordered_map<std::string, std::string> KVStore;
typedef std::unordered_map<std::string, KVStore> ValueTable;

namespace detail
{
class BlueConfig
{
public:
    explicit BlueConfig(const std::string& source_)
        : source(source_)
    {
        sourceParentPath = source.substr(0, source.find_last_of("/"));
        std::ifstream file(source.c_str());
        if (!file.is_open())
            BRION_THROW("Cannot open BlueConfig file " + source)

        std::stringstream buffer;
        buffer << file.rdbuf();

        boost::regex commentregx("#.*?\\n");
        const std::string fileString =
            boost::regex_replace(buffer.str(), commentregx, "\n");

        boost::regex regx(
            "(?<type>[a-zA-Z0-9_-]+) (?<name>[a-zA-Z0-9_-]+?)"
            "\\s+\\{(?<contents>.*?)\\}");
        const int subs[] = {1, 2, 3};
        boost::sregex_token_iterator i(fileString.begin(), fileString.end(),
                                       regx, subs);
        for (boost::sregex_token_iterator end; i != end;)
        {
            const std::string& typeStr = *i++;
            const std::string& name = *i++;
            const std::string& content = *i++;
            if (content.empty())
            {
                BRION_WARN << "Found empty section '" << typeStr << " " << name
                         << "' in BlueConfig file " << source << std::endl;
                continue;
            }

            const BlueConfigSection type =
                boost::lexical_cast<BlueConfigSection>(typeStr);
            if (type == brion::CONFIGSECTION_UNKNOWN)
            {
                BRION_DEBUG << "Found unknown section '" << typeStr
                          << "' in BlueConfig file " << source << std::endl;
                continue;
            }

            names[type].push_back(name);

            Strings lines;
            boost::split(lines, content, boost::is_any_of("\n"),
                         boost::token_compress_on);

            for (std::string line : lines)
            {
                boost::trim(line);
                if (line.empty())
                    continue;

                const std::string::size_type pos = line.find(' ', 0);
                if (pos == std::string::npos)
                {
                    BRION_WARN << "Found invalid key-value pair '" << line
                             << "' in BlueConfig file " << source << std::endl;
                    continue;
                }

                std::string value = line.substr(pos + 1);
                boost::trim(value);
                table[type][name].insert(
                    std::make_pair(line.substr(0, pos), value));
            }
        }

        if (table[CONFIGSECTION_RUN].empty())
            BRION_THROW(source + " not a valid BlueConfig file")
    }

    std::string getRun()
    {
        const brion::Strings& runs = names[brion::CONFIGSECTION_RUN];
        return runs.empty() ? std::string() : runs.front();
    }

    const std::string& get(const BlueConfigSection section,
                           const std::string& sectionName,
                           const std::string& key) const
    {
        // This function doesn't create entries in the tables in case they
        // don't exist.
        static std::string empty;
        const ValueTable::const_iterator tableIt =
            table[section].find(sectionName);
        if (tableIt == table[section].end())
            return empty;
        const KVStore& store = tableIt->second;
        const KVStore::const_iterator kv = store.find(key);
        if (kv == store.end())
            return empty;
        return kv->second;
    }

    const std::string getCircuitTarget()
    {
        const std::string& ct = get(brion::CONFIGSECTION_RUN, getRun(),
                                BLUECONFIG_CIRCUIT_TARGET_KEY);
        // Patch for sonata circuits, where the circuit target comes as
        // population_name:circuit_target
        auto colonPos = ct.find(":");
        if(colonPos != std::string::npos)
            return ct.substr(colonPos + 1);

        return ct;
    }

    const std::string getCircuitPopulation()
    {
        const std::string& ct = get(brion::CONFIGSECTION_RUN, getRun(),
                                    BLUECONFIG_CIRCUIT_TARGET_KEY);
        // Patch for sonata circuits, where the circuit target comes as
        // population_name:circuit_target
        auto colonPos = ct.find(":");
        if(colonPos != std::string::npos)
            return ct.substr(0, colonPos);

        return std::string();
    }

    const std::string& getCurrentDir()
    {
        return get(brion::CONFIGSECTION_RUN, getRun(), BLUECONFIG_CURRENT_DIR_KEY);
    }

    const std::string getOutputRoot()
    {
        return adjust_path(sourceParentPath,
                           getCurrentDir(),
                           get(brion::CONFIGSECTION_RUN,
                               getRun(),
                               BLUECONFIG_OUTPUT_PATH_KEY));
    }

    template <typename T>
    bool get(const BlueConfigSection section, const std::string& sectionName,
             const std::string& key, T& value) const
    {
        try
        {
            value = boost::lexical_cast<T>(get(section, sectionName, key));
        }
        catch (const boost::bad_lexical_cast&)
        {
            return false;
        }
        return true;
    }

    std::string source;
    std::string sourceParentPath;
    Strings names[CONFIGSECTION_ALL];
    ValueTable table[CONFIGSECTION_ALL];
};
} // namespace detail

BlueConfig::BlueConfig(const std::string& source)
    : _impl(new detail::BlueConfig(source))
{
}

BlueConfig::~BlueConfig() = default;

const std::string& BlueConfig::getSource() const
{
    return _impl->source;
}

const Strings& BlueConfig::getSectionNames(
    const BlueConfigSection section) const
{
    return _impl->names[section];
}

const std::string& BlueConfig::get(const BlueConfigSection section,
                                   const std::string& sectionName,
                                   const std::string& key) const
{
    return _impl->get(section, sectionName, key);
}

brion::Targets BlueConfig::getTargets() const
{
    Targets targets;
    const URIs& uris = getTargetSources();
    for (const URI& uri : uris)
        targets.push_back(Target(uri.getPath()));
    return targets;
}

URI BlueConfig::getCircuitSource() const
{
    const fs::path path(
        adjust_path(_impl->sourceParentPath,
                    _impl->getCurrentDir(),
                    get(CONFIGSECTION_RUN, _impl->getRun(), BLUECONFIG_CIRCUIT_PATH_KEY)));
    std::string filename = path.string();
    if (fs::exists(path) && !fs::is_regular_file(fs::canonical(path)))
    {
        filename = fs::exists(path / CIRCUIT_FILE_MVD3)
                       ? (path / CIRCUIT_FILE_MVD3).string()
                       : (path / CIRCUIT_FILE_MVD2).string();
    }
    URI uri;
    uri.setScheme("file");
    uri.setPath(filename);
    return uri;
}

URI BlueConfig::getCellLibrarySource() const
{
    // Implementation according to
    // https://bbpteam.epfl.ch/documentation/projects/Circuit%20Documentation/latest/blueconfig.html#blueconfigsection-0

    const fs::path path(
        adjust_path(_impl->sourceParentPath,
                    _impl->getCurrentDir(),
                    get(CONFIGSECTION_RUN, _impl->getRun(), BLUECONFIG_CELLLIBRARY_PATH_KEY)));
    URI uri;
    uri.setScheme("file");
    uri.setPath(path.string());
    return uri;
}

URI BlueConfig::getSynapseSource() const
{
    URI uri;
    uri.setScheme("file");
    uri.setPath(
        adjust_path(_impl->sourceParentPath,
                    _impl->getCurrentDir(),
                    get(CONFIGSECTION_RUN, _impl->getRun(), BLUECONFIG_NRN_PATH_KEY)));
    return uri;
}

std::string BlueConfig::getSynapsePopulation() const
{
    const auto& synapseSource = get(CONFIGSECTION_RUN, _impl->getRun(), BLUECONFIG_NRN_PATH_KEY);

    // Extract population name from synapses path (nrnPath)
    auto colonPos = synapseSource.find(":");
    if(colonPos != std::string::npos)
        return synapseSource.substr(colonPos + 1);

    return std::string();

}

URI BlueConfig::getProjectionSource(const std::string& name) const
{
    std::string path =
        adjust_path(_impl->sourceParentPath,
                    _impl->getCurrentDir(),
                    get(CONFIGSECTION_PROJECTION, name, BLUECONFIG_PROJECTION_PATH_KEY));
    if (path.empty())
    {
        BRION_WARN << "Invalid or missing projection  " << name << std::endl;
        return URI();
    }
    URI uri;
    uri.setScheme("file");
    uri.setPath(path);
    return uri;
}

URI BlueConfig::getMorphologySource() const
{
    URI uri(adjust_path(_impl->sourceParentPath,
                        _impl->getCurrentDir(),
                        get(CONFIGSECTION_RUN, _impl->getRun(),
                            BLUECONFIG_MORPHOLOGY_PATH_KEY)));
    if (uri.getScheme().empty())
        uri.setScheme("file");

    const fs::path barePath(uri.getPath());
    const fs::path guessedPath = barePath / MORPHOLOGY_HDF5_FILES_SUBDIRECTORY;
    if (fs::exists(guessedPath) && fs::is_directory(guessedPath))
        uri.setPath(guessedPath.string());

    return uri;
}

URI BlueConfig::getReportSource(const std::string& report) const
{
    std::string format =
        get(CONFIGSECTION_REPORT, report, BLUECONFIG_REPORT_FORMAT_KEY);
    if (format.empty())
    {
        BRION_WARN << "Invalid or missing report: " << report << std::endl;
        return URI();
    }

    boost::algorithm::to_lower(format);

    if (format == "binary" || format == "bin")
        return URI(std::string("file://") + _impl->getOutputRoot() + "/" +
                   report + ".bbp");

    if (format == "hdf5"  || format == "sonata" || format.empty() || fs::is_directory(format))
        return URI(std::string("file://") + _impl->getOutputRoot() + "/" +
                   report + ".h5");

    throw std::runtime_error("Invalid report type for report " + report + ": " +
                             format);
}

URI BlueConfig::getSpikeSource() const
{
    std::string path =
        adjust_path(_impl->sourceParentPath,
                    _impl->getCurrentDir(),
                    get(CONFIGSECTION_RUN, _impl->getRun(), BLUECONFIG_SPIKES_PATH_KEY));
    if (path.empty() || fs::is_directory(path))
        path = _impl->getOutputRoot() + SPIKE_FILE;

    // If we dont find out.dat, try out.hdf5
    if(!fs::exists(path))
        path = _impl->getOutputRoot() + SONATA_SPIKE_FILE;

    URI uri;
    uri.setScheme("file");
    uri.setPath(path);
    return uri;
}

URI BlueConfig::getMeshSource() const
{
    URI uri(adjust_path(_impl->sourceParentPath,
                        _impl->getCurrentDir(),
                        get(CONFIGSECTION_RUN, _impl->getRun(), BLUECONFIG_MESH_PATH_KEY)));
    if (uri.getScheme().empty())
        uri.setScheme("file");
    // Meshes are actually under a subdirectory named high/TXT, but the suffic
    // won't be added in prevision for other mesh formats.
    return uri;
}

brion::URIs BlueConfig::getTargetSources() const
{
    const std::string& run = _impl->getRun();

    URIs uris;
    auto nrnPath =
        adjust_path(_impl->sourceParentPath,
                    _impl->getCurrentDir(),
                    get(brion::CONFIGSECTION_RUN, run, BLUECONFIG_NRN_PATH_KEY));
    boost::trim_right_if(nrnPath, [](auto c) { return c == '/'; });

    auto circuitPath =
        adjust_path(_impl->sourceParentPath,
                    _impl->getCurrentDir(),
                    get(brion::CONFIGSECTION_RUN, run, BLUECONFIG_CIRCUIT_PATH_KEY));
    boost::trim_right_if(circuitPath, [](auto c) { return c == '/'; });

    // fs::is_directory may wrongly return true for symlinks
    // (even if these are pointing to files)

    // Check if valid nrnPath (on sonata build circuit config files,
    // nrnPath is a symlink to highfive edge file)
    if (!nrnPath.empty() && fs::exists(nrnPath + "/" + CIRCUIT_TARGET_FILE))
    {
        URI uri;
        uri.setScheme("file");
        uri.setPath(nrnPath + "/" + CIRCUIT_TARGET_FILE);
        uris.push_back(uri);
    }
    // Otherwise try to fetch start.target from circuit path
    else if(!circuitPath.empty()
            && fs::exists(circuitPath + "/" + CIRCUIT_TARGET_FILE))
    {
        URI uri;
        uri.setScheme("file");
        uri.setPath(circuitPath + "/" + CIRCUIT_TARGET_FILE);
        uris.push_back(uri);
    }

    // Finally, fetch any user defined target file
    const std::string& targetPath =
        get(brion::CONFIGSECTION_RUN, run, BLUECONFIG_TARGET_FILE_KEY);
    if (!targetPath.empty())
    {
        URI uri;
        uri.setScheme("file");
        uri.setPath(targetPath);
        uris.push_back(uri);
    }

    return uris;
}

std::string BlueConfig::getCircuitTarget() const
{
    return _impl->getCircuitTarget();
}

std::string BlueConfig::getCircuitPopulation() const
{
    return _impl->getCircuitPopulation();
}

GIDSet BlueConfig::parseTarget(const std::string& target) const
{
    return brion::Target::parse(getTargets(), target);
}

float BlueConfig::getTimestep() const
{
    const std::string& run = _impl->getRun();
    float timestep = std::numeric_limits<float>::quiet_NaN();
    _impl->get<float>(brion::CONFIGSECTION_RUN, run, BLUECONFIG_DT_KEY,
                      timestep);
    return timestep;
}

std::ostream& operator<<(std::ostream& os, const BlueConfig& config)
{
    for (size_t i = 0; i < CONFIGSECTION_ALL; ++i)
    {
        for (const ValueTable::value_type& entry : config._impl->table[i])
        {
            os << boost::lexical_cast<std::string>(BlueConfigSection(i)) << " "
               << entry.first << std::endl;
            for (const KVStore::value_type& pair : entry.second)
            {
                os << "   " << pair.first << " " << pair.second << std::endl;
            }
            os << std::endl;
        }
    }

    return os;
}
} // namespace brion
