/* Copyright (c) 2020, EPFL/Blue Brain Project
 *                     Nadir Román Guerrero <nadir.romanguerrero@epfl.ch>
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

#include "morphologyMORPHIO.h"

#include "../detail/hdf5Mutex.h"

#include "../pluginLibrary.h"

#include <boost/filesystem.hpp>

#include <algorithm>

#include <morphio/errorMessages.h>
#include <morphio/morphology.h>
#include <morphio/section.h>
#include <morphio/soma.h>

namespace brion
{
namespace plugin
{
namespace
{
std::string GetFileExtension(const std::string& path)
{
    namespace fs = boost::filesystem;
    std::string ext = fs::extension(fs::path(path));
    std::transform(ext.begin(), ext.end(), ext.begin(),
        [](unsigned char c){ return std::tolower(c); });
    return ext;
}
class PluginRegisterer
{
public:
    PluginRegisterer()
    {
        auto& pluginManager = PluginLibrary::instance().getManager<MorphologyPlugin>();
        pluginManager.registerFactory<MorphologyMORPHIO>();
        morphio::set_maximum_warnings(0);
    }
};

PluginRegisterer registerer;
}

MorphologyMORPHIO::MorphologyMORPHIO(const MorphologyInitData& data)
 : MorphologyPlugin(data)
{
}

bool MorphologyMORPHIO::handles(const MorphologyInitData& initData)
{
    const auto ext = GetFileExtension(initData.getURI().getPath());
    return ext == ".swc" || ext == ".h5" || ext == ".asc";
}

std::string MorphologyMORPHIO::getDescription()
{
    static std::string MorphIODescr ("Reads morphologies in SWC, HDF5 and ASCII using MorphIO "
                                     "(.swc|.h5|.asc)");
    return MorphIODescr;
}

void MorphologyMORPHIO::load()
{
    // Use a lock only to read (morphio constructor) if we are loading a highfive morphology
    std::unique_ptr<morphio::Morphology> morph;
    const auto ext = GetFileExtension(_data.getURI().getPath());
    if(ext == ".h5")
    {
        std::lock_guard<std::mutex> lock(detail::hdf5Mutex());
        morph = std::make_unique<morphio::Morphology>(_data.getURI().getPath());
    }
    else
        morph = std::make_unique<morphio::Morphology>(_data.getURI().getPath());

    // HANDLE SOMA
    const auto soma = morph->soma();

    _sections.emplace_back(0, -1);
    _sectionTypes.push_back(SectionType::SECTION_SOMA);
    // Fill in points
    const auto& somaPoints = soma.points();
    const auto& somaDiameters = soma.diameters();
    for(size_t i = 0; i < somaPoints.size(); ++i)
    {
        const auto& p = somaPoints[i];
        const auto d = somaDiameters[i];
        _points.emplace_back(p[0], p[1], p[2], d);
    }

    // HANDLE REST OF THE SECTIONS
    const auto& sections = morph->sections();

    for(const auto& section : sections)
    {
        // Fill in sections
        _sections.emplace_back(_points.size(), section.isRoot()? 0 : section.parent().id() + 1);

        // Fill in section types
        SectionType brionSectionType;
        switch(section.type())
        {
        case morphio::SectionType::SECTION_AXON:
            brionSectionType = SectionType::SECTION_AXON;
            break;
        case morphio::SectionType::SECTION_SOMA:
            brionSectionType = SectionType::SECTION_SOMA;
            break;
        case morphio::SectionType::SECTION_DENDRITE:
            brionSectionType = SectionType::SECTION_DENDRITE;
            break;
        case morphio::SectionType::SECTION_APICAL_DENDRITE:
            brionSectionType = SectionType::SECTION_APICAL_DENDRITE;
            break;
        default:
            // handle MorphIO enum overload
            if(morphio::SectionType::SECTION_GLIA_PROCESS == section.type())
                brionSectionType = SectionType::SECTION_GLIA_PROCESS;
            else
                brionSectionType = SectionType::SECTION_UNDEFINED;
        }
        _sectionTypes.push_back(brionSectionType);

        // Fill in points
        const auto secPoints = section.points();
        const auto secDiameters = section.diameters();
        for(size_t i = 0; i < secPoints.size(); ++i)
        {
            const auto p = secPoints[i];
            const auto d = secDiameters[i];
            _points.emplace_back(p[0], p[1], p[2], d);
        }

        // Fill in perimeters
        const auto secPerimeters = section.perimeters();
        _perimeters.insert(_perimeters.end(), secPerimeters.begin(), secPerimeters.end());
    }
}
}
}
