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

#include "morphologyHDF5.h"

#include "../detail/lockHDF5.h"
#include "../detail/morphologyHDF5.h"
#include "../detail/silenceHDF5.h"
#include "../detail/utilsHDF5.h"

#include <brion/version.h>

#include <lunchbox/debug.h>
#include <lunchbox/pluginRegisterer.h>
#include <lunchbox/scopedMutex.h>

#include <H5Cpp.h>

namespace brion
{
namespace plugin
{
namespace
{
lunchbox::PluginRegisterer<MorphologyHDF5> registerer;
}

MorphologyHDF5::MorphologyHDF5(const MorphologyInitData& initData)
    : MorphologyPlugin(initData)
    , _stage("repaired")
{
    lunchbox::ScopedWrite mutex(detail::hdf5Lock());

    const std::string path = initData.getURI().getPath();
    try
    {
        detail::SilenceHDF5 silence;
        _file.openFile(path, H5F_ACC_RDONLY);
    }
    catch (const H5::Exception& exc)
    {
        LBTHROW(std::runtime_error("Could not open morphology file " + path +
                                   ": " + exc.getDetailMsg()));
    }

    _checkVersion(path);
    _selectRepairStage();
}

MorphologyHDF5::~MorphologyHDF5()
{
    lunchbox::ScopedWrite mutex(detail::hdf5Lock());

    if (_points.getId())
        _points.close();

    if (_sections.getId())
        _sections.close();

    if (_file.getId())
        _file.close();
}

bool MorphologyHDF5::handles(const MorphologyInitData& initData)
{
    const std::string& scheme = initData.getURI().getScheme();
    if (scheme != "file" && !scheme.empty())
        return false;

    const std::string& path = initData.getURI().getPath();
    const size_t pos = path.find_last_of(".");
    if (pos == std::string::npos)
        return false;

    return path.substr(pos) == ".h5";
}

std::string MorphologyHDF5::getDescription()
{
    return "Blue Brain hdf5 morphologies:\n"
           "  [file://]/path/to/morphology.h5";
}

Vector4fsPtr MorphologyHDF5::readPoints()
{
    lunchbox::ScopedWrite mutex(detail::hdf5Lock());

    if (_data.version == MORPHOLOGY_VERSION_H5_2)
    {
        H5::DataSet dataset;
        try
        {
            dataset = _file.openDataSet("/" + _g_root + "/" + _stage + "/" +
                                        _d_points);
        }
        catch (...)
        {
            LBERROR << "Could not open points dataset for morphology file "
                    << _file.getFileName() << " repair stage " << _stage
                    << std::endl;
            return Vector4fsPtr(new Vector4fs);
        }

        hsize_t dims[2];
        const H5::DataSpace& dspace = dataset.getSpace();
        if (dspace.getSimpleExtentNdims() != 2 ||
            dspace.getSimpleExtentDims(dims) < 0 || dims[1] != _pointColumns)
        {
            LBTHROW(std::runtime_error("Reading morphology file '" +
                                       _file.getFileName() +
                                       "': bad number of dimensions in"
                                       " 'points' dataspace"));
        }

        Vector4fsPtr data(new Vector4fs(dims[0]));
        dataset.read(data->data(), H5::PredType::NATIVE_FLOAT);
        return data;
    }

    Vector4fsPtr data(new Vector4fs(_pointsDims[0]));
    _points.read(data->data(), H5::PredType::NATIVE_FLOAT);
    return data;
}

Vector2isPtr MorphologyHDF5::readSections()
{
    lunchbox::ScopedWrite mutex(detail::hdf5Lock());

    if (_data.version == MORPHOLOGY_VERSION_H5_2)
    {
        // fixes BBPSDK-295 by restoring old BBPSDK 0.13 implementation
        const std::string stage(_stage == "unraveled" ? "raw" : _stage);
        H5::DataSet dataset;
        try
        {
            dataset = _file.openDataSet("/" + _g_root + "/" + _g_structure +
                                        "/" + stage);
        }
        catch (...)
        {
            LBERROR << "Could not open sections dataset for morphology file "
                    << _file.getFileName() << " repair stage " << stage
                    << std::endl;
            return Vector2isPtr(new Vector2is);
        }

        hsize_t dims[2];
        const H5::DataSpace& dspace = dataset.getSpace();
        if (dspace.getSimpleExtentNdims() != 2 ||
            dspace.getSimpleExtentDims(dims) < 0 ||
            dims[1] != _structureV2Columns)
        {
            LBTHROW(std::runtime_error("Reading morphology file '" +
                                       _file.getFileName() +
                                       "': bad number of dimensions in"
                                       " 'structure' dataspace"));
        }

        Vector2isPtr data(new Vector2is(dims[0]));
        dataset.read(data->data(), H5::PredType::NATIVE_INT);
        return data;
    }

    const hsize_t readCount[2] = {_sectionsDims[0], 1};
    const hsize_t readOffset[2] = {0, 1};
    H5::DataSpace dspace = _sections.getSpace();
    dspace.selectHyperslab(H5S_SELECT_XOR, readCount, readOffset);

    Vector2isPtr data(new Vector2is(_sectionsDims[0]));
    const hsize_t mdim[2] = {_sectionsDims[0], 2};
    const H5::DataSpace mspace(2, mdim);
    _sections.read(data->data(), H5::PredType::NATIVE_INT, mspace, dspace);
    return data;
}

SectionTypesPtr MorphologyHDF5::readSectionTypes()
{
    lunchbox::ScopedWrite mutex(detail::hdf5Lock());

    if (_data.version == MORPHOLOGY_VERSION_H5_2)
    {
        const H5::DataSet& dataset = _file.openDataSet(
            "/" + _g_root + "/" + _g_structure + "/" + _d_type);

        hsize_t dims[2];
        const H5::DataSpace& dspace = dataset.getSpace();
        if (dspace.getSimpleExtentNdims() != 2 ||
            dspace.getSimpleExtentDims(dims) < 0 || dims[1] != 1)
        {
            LBTHROW(std::runtime_error("Reading morphology file '" +
                                       _file.getFileName() +
                                       "': bad number of dimensions in"
                                       " 'sectiontype' dataspace"));
        }

        SectionTypesPtr data(new SectionTypes(dims[0]));
        dataset.read(data->data(), H5::PredType::NATIVE_INT);
        return data;
    }

    const hsize_t readCount[2] = {_sectionsDims[0], 1};
    const hsize_t readOffset[2] = {0, 1};
    H5::DataSpace dspace = _sections.getSpace();
    dspace.selectHyperslab(H5S_SELECT_SET, readCount, readOffset);

    SectionTypesPtr data(new SectionTypes(_sectionsDims[0]));
    const hsize_t mdim = _sectionsDims[0];
    const H5::DataSpace mspace(1, &mdim);
    _sections.read(data->data(), H5::PredType::NATIVE_INT, mspace, dspace);
    return data;
}

floatsPtr MorphologyHDF5::readPerimeters()
{
    if (_data.version != MORPHOLOGY_VERSION_H5_1_1)
        return floatsPtr(new floats());

    lunchbox::ScopedWrite mutex(detail::hdf5Lock());

    try
    {
        detail::SilenceHDF5 silence;
        H5::DataSet dataset = _file.openDataSet(_d_perimeters);

        hsize_t dims;
        const H5::DataSpace& dspace = dataset.getSpace();
        if (dspace.getSimpleExtentNdims() != 1 ||
            dspace.getSimpleExtentDims(&dims) < 0)
        {
            LBTHROW(std::runtime_error("Reading morphology file '" +
                                       _file.getFileName() +
                                       "': bad number of dimensions in"
                                       " 'perimeters' dataspace"));
        }

        floatsPtr data(new floats(dims));
        dataset.read(data->data(), H5::PredType::NATIVE_FLOAT);
        return data;
    }
    catch (...)
    {
        if (getCellFamily() == FAMILY_GLIA)
            LBTHROW(
                std::runtime_error("No empty perimeters allowed for glia "
                                   "morphology"));
        return floatsPtr(new floats());
    }
}

void MorphologyHDF5::_checkVersion(const std::string& source)
{
    if (_readV11Metadata())
        return;

    if (_readV2Metadata())
        return;

    try
    {
        _resolveV1();
        _data.version = MORPHOLOGY_VERSION_H5_1;
        return;
    }
    catch (...)
    {
        LBTHROW(
            std::runtime_error("Unknown morphology file format for "
                               "file " +
                               source));
    }
}

void MorphologyHDF5::_selectRepairStage()
{
    if (_data.version != MORPHOLOGY_VERSION_H5_2)
        return;

    Strings stages{"repaired", "unraveled", "raw"};
    for (const auto& stage : stages)
    {
        try
        {
            detail::SilenceHDF5 silence;
            _file.openDataSet("/" + _g_root + "/" + stage + "/" + _d_points);
            _stage = stage;
            break;
        }
        catch (const H5::Exception&)
        {
        }
    }
    _stage = "repaired";
}

void MorphologyHDF5::_resolveV1()
{
    _points = _file.openDataSet("/" + _d_points);
    const H5::DataSpace pointsSpace = _points.getSpace();
    if (pointsSpace.getSimpleExtentNdims() != 2 ||
        pointsSpace.getSimpleExtentDims(_pointsDims) < 0 ||
        _pointsDims[1] != _pointColumns)
    {
        LBTHROW(std::runtime_error("Opening morphology file '" +
                                   _file.getFileName() +
                                   "': bad number of dimensions in"
                                   " 'points' dataspace"));
    }

    _sections = _file.openDataSet(_d_structure);
    const H5::DataSpace sectionsSpace = _sections.getSpace();
    if (sectionsSpace.getSimpleExtentNdims() != 2 ||
        sectionsSpace.getSimpleExtentDims(_sectionsDims) < 0 ||
        _sectionsDims[1] != _structureV1Columns)
    {
        LBTHROW(std::runtime_error("Opening morphology file '" +
                                   _file.getFileName() +
                                   "': bad number of dimensions in"
                                   " 'structure' dataspace"));
    }
}

bool MorphologyHDF5::_readV11Metadata()
{
    try
    {
        detail::SilenceHDF5 silence;
        const H5::Group& metadata = _file.openGroup(_g_metadata);
        const H5::Attribute& attr = metadata.openAttribute(_a_version);

        uint32_t version[2];
        attr.read(H5::PredType::STD_U32LE, &version[0]);
        if (version[0] != 1 || version[1] != 1)
            return false;

        _data.version = MORPHOLOGY_VERSION_H5_1_1;

        const H5::Attribute& familyAttr = metadata.openAttribute(_a_family);
        H5::EnumType familyEnum = metadata.openEnumType(_e_family);

        familyAttr.read(familyEnum, &_data.family);

        _resolveV1();
        return true;
    }
    catch (const H5::Exception&)
    {
        return false;
    }
}

bool MorphologyHDF5::_readV2Metadata()
{
    try
    {
        detail::SilenceHDF5 silence;
        const H5::Group& root = _file.openGroup(_g_root);
        const H5::Attribute& attr = root.openAttribute(_a_version);

        attr.read(H5::PredType::NATIVE_INT, &_data.version);

        if (_data.version == MORPHOLOGY_VERSION_H5_2)
            return true;
    }
    catch (const H5::Exception&)
    {
    }

    try
    {
        detail::SilenceHDF5 silence;
        _file.openGroup(_g_root);
        _data.version = MORPHOLOGY_VERSION_H5_2;
        return true;
    }
    catch (const H5::Exception&)
    {
        return false;
    }
}
}
}
