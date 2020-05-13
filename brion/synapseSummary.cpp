/* Copyright (c) 2013-2017, EPFL/Blue Brain Project
 *                          Daniel Nachbaur <daniel.nachbaur@epfl.ch>
 *                          Juan Hernando <juan.hernando@epfl.ch>
 *
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

#include "synapseSummary.h"

#include "log.h"

#include "detail/hdf5Mutex.h"

#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>
#include <highfive/H5Utility.hpp>

#include <boost/lexical_cast.hpp>

#include <memory>

namespace brion
{
namespace detail
{
const hsize_t NUMATTRIBUTES = 3;

class SynapseSummary
{
public:
    explicit SynapseSummary(const std::string& source)
    {
        std::lock_guard<std::mutex> lock(detail::hdf5Mutex());

        try
        {
            HighFive::SilenceHDF5 silence;
            _file.reset(new HighFive::File(source, H5F_ACC_RDONLY));
        }
        catch (...)
        {
           BRION_THROW("Could not open summary HDF5 file: '" + source + "'")
        }

        try
        {
            HighFive::SilenceHDF5 silence;
            const std::string& datasetName = _file->getObjectName(0);
            const uint32_t gid =
                boost::lexical_cast<uint32_t>(datasetName.substr(1));
            if (!_loadDataset(gid))
                throw "error";
        }
        catch (...)
        {
            BRION_THROW(source + " not a valid synapse summary file")
        }
    }

    ~SynapseSummary()
    {
        std::lock_guard<std::mutex> lock(detail::hdf5Mutex());

        if (_file)
            _file.reset();
    }

    SynapseSummaryMatrix read(const uint32_t gid)
    {
        std::lock_guard<std::mutex> lock(detail::hdf5Mutex());

        if (!_loadDataset(gid))
            return SynapseSummaryMatrix();

        SynapseSummaryMatrix values(boost::extents[_dims[0]][_dims[1]]);
        _dataset->read(values.data());
        return values;
    }

private:
    std::unique_ptr<HighFive::File> _file;
    mutable std::unique_ptr<HighFive::DataSet> _dataset;
    mutable std::vector<size_t> _dims;

    bool _loadDataset(const uint32_t gid) const
    {
        std::stringstream name;
        name << "a" << gid;

        try
        {
            HighFive::SilenceHDF5 silence;
            _dataset.reset(
                new HighFive::DataSet(_file->getDataSet(name.str())));
        }
        catch (const HighFive::DataSetException&)
        {
            BRION_WARN << "Could not find synapse summary dataset for "
                     << name.str() << ": " << std::endl;
            return false;
        }

        auto dataspace = _dataset->getSpace();
        _dims = dataspace.getDimensions();

        if (_dims.size() != 2)
        {
            BRION_ERROR << "Synapse summary dataset is not 2 dimensional" << std::endl;
            return false;
        }
        if (_dims[1] != NUMATTRIBUTES)
        {
            BRION_ERROR << "Synapse summary dataset does not have " << NUMATTRIBUTES
                      << " attributes" << std::endl;
            return false;
        }
        if (_dims[0] == 0)
        {
            BRION_INFO << "No synapse summary for GID " << gid << std::endl;
            return false;
        }

        return true;
    }
};
}

SynapseSummary::SynapseSummary(const std::string& source)
    : _impl(new detail::SynapseSummary(source))
{
}

SynapseSummary::~SynapseSummary()
{
    delete _impl;
}

SynapseSummaryMatrix SynapseSummary::read(const uint32_t gid) const
{
    return _impl->read(gid);
}
}
