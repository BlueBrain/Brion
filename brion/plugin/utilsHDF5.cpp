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

#include <brion/detail/hdf5Mutex.h>
#include <brion/log.h>
#include <brion/types.h>

#include <highfive/H5File.hpp>
#include <highfive/H5Utility.hpp>

#include <boost/filesystem.hpp>

namespace brion
{
namespace plugin
{
HighFive::File openFile(const std::string& filepath, const int accessMode,
                        const bool lockH5)
{
    HighFive::SilenceHDF5 silence;
    std::unique_ptr<std::lock_guard<std::mutex>> lock;
    if (lockH5)
        lock.reset(new std::lock_guard<std::mutex>(detail::hdf5Mutex()));

    if (accessMode & MODE_WRITE)
    {
        namespace fs = boost::filesystem;
        const bool exists = fs::exists(fs::path(filepath));
        if (exists && (accessMode & MODE_OVERWRITE) != MODE_OVERWRITE)
            BRION_THROW("Cannot overwrite existing file " + filepath)

        const auto writeFlag =
            exists ? HighFive::File::Truncate : HighFive::File::Excl;
        return HighFive::File(filepath, HighFive::File::Create | writeFlag);
    }
    return HighFive::File(filepath, HighFive::File::ReadOnly);
}

bool isHDF5File(const URI& uri)
{
    if (!uri.getScheme().empty() && uri.getScheme() != "file")
        return false;
    const boost::filesystem::path ext =
        boost::filesystem::path(uri.getPath()).extension();
    return ext == ".h5" || ext == ".hdf5";
}
}
}
