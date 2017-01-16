/* Copyright (c) 2006-2016, Ahmet Bilgili <ahmet.bilgili@epfl.ch>
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

#include <boost/python.hpp>

#include <brain/types.h>
#include <brain/spikeReportReader.h>

namespace bp = boost::python;

namespace brain
{

namespace
{
SpikeReportReaderPtr _initURI( const std::string& uri )
{
    return SpikeReportReaderPtr( new SpikeReportReader( brion::URI( uri )));
}
}

void export_SpikeReportReader()
{

bp::class_< SpikeReportReader, boost::noncopyable >(
    "SpikeReportReader", bp::no_init )
    .def( "__init__", bp::make_constructor( _initURI ))
    .def( "close", &SpikeReportReader::close )        
    .def( "getSpikes",
          ( brion::Spikes (SpikeReportReader::* )( float, float ))
          &SpikeReportReader::getSpikes )
    .def( "hasEnded", &SpikeReportReader::hasEnded );
}

}
