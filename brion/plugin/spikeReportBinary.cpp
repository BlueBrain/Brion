/* Copyright (c) 2015-2017, EPFL/Blue Brain Project
 *                          Stefan.Eilemann@epfl.ch
 *                          Mohamed-Ghaith Kaabi <mohamed.kaabi@epfl.ch>
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

#include "spikeReportBinary.h"
#include "spikeReportTypes.h"

#include <lunchbox/memoryMap.h>
#include <lunchbox/pluginRegisterer.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <fstream>

namespace brion
{
namespace plugin
{
namespace
{
lunchbox::PluginRegisterer< SpikeReportBinary > registerer;
}

using brion::Spike;

namespace fs = boost::filesystem;

class Header
{
public:
    bool isValid() const
    {
        return _magic == 0xf0a && _version == 1;
    }

private:
    uint32_t _magic = 0xf0a;
    uint32_t _version = 1;
};

void BinaryReportFile::mapForRead( const std::string& path )
{
    _map.remap( path );
    const size_t totalSize = _map.getSize();

    if( totalSize < sizeof( Header ) ||
        ( totalSize % sizeof( uint32_t ) ) != 0 )
    {
        LBTHROW( std::runtime_error( "Incompatible binary report: " + path ) );
    }

    const Header* header = _map.getAddress< Header >();
    if ( !header->isValid( ))
    {
        LBTHROW( std::runtime_error( "Invalid binary spike report header: " + path ) );
    }

    _mappedForRead = true;
}

void BinaryReportFile::mapForReadWrite( const std::string& path,
                                        const size_t spikeCount )
{
    const size_t newSize = sizeof( Header ) + sizeof( Spike ) * spikeCount;
    if( _map.getAddress && !_mappedForRead )
        _map.resize( newSize )
    else
        _map.recreate( path, newSize );

    *(_map.getAddress< Header >( )) = Header();
    _mappedForRead = false;
}

size_t BinaryReportFile::getSpikeCount() const
{
    return ( _map.getSize() - sizeof( Header ) ) / sizeof( Spike );
}

Spike* BinaryReportFile::getSpikes() const
{
    return reinterpret_cast< Spike* >( _map.getAddress< uint8_t >() +
                                       sizeof( Header ));
}

bool BinaryReportFile::isMappedForRead() const
{
    return _mappedForRead;
}

BinaryReportFile::operator bool() const
{
    return _map.getAddress() != nullptr;
}

SpikeReportBinary::SpikeReportBinary( const SpikeReportInitData& initData )
    : SpikeReportPlugin( initData )
{
    if ( _accessMode == MODE_READ )
    {
        if ( !fs::exists( getURI().getPath() ) )
        {
            LBTHROW(
                std::runtime_error( "Cannot find file:'" + getURI().getPath() + "'." ) );
        }
        _memFile.mapForRead( getURI().getPath() );
    }
    else
    {
        if ( !boost::filesystem::exists( getURI().getPath() ) )
        {
            std::fstream fs;
            fs.open( getURI().getPath(), std::ios::out );
            if ( !fs )
                LBTHROW( std::runtime_error( "Failed to create file: '" +
                                             getURI().getPath() + "'." ) );
            fs.close();
            _memFile.mapForReadWrite( getURI().getPath(), 0 ); // create an empty file
        }
    }
}

bool SpikeReportBinary::handles( const SpikeReportInitData& initData )
{
    const URI& uri = initData.getURI();
    if ( !uri.getScheme().empty() && uri.getScheme() != "file" )
        return false;

    const auto ext = boost::filesystem::path( uri.getPath() ).extension();
    return ext == brion::plugin::BINARY_REPORT_FILE_EXT;
}

std::string SpikeReportBinary::getDescription()
{
    return "Blue Brain binary spike reports:\n"
           "  [file://]/path/to/report" +
           std::string( BINARY_REPORT_FILE_EXT );
}
void SpikeReportBinary::close()
{
}

Spikes SpikeReportBinary::read( const float )
{
    // In file based reports, this function reads all remaining data.
    Spikes spikes;
    Spike* spikeArray = _memFile.getSpikes();
    const size_t nElems = _memFile.getSpikeCount();

    for ( ; _startIndex < nElems; ++_startIndex )
        pushBack( spikeArray[_startIndex], spikes );

    _currentTime = UNDEFINED_TIMESTAMP;
    _state = State::ended;
    return spikes;
}

Spikes SpikeReportBinary::readUntil( const float max )
{
    Spikes spikes;

    Spike* spikeArray = _memFile.getSpikes();
    const size_t nElems = _memFile.getSpikeCount();

    for ( ; _startIndex < nElems; ++_startIndex )
    {
        if ( spikeArray[_startIndex].first > max )
        {
            _currentTime = spikeArray[_startIndex].first;
            break;
        }
        pushBack( spikeArray[_startIndex], spikes );
    }

    if ( _startIndex == nElems )
    {
        _currentTime = UNDEFINED_TIMESTAMP;
        _state = State::ended;
    }

    return spikes;
}

void SpikeReportBinary::readSeek( const float toTimeStamp )
{
    if ( !_memFile || !_memFile.isMappedForRead() )
        _memFile.mapForRead( getURI().getPath() );

    Spike* spikeArray = _memFile.getSpikes();
    const size_t nElems = _memFile.getSpikeCount();

    Spike* position = nullptr;

    if ( toTimeStamp < _currentTime )
    {
        position = std::lower_bound(
            spikeArray, spikeArray + _startIndex, toTimeStamp,
            []( const Spike& spike, const float val ) { return spike.first < val; } );
    }
    else
    {
        position = std::lower_bound(
            spikeArray + _startIndex, spikeArray + nElems, toTimeStamp,
            []( const Spike& spike, const float val ) { return spike.first < val; } );
    }

    if ( position == ( spikeArray + nElems ) ) // end
    {
        _startIndex = nElems;
        _state = State::ended;
        _currentTime = UNDEFINED_TIMESTAMP;
    }
    else
    {
        _state = State::ok;
        _startIndex = std::distance( spikeArray, position );
        _currentTime = toTimeStamp;
    }
}

void SpikeReportBinary::writeSeek( float toTimeStamp )
{
    if ( !_memFile )
        _memFile.mapForReadWrite( getURI().getPath(),
                                  fs::file_size( getURI().getPath() ) );
    readSeek( toTimeStamp );
}

void SpikeReportBinary::write( const Spikes& spikes )
{
    size_t totalSpikeCount = _startIndex + spikes.size();

    if ( spikes.empty() )
        return;

    // create or resize the file
    if ( !_memFile || _memFile.getSpikeCount() != totalSpikeCount )
        _memFile.mapForReadWrite( getURI().getPath(), totalSpikeCount );

    Spike* spikeArray = _memFile.getSpikes();

    for ( const Spike& spike : spikes )
        spikeArray[_startIndex++] = spike;

    _currentTime = spikes.rbegin()->first + std::numeric_limits< float >::epsilon();
}
}
} // namespaces
