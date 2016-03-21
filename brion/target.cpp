/* Copyright (c) 2013-2016, EPFL/Blue Brain Project
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

#include "target.h"

#include <lunchbox/log.h>
#include <lunchbox/stdExt.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <fstream>

namespace boost
{
template<>
inline brion::TargetType lexical_cast( const std::string& s )
{
    if( s == "Cell" )
        return brion::TARGET_CELL;
    if( s == "Compartment" )
        return brion::TARGET_COMPARTMENT;
    throw boost::bad_lexical_cast();
}
}

using boost::lexical_cast;

namespace brion
{
namespace detail
{
class Target
{
public:
    explicit Target( const std::string& source )
    {
        std::ifstream file( source.c_str( ));
        if( !file.is_open( ))
            LBTHROW( std::runtime_error( "Cannot open target file " + source ));
        std::stringstream buffer;
        buffer << file.rdbuf();

        boost::regex commentregx( "#.*?\\n" );
        const std::string fileString = boost::regex_replace( buffer.str(),
                                                             commentregx , "" );

        boost::regex regx( "Target (?<type>[a-zA-Z0-9_]+) (?<name>.*?)\\s+"
                           "\\{(?<contents>.*?)\\}" );
        const int subs[] = {1, 2, 3};
        boost::sregex_token_iterator i( fileString.begin(), fileString.end(),
                                        regx, subs );
        for( boost::sregex_token_iterator j; i != j; )
        {
            const std::string& typeStr = *i++;
            const std::string& name = *i++;
            std::string content = *i++;

            const TargetType type = lexical_cast< TargetType >( typeStr );
            _targetNames[type].push_back( name );
            boost::trim( content );
            boost::split( _targetValues[name], content, boost::is_any_of("\n "),
                          boost::token_compress_on );
        }

        if( _targetNames.empty( ))
            LBTHROW( std::runtime_error( source + " not a valid target file" ));
    }

    const Strings& getTargetNames( const TargetType type ) const
    {
        NameTable::const_iterator i = _targetNames.find( type );
        if( i != _targetNames.end( ))
            return i->second;
        static Strings empty;
        return empty;
    }

    const Strings& get( const std::string& name ) const
    {
        ValueTable::const_iterator i = _targetValues.find( name );
        if( i != _targetValues.end( ))
            return i->second;
        throw std::runtime_error( name + " not a valid target" );
    }

private:
    typedef stde::hash_map< uint32_t, Strings > NameTable;
    typedef stde::hash_map< std::string, Strings > ValueTable;
    NameTable _targetNames;
    ValueTable _targetValues;
};
}

Target::Target( const Target& from )
    : _impl( new detail::Target( *from._impl ))
{}

Target::Target( const std::string& source )
    : _impl( new detail::Target( source ))
{
}

Target::~Target()
{
    delete _impl;
}

Target& Target::operator = ( const Target& rhs )
{
    if( this == &rhs )
        return *this;

    delete _impl;
    _impl = new detail::Target( *rhs._impl );
    return *this;
}


const Strings& Target::getTargetNames( const TargetType type ) const
{
    return _impl->getTargetNames( type );
}

const Strings& Target::get( const std::string& name ) const
{
    return _impl->get( name );
}

namespace
{
void _parse( const Targets& targets,
             const std::string& name,
             GIDSet& gids,
             bool& found )
{
    BOOST_FOREACH( const Target& target, targets )
    {
        try
        {
            const brion::Strings& values = target.get( name );
            found = true;
            BOOST_FOREACH( const std::string& value, values )
            {
                try
                {
                    gids.insert( lexical_cast< uint32_t >( value.substr( 1 )));
                }
                catch( ... )
                {
                    if( value != name )
                        _parse( targets, value, gids, found );
                }
            }
        }
        catch( ... ) {}
    }
}
}

GIDSet Target::parse( const Targets& targets, const std::string& name )
{
    brion::GIDSet gids;
    bool found = false;
    _parse( targets, name, gids, found );
    if( !found )
        LBTHROW( std::runtime_error( name + " not a valid target" ));
    return gids;
}

std::ostream& operator << ( std::ostream& os, const Target& target )
{
    const Strings& targetNames = target.getTargetNames( brion::TARGET_CELL );
    BOOST_FOREACH( const std::string& name, targetNames )
    {
        const Strings& values = target.get( name );
        os << "Target " << name << ": ";
        BOOST_FOREACH( const std::string& value, values )
        {
            os << value << " ";
        }
        os << std::endl;
    }
    return os << std::endl;
}

}
