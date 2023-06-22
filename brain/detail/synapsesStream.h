/* Copyright (c) 2013-2016, EPFL/Blue Brain Project
 *                          Daniel.Nachbaur@epfl.ch
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

#ifndef BRAIN_DETAIL_SYNAPSESSTREAM
#define BRAIN_DETAIL_SYNAPSESSTREAM

#include <brain/circuit.h>
#include <brain/synapses.h>

#include <algorithm>
#include <future>

namespace
{
class GidSelector
{
public:
    static brion::GIDSet select(const brion::GIDSet& input, const brion::GIDSet& filter)
    {
        if (input.empty() || filter.empty())
        {
            return {};
        }

        brion::GIDSet result;
        std::set_intersection(input.begin(), input.end(), filter.begin(), filter.end(),
                              std::inserter(result, result.begin()));
        return result;
    }
};
} // namespace

namespace brain
{
namespace detail
{
struct SynapsesStream
{
    SynapsesStream(const Circuit& circuit, const GIDSet& gids, bool afferent, SynapsePrefetch prefetch)
        : _circuit(circuit)
        , _afferent(afferent)
        , _gids(gids)
        , _prefetch(prefetch)
        , _it(_gids.begin())
    {
    }

    SynapsesStream(const Circuit& circuit, const GIDSet& preGIDs, const GIDSet& postGIDs, SynapsePrefetch prefetch)
        : _circuit(circuit)
        , _afferent(preGIDs.empty() || (postGIDs.size() <= preGIDs.size()))
        , _gids(_afferent ? GidSelector::select(postGIDs, preGIDs) : GidSelector::select(preGIDs, postGIDs))
        , _prefetch(prefetch)
        , _it(_gids.begin())
    {
    }

    SynapsesStream(const Circuit& circuit, const GIDSet& gids, const std::string& source, SynapsePrefetch prefetch)
        : _circuit(circuit)
        , _afferent(true)
        , _gids(gids)
        , _externalSource(source)
        , _prefetch(prefetch)
        , _it(_gids.begin())
    {
    }

    const Circuit& _circuit;
    const bool _afferent;
    const GIDSet _gids;
    // Source name for external afferent projections
    const std::string _externalSource;
    const SynapsePrefetch _prefetch;
    GIDSet::const_iterator _it;

    size_t getRemaining() const { return size_t(std::abs(std::distance(_it, _gids.end()))); }

    std::future<Synapses> read(size_t count)
    {
        count = std::min(count, getRemaining());
        GIDSet::const_iterator start = _it;
        std::advance(_it, count);
        GIDSet::const_iterator end = _it;

        if (_externalSource.empty())
        {
            return std::async(std::launch::async,
                              [&, start, end] { return Synapses(_circuit, GIDSet(start, end), _afferent, _prefetch); });
        }
        return std::async(std::launch::async, [&, start, end]
                          { return Synapses(_circuit, GIDSet(start, end), _externalSource, _prefetch); });
    }
};
} // namespace detail
} // namespace brain

#endif
