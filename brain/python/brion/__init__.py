# Copyright (c) 2020, EPFL/Blue Brain Project
#                          Nadir Roman Guerrero <nadir.romanguerrero@epfl.ch>
#
# This file is part of Brion <https://github.com/BlueBrain/Brion>
#
# This library is free software; you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License version 3.0 as published
# by the Free Software Foundation.
#
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

import sys as _sys

from ._brion import *

from . import neuron

__version__ = '3.3.0'

# Import the test helper module if present
try:
    from ._brion import _test as test
except ImportError as e:
    pass
