#!/usr/bin/env python

import h5py
import numpy

def h5enum(class_):
    class_.dtype = h5py.new_enum('i',  {x: getattr(class_, x)
                                        for x in dir(class_) if x[0:2] != '__'})
    return class_

@h5enum
class Sorting:
    none = 0
    by_gid = 1
    by_time = 2

gids = numpy.arange(0,7)
times = numpy.array([0.4, 0.3, 0.3, 0.3, 0.2, 0.1, 0.0])

sorted_idx_time = numpy.array(numpy.argsort(times))
sorted_idx_gids = numpy.array(numpy.argsort(gids))

gid_by_gid = gids[sorted_idx_gids][:]
gid_by_time = gids[sorted_idx_time][:]
gid_none = gids[:]

time_by_gid = times[sorted_idx_gids][:]
time_by_time = times[sorted_idx_time][:]
time_none = times[:]

sort_names = [ "none"      , "by_gid"      , "by_time"       ]
sort_enums = [ Sorting.none, Sorting.by_gid, Sorting.by_time ]
sort_times = [ time_none   , time_by_gid   , time_by_time    ]
sort_gids  = [ gid_none    , gid_by_gid    , gid_by_time     ]

# Create random sorting
for (i,j) in [(0,3),(1,2),(4,6),(5,2)]:
    time_none[i], time_none[j] = time_none[j], time_none[i]
    gid_none[i], gid_none[j] = gid_none[j], gid_none[i]

for i in range(0,3):
    sort_by = sort_names[i]
    id = sort_enums[i]
    gids = sort_gids[i]
    times = sort_times[i]

    out = h5py.File("simple_spikes_sort_" + sort_by + ".h5", "w")

    group_spikes = out.create_group("spikes");

    timestamps_arr = numpy.array(times, dtype="double")
    gids_arr = numpy.array(gids, dtype="int64")

    dataset_timestamps = group_spikes.create_dataset("timestamps", data=timestamps_arr)
    dataset_gids = group_spikes.create_dataset("gids", data=gids_arr)

    group_spikes.attrs.create('sorting', id, dtype=Sorting.dtype)
