#!/usr/bin/env python

import h5py
import numpy
import struct

def h5enum(class_):
    class_.dtype = h5py.new_enum('i',  {x: getattr(class_, x)
                                        for x in dir(class_) if x[0:2] != '__'})
    return class_

@h5enum
class Sorting:
    none = 0
    by_gid = 1
    by_time = 2

# Generating a random array of 500 spikes
numpy.random.seed(0)
gids = numpy.random.randint(1, 300, 500, dtype="u4")
times = numpy.random.randint(1, 2000, 500) * 0.005
times = times.astype("f4")

print(min(times), max(times))
print(min(gids), max(gids))

gid_sorting = gids.argsort()
timestamp_sorting = times.argsort()

sorted_gids = gids[timestamp_sorting]
print(sorted_gids[0], sorted_gids[-1])
sorted_times = times[timestamp_sorting]

# Bluron spikes: ASCII .out file
with open("spikes.dat", "w") as out:
    out.write("/scatter\n")
    for gid, t in zip(sorted_gids, sorted_times):
            out.write("%f %d\n" % (t, gid))

# NEST spikes: ten ASCI .gdf files
for i in range(10):
    with open("spikes-%d.gdf" % i, "w") as out:
        for gid, t in zip(sorted_gids[i::10], sorted_times[i::10]):
            out.write("%d %f\n" % (gid, t))

# Binary spikes
with open("binary.spikes", "wb") as out:
    # header
    out.write(struct.pack("ii", 0xf0a, 1))
    for gid, t in zip(sorted_gids, sorted_times):
        out.write(struct.pack("fI", t, gid))

# SONATA spikes (h5)
spikes = [(gids, times),
          (gids[timestamp_sorting], times[timestamp_sorting]),
          (gids[gid_sorting], times[gid_sorting])]
names = ["unsorted", "by_time", "by_gid"]
attribute = [Sorting.none, Sorting.by_time, Sorting.by_gid]

for (gids, times), name, attribute in zip(spikes, names, attribute):
    with h5py.File("spikes_" + name + ".h5", "w") as out:
        group = out.create_group("spikes")
        group.create_dataset("timestamps", data=times)
        group.create_dataset("gids", data=gids)
        group.attrs.create('sorting', attribute, dtype=Sorting.dtype)
