#/usr/bin/env pythone

import h5py
import numpy

report = h5py.File("compartment_report.h5", mode="w")

report.attrs["version"] = [0, 1]
report.attrs["magic"] = 0x0A7A

mapping = report.create_group("mapping")
time = mapping.create_dataset("time", data=[0, 1, 0.1], dtype="f8")
time.attrs["units"] = "ms"

gids = [0, 1, 2]
offsets = [0, 4, 9]
counts = [4, 5, 4]
num_compartments = numpy.sum(counts)
section_ids = [0, 1, 1, 2, # gid 0
               0, 4, 4, 3, 2, # gid 1
               0, 1, 2, 3] # gid 2
gids = mapping.create_dataset("gids", data=gids, dtype="u8")
gids.attrs["sorted"] = True
mapping.create_dataset("index_pointer", data=offsets, dtype="u8")
mapping.create_dataset("element_id", data=section_ids, dtype="u4")

frames = 10

# The specification suggest using chunking, but it doesn't make sense for
# such a small dataset
data = report.create_dataset("data", (frames, num_compartments), dtype="f4")
data.attrs["units"] = "mV"

for f in range(frames):
    frame = numpy.zeros((num_compartments), dtype="f4")
    for gid, offset, count in zip(gids, offsets, counts):
        chunk = numpy.array([f * 100 + gid * 10] * count, dtype="f4") \
                + numpy.array(section_ids[offset:offset+count])
        frame[offset:offset + count] = chunk
    data[f,:] = frame
