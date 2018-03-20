#!/usr/bin/env python

import h5py
import numpy
import csv

def h5enum(class_):
    class_.dtype = h5py.new_enum('i',  {x: getattr(class_, x)
                                        for x in dir(class_) if x[0:2] != '__'})
    return class_

@h5enum
class ModelType:
    biophysical = 0
    virtual = 1
    single_compartment = 2
    point_process = 3

out = h5py.File("simple_nodes.h5", "w")

# Single population consisting of two groups of 10 cells with one different node
# types each one and a third common one with parameters to be overriden.
# Nodes are shuffled on purpose, as the spec allows it.
population = out.create_group("nodes/simple")

node_group_id = numpy.array([
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1], dtype="u4")
node_group_index = numpy.array([
    9, 0, 8, 1, 7, 2, 6, 3, 5, 4, 9, 0, 8, 1, 7, 2, 6, 3, 5, 4], dtype="u4")
# Node ids are optional, but in this example we will make them explicit,
# non ordered, non contiguous
node_id = numpy.array([
    25, 65, 60, 64, 69, 57, 33, 71, 28, 84, 69, 64, 25, 43, 70, 44, 93, 41,
    63, 92], dtype="u8")
# Arguably, this node attribute is also at the top level
node_type_id = numpy.array([
    0, 1, 0, 1, 2, 2, 0, 1, 0, 1, 2, 2, 0, 1, 0, 1, 2, 2, 0, 1], dtype="u4")

population.create_dataset("node_group_id", data=node_group_id)
population.create_dataset("node_group_index", data=node_group_index)
population.create_dataset("node_id", data=node_id)
population.create_dataset("node_type_id", data=node_type_id)

group = population.create_group("0")

# model_type is inherited from the node types

x = numpy.array([0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5], dtype="float")
y = numpy.array([0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1], dtype="float")
z = numpy.array([0.5, 0.5, 0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5, -0.5,],
                dtype="float")
# Not clear how this fields go
rotation_angle_x = numpy.array([0.1] * 10, dtype="float")
rotation_angle_y = numpy.array([0.2] * 10, dtype="float")
rotation_angle_z = numpy.array([0.3] * 10, dtype="float")
morphology_file = numpy.array(["morph_A", "morph_B", "morph_C", "morph_A",
                               "morph_B", "morph_C", "morph_D", "morph_B",
                               "morph_A", "morph_C"], dtype=bytes)

group.create_dataset("x", data=x)
group.create_dataset("y", data=y)
group.create_dataset("z", data=z)
group.create_dataset("rotation_angle_x", data=rotation_angle_x)
group.create_dataset("rotation_angle_y", data=rotation_angle_y)
group.create_dataset("rotation_angle_z", data=rotation_angle_z)
group.create_dataset("morphology_file", data=morphology_file)
# Not used yet
# morph_translate =
# model_template =

dynamic_params = group.create_group("dynamics_params")
my_dataset = numpy.array([0.1]*10, dtype="float")
dynamic_params.create_dataset("my_dataset", data=my_dataset)

group = population.create_group("1")
model_type = numpy.array([ModelType.virtual] * 10, dtype=ModelType.dtype)
group.create_dataset("model_type", data=model_type)

## CSV file

out = open("node_types.csv", "+w")

columns = {
    "node_type_id": [0, 1, 2],
    "population": ["simple"] * 3,
    "model_type": ["biophysical", "virtual", "biophysical"],
    # These are extra user custom column"
    "mtype": ["pyramidal", "none", "interneuron"],
    "etype": ["fast", "slow", "fast"]}

writer = csv.writer(out, delimiter=' ',
                    quotechar='|', quoting=csv.QUOTE_MINIMAL)

writer.writerow(columns.keys())
def get_row(columns, index):
    return [c[index] for c in columns.values()]
for i in range(3):
    writer.writerow(get_row(columns, i))



