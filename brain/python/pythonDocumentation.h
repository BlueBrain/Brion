/* Copyright (c) 2020, EPFL/Blue Brain Project
 *                          Nadir Román <nadir.romanguerrero@epfl.ch>
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

#ifndef BRAIN_PYTHONDOCUMENTATION_H
#define BRAIN_PYTHONDOCUMENTATION_H

#define CIRCUIT_CLASS_DOXY \
"\nRead access to a circuit database\n\nThis class provides convenience functions to access " \
"information about the cells inside the circuit and their morphologies. \n" \

#define CIRCUIT_CONSTRUCTOR_DOXY \
"\nOpens a circuit for read access.\n\n**Parameters**\n\n     ``source`` -       the URI to " \
"the CircuitConfig or BlueConfig file. \n\n" \

#define CIRCUIT_GETSOURCE_DOXY \
"\n**Return**\n    the URI from which this circuit has been opened. \n\n" \

#define CIRCUIT_GETGIDS_DOXY "\n**Return**\n    The arrayof all GIDs held by the circuit. \n\n"

#define CIRCUIT_GETGIDS_TARGET_DOXY \
"\n**Return**\n    The arrayof GIDs for the given target name. \n\n**Exceptions**\n\n     " \
"``std.runtime_error`` -       if the target cannot be found. \n\n" \

#define CIRCUIT_GETRANDOMGIDS_TARGET_DOXY \
"\n**Return**\n    A random fraction of GIDs from the given target name. Environmental variables:" \
" BRAIN_CIRCUIT_SEED set the seed for deterministic randomness \n\n**Exceptions**\n\n     " \
"``std.runtime_error`` -       if the fraction is not in the range [0,1]. \n\n     " \
"``std.runtime_error`` -       if the target cannot be found. \n\n" \

#define CIRCUIT_GETRANDOMGIDS_DOXY \
"\n**Return**\n    A random fraction of GIDs from the circuit. Environmental variables: " \
"BRAIN_CIRCUIT_SEED set the seed for deterministic randomness \n\n**Exceptions**\n\n" \
"     ``std.runtime_error`` -       if the fraction is not in the range [0,1]. \n\n" \

#define CIRCUIT_GETMORPHOLOGYURIS_DOXY \
"\n**Return**\n    The set of URIs to access the morphologies of the given cells \n\n" \

#define CIRCUIT_LOADMORPHOLOGIES_DOXY \
"\n**Return**\n    The list of morpholgies for the GID set. If local coordinates are " \
"requested, morphologies that are repeated in the circuit will shared the same Morphology " \
"object in the list. If global coordinates are requested, all Morphology objects are unique. " \
"\n\n" \

#define CIRCUIT_GETPOSITIONS_DOXY \
"\n**Return**\n    The positions of the given cells in the iteration order of the input gids. \n\n" \

#define CIRCUIT_GETMORPHOLOGYTYPES_DOXY \
"\n**Return**\n    The morphology type indices of the given cells in the iteration order of the " \
"input gids. \n\n" \

#define CIRCUIT_GETMORPHOLOGYTYPENAMES_DOXY \
"\n**Return**\n    The morphology type names of the circuit, indexed by :std std-ref:`" \
"getMorphologyTypes()`. \n\n" \

#define CIRCUIT_GETELECTROPHYSIOLOGYTYPES_DOXY \
"\n**Return**\n    The electrophysiology type indices of the given cells in the iteration " \
"order of the input gids. \n\n" \

#define CIRCUIT_GETELECTROPHYSIOLOGYTYPENAMES_DOXY \
"\n**Return**\n    The electrophysiology type names of the circuit, indexed by :std std-ref:" \
"`getElectrophysiologyTypes()`. \n\n" \

#define CIRCUIT_GETRANSFORMS_DOXY \
"\n**Return**\n    A Nx4x4 numpy array with the local to world transformations of the given " \
"cells in their iteration order. \n\n" \

#define CIRCUIT_GETROTATIONS_DOXY \
"\n**Return**\n    A Nx4 numpy array with the local to world rotation of the given cells as " \
"xyzw quaternions \n\n" \

#define CIRCUIT_GETNUMNEURONS_DOXY "\n**Return**\n    The number of neurons in the circuit. \n\n"

#define CIRCUIT_GETAFFERENTSYNAPSES_DOXY \
"\nAccess all afferent synapses of the given GIDs.\n\n**Parameters**\n\n     ``gids`` -       " \
"the gids to load afferent synapses for \n\n     ``prefetch`` -       which synapse data to " \
"preload \n\n" \

#define CIRCUIT_GETEXTERNALAFFERENTSYNAPSES_DOXY \
"\nAccess all afferent synapses projected from another circuit into the given GIDs.\n\n**" \
"Parameters**\n\n     ``gids`` -       the gids of the post-synaptic cells \n\n     ``" \
"source`` -       the name of the projecting circuit. This corresponds to the label of a " \
"Projection section in the CircuitConfig. If the source doesn’t exist an exception will be " \
"thrown by the first operation that tries to access the data. \n\n     ``prefetch`` -       " \
"which synapse data to preload \n\n" \

#define CIRCUIT_GETEFFERENTSYNAPSES_DOXY \
"\nAccess all efferent synapses of the given GIDs.\n\n**Parameters**\n\n     ``gids`` -" \
"       the gids to load efferent synapses for \n\n     ``prefetch`` -       which " \
"synapse data to preload \n\n" \

#define CIRCUIT_GETPROJECTEDSYNAPSES_DOXY \
"\nAccess all synapses along the projection from the pre- to the postGIDs.\n\n**" \
"Parameters**\n\n     ``preGIDs`` -       the gids to load the efferent synapses for \n\n" \
"     ``postGIDs`` -       the gids to load the afferent synapses for \n\n     ``prefetch``" \
" -       which synapse data to preload \n\n" \

// ===============================================================================

#define COMPARTMENTREPORT_CONSTRUCTOR_DOXY \
"\nOpen a report in read mode. **Version**\n    2.0 \n\n**Parameters**\n\n     ``uri`` -" \
"       URI to compartment report \n\n**Exceptions**\n\n     ``std.runtime_error`` -" \
"       if compartment report could be opened for read or it is not valid. \n\n" \

#define COMPARTMENTREPORT_GEMETADATA_DOXY \
"\n**Return**\n    the metadata of the report\n     *start_time* (Numeric) : The start time " \
"of the report\n\n     *end_time* (Numeric) : The end time of the report\n\n     " \
"*time_step* (Numeric) : The sampling time interval of the report\n\n     *time_unit* " \
"(String) : The time unit of the report\n\n     *data_unit* (String) : The data unit of the " \
"report\n\n     *frame_count* (Numeric) : The total frame count in the report \n\n\n" \

#define COMPARTMENTREPORT_GETGIDS_DOXY "\n**Return**\n    the GIDs of the report \n\n"

#define COMPARTMENTREPORT_GETCELLCOUNT_DOXY \
"\n**Return**\n    the number of cells in the report. \n\n**Version**\n    3.0 \n\n" \

#define COMPARTMENTREPORT_CREATEVIEW_DOXY \
"\nCreate a view of a subset of neurons. An empty gid set creates a view containing all" \
" the data.\n\n**Version**\n    2.0 \n\n**Parameters**\n\n     ``gids`` -       the neurons " \
"of interest \n\n**Exceptions**\n\n     ``std.runtime_error`` -       if invalid GID set. \n\n" \

#define COMPARTMENTREPORT_CREATEVIEW_EMPTY_DOXY \
"\nCreate a view with all the neurons in the report.\n\n**Version**\n    2.0 \n\n" \

// ===============================================================================

#define COMPARTMENTREPORTMAPPINGPROXY_GETNUMCOMPARTMENTS_DOXY \
"\nGet the number of compartments for the given neuron. **Return**\n    number of " \
"compartments for the given neuron \n\n**Version**\n    2.0 \n\n**Parameters**\n\n     " \
"``index`` -       neuron index per current GID set \n\n" \

#define COMPARTMENTREPORTMAPPINGPROXY_GETINDEX_DOXY \
"\n**Return**\n    return the index of the all the neurons in the view. \n\n**Version**\n    " \
"2.0 \n\n" \

#define COMPARTMENTREPORTMAPPINGPROXY_GETOFFSETS_DOXY \
"\nGet the current mapping of each section of each neuron in each simulation frame buffer. For " \
"instance, :std std-ref:`getOffsets()`\\[1][15] retrieves the lookup index for the frame buffer " \
"for section 15 of neuron with index 1. The neuron index is derived from its positions in the " \
"sorted list of GIDs provided in the view constructor.\n\n**Return**\n    the offset for each " \
"section for each neuron \n\n**Version**\n    2.0 \n\n" \

#define COMPARTMENTREPORTMAPPINGPROXY_GETFRAMESIZE_DOXY \
"\n**Return**\n    The total number of compartments in a frame. \n\n**Version**\n    3.0 \n\n" \

#define COMPARTMENTREPORTMAPPINGPROXY_GETCOMPARTMENTCOUNTS_DOXY \
"\nGet the number of compartments for each section of each neuron in the view **Return**\n    " \
"the compartment counts for each section for each neuron \n\n**Version**\n    2.0 \n\n" \

// ===============================================================================

#define COMPARTMENTREPORTVIEW_GETGIDS_DOXY \
"\n**Return**\n    the considered GIDs \n\n**Version**\n    2.0 \n\n" \

#define COMPARTEMENTREPORTVIEW_GETMAPPING_DOXY \
"\n**Return**\n    the data mapping of the view. \n\n**Version**\n    2.0 \n\n" \

#define COMPARTMENTREPORTVIEW_LOAD_DOXY \
"\nLoad a frame at the given time stamp.\n\n**Return**\n    a frame containing the " \
"data if found at timestamp, an empty frame otherwise \n\n**Version**\n    2.0 \n\n" \
"**Parameters**\n\n     ``timestamp`` -       the time stamp of interest \n\n" \

#define COMPARTMENTREPORTVIEW_LOAD_DBLDBL_DOXY \
"\nLoad frames between start and end time stamps.\n\n**Return**\n    the frames " \
"overlapped by the given time window. The start time doesn’t need to be aligned with " \
"the report timestep and the time interval is open on the right. The result may be empty" \
" if the time window falls out of the report window. \n\n**Version**\n    2.0 \n\n" \
"**Parameters**\n\n     ``start`` -       the start time stamp \n\n     ``end`` -       " \
"the end time stamp \n\n**Exceptions**\n\n     ``std.logic_error`` -       if invalid " \
"interval \n\n" \

#define COMPARTMENTREPORTVIEW_LOAD_DBLDBLDBL_DOXY \
"\nLoad frames between start and end time stamps.\n\n**Return**\n    the frames overlapped" \
" by the given time window, spaced by a given step. The start time doesn’t need to be " \
"aligned with the step and the time interval is open on the right. The result may be empty" \
" if the time window falls out of the report window. \n\n**Version**\n    2.1 \n\n" \
"**Parameters**\n\n     ``start`` -       the start time stamp with a time step \n\n     " \
"``end`` -       the end time stamp \n\n     ``step`` -       the time step \n\n" \
"**Exceptions**\n\n     ``std.logic_error`` -       if invalid interval or step < timeStep" \
" or step is not a multiple of timeStep \n\n" \

#define COMPARTMENTREPORTVIEW_LOADALL_DOXY \
"\nLoad all the frames. This is equivalent to call load(starTime, endTime) **Version**\n    2.0" \
" \n\n" \

// ===============================================================================

#define SOMA_CLASS_DOXY \
"\nA class to represent a neuron soma.\n\nThis class provides functions to query information " \
"about the soma of a neuron.\n\nTypically the soma is described as the poly-line of the " \
"projection of the soma onto a plane, where the plane normal points in the vertical direction" \
" in the local coordinate system of the morphology. In other cases the poly-line is not projected" \
" onto a plane, but is an approximation of the countour of the soma as seen in an orhogonal " \
"projection down the vertical axis (this is basically the same as before, but the vertical " \
"coordinate is not 0 for all the points). This class can also be used for both descriptions as " \
"well as somas simply approximated as spheres.\n\nThe coordinates system used by a soma will be " \
"in the same as the brain.Morphology from where it comes.\n\n**Version**\n    unstable \n\n" \

#define SOMA_GETPROFILEPOINTS_DOXY \
"\nReturn the x,y,z and radius of the points of the soma profile as a 4xN numpy array. \n" \

#define SOMA_GETMEANRADIUS_DOXY \
"\nReturn the mean distance between the profile points and the centroid. \n" \

#define SOMA_GETMAXRADIUS_DOXY \
"\nReturn the max distance between any profile point to the centroid. \n" \

#define SOMA_GETCENTROID_DOXY "\nReturn the average of the profile points. \n"

#define SOMA_GETCHILDREN_DOXY "\nReturn the first order sections starting from the soma. \n"

// ===============================================================================

#define SECTION_CLASS_DOXY \
"\nA class to represent a morphological section.\n\nA Section is an unbranched piece of a " \
"morphological skeleton. This class provides functions to query information about the sample " \
"points that compose the section and functions to obtain the parent and children sections.\n\n" \
"The cell soma is also considered a section, but some functions have special meaning for it.\n\n" \
"Sections cannot be directly created, but are returned by several brain.Morphology and brain." \
"Section methods.\n\nThis is a lightweight object with STL container style thread safety. It is " \
"also safe to use a section after the morphology from where it comes has been deallocated. The " \
"morphological data will be kept as long as there is a Section referring to it. \n" \

#define SECTION_GETID_DOXY "\nReturn the ID of this section. \n"

#define SECTION_GETTYPE_DOXY \
"\nReturn the morphological type of this section (dendrite, axon, …). \n" \

#define SECTION_GETLENGTH_DOXY \
"\nReturn the total length of this section in microns.\n\nIf this section is a soma section the" \
" length is ill-defined and this function will return 0. \n" \

#define SECTION_GETSAMPLES_DOXY \
"\nReturn the list of all point samples that define this section.\n\nIf this sections is a soma " \
"section return the list of points of the soma profile poly-line.\n\n**Return**\n    A list of " \
"point positions with diameter. For a section consisting of n segments, this list will have n + 1 " \
"points. \n\n" \

#define SECTION_GETSAMPLES_FLT_DOXY \
"\nReturn a list of points sampling this section at discrete locations.\n\nIf the section is a " \
"soma section this function will return the soma position for all sampling positions. The soma " \
"position is assumed to be (0, 0, 0) unless the origin morphology has been transformed.\n\n" \
"**Return**\n    The section sampled at the given relative positions. \n\n**Parameters**\n\n" \
"     ``points`` -       Normalized positions of the sample points along the section. Values " \
"will be clampled to [0, 1] before sampling. \n\n" \

#define SECTION_GETNUMSAMPLES_DOXY \
"\nReturn the number of samples of this section. Provided as a more efficient alternative to " \
"getSamples.size(). \n" \

#define SECTION_GETDISTANCETOSOMA_DOXY \
"\nReturn the absolute distance from the start of the section to the soma. \n" \

#define SECTION_GETSAMPLEDISTANCESTOSOMA_DOXY \
"\nReturn the absolute distances to the soma in microns for all sample positions.\n\n" \
"**Return**\n    A list of distances. For a section consisting of n segments, this list " \
"will have n + 1 values. The section length is equal to the difference between the first " \
"and last values of the list. \n\n" \

#define SECTION_GETPARENT_DOXY \
"\nReturn the parent section of this section or None if doesn’t have any. \n" \

#define SECTION_GETCHILDREN_DOXY \
"\nReturn a vector with all the direct children of this section. The container will be empty" \
" for terminal sections. \n" \

// ===============================================================================

#define MORPHOLOGY_CLASS_DOXY \
"\nWrapper around brion.Morphology with higher level functions.\n\nThis class provides methods" \
" to facilitate some queries about morphologies in the context of circuits. Morphologies can be" \
" loaded with a transformation applied to its points, which is useful for operating in global" \
" circuit coordinates. The transformation is applied at construction so it cannot be modified" \
" or reverted.\n\nAccess to the raw data fields is still provided by getter functions.\n\n" \
"**Version**\n    unstable \n\n" \

#define MORPHOLOGY_CONSTRUCTOR_DOXY \
"\nCreate a morphology from a URI and load all the data.\n\n**Parameters**\n\n     ``source``" \
" -       URI of the morphology data source. \n\n**Exceptions**\n\n     ``runtime_error`` -" \
"       if an inconsistency is detected in the input file. \n\n" \

#define MORPHOLOGY_CTOR_MATRIX_URI_DOXY \
"\nCreate a morphology from a URI, load all the data and transform the points.\n\n**Parameters**" \
"\n\n     ``source`` -       URI of the morphology data source. \n\n     ``transform`` -       " \
"the transformation matrix to apply to the points. Radii will not be affected by this " \
"transformation. \n\n**Exceptions**\n\n     ``runtime_error`` -       if an inconsistency is " \
"detected in the input file. \n\n" \

#define MORPHOLOGY_GETPOINTS_DOXY \
"Return a 4xN numpy array with the x,y,z and radius of all the points of this morphology." \

#define MORPHOLOGY_GETSECTIONS_DOXY \
"Return a 2xN numpy array with the parent ID and first point offset of each section." \

#define MORPHOLOGY_GETSECTIONTYPES_DOXY "Return a numpy array with the section types."

#define MORPHOLOGY_GETSECTIONIDS_DOXY "\nReturn the list of ids for the given section types. \n"

#define MORPHOLOGY_GETSECTIONS_BYTYPE_DOXY \
"\nReturn the sections which have any of the given section types. No sections are returned for " \
"the type SectionType.Soma. \n" \

#define MORPHOLOGY_GETSECTIONS_BYTYPE_NC_DOXY \
"\nReturn the sections which have the given section type. If type is SectionType.Soma an empty " \
"list is returned. \n" \

#define MORPHOLOGY_GETSOMA_DOXY "\nReturn the object with the information about the neuron soma \n"

#define MORPHOLOGY_GETTRANSFORMATION_DOXY \
"\nReturn a 4x4 numpy array with the transformation that was passed to the constructor or the" \
" identity matrix is no transformation was given. \n" \

// ===============================================================================

#define SYNAPSE_CLASS_DOXY \
"\nA proxy object returned by the Synapses container to access data for a particular synapse." \
"\n\nThe lifetime of this object is stricly bound to the synapses container it comes from. \n" \

#define SYNAPSE_GETGID_DOXY \
"\n**Return**\n    the synapse GID containing GID of the post-synaptic cell and the index in " \
"the afferent contacts array. \n\n**Exceptions**\n\n     ``std.runtime_error`` -       if " \
"index information not found in the synapse source of the circuit. \n\n" \

#define SYNAPSE_GETPRESYNAPTICGID_DOXY "\n**Return**\n    the GID of the presynaptic neuron. \n\n"

#define SYNAPSE_GETPRESYNAPTICSECIONID_DOXY \
"\n**Return**\n    the section ID on the presynaptic neuron. \n\n" \

#define SYNAPSE_GETPRESYNAPTICSEGMENTID_DOXY \
"\n**Return**\n    the segment ID on the presynaptic neuron. \n\n" \

#define SYNAPSE_GETPRESYNAPTICDISTANCE_DOXY \
"\n**Return**\n    the distance from the beginning of the presynaptic segment to the synapse in " \
"micrometers. \n\n" \

#define SYNAPSE_GETPRESYNAPTICCENTERPOSITION_DOXY \
"\n**Return**\n    the presynaptic touch position in the center of the segment. \n\n" \

#define SYNAPSE_GETPRESYNAPTICSURFACEPOSITION_DOXY \
"\n**Return**\n    the presynaptic touch position on the surface of the segment. \n\n" \

#define SYNAPSE_GETPOSTSYNAPTICGID_DOXY \
"\n**Return**\n    the GID of the postsynaptic neuron. \n\n" \

#define SYNAPSE_GETPOSTSYNAPTICSECTIONID_DOXY \
"\n**Return**\n    the section ID on the postsynaptic neuron. \n\n" \

#define SYNAPSE_GETPOSTSYNAPTICSEGMENTID_DOXY \
"\n**Return**\n    the segment ID on the postsynaptic neuron. \n\n" \

#define SYNAPSE_GETPOSTSYNAPTICDISTANCE_DOXY \
"\n**Return**\n    the distance from the beginning of the postsynaptic segment to the synapse" \
" in micrometers. \n\n" \

#define SYNAPSE_GETPOSTSYNAPTICCENTERPOSITION_DOXY \
"\n**Return**\n    the postsynaptic touch position in the center of the segment. \n\n" \

#define SYNAPSE_GETPOSSYNAPTICSURFACEPOSITION_DOXY \
"\n**Return**\n    the postsynaptic touch position on the surface of the segment. \n\n" \

#define SYNAPSE_GETDELAY_DOXY "\n**Return**\n    the axonal delay in milliseconds. \n\n"

#define SYNAPSE_GETCONDUCTANCE_DOXY "\n**Return**\n    the conductance in nanosiemens. \n\n"

#define SYNAPSE_GETUTILIZATION_DOXY \
"\n**Return**\n    the neuro-transmitter release probability. \n\n" \

#define SYNAPSE_GETDEPRESSION_DOXY \
"\n**Return**\n    the depression time constant in milliseconds. \n\n" \

#define SYNAPSE_GETFACILITATION_DOXY \
"\n**Return**\n    the facilitation time constant in milliseconds. \n\n" \

#define SYNAPSE_GETDECAY_DOXY "\n**Return**\n    the decay time constant in milliseconds. \n\n"

#define SYNAPSE_GETEFFICACY_DOXY \
"\n**Return**\n    the absolute synaptic efficacy in millivolts. \n\n" \

// ===============================================================================

#define SYNAPSES_CLASS_DOXY \
"\nA container providing read-only access to Synapses retrieved by getXXXSynapses() functions" \
" from brain.Circuit. It provides per-object and per-array access on the various synapses " \
"attributes. Data which was not prefetched will be loaded on-demand.\n\nThis container can be" \
" iterated as well as random accessed using the operator [].\n\nThis class is thread-safe, " \
"moveable and copyable. \n" \

#define SYNAPSES_EMPTY_DOXY "\n**Return**\n    size() == 0. \n\n"

#define SYNAPSES_INDICES_DOXY \
"\n**Return**\n    the index part of the synapses GID (this is the index in each post-synaptic" \
" dataset array). \n\n**Exceptions**\n\n     ``std.runtime_error`` -       if index information" \
" not found in the synapse source of the circuit. \n\n" \

#define SYNAPSES_PREGIDS_DOXY "\n**Return**\n    the GIDs of the presynaptic neurons. \n\n"

#define SYNAPSES_PRESECTIONIDS_DOXY \
"\n**Return**\n    the section IDs on the presynaptic neurons. \n\n" \

#define SYNAPSES_PRESEGMENTIDS_DOXY \
"\n**Return**\n    the segment IDs on the presynaptic neurons. \n\n" \

#define SYNAPSES_PREDISTANCES_DOXY \
"\n**Return**\n    the distances between each synapse and the beginning of their presynaptic " \
"segments in micrometers. \n\n" \

#define SYNAPSES_PRESURFACEXPOSITIONS_DOXY \
"\n**Return**\n    the presynaptic touch position x-coordinates on the surfaces of the segments." \
" May be null in old circuits. \n\n" \

#define SYNAPSES_PRESURFACEYPOSITIONS_DOXY \
"\n**Return**\n    the presynaptic touch position y-coordinates on the surfaces of the segments." \
" May be null in old circuits. \n\n" \

#define SYNAPSES_PRESURFACEZPOSITIONS_DOXY \
"\n**Return**\n    the presynaptic touch position z-coordinates on the surfaces of the segments." \
" May be null in old circuits. \n\n" \

#define SYNAPSES_PRECENTERXPOSITIONS_DOXY \
"\n**Return**\n    the presynaptic touch position x-coordinates in the center of the segments." \
" \n\n" \

#define SYNAPSES_PRECENTERYPOSITIONS_DOXY \
"\n**Return**\n    the presynaptic touch position y-coordinates in the center of the segments." \
" \n\n" \

#define SYNAPSES_PRECENTERZPOSITIONS_DOXY \
"\n**Return**\n    the presynaptic touch position z-coordinates in the center of the segments." \
" \n\n" \

#define SYNAPSES_POSTGIDS_DOXY "\n**Return**\n    the GIDs of the postsynaptic neurons. \n\n"

#define SYNAPSES_POSTSECTIONIDS_DOXY \
"\n**Return**\n    the section IDs on the postsynaptic neurons. \n\n" \

#define SYNAPSES_POSTSEGMENTIDS_DOXY \
"\n**Return**\n    the segment IDs on the postsynaptic neurons. \n\n" \

#define SYNAPSES_POSTDISTANCES_DOXY \
"\n**Return**\n    the distances between each synapse and the beginning of theirs postsynaptic " \
"segments in micrometers. \n\n" \

#define SYNAPSES_POSTSURFACEXPOSITIONS_DOXY \
"\n**Return**\n    the postsynaptic touch position x-coordinates on the surfaces of the " \
"segments. May be null in old circuits. \n\n" \

#define SYNAPSES_POSTSURFACEYPOSITIONS_DOXY \
"\n**Return**\n    the postsynaptic touch position y-coordinates on the surfaces of the " \
"segments. May be null in old circuits. \n\n" \

#define SYNAPSES_POSTSURFACEZPOSITIONS_DOXY \
"\n**Return**\n    the postsynaptic touch position z-coordinates on the surfaces of the " \
"segments. May be null in old circuits. \n\n" \

#define SYNAPSES_POSTCENTERXPOSITIONS_DOXY \
"\n**Return**\n    the postsynaptic touch position x-coordinates in the center of the " \
"segments. \n\n" \

#define SYNAPSES_POSTCENTERYPOSITIONS_DOXY \
"\n**Return**\n    the postsynaptic touch position y-coordinates in the center of the " \
"segments. \n\n" \

#define SYNAPSES_POSTCENTERZPOSITIONS_DOXY \
"\n**Return**\n    the postsynaptic touch position z-coordinates in the center of the " \
"segments. \n\n" \

#define SYNAPSES_DELAYS_DOXY "\n**Return**\n    the axonal delays in milliseconds. \n\n"

#define SYNAPSES_CONDUCTANCES_DOXY "\n**Return**\n    the conductances in nanosiemens. \n\n"

#define SYNAPSES_UTILIZATIONS_DOXY \
"\n**Return**\n    the neuro-transmitter release probabilities. \n\n" \

#define SYNAPSES_DEPRESSIONS_DOXY \
"\n**Return**\n    the depression time constants in milliseconds. \n\n" \

#define SYNAPSES_FACILITATIONS_DOXY \
"\n**Return**\n    the facilitation time constants in milliseconds. \n\n" \

#define SYNAPSES_DECAYS_DOXY "\n**Return**\n    the decay time constants in milliseconds. \n\n"

#define SYNAPSES_EFFICACIES_DOXY \
"\n**Return**\n    the absolute synaptic efficacies in millivolts. \n\n" \

// ===============================================================================

#define SIMULATION_CLASS_DOXY \
"\nSimulation configuration parser and entry point.\n\nThis object parses a simulation " \
" and gives access to its circuit, reports and cell sets. \n" \

#define SIMULATION_CONSTRUCTOR_DOXY \
"\nOpen a simulation configuration\n\n**Parameters**\n\n     ``source`` -       the URI " \
"to the BlueConfig file. \n\n" \

#define SIMULATION_OPENCIRCUIT_DOXY "\nOpen the circuit associated to this simulation. \n"

#define SIMULATION_OPENCOMPARTMENTREPORT_DOXY \
"\nOpen a compartment report with the given name. **Parameters**\n\n     ``name`` -       " \
"the report name as it appears in the configuration. \n\n" \

#define SIMULATION_GETGIDS_DOXY "\n**Return**\n    the gids of the default circuit target. \n\n"

#define SIMULATION_GETGIDS_TARGET_DOXY \
"\n**Return**\n    the gids of a cell set given its name. \n\n" \

#define SIMULATION_GETCOMPARTMENTREPORTNAMES_DOXY \
"\n**Return**\n    the names of the reports listed in the config. \n\n" \

#define SIMULATION_GETTARGETNAMES_DOXY \
"\n**Return**\n    the names of all cell sets referred by this simulation. \n\n" \

// ===============================================================================

#define SPIKEREPORTREADER_CONSTRUCTOR_DOXY \
"\nConstruct a new reader opening a spike data source. **Version**\n    1.0 \n\n" \
"**Parameters**\n\n     ``uri`` -       URI to spike report (can contain a wildcard" \
" to specify several files). \n\n**Exceptions**\n\n     ``std.runtime_error`` " \
"-       if source is invalid. \n\n" \

#define SPIKEREPORTREADER_CTOR_URI_GIDSET_DOXY \
"\nConstruct a new reader opening a spike data source. **Version**\n    1.0 \n\n" \
"**Parameters**\n\n     ``uri`` -       URI to spike report (can contain a wildcard" \
" to specify several files). \n\n     ``subset`` -       Subset of cells to be reported." \
" files). \n\n**Exceptions**\n\n     ``std.runtime_error`` -       if source is invalid. \n\n" \

#define SPIKEREPORTREADER_CLOSE_DOXY \
"\nClose the data source.\n\nAny thread blocked in getSpikes will return immediately, possibly " \
"returning an empty container. This method may be called concurrently to both :std std-ref:" \
"`getSpikes()` functions and :std std-ref:`hasEnded()`.\n\n**Version**\n    1.0 \n\n" \

#define SPIKEREPORTREADER_GETSPIKES_DOXY \
"\nGet all spikes inside a time window.\n\nFor stream reports this method will wait until " \
"the first spike with a time larger or equal to end arrives. The time interval is open on " \
"the right, so assuming that spikes arrive in order, this method will return a full snapshot " \
"of the spikes between [start, end). Precondition : start < end **Return**\n    A numpy array " \
"of dytpe = “f4, u4” \n\n**Version**\n    1.0 \n\n**Exceptions**\n\n     ``std.logic_error`` " \
"-       if the precondition is not fulfilled. \n\n" \

#define SPIKEREPORTREADER_GETENDTIME_DOXY \
"\n**Return**\n    the end timestamp of the report. This is the timestamp of the last spike " \
"known to be available or larger if the implementation has more metadata available. For " \
"stream reports this time is 0 and it is updated when getSpikes is called. \n\n" \
"**Version**\n    1.0 \n\n" \

#define SPIKEREPORTREADER_HASENDED_DOXY \
"\n**Return**\n    true if any of the versions of :std std-ref:`getSpikes()` reaches the end " \
"of the stream, if the report is static or if closed has been called. \n\n**Version**\n    " \
"1.0 \n\n" \

// ===============================================================================

#define SPIKEREPORTWRITER_CONSTRUCTOR_DOXY \
"\nConstruct a new writer for the given URI. **Version**\n    1.0 \n\n**Parameters**\n\n     " \
"``uri`` -       URI to spike report \n\n     ``accessMode`` -       Access mode \n\n" \

#define SPIKEREPORTWRITER_CLOSE_DOXY \
"\nCloses the report. ( It is implicitly called on destruction ).\n\n**Version**\n    1.0 \n\n" \

#define SPIKEREPORTWRITER_WRITESPIKES_DOXY \
"\nWrites the spike times and cell GIDs.\n\n**Version**\n    1.0 \n\n**Parameters**\n\n     " \
"``spikes`` -       Spikes to write. \n\n" \

#endif // BRAIN_PYTHONDOCUMENTATION_H
