Changelog {#Changelog}
=========

# Release 3.3.0

* [305](https://github.com/BlueBrain/Brion/pull/305)
  Preparation for 3.3.0 release.
* [304](https://github.com/BlueBrain/Brion/pull/304)
  Rename brain python package to brion.
* [303](https://github.com/BlueBrain/Brion/pull/303)
  Improve SONATA syanapse support.
* [302](https://github.com/BlueBrain/Brion/pull/302)
  Fix glm targets not installed properly.
* [300](https://github.com/BlueBrain/Brion/pull/300)
  Initial commit for python wheel distribution package support.
* [299](https://github.com/BlueBrain/Brion/pull/299)
  Removing unused functions from old SONATA circuit implementation.
* [298](https://github.com/BlueBrain/Brion/pull/298)
  Support for relative paths in BlueConfig/CircuitConfig.
* [297](https://github.com/BlueBrain/Brion/pull/297)
  SONATA circuit support.
* [295](https://github.com/BlueBrain/Brion/pull/295)
  Misc. fixes and updates.
* [293](https://github.com/BlueBrain/Brion/pull/293)
  Fix mvdtool bug when mecombo file is empty.
* [290](https://github.com/BlueBrain/Brion/pull/290)
  Replacing morphology readers and tests with MorphIO.
* [289](https://github.com/BlueBrain/Brion/pull/289)
  Update README.md.
* [288](https://github.com/BlueBrain/Brion/pull/288)
  Update MVDTool submodule.


# Release 3.2.0 (10-06-2020)

* [287](https://github.com/BlueBrain/Brion/pull/287)
  Fix wrong file access mode when opening spike report for reading.
* [286](https://github.com/BlueBrain/Brion/pull/286)
  Fix cmake build with newer boost, python and cmake version. Fix tests.
* [285](https://github.com/BlueBrain/Brion/pull/285)
  Attempt to fix intel compile errors.
* [284](https://github.com/BlueBrain/Brion/pull/284)
  Fix installation structure from CMake.
* [283](https://github.com/BlueBrain/Brion/pull/283)
  Preparation for 3.2.0 release.
* [282](https://github.com/BlueBrain/Brion/pull/282)
  Few fixes related to glm.
* [281](https://github.com/BlueBrain/Brion/pull/281)
  Strips Pydoxygen, Servus and CMake/common.
* [280](https://github.com/BlueBrain/Brion/pull/280)
  Bumps libsonata submodule.
* [279](https://github.com/BlueBrain/Brion/pull/279):
  Removed lunchbox library from Brion.
* [276](https://github.com/BlueBrain/Brion/pull/276):
  Removed vmmlib library from Brion. Replaced with GLM.
* [275](https://github.com/BlueBrain/Brion/pull/275):
  Removed Keyv library from Brion. This change also means the end of
  support for BlueBrain's key value map based reports.
* [273](https://github.com/BlueBrain/Brion/pull/273):
  Fixes to improve compilation against libsonata.

# Release 3.1.0 (07-01-2020)

* [269](https://github.com/BlueBrain/Brion/pull/269):
  Introduce SONATA support
* [263](https://github.com/BlueBrain/Brion/pull/263):
  Detect start.target in CircuitPath
* [255](https://github.com/BlueBrain/Brion/pull/255):
  Adding Python 3.7 and removing 3.4 from wheels

# Release 3.0.0 (20-03-2019)

* [250](https://github.com/BlueBrain/Brion/pull/250):
  Added new overloads of brain::Simulation::getGIDs to get random GID sets.
* [249](https://github.com/BlueBrain/Brion/pull/249):
  - brion::CompartmentReport::loadFrame changed to return brion::Frame
  - Implemented missing brain::CompartmentReportView::getReport
* [239](https://github.com/BlueBrain/Brion/pull/239):
  - Added two query options to configure chunking and chunk cache size when
    opening/creating SONATA compartment reports.
  - Added a new overload of brion::CompartmentReport::write to allow
    optimizations for full frame writes.
* [229](https://github.com/BlueBrain/Brion/pull/229):
  - Added a method to brain::Circuit to retrive custom node attributes. This is
    only supported in C++ for SONATA circuits.
  - Implemented morphology recentering for SONATA circuits.
* [227](https://github.com/BlueBrain/Brion/pull/226):
  Read/write support for SONATA compartment reports. This file format replaces
  the old file format when writing reports.
* [225](https://github.com/BlueBrain/Brion/pull/225):
  Addition of some new methods and classes:
  - Added the class Simulation for accesing the contents of simulation
    configurations
  - Circuit::getSource
  - Circuit::getMorphologyNames
  - Made CompartmentReport and SpikeReporReader movable
  - Soma::getMaxRadius
  - neuron::Section::getNumSamples
  - neuron::Section::operator\[\]
  - Support morphologies with single point somas
  - Added default Synapses::iterator constructor
  - Changed Synapses and Synapse memory magement implementation to allow
    use in Python wrappings of client code.
* [222](https://github.com/BlueBrain/Brion/pull/222):
  Implemented basic support for reading circuit in SONATA file format
  - The implementation is limited to node, no edges are read
  - Only one population and one node group are read, the rest are ignored.
* [204](https://github.com/BlueBrain/Brion/pull/204):
  Fixed brain::Circuit methods to not crash with empty gids lists.
* [191](https://github.com/BlueBrain/Brion/pull/191):
  - Optimizations in brain::CompartmentReport opening, only the metadata is
    read.
  - Removed compartmentCount and cellCount from brain::CompartmentReportMetadata
    because they can't be figured out efficiently in all report types.
  - New function getFrameSize added to CompartmentReportMapping to replace
    compartmentCount.
  - New function getCellCount added to CompartmentReport to replace cellCount.
* [180](https://github.com/BlueBrain/Brion/pull/180):
  Bugfixes in compartment reports:
  - The report converter was messing up the order of the data when writing to
    .h5 from an out-of-order binary.
  - Timestamps reported were wrong for reports not starting at 0.
  - Read time and data unit attributes from H5 reports is they exist.
  - Do not assume the first section is always present post-processing the
    mapping in H5 reports.
  - Avoid crashing when calling getNumCompartments from Python with and out of
    bounds index.
  - Corrections in compartmentReport comparison code.
* [176](https://github.com/BlueBrain/Brion/pull/176):
  Removed the dependency on HDF5++ and use HighFive instead.
* [172](https://github.com/BlueBrain/Brion/pull/172):
  - zeroeq::Server/Client based morphology loader.
  - New brion::Morphology read API and removed write API (see doc/feature/morphologyRead.md)
  - Removed apical point API
  - morphologyConverter can only write H5 V1.1 morphologies
* [171](https://github.com/BlueBrain/Brion/pull/171):
  Ban obsolete morphology repair stage from API: The stage parameter from
  certain Morphology methods has been removed. In the case of h5v2 morphologies,
  the implementation will pick the most complete repair stage.

# Release 2.0.0 (30-05-2017)

* [159](https://github.com/BlueBrain/Brion/pull/159):
  Changed mapping index according to Pandas requirements for indexing.
* [157](https://github.com/BlueBrain/Brion/pull/157):
  - Add Synapse.gid() python wrapping
  - Improve synapse attribute reading performance
* [154](https://github.com/BlueBrain/Brion/pull/154):
  - Improved help and command line inverface for spikeConverter and
    compartmentConverter.
  - Timestamps in brion::CompartmentReport changed from float to double.
  - New implementation of binary compartment report reader using POSIX AIO.
* [153](https://github.com/BlueBrain/Brion/pull/153):
  Updated Python wrapping of SpikeReportWriter to handle nympy arrays of spikes
* [152](https://github.com/BlueBrain/Brion/pull/152):
  Added the function brain.neuron.compute_morphological_samples to help getting
  sample positions from morphologies
* [151](https://github.com/BlueBrain/Brion/pull/151):
  brain::Circuit accepts full paths to mvd files in CircuitPath
* [150](https://github.com/BlueBrain/Brion/pull/150):
  - Added a new method in brain::Circuit to obtain synapses for afferent
    projections from outside the circuit (e.g. thalamocortical projections).
  - Support for Projections sections in brion::BlueConfig
* [141](https://github.com/BlueBrain/Brion/pull/141):
  Fixed documentation to specify that morphology section samples report
  diameters and not radii.
* [130](https://github.com/BlueBrain/Brion/pull/130),[138](https://github.com/BlueBrain/Brion/pull/138):
  Added brain::CompartmentReportReader and its Python wrapping
  brain::CompartmentReport : python API v2
* [136](https://github.com/BlueBrain/Brion/pull/136):
  Implement readable null:// compartment report for benchmarking
* [120](https://github.com/BlueBrain/Brion/pull/120), [131](https://github.com/BlueBrain/Brion/pull/131):
  - New SpikeReport API.
  - Reimplementation of the high level SpikeReportReader. The new implementation
    uses numpy arrays to provide the requested spikes.
* [126](https://github.com/BlueBrain/Brion/pull/126):
  Add erase for map compartment reports
* [122](https://github.com/BlueBrain/Brion/pull/122):
  Support loading of individual neurons for binary and map compartment reports
* [121](https://github.com/BlueBrain/Brion/pull/121):
  Performance optimizations for map and binary compartment reports, fix
  sub target loading of map compartment reports
* [119](https://github.com/BlueBrain/Brion/pull/119):
  Optimized brain::Circuit::loadMorphologies for slow stat'ing
  filesystems (GPFS)

# Release 1.9.0 (09-12-2016)

* [117](https://github.com/BlueBrain/Brion/pull/117):
  Changes and fixes in Brain Python module:
  - Functions that take gids now preserve the iteration order of the input in
    the output arrays/lists
  - Replaced Python sets with numpy arrays in functions returning GID sets
  - Circuit::getXYZNames functions renamed to Circuit::getXYZTypeNames
* [113](https://github.com/BlueBrain/Brion/pull/113):
  Support for old circuits containing only synapse center positions
* [110](https://github.com/BlueBrain/Brion/pull/110):
  Break PersistentMap out into keyv::Map
* [107](https://github.com/BlueBrain/Brion/pull/107):
  Added Sphinx generated documentation of the brain python module
* [102](https://github.com/BlueBrain/Brion/pull/102):
  Use PersistentMap for caching synapse positions loaded from brain::Circuit
* [94](https://github.com/BlueBrain/Brion/pull/94):
  Fixed SWC morphology parser for morphologies with soma contour. The parser was
  creating invalid soma sections when the first order sections where connected
  to arbitrary soma sample points.
* [89](https://github.com/BlueBrain/Brion/pull/89):
  Python wrapping of brain classes
* [88](https://github.com/BlueBrain/Brion/pull/88):
  - Brain namespace enums made strongly typed
  - Fix for brain::Section::getSamples for sections where the first point
    appears repeated
* [85](https://github.com/BlueBrain/Brion/pull/85):
  Implement brain synapses specification
* [84](https://github.com/BlueBrain/Brion/pull/84):
  Brain synapses specification
* [83](https://github.com/BlueBrain/Brion/pull/83):
  Add brain::Circuit::getRandomGIDs()
* [82](https://github.com/BlueBrain/Brion/pull/82):
  Improve performance when reading synapse attributes from non-merged files
* [81](https://github.com/BlueBrain/Brion/pull/81):
  Fix GID out-of-bounds handling for MVD3 in brain::Circuit
* [79](https://github.com/BlueBrain/Brion/pull/79):
  Use PersistentMap for cache in brain::Circuit::loadMorphologies(); add thread
  safety w/ synapses cache

# Release 1.8.0 (30-06-2016)

* [77](https://github.com/BlueBrain/Brion/pull/77):
  Add brain::Circuit::getMorphologyTypes() and brain::Circuit::getElectrophysiologyTypes()
* [75](https://github.com/BlueBrain/Brion/pull/75):
  Implement morphology version 1.1 specification
* [74](https://github.com/BlueBrain/Brion/pull/74):
  Remove deprecated enums and functions:
  - `CompartmentReport( const std::string&, const GIDSet& )` and
    `CompartmentReport( const std::string&,
    const brion::CompartmentReportFormat, const bool )` constructors; use
    brion::CompartmentReport::CompartmentReport( const brion::URI&, int, const brion::GIDSet& )
    instead
  - `enum CompartmentReportFormat`; use brion::BlueConfig::getReportSource()
    instead
* [71](https://github.com/BlueBrain/Brion/pull/71):
  Transparent caching of Synapse loading. See
  [Lunchbox#263](https://github.com/Eyescale/Lunchbox/pull/263) for
  configuration.
* [69](https://github.com/BlueBrain/Brion/pull/69):
  Speedup brain::Circuit::getGIDs(), add brion::BlueConfig::getTargetSources()
* [63](https://github.com/BlueBrain/Brion/pull/63):
  Moved old BBPSDK/Monsteer spike report to Brain (pending refactoring)

# Release 1.7.0 (24-03-2016)

* [56](https://github.com/BlueBrain/Brion/pull/56):
  Improved target parser
* [55](https://github.com/BlueBrain/Brion/pull/55):
  Add basic provenance metadata for written HDF5 compartment reports
* [49](https://github.com/BlueBrain/Brion/pull/49):
  Added the method brain::Circuit::getRotations
* [46](https://github.com/BlueBrain/Brion/pull/46):
  Fixed a bug in enum to string conversions affecting morphologyConverter.
* [45](https://github.com/BlueBrain/Brion/pull/45):
  Made targets optional.
  - brain::Circuit::getGIDs() returns all GIDs handled by the circuit.
  - brain::Circuit::getGIDs( target ) returns GIDS for a specified target.
  - brain::Circuit::getNumNeurons() returns the total number of neurons in the
    circuit.
* [43](https://github.com/BlueBrain/Brion/pull/43):
  Add MVD3 support to brain::circuit
* [39](https://github.com/BlueBrain/Brion/pull/39):
  Add compartment report converter tool
* [30](https://github.com/BlueBrain/Brion/pull/30),
  [35](https://github.com/BlueBrain/Brion/pull/35):
  Added a new library, Brain, to provide higher level functions and classes.
  The library provides:
  - A Circuit class to get basic information from cells and targets and load
    morphologies at circuit level.
  - A brain::cell namspace with a Morphology class plus other related classes
    to access morphological data about neurons.
* [38](https://github.com/BlueBrain/Brion/pull/38):
  Fix crash while reading more than `ulimit -Sn` (1024 default) morphologies
* [37](https://github.com/BlueBrain/Brion/pull/37):
  Added support for synapse nrn_extra.h5 files.
* [31](https://github.com/BlueBrain/Brion/pull/31):
  Fix crash while reading more than `ulimit -Sn` (1024 default) NEST gdf files
* [29](https://github.com/BlueBrain/Brion/pull/29):
  New member functions in brion::BlueConfig to provide a semantic API.
* [28](https://github.com/BlueBrain/Brion/pull/28):
  SpikeReport continues parsing files that have broken lines

# Release 1.6.0 (9-11-2015)

* [24](https://github.com/BlueBrain/Brion/pull/24):
  Add getURI() method in SpikeReport.
* [22](https://github.com/BlueBrain/Brion/pull/22):
  spikeConverter can process spikes to and from stream-type SpikeReport plugins.
* [22](https://github.com/BlueBrain/Brion/pull/22):
  SpikeReport DSO plugins in the LD_LIBRARY_PATH are loaded automatically.
* [12](https://github.com/BlueBrain/Brion/pull/12):
  Extended brion::Synapse to also support non-merged synapse files
* [12](https://github.com/BlueBrain/Brion/pull/12):
  Add brion::Target::parse() to resolve a given target name
* [9](https://github.com/BlueBrain/Brion/issues/9):
  Extend SWC parser to support fork and end points and undefined section points.
  The Brion::SectionType enum has not been extended to include end and fork
  points, these types are translated into the most reasonable one based on the
  point ancestors.

# Release 1.5.0 (7-07-2015)

* Add RESTING_VOLTAGE constant.
* Add support for binary spike reports.
* Add support for spike writer plugin.
* Add async IO support in brion::CompartmentReportMap.
* brion::CompartmentReportBinary::updateMapping() more robust.
* brion::SpikeReport API extended to support stream-based reports. A reference
  implementation of a stream-based spike report plugin is also provided.
* Ensure thread-safety with non-threadsafe HDF5 library.
* Morphology class implementation changed to support the Lunchbox plugin
  mechanism.
* New morphology plugin to read SWC morphologies.
* [#3](https://github.com/BlueBrain/Brion/pull/3):
  New morphology converter tool.
* New null compartment report for benchmarks.
* New spike converter tool.
* Optimized function Circuit::get for large circuits (cut down the loading time
  of the 3.1 M neuron circuit by 6.5 seconds in a Core i7-4930K @ 3.40GHz).
* Replace use of iostreams by lunchbox::MemoryMap.
* Several bugfixes and cleanups.

# Release 1.4.0 (8-10-2014)

* Old report readers have been adapted to be static plugins using
  lunchbox::IOPluginFactory.
* New spike report reader plugin for NEST spike reports. The plugin can
  parse multiple report file using shell wildcards (*, ?) in the
  filepath provided.
* New constructor for brion::CompartmentReport accepting an URI and the desired
  access mode deprecates the old read and write constructors.
* New compartment report based on key-value stores supported by
  Lunchbox::PersistentMap.
* Spike loading has been optimized (in a 12-core GPU the expected
  speed-up is above 4x for large files).
