# README

## Dependencies
The dependencies are listed in `dependencies.yml`, it's recommended to create a conda environment using this file.

### Data
Almost all required data is included within the `data` folder. However, the Ontario Road Network, which is required to compute inter-regional trips, must be downloaded from the following [link](https://opendata.arcgis.com/datasets/923cb3294384488e8a4ffbeb3b8f6cb2_32.zip). Extract the zip file into the following path `data/ontario_road_network/`. 

## Analysis/Visualization
There is some analysis and visualization in the Jupyter notebook. Trips can be generated by executing the notebook end to end but this is no longer recommended.

## Generating Trips
To generate trips, run:
```
python generate_grid.py [config dir] 
```

The default configuration files are located in the ```configs``` folder. The default configuration files can be modified, or new configuration files can be made and passed into the program as so:

```
usage: generate_tts_trips.py [-h] [--sim-net-config SIM_NET_CONFIG] [--trip-planner-config TRIP_PLANNER_CONFIG] [--tts-data-config TTS_DATA_CONFIG] [--traffic-regions-config TRAFFIC_REGIONS_CONFIG] config

Generate SUMO Trips from Toronto Transit Survey Data

positional arguments:
  config                Config folder

optional arguments:
  -h, --help            show this help message and exit
  --sim-net-config SIM_NET_CONFIG
                        to change default simulation net config file
  --trip-planner-config TRIP_PLANNER_CONFIG
                        to change default trip planner config file
  --tts-data-config TTS_DATA_CONFIG
                        to change default tts data config file
  --traffic-regions-config TRAFFIC_REGIONS_CONFIG
```

Once trips are generated, ```DUAROUTER``` needs to be run to generate routes for SUMO to work with. 

** CAUTION: It is suspected that using DUAROUTER in this way is slightly incorrect, as it leads to major traffic jams and underutilization of the traffic network. But for small scale simulations it gets the job done FOR NOW. **

Run:
```
duarouter --unsorted-input --ignore-errors --repair true --repair.from true --repair.to true -n data/osm_to_sumo/osm_pruned.net.xml --route-files [path to trip files] -o [path to output routes] --routing-threads [num routing threads]
```

### Simulation Network
The simulation network loads the generated SUMO Network from the OSM map of Downtown Toronto. The AOI Downtown boundary is defined in the ```simulation_net.cfg``` and should not be modified unless the AOI is changed. The traffic network nodes and edges are pruned based on the defined boundary and a new SUMO network is generated using ```NETCONVERT``` utility from SUMO. (*Details on how to prune are located in the **Prune SUMO Network** section of the README*)

### Toronto Transportation Survey (TTS) Data
The TTS Data comes in a txt format that is copied from the database output of the TTS portal. This data has to be parsed and processed in ```tts_data.py```. The data processing includes smoothing of the trip counts with a triangle filter, as the counts are very bursty at 30 minute intervals due to the nature of the survey data collection. 

The configuration file allows the user to define the interval size for trips (default to 5 minute) and the filter width for smoothing the trip counts (defaults to 7 to take a 15 minute before/after weighted average of trip counts). Additionally, file paths are defined in the configuration file.

### Traffic Regions
All traffic region geometry is handled within the Traffic Regions class. The Traffic Regions class contains GeoPandas Dataframes and Series for the traffic zones, planning districts, regions, and highways, as determined by the [TTS defined zones](http://dmg.utoronto.ca/survey-boundary-files#2006_zone) and the Ontario Road Network.

### Region Graph
The Region Graph is used to determine if trips from and/or to external regions to the AOI, actually travel through the AOI. The Region Graph class builds a multigraph of planning districts as nodes and *trivial* and/or *highway* roads edges. The edges represent planning districts that are adjacent and defines multiple edges between planning districts. Adjacent planning districts by default have trivial edges which represent any non-highway road, if there is a highway connecting them another edge is added for the highway that has a lower weight (representing greater speed/capacity).

### Trip Planner
The Trip Planner does the bulk of the lifting in actually generating trips for SUMO. The configuration file allows the user to define:
 * the inflow and outflow nodes for highways intersecting the AOI (this shouldn't be touched for the most part)
 * the average trivial and highway speed (in km/h)
 * the speed minimum and maximum factors (cutoffs for the truncated normal distribution that speed is sampled from)
 * speed deviation for the normal distribution
 * trip start and end timings (for generating a smaller time period)
 * the minimum percentage of a trip's length that has to intersect the AOI to be considered as passing through AOI (only for straight line trivial trips)
 * The maximum distance to locate inflow/outflow nodes for a straight line trip (samples from the closest nodes)
 * The number of closest nodes to consider when sampling inflow/outflow node (weighted based on capacity = road_speed * lanes)
 * The path length when doing inter planning district travel to consider a trip not to take highways (path length of 3 means path of [origin, via, destination])

To generate all trips, the Trip Planner takes about 8 hours on a Ryzen 2700x @ 3.7 GHz base. Work is being considered to multi thread this.


## Prune SUMO Network
Although not strictly something implemented in this library, pruning of the SUMO network after conversion from OSM proved to be a task that challenged us. The easy way to do it is actually to use ```NETCONVERT``` from SUMO and convert from a SUMO network to another SUMO network.

To prune the SUMO network within a boundary or from a list of edges refer to [this](https://sumo.dlr.de/docs/Networks/Further_Options.html#pruning_the_imported_network) documentation and look at the NETCONVERT [documentation](https://sumo.dlr.de/docs/NETCONVERT.html#edge_removal) in the edge removal section.
The command used in this library (in the ```prune_network``` function of  the class ```Simulation_Net```) was:

```
netconvert -s osm.net.xml --keep-edges.input-file edges_to_keep.txt -o osm_pruned.net.xml  ```
