# README

## Dependencies
The dependencies are listed in `dependencies.yml`, it's recommended to create a conda environment using this file.

### Data
Almost all required data is included within the `data` folder. However, the Ontario Road Network, which is required to compute inter-regional trips, must be downloaded from the following [link](https://opendata.arcgis.com/datasets/923cb3294384488e8a4ffbeb3b8f6cb2_32.zip). Extract the zip file into the following path `data/ontario_road_network/`. 

## Visualization
There is some analysis and visualization in the Jupyter notebook, however most of the code is under development to generate a full SUMO scenario from the trip and map data.