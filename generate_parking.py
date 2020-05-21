import os
import sys

import generateParkingAreasFromOSM


# Generates parking areas and rerouters for toronto OSM

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    import generateParkingAreaRerouters 

else:
    sys.exit("please declare environment variable 'SUMO_HOME'")



parking_options = ['--osm', 'toronto-data/osm_bbox.osm.xml', '--net', 'toronto-data/osm.net.xml', '--out', 'toronto-data/parking_areas.add.xml'] 
generateParkingAreasFromOSM.main(parking_options)

rerouters_options = ['-a', 'toronto-data/complete_parking_areas.add.xml', 
                    '-n', 'toronto-data/osm.net.xml', 
                    '--max-number-alternatives', '10', 
                    '--max-distance-alternatives', '1000.0', 
                    '--min-capacity-visibility-true', '50', 
                    '--max-distance-visibility-true', '1000.0', 
                    '--processes', str(2), 
                    '-o', 'toronto-data/parking_rerouters.add.xml'
                    , '--tqdm']
generateParkingAreaRerouters.main(rerouters_options)
