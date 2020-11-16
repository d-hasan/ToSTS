import xml.etree.ElementTree as ET
import numpy as np
trip_path = 'data/tts_sumo/mini_trip_samples.xml'
tree = ET.parse(trip_path)
root = tree.getroot()

for child in root:
    child.attrib.pop('type')

tree.write( 'data/tts_sumo/mini_trip_samples.xml')