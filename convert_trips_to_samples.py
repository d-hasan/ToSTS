import xml.etree.ElementTree as ET
import string 
from collections import defaultdict

trips_path = 'data/tts_sumo/dua_mini_trip_samples.xml'
output_path = 'data/tts_sumo/trip_samples_mini.xml'
xml_version=  '<?xml version="1.0" encoding="UTF-8"?>\n'
trip_header = '<trips>\n'

trip_template = string.Template(
    '\t<trip id="$trip_id" depart="$departure_time" from="$from_edge" \
    to="$to_edge" />\n')

tree = ET.parse(trips_path)
root = tree.getroot()

od_dict = defaultdict(list)

for trip in root:
    trip_id = trip.attrib['id']
    od = '-'.join(trip_id.split('-')[:2])
    od_dict[od].append(trip.attrib)


with open(output_path, 'w') as f:
    f.write(xml_version)
    f.write(trip_header)

for od in od_dict:
    origin, dest = od.split('-')

    orig_dest_string = '\t<OD origin="{}" dest="{}">\n'.format(origin, dest)

    trip_strings = []
    for trip in od_dict[od]:
        trip_string = trip_template.substitute(
            trip_id=trip['id'],
            from_edge=trip['from'],
            to_edge=trip['to'],
            departure_time=trip['depart']    
        )
        trip_strings.append(trip_string)
    trip_strings = ''.join(trip_strings)

    with open(output_path, 'a') as f:
        f.write(orig_dest_string)
        f.write(trip_strings)   
        f.write('\t</OD>\n') 

with open(output_path, 'a') as f:
    f.write('</trips>')
