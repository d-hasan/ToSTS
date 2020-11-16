import xml.etree.ElementTree as ET
import string 

output_path = 'data/tts_sumo/mini_trip_samples.xml'
trip_samples_path = 'data/tts_sumo/trip_samples_max50.xml'
xml_version=  '<?xml version="1.0" encoding="UTF-8"?>\n'
trip_header = '<trips>\n'

tree = ET.parse(trip_samples_path)
root = tree.getroot()

trips = []
for od in root:
    for trip_sample in od:
        trips.append(trip_sample.attrib)


trip_template = string.Template(
    '\t<trip id="$trip_id" depart="$departure_time" from="$from_edge" \
    departPos="$from_pos" to="$to_edge" arrivalPos="$to_pos" \
    fromJunction="$from_node" toJunction="$to_node"/>\n')

trip_strings = []
for trip in trips:
    trip_string = trip_template.substitute(
                trip_id=trip['id'],
                departure_time=trip['inflowTime'], 
                from_edge=trip['from'],
                from_pos=trip['departPos'],
                to_edge=trip['to'],
                to_pos=trip['arrivalPos'],
                from_node=trip['fromJunction'],
                to_node=trip['toJunction']
            )
    trip_strings.append(trip_string)

with open(output_path, 'w') as f:
    f.write(xml_version)
    f.write(trip_header)
            
    trip_strings = ''.join(trip_strings)
    f.write(trip_strings)

    f.write('</trips>')
    


