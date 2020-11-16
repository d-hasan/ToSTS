import string 


trip_template = string.Template(
    '\t<trip id="$trip_id" depart="$departure_time" from="$from_edge" \
    departPos="$from_pos" to="$to_edge" arrivalPos="$to_pos" \
    fromJunction="$from_node" toJunction="$to_node"/>\n')


# trip_template = string.Template(
#     '\t<trip id="$trip_id" depart="$departure_time" \
#     fromJunction="$from_node" toJunction="$to_node"/>\n')

class SUMOTrip():

    ## ORIGINAL 
    def __init__(self, origin, dest, departure_time, trip_num):
        self.origin_edge = origin[0]
        self.origin_edge_pos = origin[1]
        self.origin_node = origin[2]
        self.origin_point = origin[3]
        self.origin_taz_original = origin[4]
        # self.origin_taz = origin[4]
        
        self.dest_edge = dest[0]
        self.dest_edge_pos = dest[1]
        self.dest_node = dest[2]
        self.dest_point = dest[3]
        self.dest_taz_original = dest[4]
        # self.dest_taz = dest[4]

        self.original_departure_time = departure_time
        self.departure_time = int(departure_time)
        self.trip_num = trip_num
        self.trip_id = '{}-{}-{}-{}'.format(
            self.origin_taz_original, 
            self.dest_taz_original, 
            self.departure_time,
            self.trip_num)
        
        
    def construct_trip_string(self):
        trip_string = trip_template.substitute(
            trip_id=self.trip_id,
            departure_time=self.departure_time, 
            from_edge=self.origin_edge.getID(),
            from_pos=self.origin_edge_pos,
            to_edge=self.dest_edge.getID(),
            to_pos=self.dest_edge_pos,
            from_node=self.origin_node.getID(),
            to_node=self.dest_node.getID())
        return trip_string 