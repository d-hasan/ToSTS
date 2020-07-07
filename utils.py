import configparser


configparser

def convert_clock_to_minutes(clock_time):
    hour = clock_time // 100
    minute_offset = (clock_time % 100) % 60

    return int(hour * 60 + minute_offset )


def convert_minutes_to_clock(minutes):
    hour = (minutes // 60) * 100
    minute_offset = minutes % 60

    return int(hour + minute_offset)

def get_config(config_path):
    config = configparser.ConfigParser()
    config.read(config_path)

    return config 