import math

EARTH_RADIUS_M = 6378000


class Position:
    def __init__(self, latitude: float, longitude: float):
        self.latitude = latitude
        self.longitude = longitude

    def __repr__(self):
        return f'{self.latitude} | {self.longitude}'

    def __eq__(self, other):
        if isinstance(other, Position):
            return (self.latitude == other.latitude) and (self.longitude == other.longitude)
        return False

def calculate_distance_meters(prev_pos: Position, next_pos: Position) -> float:
    # source: https://stackoverflow.com/questions/365826/calculate-distance-between-2-gps-coordinates

    # Convert degrees to radians
    lat1_rad = math.radians(prev_pos.latitude)
    lat2_rad = math.radians(next_pos.latitude)

    delta_lat_rad = math.radians(next_pos.latitude - prev_pos.latitude)
    delta_lon_rad = math.radians(next_pos.longitude - prev_pos.longitude)

    # Haversine formula
    a = math.sin(delta_lat_rad / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon_rad / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS_M * c


def generate_initial_positions(n_drones, dist_between_drones, lat_range, lon_range):
    from_lat, to_lat = lat_range
    from_lon, to_lon = lon_range

    positions = []
    step_lon = dist_between_drones / (EARTH_RADIUS_M * math.cos(math.radians(from_lat))) * (180 / math.pi)

    current_lat = from_lat
    current_lon = from_lon + step_lon  # keep some

    for i in range(n_drones):
        positions.append(Position(current_lat, current_lon))
        current_lon += step_lon

    return positions
