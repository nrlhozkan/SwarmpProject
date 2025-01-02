import math
from pyproj import Transformer

EARTH_RADIUS_M = 6378000

# Define simulation boundaries
BORDER_LAT_RANGE = (48.0000, 48.0045)
BORDER_LON_RANGE = (14.0000, 14.00671)

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

class CoordinateConverter:
    """
    Handles coordinate conversions between lat/lon (WGS84) and a local UTM projection.
    """
    def __init__(self, ref_lat, ref_lon):
        utm_zone = int((ref_lon + 180) / 6) + 1
        hemisphere = 'north' if ref_lat >= 0 else 'south'
        epsg_code = 32600 + utm_zone if hemisphere == 'north' else 32700 + utm_zone
        self.transformer_to_utm = Transformer.from_crs("epsg:4326", f"epsg:{epsg_code}", always_xy=True)
        self.transformer_to_gps = Transformer.from_crs(f"epsg:{epsg_code}", "epsg:4326", always_xy=True)

    def latlon_to_xy(self, lat, lon):
        x, y = self.transformer_to_utm.transform(lon, lat)
        return x, y

    def xy_to_latlon(self, x, y):
        lon, lat = self.transformer_to_gps.transform(x, y)
        return lat, lon


def calculate_distance_meters(ref, prev_pos: Position, next_pos: Position) -> float:
    """
    Calculate the distance between two GPS coordinates in meters.
    """
    converter = CoordinateConverter(ref[0], ref[1])
        
    x1, y1 = converter.latlon_to_xy(prev_pos.latitude, prev_pos.longitude)
    x2, y2 = converter.latlon_to_xy(next_pos.latitude, next_pos.longitude)
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# def calculate_distance_meters(prev_pos: Position, next_pos: Position) -> float:
#     # source: https://stackoverflow.com/questions/365826/calculate-distance-between-2-gps-coordinates

#     # Convert degrees to radians
#     lat1_rad = math.radians(prev_pos.latitude)
#     lat2_rad = math.radians(next_pos.latitude)

#     delta_lat_rad = math.radians(next_pos.latitude - prev_pos.latitude)
#     delta_lon_rad = math.radians(next_pos.longitude - prev_pos.longitude)

#     # Haversine formula
#     a = math.sin(delta_lat_rad / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon_rad / 2) ** 2
#     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

#     return EARTH_RADIUS_M * c


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
