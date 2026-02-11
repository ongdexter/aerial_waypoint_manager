import numpy as np
import pickle


class PixGpsMap:    
    def __init__(self, pkl_path):
        with open(pkl_path, 'rb') as f:
            map_data = pickle.load(f)
        
        self.image = map_data['image']
        self.bounds = map_data['bounds']
        self.image_shape = map_data['image_shape']
        self.ao_coords = map_data.get('ao_coords', [])
        self.nfz_coords_list = map_data.get('nfz_coords_list', [])
        self.nfz_names = map_data.get('nfz_names', [])
        
        self.north = self.bounds['north']
        self.south = self.bounds['south']
        self.east = self.bounds['east']
        self.west = self.bounds['west']
        
        self.lat_span = self.north - self.south
        self.lon_span = self.east - self.west
        
        self.height, self.width = self.image_shape[:2]
    
    def gps_to_pixel(self, lat, lon):
        lat_norm = (lat - self.south) / self.lat_span
        lon_norm = (lon - self.west) / self.lon_span
        
        px = int(np.clip(self.width * lon_norm, 0, self.width - 1))
        py = int(np.clip(self.height * (1 - lat_norm), 0, self.height - 1))
        
        return px, py
    
    def pixel_to_gps(self, px, py):
        lat_norm = 1 - (py / self.height)
        lon_norm = px / self.width
        
        lat = self.south + lat_norm * self.lat_span
        lon = self.west + lon_norm * self.lon_span
        
        return lat, lon
    
    def is_in_bounds(self, lat, lon):
        return (self.south <= lat <= self.north and 
                self.west <= lon <= self.east)
    
    def get_pixel_value(self, lat, lon):
        px, py = self.gps_to_pixel(lat, lon)
        return self.image[py, px]
    
    def get_image(self):
        return self.image
