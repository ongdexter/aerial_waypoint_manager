import numpy as np
import os
from PIL import Image
import pickle

class PixGpsMap:    
    def __init__(self, pkl_path):
        # Load all data
        with open(pkl_path, 'rb') as f:
            map_data = pickle.load(f)
        
        self.image = map_data['image']
        self.bounds = map_data['bounds']
        self.image_shape = map_data['image_shape']
        
        # Extract bounds
        self.north = self.bounds['north']
        self.south = self.bounds['south']
        self.east = self.bounds['east']
        self.west = self.bounds['west']
        
        # Calculate spans
        self.lat_span = self.north - self.south
        self.lon_span = self.east - self.west
        
        # Image dimensions
        self.height, self.width = self.image_shape[:2]
        
        print(f"PixGpsMap loaded:")
        print(f"  Bounds: N={self.north:.6f}, S={self.south:.6f}, E={self.east:.6f}, W={self.west:.6f}")
        print(f"  Image: {self.width}x{self.height} px")
        print(f"  Resolution: ~{self.lat_span*111000/self.height:.2f}m/pixel")
    
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
    
    def save_image(self, filepath):
        Image.fromarray(self.image).save(filepath)
        print(f"Image saved to: {filepath}")
