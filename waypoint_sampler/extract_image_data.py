import folium
import numpy as np
from PIL import Image, ImageDraw
from xml.etree import ElementTree as etree
import tempfile
import os
import io
from playwright.sync_api import sync_playwright
import time
from pix_gps_map import PixGpsMap
import pickle

def parse_kml(kml_file: str):
    tree = etree.parse(kml_file)
    root = tree.getroot()
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}


    ao_coords = []
    nfz_coords_list = []
    nfz_names = []


    for pm in root.findall('.//kml:Placemark', ns):
        name_elem = pm.find('.//kml:name', ns)
        if name_elem is not None:
            name = name_elem.text.strip()
            coords_elem = pm.find('.//kml:coordinates', ns)
            if coords_elem is not None:
                coords_text = coords_elem.text.strip()
                coords = []
                for c in coords_text.split():
                    parts = c.split(',')
                    if len(parts) >= 2:
                        lon = float(parts[0])
                        lat = float(parts[1])
                        coords.append((lat, lon))
                if name.lower().startswith('ao'):
                    ao_coords = coords
                elif name.lower().startswith('nfz_'):
                    nfz_coords_list.append(coords)
                    nfz_names.append(name)


    return ao_coords, nfz_coords_list, nfz_names


def get_clean_satellite_with_bounds(ao_coords, nfz_coords_list, image_size=(1200, 800)):
    all_coords = ao_coords + [c for coords in nfz_coords_list for c in coords]
    lats = [lat for lat, lon in all_coords]
    lons = [lon for lat, lon in all_coords]
    
    lat_min, lat_max = min(lats), max(lats)
    lon_min, lon_max = min(lons), max(lons)
    
    lat_pad = 0
    lon_pad = 0
    
    bounds_sw = [lat_min - lat_pad, lon_min - lon_pad]
    bounds_ne = [lat_max + lat_pad, lon_max + lon_pad]
    
    center_lat = (bounds_sw[0] + bounds_ne[0]) / 2
    center_lon = (bounds_sw[1] + bounds_ne[1]) / 2
    
    print(f"Fit bounds: SW={bounds_sw}, NE={bounds_ne}")
    
    tiles = "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
    
    m = folium.Map(
        tiles=tiles,
        attr='Google Satellite',
        width=image_size[0],
        height=image_size[1]
    )
    
    m.fit_bounds([bounds_sw, bounds_ne])
    
    html_path = tempfile.mktemp(suffix='.html')
    m.save(html_path)
    
    image = None
    bounds_data = None
    
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()
        page.set_viewport_size({"width": image_size[0], "height": image_size[1]})
        page.goto(f'file://{os.path.abspath(html_path)}')
        page.wait_for_load_state('networkidle')
        page.wait_for_timeout(2000)
        
        bounds_data = page.evaluate("""
            () => {
                let mapObj = window.map || document.querySelector('.folium-map')?._leaflet_map;
                
                if (!mapObj) {
                    for (let key in window) {
                        if (window[key] && window[key]._container && window[key].getBounds) {
                            mapObj = window[key];
                            break;
                        }
                    }
                }
                
                if (mapObj && mapObj.getBounds) {
                    const bounds = mapObj.getBounds();
                    return {
                        north: bounds.getNorth(),
                        south: bounds.getSouth(),
                        east: bounds.getEast(),
                        west: bounds.getWest()
                    };
                }
                
                return null;
            }
        """)
        
        if bounds_data:
            print(f"Extracted JS bounds")
        else:
            raise Exception("Map object not found")        
        
        screenshot_bytes = page.screenshot(full_page=False)
        image = Image.open(io.BytesIO(screenshot_bytes))
        
        browser.close()
    
    os.unlink(html_path)
    
    return np.array(image), bounds_data


def save_map_data(image, bounds_data, pkl_path='satellite_map.pkl'):
    map_data = {
        'image': image,
        'bounds': bounds_data,
        'image_shape': image.shape
    }
    
    with open(pkl_path, 'wb') as f:
        pickle.dump(map_data, f)
    
    return pkl_path


def draw_polygons(map_converter, ao_coords, nfz_coords_list, nfz_names):
    # Get the base image
    image = map_converter.get_image()
    
    img_pil = Image.fromarray(image).convert('RGBA')
    overlay = Image.new('RGBA', img_pil.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    
    # Convert AO coords to pixels using PixGpsMap
    ao_pixels = [map_converter.gps_to_pixel(lat, lon) for lat, lon in ao_coords]
    
    ao_x = [p[0] for p in ao_pixels]
    ao_y = [p[1] for p in ao_pixels]
    
    # Draw AO (cyan, semi-transparent)
    if len(ao_pixels) > 2:
        ao_pixels_closed = ao_pixels + [ao_pixels[0]]
        draw.polygon(ao_pixels, fill=(0, 255, 255, 50), outline=(0, 255, 255, 255))
        draw.line(ao_pixels_closed, fill=(0, 255, 255, 255), width=4)
    
    # Draw NFZs (red, semi-transparent)
    for i, (nfz_coords, name) in enumerate(zip(nfz_coords_list, nfz_names)):
        nfz_pixels = [map_converter.gps_to_pixel(lat, lon) for lat, lon in nfz_coords]        
        nfz_x = [p[0] for p in nfz_pixels]
        nfz_y = [p[1] for p in nfz_pixels]
        
        if len(nfz_pixels) > 2:
            nfz_pixels_closed = nfz_pixels + [nfz_pixels[0]]
            draw.polygon(nfz_pixels, fill=(255, 0, 0, 50), outline=(255, 0, 0, 255))
            draw.line(nfz_pixels_closed, fill=(255, 0, 0, 255), width=3)
        
    img_pil = Image.alpha_composite(img_pil, overlay)
    final_image = np.array(img_pil.convert('RGB'))
    
    return final_image


# MAIN
if __name__ == "__main__":
    kml_file = 'pennovation.kml'

    ao_coords, nfz_coords_list, nfz_names = parse_kml(kml_file)
    
    # get clean satellite + image bounds
    image, bounds_data = get_clean_satellite_with_bounds(ao_coords, nfz_coords_list)
    
    # save everything in one file
    pkl_path = save_map_data(image, bounds_data, 'satellite_map.pkl')
    
    # load with PixGpsMap and draw polygons    
    map_converter = PixGpsMap('satellite_map.pkl')
    final_image = draw_polygons(
        map_converter, 
        ao_coords, 
        nfz_coords_list, 
        nfz_names
    )
    
    output_path = 'map_with_polygons.png'
    Image.fromarray(final_image).save(output_path)
