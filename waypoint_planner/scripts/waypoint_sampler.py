#!/usr/bin/env python3
"""
Waypoint Sampler - Sample waypoints from a KML file containing AO and NFZ polygons.

Usage:
    python waypoint_sampler.py --kml area.kml --resolution 2.0 --output waypoints_data.json
"""

import argparse
import json

from lxml import etree
from shapely.geometry import Polygon, Point
import os
import sys
# Run directly from the source tree as well as from an installed workspace.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from waypoint_planner.utm_zone import transformers_for, zone_name
import matplotlib.pyplot as plt


def parse_kml(kml_file: str):
    """Parse KML file and extract AO and NFZ polygons."""
    tree = etree.parse(kml_file)
    root = tree.getroot()
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}

    ao_coords = None
    nfz_coords_list = []

    for pm in root.findall('.//kml:Placemark', ns):
        name_elem = pm.find('.//kml:name', ns)
        if name_elem is not None:
            name = name_elem.text
            print(f"Placemark name: {name}")
            coords_elem = pm.find('.//kml:coordinates', ns)
            if coords_elem is not None:
                coords_text = coords_elem.text.strip()
                coords = [tuple(map(float, c.split(','))) for c in coords_text.split()]
                if name.startswith('ao'):
                    ao_coords = coords
                elif name.startswith('nfz_'):
                    nfz_coords_list.append(coords)

    if ao_coords is None:
        raise ValueError("No AO polygon found (name must start with 'ao')")

    return ao_coords, nfz_coords_list


def sample_waypoints(ao_coords, nfz_coords_list, spacing: float):
    """Sample waypoints within AO, excluding NFZs."""
    # UTM transformer, zoned from the AO rather than fixed.
    ref_lon, ref_lat = ao_coords[0][0], ao_coords[0][1]
    transformer_to_utm, transformer_from_utm, epsg = transformers_for(ref_lon, ref_lat)
    print(f"Projecting with EPSG:{epsg} (UTM zone {zone_name(ref_lon, ref_lat)})")

    def transform_coords(coords, transformer):
        return [(transformer.transform(lon, lat)[0], transformer.transform(lon, lat)[1]) 
                for lon, lat, alt in coords]

    ao_utm = transform_coords(ao_coords, transformer_to_utm)
    nfzs_utm = [transform_coords(nfz, transformer_to_utm) for nfz in nfz_coords_list]

    ao_poly = Polygon(ao_utm)
    nfz_polys = [Polygon(nfz_utm) for nfz_utm in nfzs_utm]

    # Sample waypoints on grid
    bounds = ao_poly.bounds
    waypoints_utm = []

    x = bounds[0]
    while x <= bounds[2]:
        y = bounds[1]
        while y <= bounds[3]:
            point = Point(x, y)
            if ao_poly.contains(point):
                in_nfz = any(nfz.contains(point) for nfz in nfz_polys)
                if not in_nfz:
                    waypoints_utm.append((x, y))
            y += spacing
        x += spacing

    # Convert back to lat/lon
    waypoints_latlon = [transformer_from_utm.transform(wx, wy) for wx, wy in waypoints_utm]

    return waypoints_utm, waypoints_latlon, ao_poly, nfz_polys


def visualize(ao_poly, nfz_polys, waypoints_utm, output_image: str = None, show=True):
    """Visualize AO, NFZs, and sampled waypoints."""
    plt.figure(figsize=(10, 8))

    # Plot AO
    x, y = ao_poly.exterior.xy
    plt.plot(x, y, 'b-', linewidth=2, label='Area of Operations (AO)')

    # Plot NFZs
    for i, nfz in enumerate(nfz_polys):
        x, y = nfz.exterior.xy
        label = 'No-Fly Zone (NFZ)' if i == 0 else None
        plt.fill(x, y, 'r-', alpha=0.5, label=label)

    # Plot waypoints
    if waypoints_utm:
        wx, wy = zip(*waypoints_utm)
        plt.scatter(wx, wy, c='g', s=1, label='Waypoints')

    plt.legend()
    plt.axis('equal')
    plt.title('AO, NFZs, and Sampled Waypoints')
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    
    if output_image:
        plt.savefig(output_image, dpi=300, bbox_inches='tight')
        print(f"Saved visualization to {output_image}")
    
    if show:
        plt.show()
    else:
        plt.close()


def main():
    parser = argparse.ArgumentParser(description='Sample waypoints from KML polygons')
    parser.add_argument('--kml', type=str, default='pennovation.kml',
                        help='Path to KML file with AO and NFZ polygons')
    parser.add_argument('--resolution', type=float, default=2.0,
                        help='Waypoint spacing in meters (default: 2.0)')
    parser.add_argument('--output', type=str, default='waypoints_data.json',
                        help='Output JSON file path (default: waypoints_data.json)')
    parser.add_argument('--output_image', type=str, default='waypoints.png',
                        help='Output visualization image (default: waypoints.png)')
    parser.add_argument('--viz', action='store_true',
                        help='Skip visualization')
    
    args = parser.parse_args()

    print(f"Parsing KML file: {args.kml}")
    ao_coords, nfz_coords_list = parse_kml(args.kml)

    print(f"Sampling waypoints with {args.resolution}m spacing...")
    waypoints_utm, waypoints_latlon, ao_poly, nfz_polys = sample_waypoints(
        ao_coords, nfz_coords_list, args.resolution
    )

    print(f"Number of sampled waypoints: {len(waypoints_utm)}")

    # Save to JSON
    data = {
        "ao": ao_coords,
        "nfzs": nfz_coords_list,
        "waypoints": waypoints_latlon
    }
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Saved waypoint data to {args.output}")

    # Visualization
    if args.viz:
        visualize(ao_poly, nfz_polys, waypoints_utm, args.output_image)


if __name__ == '__main__':
    main()
