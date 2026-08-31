#!/usr/bin/env python3
"""Build all waypoint-planner data from one KML file into one pickle file."""

import argparse
import pickle

from extract_image_data import get_clean_satellite_with_bounds
from waypoint_graph import build_graph
from waypoint_sampler import parse_kml, sample_waypoints, visualize


def polygon_for_map(coords):
    """Convert KML (longitude, latitude, altitude) coordinates for the GUI."""
    return [(lat, lon) for lon, lat, *_ in coords]


def build_waypoint_data(kml_file, output_file, resolution=2.0,
                        max_distance=5.0, image_size=(1200, 800),
                        visualization_file=None):
    """Sample points, build their graph, capture the map, and save one pickle."""
    ao_coords, nfz_coords_list = parse_kml(kml_file)
    waypoints_utm, waypoints, ao_poly, nfz_polys = sample_waypoints(
        ao_coords, nfz_coords_list, resolution
    )
    if not waypoints:
        raise ValueError('No waypoints were sampled; check the KML and resolution')

    graph_input = {
        'waypoints': waypoints,
        'ao': ao_coords,
        'nfzs': nfz_coords_list,
    }
    graph = build_graph(graph_input, max_dist=max_distance)

    ao_map_coords = polygon_for_map(ao_coords)
    nfz_map_coords = [polygon_for_map(coords) for coords in nfz_coords_list]
    image, bounds = get_clean_satellite_with_bounds(
        ao_map_coords, nfz_map_coords, image_size=image_size
    )

    data = {
        'format_version': 1,
        'graph': graph,
        'waypoints': waypoints,
        'ao': ao_coords,
        'nfzs': nfz_coords_list,
        'image': image,
        'bounds': bounds,
        'image_shape': image.shape,
        'ao_coords': ao_map_coords,
        'nfz_coords_list': nfz_map_coords,
        'nfz_names': [f'nfz_{index + 1}' for index in range(len(nfz_map_coords))],
        'resolution': resolution,
        'max_graph_distance': max_distance,
        'source_kml': kml_file,
    }
    with open(output_file, 'wb') as output:
        pickle.dump(data, output, protocol=pickle.HIGHEST_PROTOCOL)

    print(
        f'Saved {len(waypoints)} waypoints, {graph.number_of_edges()} graph edges, '
        f'and a {image.shape[1]}x{image.shape[0]} map to {output_file}'
    )

    if visualization_file:
        visualize(ao_poly, nfz_polys, waypoints_utm, visualization_file, show=False)

    return data


def main():
    parser = argparse.ArgumentParser(
        description='Build waypoint graph and satellite map in one pickle file'
    )
    parser.add_argument('--kml', required=True, help='KML containing ao and nfz_* polygons')
    parser.add_argument('--output', default='waypoint_data.pkl', help='Output pickle path')
    parser.add_argument('--resolution', type=float, default=2.0, help='Waypoint spacing in meters')
    parser.add_argument('--max-distance', type=float, default=5.0, help='Maximum graph edge length in meters')
    parser.add_argument('--image-width', type=int, default=1200)
    parser.add_argument('--image-height', type=int, default=800)
    parser.add_argument('--visualization', help='Optional sampled-waypoint preview image')
    args = parser.parse_args()

    build_waypoint_data(
        args.kml,
        args.output,
        resolution=args.resolution,
        max_distance=args.max_distance,
        image_size=(args.image_width, args.image_height),
        visualization_file=args.visualization,
    )


if __name__ == '__main__':
    main()
