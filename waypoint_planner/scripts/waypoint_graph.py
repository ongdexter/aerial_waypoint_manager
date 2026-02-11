import json
import argparse
import networkx as nx
from shapely.geometry import Polygon, Point
from pyproj import Transformer
import matplotlib.pyplot as plt
import os
from scipy.spatial import KDTree
import pickle
import random
import time

def create_graph(waypoint_data_file, graph_file):
    # load sampled waypoints
    with open(waypoint_data_file, 'r') as f:
        data = json.load(f)

    waypoints = data['waypoints']  # list of (lon, lat)
    ao_coords = data['ao']
    nfz_coords_list = data['nfzs']

    # UTM transformer
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32618", always_xy=True)

    # transform to UTM
    waypoints_utm = [transformer_to_utm.transform(lon, lat) for lon, lat in waypoints]
    ao_utm = [transformer_to_utm.transform(lon, lat) for lon, lat, alt in ao_coords]
    nfzs_utm = [[transformer_to_utm.transform(lon, lat) for lon, lat, alt in nfz] for nfz in nfz_coords_list]

    # build graph
    print("Building graph...")
    G = nx.Graph()
    for i, wp in enumerate(waypoints_utm):
        G.add_node(i, pos=wp)

    tree = KDTree(waypoints_utm)
    max_dist = 5.0  # meters
    for i, wp in enumerate(waypoints_utm):
        neighbors = tree.query_ball_point(wp, max_dist)
        for j in neighbors:
            if i < j:
                dist = Point(wp).distance(Point(waypoints_utm[j]))
                G.add_edge(i, j, weight=dist)
    
    # Save consolidated data (graph + waypoint data) in single file
    consolidated_data = {
        'graph': G,
        'waypoints': waypoints,
        'ao': ao_coords,
        'nfzs': nfz_coords_list
    }
    with open(graph_file, 'wb') as f:
        pickle.dump(consolidated_data, f)
    print(f"Graph built and saved to {graph_file} (consolidated format)")

def test(graph_file, waypoint_data_file=None):
    """Test path planning. If graph_file is consolidated format, waypoint_data_file is ignored."""
    if not os.path.exists(graph_file):
        print("Graph file not found")
        return
    
    with open(graph_file, 'rb') as f:
        loaded_data = pickle.load(f)
    
    # Check if consolidated format (dict) or legacy format (just graph)
    if isinstance(loaded_data, dict):
        print("Loaded consolidated graph file")
        G = loaded_data['graph']
        waypoints = loaded_data['waypoints']
        ao_coords = loaded_data['ao']
        nfz_coords_list = loaded_data['nfzs']
    else:
        # Legacy format: graph only, need separate waypoint file
        print("Loaded legacy graph format, loading waypoints from separate file")
        G = loaded_data
        if waypoint_data_file is None or not os.path.exists(waypoint_data_file):
            print("Waypoint data file required for legacy format but not found")
            return
        with open(waypoint_data_file, 'r') as f:
            data = json.load(f)
        waypoints = data['waypoints']
        ao_coords = data['ao']
        nfz_coords_list = data['nfzs']

    # UTM transformer
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32618", always_xy=True)

    # transform to UTM
    waypoints_utm = [transformer_to_utm.transform(lon, lat) for lon, lat in waypoints]
    ao_utm = [transformer_to_utm.transform(lon, lat) for lon, lat, alt in ao_coords]
    nfzs_utm = [[transformer_to_utm.transform(lon, lat) for lon, lat, alt in nfz] for nfz in nfz_coords_list]

    # create polygons for viz
    ao_poly = Polygon(ao_utm)
    nfz_polys = [Polygon(nfz_utm) for nfz_utm in nfzs_utm]

    num_trials = 10
    for _ in range(num_trials):
        start_idx = random.randint(0, len(waypoints) - 1)
        goal_idx = random.randint(0, len(waypoints) - 1)

        if not (0 <= start_idx < len(waypoints) and 0 <= goal_idx < len(waypoints)):
            raise ValueError("Invalid waypoint indices")

        # Plan path
        try:
            start_time = time.time()
            path = nx.shortest_path(G, start_idx, goal_idx, weight='weight')
            end_time = time.time()
            print(f"Path found in {end_time - start_time:.4f} seconds")
            print(f"Path found with {len(path)} waypoints")
        except nx.NetworkXNoPath:
            raise ValueError("No path found between start and goal")

        # Visualization
        plt.figure(figsize=(10, 8))

        # Plot AO
        x, y = ao_poly.exterior.xy
        plt.plot(x, y, 'orange', linewidth=2, label='Area of Operations (AO)')

        # Plot NFZs
        for i, nfz in enumerate(nfz_polys):
            x, y = nfz.exterior.xy
            label = 'No-Fly Zone (NFZ)' if i == 0 else None
            plt.fill(x, y, 'r-', alpha=0.5, label=label)
            plt.plot(x, y, 'k-', linewidth=1)

        # Plot all waypoints
        wx, wy = zip(*waypoints_utm)
        plt.scatter(wx, wy, c='g', s=1, label='Waypoints')

        # Plot path
        path_utm = [waypoints_utm[i] for i in path]
        px, py = zip(*path_utm)
        plt.plot(px, py, 'r-', linewidth=3, label='Planned Path')

        # Mark start and end
        start_pos = waypoints_utm[path[0]]
        end_pos = waypoints_utm[path[-1]]
        plt.scatter(start_pos[0], start_pos[1], c='blue', s=100, marker='o', label='Start', zorder=5)
        plt.scatter(end_pos[0], end_pos[1], c='yellow', s=150, marker='*', label='Goal', zorder=5)

        plt.legend()
        plt.axis('equal')
        plt.title(f'Planned Path from Waypoint {start_idx} to {goal_idx}')
        plt.xlabel('Easting (m)')
        plt.ylabel('Northing (m)')
        plt.show()

        print("Path visualization saved to planned_path.png")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Waypoint Planner")
    parser.add_argument('--test', action='store_true', help='Test path planning on the graph')
    parser.add_argument('--waypoint_data', type=str, default='waypoints_data.json', help='Path to waypoint data JSON file')
    parser.add_argument('--graph_file', type=str, default='graph.pkl', help='Path to save/load the graph pickle file')

    args = parser.parse_args()    

    if not os.path.exists(args.waypoint_data):
        raise FileNotFoundError(f"Waypoint data file {args.waypoint_data} not found")
    if not os.path.exists(args.graph_file):            
        create_graph(args.waypoint_data, args.graph_file)
    else:
        print(f"Graph file {args.graph_file} already exists.")
    
    if args.test:
        test(args.graph_file, args.waypoint_data)