"""Read KML documents directly or from self-contained waypoint bundles."""

import os
from pathlib import Path
import pickle


def read_kml(filename):
    """Return original KML bytes. Pickle inputs must be trusted waypoint bundles."""
    path = Path(os.path.expandvars(str(filename))).expanduser()
    if path.suffix.lower() != '.pkl':
        return path.read_bytes()
    with path.open('rb') as stream:
        data = pickle.load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('kml_data'), (bytes, str)):
        raise ValueError(
            f'{path} has no embedded KML; rebuild it with build_waypoint_data.py'
        )
    content = data['kml_data']
    return content.encode('utf-8') if isinstance(content, str) else content


def select_site_file(pkl_file, kml_file):
    """Choose a launch input, rejecting ambiguous site overrides."""
    if pkl_file and kml_file:
        raise ValueError('Specify only one of pkl_file or kml_file')
    return pkl_file or kml_file
