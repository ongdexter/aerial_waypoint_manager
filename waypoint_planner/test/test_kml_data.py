"""Regression coverage for portable waypoint bundles."""
import pickle
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import build_waypoint_data as builder
from extract_image_data import parse_kml as parse_map_kml
from waypoint_sampler import parse_kml
from waypoint_planner.kml_data import read_kml, select_site_file


KML = b'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><Folder>
<Placemark><name>ao_main</name><Polygon><outerBoundaryIs><LinearRing>
<coordinates>-75,40,0 -74.9998,40,0 -74.9998,40.0002,0 -75,40.0002,0 -75,40,0</coordinates>
</LinearRing></outerBoundaryIs></Polygon></Placemark>
<Placemark><name>nfz_building</name><Polygon><outerBoundaryIs><LinearRing>
<coordinates>-75,40,0 -74.99999,40,0 -74.99999,40.00001,0 -75,40,0</coordinates>
</LinearRing></outerBoundaryIs></Polygon></Placemark>
</Folder></Document></kml>'''


def test_bundle_survives_missing_source(tmp_path, monkeypatch):
    source = tmp_path / 'site.kml'
    bundle = source.with_suffix('.pkl')
    source.write_bytes(KML)
    expected = parse_kml(source)
    expected_map = parse_map_kml(source)
    monkeypatch.setattr(builder, 'get_clean_satellite_with_bounds',
                        lambda *args, **kwargs: (np.zeros((2, 3, 3), dtype=np.uint8), {}))
    with patch.object(sys, 'argv', ['build_waypoint_data.py', '--kml', str(source)]):
        builder.main()
    source.unlink()
    with bundle.open('rb') as stream:
        data = pickle.load(stream)
    assert data['format_version'] == 2
    assert data['graph'].number_of_nodes() == len(data['waypoints']) > 0
    assert data['nfz_names'] == ['nfz_building']
    assert read_kml(bundle) == KML
    assert parse_kml(bundle) == expected
    assert parse_map_kml(bundle) == expected_map


def test_old_bundle_requires_rebuild(tmp_path):
    bundle = tmp_path / 'old.pkl'
    bundle.write_bytes(pickle.dumps({'source_kml': 'missing.kml', 'ao': []}))
    with pytest.raises(ValueError, match='rebuild'):
        read_kml(bundle)


def test_launch_input_selection():
    assert select_site_file('site.pkl', '') == 'site.pkl'
    assert select_site_file('', 'site.kml') == 'site.kml'
    assert select_site_file('', '') == ''
    with pytest.raises(ValueError, match='only one'):
        select_site_file('site.pkl', 'other.kml')
