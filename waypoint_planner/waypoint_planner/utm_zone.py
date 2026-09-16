"""UTM zone selection from a reference coordinate.

The zone used here has to agree with the one every other component derives for
the same site -- gnocchi's datum, air_sem_explorer's KML zones, plan_env's
restricted zones -- because those all exchange coordinates through the same
world frame. Hardcoding a zone is what makes them silently disagree the first
time the vehicle flies somewhere else, so each side derives it from its own
data and logs it, and the logs are what make a mismatch visible.
"""


def epsg_for(lon: float, lat: float) -> int:
    """EPSG code of the WGS84 UTM zone containing (lon, lat)."""
    zone = int((lon + 180.0) / 6.0) + 1
    return (32600 if lat >= 0.0 else 32700) + zone


def zone_name(lon: float, lat: float) -> str:
    """Human-readable zone, e.g. '18N' -- for logging alongside the EPSG code."""
    zone = int((lon + 180.0) / 6.0) + 1
    return f"{zone}{'N' if lat >= 0.0 else 'S'}"


def transformers_for(lon: float, lat: float):
    """(to_utm, from_utm, epsg) for the zone containing the reference point."""
    from pyproj import Transformer

    epsg = epsg_for(lon, lat)
    to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    from_utm = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)
    return to_utm, from_utm, epsg
