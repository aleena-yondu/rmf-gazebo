import os

VALID_LOCATIONS = ('YONDU', 'SHIPBOTS', 'GPA')
VALID_LIDARS = ('LIVOX', 'OUSTER')
DEFAULT_LOCATION = 'YONDU'
DEFAULT_LIDAR = 'LIVOX'
DEFAULT_MAP_NAME = 'map'
# Lidar-aware raw IMU source fed into online_imu_bias_estimator. Keeping this
# per-lidar prevents an OUSTER run from silently consuming the Livox IMU when
# $IMU_SOURCE is unset (the estimator's input_topic is always overridden from
# here in sensor_pipeline.py, so a wrong default here wins over the YAML).
DEFAULT_IMU_SOURCE_BY_LIDAR = {
    'LIVOX': '/livox/imu',
    'OUSTER': '/ouster/imu',
}
# Fallback used if the lidar is somehow unrecognized.
DEFAULT_IMU_SOURCE = '/livox/imu'


def get_location():
    loc = os.environ.get('LOCATION', DEFAULT_LOCATION).upper()
    if loc not in VALID_LOCATIONS:
        print(f"[config_resolver] Unknown LOCATION='{loc}', defaulting to {DEFAULT_LOCATION}")
        return DEFAULT_LOCATION
    return loc


def get_lidar():
    lidar = os.environ.get('LIDAR', DEFAULT_LIDAR).upper()
    if lidar not in VALID_LIDARS:
        print(f"[config_resolver] Unknown LIDAR='{lidar}', defaulting to {DEFAULT_LIDAR}")
        return DEFAULT_LIDAR
    return lidar


def get_map_name():
    """Resolve MAP_NAME env var to a sanitized map slug.

    Strips any directory components and recognized extensions so callers can
    safely append `.posegraph`/`.yaml`. Falls back to DEFAULT_MAP_NAME if the
    env var is empty after sanitization.
    """
    raw = os.environ.get('MAP_NAME', DEFAULT_MAP_NAME).strip()
    if not raw:
        return DEFAULT_MAP_NAME
    # Disallow directory traversal / paths -- map_name is a filename slug only.
    name = os.path.basename(raw)
    # Strip recognized extensions so MAP_NAME=foo.posegraph and MAP_NAME=foo
    # both resolve to the same base path.
    for ext in ('.posegraph', '.data', '.yaml', '.pgm'):
        if name.endswith(ext):
            name = name[: -len(ext)]
            break
    if not name:
        print(f"[config_resolver] MAP_NAME='{raw}' sanitized to empty, defaulting to {DEFAULT_MAP_NAME}")
        return DEFAULT_MAP_NAME
    return name


def get_imu_source(lidar=None):
    """Topic name to feed into the online IMU bias estimator (input_topic).

    Precedence:
      1. Explicit $IMU_SOURCE env var (field experiments / source swaps).
      2. Lidar-aware default (LIVOX -> /livox/imu, OUSTER -> /ouster/imu).

    The lidar-aware default matters because sensor_pipeline.py passes this
    value AFTER the YAML in the parameter list, so it wins over the per-lidar
    imu_bias_estimator.yaml input_topic. A flat /livox/imu default silently
    fed the Livox IMU into an OUSTER bias estimator when $IMU_SOURCE was unset.
    """
    src = os.environ.get('IMU_SOURCE', '').strip()
    if src:
        return src
    lid = (lidar or get_lidar()).upper()
    return DEFAULT_IMU_SOURCE_BY_LIDAR.get(lid, DEFAULT_IMU_SOURCE)


def get_map_path(pkg_share, location=None, lidar=None, map_name=None, with_extension=True):
    """Returns path to map file. Without extension for SLAM Toolbox, with .yaml for Nav2."""
    loc = (location or get_location()).lower()
    lid = (lidar or get_lidar()).lower()
    name = map_name or get_map_name()
    base = os.path.join(pkg_share, 'maps', loc, lid, name)
    return base + '.yaml' if with_extension else base


def get_warehouse_config_path(pkg_share, location=None, lidar=None):
    loc = (location or get_location()).lower()
    lid = (lidar or get_lidar()).lower()
    return os.path.join(pkg_share, 'config', 'warehouse_configs', loc, lid, 'warehouse.yaml')


def get_ik_config_path(pkg_share, location=None, lidar=None):
    loc = (location or get_location()).lower()
    lid = (lidar or get_lidar()).lower()
    return os.path.join(pkg_share, 'config', 'warehouse_configs', loc, lid, 'ik.yaml')


def get_tag_detections_config_path(pkg_share, location=None, lidar=None):
    loc = (location or get_location()).lower()
    lid = (lidar or get_lidar()).lower()
    return os.path.join(pkg_share, 'config', 'warehouse_configs', loc, lid, 'tag_detections.yaml')


def get_nav_config_path(pkg_share, filename, lidar=None):
    """Returns path to a lidar-specific nav config file under {lidar}/ subfolder."""
    lid = (lidar or get_lidar()).lower()
    return os.path.join(pkg_share, 'config', 'nav_configs', lid, filename)


def get_shared_nav_config_path(pkg_share, filename):
    """Returns path to a shared (lidar-agnostic) nav config file at the root of nav_configs/."""
    return os.path.join(pkg_share, 'config', 'nav_configs', filename)
